#include "MonitorIntervalQueue.h"
#include <iostream>

//#define DEBUG_UTILITY_CALC
//#define DEBUG_MONITOR_INTERVAL_QUEUE_ACKS
//#define DEBUG_MONITOR_INTERVAL_QUEUE_LOSS
//#define DEBUG_INTERVAL_SIZE

namespace
{
	// Number of probing MonitorIntervals necessary for Probing.
	//const size_t kRoundsPerProbing = 4;
	// Tolerance of loss rate by utility function.
	const float kLossTolerance = 0.05f;
	// Coefficeint of the loss rate term in utility function.
	const float kLossCoefficient = -1000.0f;
	// Coefficient of RTT term in utility function.
	const float kRTTCoefficient = -200.0f;
	// Number of microseconds per second.
	const float kNumMicrosPerSecond = 1000000.0f;
	// Coefficienty of the latency term in the utility function.
	const float kLatencyCoefficient = 1;
	// Alpha factor in the utility function.
	const float kAlpha = 1;
	// An exponent in the utility function.
	const float kExponent = 0.9;
	// An exponent in the utility function.
	const size_t kMegabit = 1024 * 1024;
} // namespace

PacketRttSample::PacketRttSample(QuicPacketNumber packet_number, QuicTime rtt) : 
	packet_number(packet_number),
	sample_rtt(rtt) 
{
}

MonitorInterval::MonitorInterval(QuicBandwidth sending_rate,
				 bool is_useful,
				 float rtt_fluctuation_tolerance_ratio,
				 int64_t rtt_us,
				 QuicTime end_time) :
	sending_rate(sending_rate),
	is_useful(is_useful),
	rtt_fluctuation_tolerance_ratio(rtt_fluctuation_tolerance_ratio),
	end_time(end_time),
	rtt_on_monitor_start_us(rtt_us),
	rtt_on_monitor_end_us(rtt_us)
{
}

UtilityInfo::UtilityInfo(QuicBandwidth rate, float utility) :
	sending_rate(rate),
	utility(utility) 
{
}

MonitorIntervalQueue::MonitorIntervalQueue(MonitorIntervalQueueDelegateInterface& delegate) :
	delegate_(delegate) 
{
}

void MonitorIntervalQueue::EnqueueNewMonitorInterval(QuicBandwidth sending_rate,bool is_useful, float rtt_fluctuation_tolerance_ratio, int64_t rtt_us, QuicTime end_time)
{
	if (is_useful)
		++num_useful_intervals_;

	monitor_intervals_.emplace_back(sending_rate, is_useful, rtt_fluctuation_tolerance_ratio, rtt_us, end_time);
}

void MonitorIntervalQueue::OnPacketSent(QuicTime sent_time, QuicPacketNumber packet_number, QuicByteCount bytes)
{
	if (monitor_intervals_.empty())
		return;

	if (monitor_intervals_.back().bytes_sent == 0)
	{
		// This is the first packet of this interval.
		monitor_intervals_.back().first_packet_sent_time = sent_time;
		monitor_intervals_.back().first_packet_number = packet_number;
	}

	monitor_intervals_.back().last_packet_sent_time = sent_time;
	monitor_intervals_.back().last_packet_number = packet_number;
	monitor_intervals_.back().bytes_sent += bytes;
	++monitor_intervals_.back().n_packets;
#if ! defined(QUIC_PORT) && defined(DEBUG_INTERVAL_SIZE)
	if (monitor_intervals_.back().is_useful)
	{
		std::cerr << "Added packet " << packet_number
			<< " to monitor interval, now "
			<< monitor_intervals_.back().bytes_sent << " bytes " << std::endl;
	}
#endif
}

void MonitorIntervalQueue::OnCongestionEvent( const AckedPacketVector& acked_packets, const LostPacketVector& lost_packets, int64_t rtt_us, QuicTime event_time)
{
	num_available_intervals_ = 0;
	if (num_useful_intervals_ == 0)
		// Skip all the received packets if no intervals are useful.
		return;

	bool has_invalid_utility = false;
	for (MonitorInterval& interval : monitor_intervals_)
	{
		if (!interval.is_useful)
			// Skips useless monitor intervals.
			continue;

		if (IsUtilityAvailable(interval, event_time))
		{
			// Skips intervals that have available utilities.
			++num_available_intervals_;
			continue;
		}

		for (const LostPacket& lost_packet : lost_packets)
		{
			if (IntervalContainsPacket(interval, lost_packet.packet_number))
			{
				interval.bytes_lost += lost_packet.bytes_lost;
#if (! defined(QUIC_PORT)) && defined(DEBUG_MONITOR_INTERVAL_QUEUE_LOSS)
				std::cerr << "\tattributed bytes to an interval" << std::endl;
				std::cerr << "\tacked " << interval.bytes_acked << "/"
					<< interval.bytes_sent << std::endl;
				std::cerr << "\tlost " << interval.bytes_lost << "/"
					<< interval.bytes_sent << std::endl;
				std::cerr << "\ttotal " << interval.bytes_lost + interval.bytes_acked
					<< "/" << interval.bytes_sent << std::endl;
#endif
			}
		}

		for (const AckedPacket& acked_packet : acked_packets)
		{
			if (IntervalContainsPacket(interval, acked_packet.packet_number))
			{
				interval.bytes_acked += acked_packet.bytes_acked;
				interval.packet_rtt_samples.push_back(PacketRttSample(acked_packet.packet_number, rtt_us));

#if (! defined(QUIC_PORT)) && defined(DEBUG_MONITOR_INTERVAL_QUEUE_ACKS)
				std::cerr << "\tattributed bytes to an interval" << std::endl;
				std::cerr << "\tacked " << interval.bytes_acked << "/"
					<< interval.bytes_sent << std::endl;
				std::cerr << "\tlost " << interval.bytes_lost << "/"
					<< interval.bytes_sent << std::endl;
				std::cerr << "\ttotal " << interval.bytes_lost + interval.bytes_acked
					<< "/" << interval.bytes_sent << std::endl;
#endif
			}
		}

		if (IsUtilityAvailable(interval, event_time))
		{
			interval.rtt_on_monitor_end_us = rtt_us;
			has_invalid_utility = !CalculateUtility(&interval);
			if (has_invalid_utility)
				break;
			++num_available_intervals_;
		}
	}

#if (!defined(QUIC_PORT)) && defined(DEBUG_INTERVAL_SIZE)
	std::cerr << "Num useful = " << num_useful_intervals_ << ", num avail = "
		<< num_available_intervals_ << std::endl;
#endif

	if (num_useful_intervals_ > num_available_intervals_ && !has_invalid_utility)
		return;

	if (!has_invalid_utility)
	{
		std::vector<UtilityInfo> utility_info;
		for (const MonitorInterval& interval : monitor_intervals_)
		{
			if (!interval.is_useful)
				continue;
			// All the useful intervals should have available utilities now.
			utility_info.push_back(UtilityInfo(interval.sending_rate, interval.utility));
		}

		delegate_.OnUtilityAvailable(utility_info);
	}

	// Remove MonitorIntervals from the head of the queue,
	// until all useful intervals are removed.
	while (num_useful_intervals_ > 0)
	{
		if (monitor_intervals_.front().is_useful)
			--num_useful_intervals_;
		monitor_intervals_.pop_front();
	}
	num_available_intervals_ = 0;
}

const MonitorInterval& MonitorIntervalQueue::current() const
{
	return monitor_intervals_.back();
}

bool MonitorIntervalQueue::empty() const
{
	return monitor_intervals_.empty();
}

size_t MonitorIntervalQueue::size() const
{
	return monitor_intervals_.size();
}

void MonitorIntervalQueue::OnRttInflationInStarting()
{
	monitor_intervals_.clear();
	num_useful_intervals_ = 0;
	num_available_intervals_ = 0;
}

bool MonitorIntervalQueue::IsUtilityAvailable(const MonitorInterval& interval, QuicTime event_time) const
{
	return (event_time >= interval.end_time && interval.bytes_acked + interval.bytes_lost == interval.bytes_sent);
}

bool MonitorIntervalQueue::IntervalContainsPacket(const MonitorInterval& interval, QuicPacketNumber packet_number) const
{
#if ! defined(QUIC_PORT) && (defined(DEBUG_MONITOR_INTERVAL_QUEUE_LOSS) || defined(DEBUG_MONITOR_INTERVAL_QUEUE_ACKS))
	std::cerr << "Checking for packet " << packet_number << " in interval: ["
		<< interval.first_packet_number << ", "
		<< interval.last_packet_number << "]" << std::endl;
#endif
	return (packet_number >= interval.first_packet_number && packet_number <= interval.last_packet_number);
}

/*
bool MonitorIntervalQueue::CalculateUtility(MonitorInterval* interval)
{
	if (interval->last_packet_sent_time == interval->first_packet_sent_time)
	{
		// Cannot get valid utility if interval only contains one packet.
		return false;
	}
	const int64_t kMinTransmissionTime = 1l;
	int64_t mi_duration = std::max(
				       kMinTransmissionTime,
				       (interval->last_packet_sent_time - interval->first_packet_sent_time)
				       .ToMicroseconds());

	float rtt_ratio = static_cast<float> (interval->rtt_on_monitor_start_us) /
		static_cast<float> (interval->rtt_on_monitor_end_us);
	if (rtt_ratio > 1.0 - interval->rtt_fluctuation_tolerance_ratio &&
		rtt_ratio < 1.0 + interval->rtt_fluctuation_tolerance_ratio)
	{
		rtt_ratio = 1.0;
	}
	float latency_penalty =
		1.0 - 1.0 / (1.0 + exp(kRTTCoefficient * (1.0 - rtt_ratio)));

	float bytes_acked = static_cast<float> (interval->bytes_acked);
	float bytes_lost = static_cast<float> (interval->bytes_lost);
	float bytes_sent = static_cast<float> (interval->bytes_sent);
	float loss_rate = bytes_lost / bytes_sent;
	float loss_penalty =
		1.0 - 1.0 / (1.0 + exp(kLossCoefficient * (loss_rate - kLossTolerance)));

	interval->utility = (bytes_acked / static_cast<float> (mi_duration) *
		loss_penalty * latency_penalty -
		bytes_lost / static_cast<float> (mi_duration)) *
		1000.0;

	return true;
}
 */

bool MonitorIntervalQueue::CalculateUtility(MonitorInterval* interval)
{
	if (interval->last_packet_sent_time == interval->first_packet_sent_time)
		// Cannot get valid utility if interval only contains one packet.
		return false;
	
	const int64_t kMinTransmissionTime = 1l;
	int64_t mi_duration = std::max(kMinTransmissionTime, (interval->last_packet_sent_time - interval->first_packet_sent_time));

	float mi_time_seconds = static_cast<float> (mi_duration) / kNumMicrosPerSecond;
	float bytes_lost = static_cast<float> (interval->bytes_lost);
	float bytes_sent = static_cast<float> (interval->bytes_sent);

	float sending_rate_bps = bytes_sent * 8.0f / mi_time_seconds;
	float sending_factor = kAlpha * pow(sending_rate_bps / kMegabit, kExponent);

	// Approximate the derivative at each point by computing the slope of RTT to
	// the following point and average these values.
	float rtt_first_half_sum = 0.0;
	float rtt_second_half_sum = 0.0;
	int half_samples = interval->packet_rtt_samples.size() / 2;
	for (int i = 0; i < half_samples; ++i)
	{
		rtt_first_half_sum += static_cast<float> (interval->packet_rtt_samples[i].sample_rtt);
		rtt_second_half_sum += static_cast<float> (interval->packet_rtt_samples[i + half_samples].sample_rtt);
	}
	float latency_inflation = 2.0 * (rtt_second_half_sum - rtt_first_half_sum) / (rtt_first_half_sum + rtt_second_half_sum);

	float rtt_penalty = int(int(latency_inflation * 100) / 100.0 * 100) / 2 * 2 / 100.0;
	float rtt_contribution = kLatencyCoefficient * 11330 * bytes_sent * (pow(rtt_penalty, 1));

	float loss_rate = bytes_lost / bytes_sent;
	float loss_contribution = interval->n_packets * (11.35 * (pow((1 + loss_rate), 1) - 1));
	if (loss_rate <= 0.03)
		loss_contribution = interval->n_packets * (1 * (pow((1 + loss_rate), 1) - 1));
	float current_utility = sending_factor - (loss_contribution + rtt_contribution) * (sending_rate_bps / kMegabit) / static_cast<float> (interval->n_packets);

#if !defined(QUIC_PORT) && defined(DEBUG_UTILITY_CALC)
	std::cerr << "Calculate utility:" << std::endl;
	std::cerr << "\tutility           = " << current_utility << std::endl;
	std::cerr << "\tn_packets         = " << interval->n_packets << std::endl;
	std::cerr << "\ttarget send_rate  = " << interval->sending_rate / 1000000.0f
		<< std::endl;
	std::cerr << "\tactual send_rate  = "
		<< bytes_sent * 8.0f / (mi_time_seconds * 1000000.0f) << std::endl;
	std::cerr << "\tthroughput        = "
		<< (bytes_sent - bytes_lost) * 8.0f / (mi_time_seconds * 1000000.0f)
		<< std::endl;
	std::cerr << "\tthroughput factor = " << sending_factor << std::endl;
	std::cerr << "\tavg_rtt           = "
		<< (rtt_first_half_sum + rtt_second_half_sum) / (2 * half_samples)
		<< std::endl;
	std::cerr << "\tlatency_infla.    = " << latency_inflation << std::endl;
	std::cerr << "\trtt_contribution  = " << rtt_contribution << std::endl;
	std::cerr << "\tloss_rate         = " << loss_rate << std::endl;
	std::cerr << "\tloss_contribution = " << loss_contribution << std::endl;
#endif

	interval->utility = current_utility;
	return true;
}
