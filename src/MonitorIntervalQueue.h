#ifndef THIRD_PARTY_PCC_QUIC_PCC_MONITOR_QUEUE_H_
#define THIRD_PARTY_PCC_QUIC_PCC_MONITOR_QUEUE_H_

#include <deque>
#include <utility>
#include <vector>

#include <cstdint>
#include <cstdlib>
#include <cmath>

typedef int32_t QuicPacketCount;
typedef int32_t QuicPacketNumber;
typedef int64_t QuicByteCount;
typedef int64_t QuicTime;
typedef double QuicBandwidth;

struct CongestionEvent
{
	int32_t packet_number;
	int32_t bytes_acked;
	int32_t bytes_lost;
	uint64_t time;
};

typedef CongestionEvent AckedPacket;
typedef CongestionEvent LostPacket;
typedef std::vector<CongestionEvent> AckedPacketVector;
typedef std::vector<CongestionEvent> LostPacketVector;


// PacketRttSample, stores the packet number and its corresponding RTT

struct PacketRttSample
{
	PacketRttSample(QuicPacketNumber packet_number, QuicTime rtt);

	// Packet number of the sampled packet.
	QuicPacketNumber packet_number = 0;
	QuicTime sample_rtt = 0;
};

// MonitorInterval, as the queue's entry struct, stores the information
// of a PCC monitor interval (MonitorInterval) that can be used to
// - pinpoint a acked/lost packet to the corresponding MonitorInterval,
// - calculate the MonitorInterval's utility value.

struct MonitorInterval
{
	MonitorInterval(QuicBandwidth sending_rate,
		bool is_useful,
		float rtt_fluctuation_tolerance_ratio,
		int64_t rtt_us,
		QuicTime end_time);

	// Sending rate.
	QuicBandwidth sending_rate = 0;
	// True if calculating utility for this MonitorInterval.
	bool is_useful = false;
	// The tolerable rtt fluctuation ratio.
	float rtt_fluctuation_tolerance_ratio = 0.0f;
	// The end time for this monitor interval in microseconds.
	QuicTime end_time = 0;

	// Sent time of the first packet.
	QuicTime first_packet_sent_time = 0;
	// Sent time of the last packet.
	QuicTime last_packet_sent_time = 0;

	// PacketNumber of the first sent packet.
	QuicPacketNumber first_packet_number = 0;
	// PacketNumber of the last sent packet.
	QuicPacketNumber last_packet_number = 0;

	// Number of bytes which are sent in total.
	QuicByteCount bytes_sent = 0;
	// Number of bytes which have been acked.
	QuicByteCount bytes_acked = 0;
	// Number of bytes which are considered as lost.
	QuicByteCount bytes_lost = 0;

	// RTT when the first packet is sent.
	int64_t rtt_on_monitor_start_us = 0;
	// RTT when all sent packets are either acked or lost.
	int64_t rtt_on_monitor_end_us = 0;

	// Utility value of this MonitorInterval, which is calculated
	// when all sent packets are either acked or lost.
	float utility = 0.0f;

	// The number of packets in this monitor interval.
	int n_packets = 0;
	// A sample of the RTT for each packet.
	std::vector<PacketRttSample> packet_rtt_samples;
};

// UtilityInfo is used to store <sending_rate, utility> pairs

struct UtilityInfo
{
	UtilityInfo() = default;
	UtilityInfo(QuicBandwidth rate, float utility);
	QuicBandwidth sending_rate = 0;
	float utility = 0.0f;
};

class MonitorIntervalQueueDelegateInterface
{
public:
	virtual ~MonitorIntervalQueueDelegateInterface() = default;
	virtual void OnUtilityAvailable(const std::vector<UtilityInfo>& utility_info) = 0;
	
};

// MonitorIntervalQueue contains a queue of MonitorIntervals.
// New MonitorIntervals are added to the tail of the queue.
// Existing MonitorIntervals are removed from the queue when all
// 'useful' intervals' utilities are available.

class MonitorIntervalQueue
{
public:
	explicit MonitorIntervalQueue(MonitorIntervalQueueDelegateInterface& delegate);
	MonitorIntervalQueue(const MonitorIntervalQueue&) = delete;
	MonitorIntervalQueue& operator=(const MonitorIntervalQueue&) = delete;
	MonitorIntervalQueue(MonitorIntervalQueue&&) = delete;
	MonitorIntervalQueue& operator=(MonitorIntervalQueue&&) = delete;

	// Creates a new MonitorInterval and add it to the tail of the
	// monitor interval queue, provided the necessary variables
	// for MonitorInterval initialization.
	void EnqueueNewMonitorInterval(QuicBandwidth sending_rate,
		bool is_useful,
		float rtt_fluctuation_tolerance_ratio,
		int64_t rtt_us,
		QuicTime end_time);

	// Called when a packet belonging to current monitor interval is sent.
	void OnPacketSent(QuicTime sent_time,
		QuicPacketNumber packet_number,
		QuicByteCount bytes);

	// Called when packets are acked or considered as lost.
	void OnCongestionEvent(const AckedPacketVector& acked_packets,
		const LostPacketVector& lost_packets,
		int64_t rtt_us,
		QuicTime event_time);

	// Called when RTT inflation ratio is greater than
	// max_rtt_fluctuation_tolerance_ratio_in_starting.
	void OnRttInflationInStarting();

	// Returns the most recent MonitorInterval in the tail of the queue
	const MonitorInterval& current() const;
	size_t num_useful_intervals() const { return num_useful_intervals_; }
	size_t num_available_intervals() const { return num_available_intervals_; }
	bool empty() const;
	size_t size() const;

private:
	// Returns true if the utility of |interval| is available, i.e.,
	// when all the interval's packets are either acked or lost.
	bool IsUtilityAvailable(const MonitorInterval& interval,
		QuicTime cur_time) const;

	// Retruns true if |packet_number| belongs to |interval|.
	bool IntervalContainsPacket(const MonitorInterval& interval,
		QuicPacketNumber packet_number) const;

#ifdef QUIC_PORT
	// Calculates utility for |interval|. Returns true if |interval| has valid
	// utility, false otherwise.
	bool CalculateUtility(MonitorInterval* interval);
	// Calculates utility for |interval| using version-2 utility function. Returns
	// true if |interval| has valid utility, false otherwise.
	bool CalculateUtility2(MonitorInterval* interval);
#else
	// Calculates utility for |interval|. Returns true if |interval| has valid
	// utility, false otherwise.
	bool CalculateUtility(MonitorInterval* interval);
#endif

	std::deque<MonitorInterval> monitor_intervals_;
	// Number of useful intervals in the queue.
	size_t num_useful_intervals_ = 0;
	// Number of useful intervals in the queue with available utilities.
	size_t num_available_intervals_ = 0;
	// Delegate interface, not owned.
	MonitorIntervalQueueDelegateInterface& delegate_;
};

#endif  // THIRD_PARTY_PCC_QUIC_PCC_MONITOR_QUEUE_H_
