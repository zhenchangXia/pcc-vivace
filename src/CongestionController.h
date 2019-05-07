// PCC (Performance Oriented Congestion Control) algorithm

#ifndef NET_QUIC_CORE_CONGESTION_CONTROL_PCC_SENDER_H_
#define NET_QUIC_CORE_CONGESTION_CONTROL_PCC_SENDER_H_

#include <vector>
#include <queue>

#include "MonitorIntervalQueue.h"

// CongestionController implements the PCC congestion control algorithm.
// CongestionController evaluates the benefits of different sending rates by 
// comparing their utilities, and adjusts the sending rate towards the direction
// of  higher utility.
class CongestionController : public MonitorIntervalQueueDelegateInterface
{
public:
	// Sender's mode during a connection.

	enum SenderMode
	{
		// Initial phase of the connection. Sending rate gets doubled as
		// long as utility keeps increasing, and the sender enters
		// PROBING mode when utility decreases.
		STARTING,
		// Sender tries different sending rates to decide whether higher
		// or lower sending rate has greater utility. Sender enters
		// DECISION_MADE mode once a decision is made.
		PROBING,
		// Sender keeps increasing or decreasing sending rate until
		// utility decreases, then sender returns to PROBING mode.
		// TODO(tongmeng): a better name?
		DECISION_MADE
	};

	// Indicates whether sender should increase or decrease sending rate.
	enum RateChangeDirection
	{
		INCREASE, DECREASE
	};

	CongestionController(QuicTime initial_rtt_us, QuicPacketCount initial_congestion_window, QuicPacketCount max_congestion_window);
	CongestionController(const CongestionController&) = delete;
	CongestionController& operator=(const CongestionController&) = delete;
	CongestionController(CongestionController&&) = delete;
	CongestionController& operator=(CongestionController&&) = delete;

	
	void OnCongestionEvent(QuicTime event_time,
		QuicTime rtt,
		const AckedPacketVector& acked_packets,
		const LostPacketVector& lost_packets);
	
	void OnPacketSent(QuicTime sent_time,
		QuicPacketNumber packet_number,
		QuicByteCount bytes,
		bool is_retransmittable);

	QuicBandwidth PacingRate() const;
	QuicByteCount GetCongestionWindow() const;
	QuicTime ComputeMonitorDuration(QuicBandwidth sending_rate, QuicTime rtt);
	QuicBandwidth ComputeRateChange(const UtilityInfo& utility_sample_1,const UtilityInfo& utility_sample_2);

	void UpdateAverageGradient(float new_gradient);

	// Implementation of MonitorIntervalQueueDelegate.
	// Called when all useful intervals' utilities are available,
	// so the sender can make a decision.
	void OnUtilityAvailable(const std::vector<UtilityInfo>& utility_info) override;

private:
	// Returns true if next created monitor interval is useful,
	// i.e., its utility will be used when a decision can be made.
	bool CreateUsefulInterval() const;
	// Maybe set sending_rate_ for next created monitor interval.
	void MaybeSetSendingRate();

	// Returns true if the sender can enter DECISION_MADE from PROBING mode.
	bool CanMakeDecision(const std::vector<UtilityInfo>& utility_info) const;
	// Set the sending rate to the central rate used in PROBING mode.
	void EnterProbing();
	// Set the sending rate when entering DECISION_MADE from PROBING mode.
	void EnterDecisionMade(QuicBandwidth new_rate);

	// Current mode of CongestionController.
	SenderMode mode_ = STARTING;
	// Sending rate in Mbit/s for the next monitor intervals.
	QuicBandwidth sending_rate_;
	// Most recent utility used when making the last rate change decision.
	UtilityInfo latest_utility_info_;
	// Duration of the current monitor interval.
	QuicTime monitor_duration_ = 0;
	// Current direction of rate changes.
	RateChangeDirection direction_ = INCREASE;
	// Number of rounds sender remains in current mode.
	size_t rounds_ = 1;
	// Queue of monitor intervals with pending utilities.
	MonitorIntervalQueue interval_queue_;
	// Maximum congestion window in bits, used to cap sending rate.
	uint32_t max_cwnd_bits_;
	// The current average of several utility gradients.
	float avg_gradient_ = 0.0f;
	// The gradient samples that have been averaged.
	std::queue<float> gradient_samples_;

	QuicTime initial_rtt_ = 0;
	QuicTime avg_rtt_ = 0;

	// The number of consecutive rate changes in a single direction
	// before we accelerate the rate of change.
	size_t swing_buffer_ = 0;
	// An acceleration factor for the rate of change.
	float rate_change_amplifier_ = 0.0f;
	// The maximum rate change as a proportion of the current rate.
	size_t rate_change_proportion_allowance_ = 0;
	// The most recent change made to the sending rate.
	QuicBandwidth previous_change_ = 0;
};

#endif
