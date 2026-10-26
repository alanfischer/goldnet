// Automatic rate and command rate (auto_rate.h): the meters that read a link's queue and capacity,
// and the controllers that size a client's rate and command rate from them.

#include "test_support.h"

#include "goldnet/auto_rate.h"

#include <cmath>
#include <cstdlib>

using namespace goldnet;

// --- QueueMeter: a steady stream reads no lag; a building queue reads as its extra wait ---

static const double TICK = 1000.0 / 60.0;

static void test_queue_steady_stream_reads_no_lag() {
	QueueMeter m;
	uint32_t lag = 0;
	for (int seq = 1; seq < 300; seq++) {
		lag = m.note((int64_t)(seq * TICK), 5000 + (uint32_t)(seq * TICK) + 40); // a constant 40 ms trip
	}
	CHECK(lag <= 1);
	printf("  a steady stream reads no lag: ok\n");
}

static void test_queue_building_reads_its_extra_wait() {
	QueueMeter m;
	for (int seq = 1; seq < 120; seq++) {
		m.note((int64_t)(seq * TICK), 5000 + (uint32_t)(seq * TICK) + 40);
	}
	uint32_t lag = 0;
	for (int k = 0; k < 60; k++) { // each packet now waits 5 ms longer than the last
		const int seq = 120 + k;
		lag = m.note((int64_t)(seq * TICK), 5000 + (uint32_t)(seq * TICK) + 40 + 5 * k);
	}
	CHECK(std::abs((int)lag - 295) <= 2);
	printf("  a building queue reads as its extra wait (%u): ok\n", lag);
}

static void test_queue_pause_is_not_lag() {
	QueueMeter m;
	for (int seq = 1; seq < 120; seq++) {
		m.note((int64_t)(seq * TICK), 5000 + (uint32_t)(seq * TICK) + 40);
	}
	// Silent for 3 s, and the next packets are stamped as if no time had passed on the sender.
	const uint32_t t0 = 5000 + (uint32_t)(120 * TICK) + 40 + 3000;
	uint32_t lag = 0;
	for (int k = 0; k < 30; k++) {
		lag = m.note((int64_t)((120 + k) * TICK), t0 + (uint32_t)(k * TICK));
	}
	CHECK(lag <= 1);
	printf("  a pause starts the floor over: ok\n");
}

static void test_queue_floor_follows_a_new_trip() {
	QueueMeter m;
	double t = 5000.0;
	uint32_t lag = 0;
	for (int i = 0; i < 300; i++, t += TICK) { // 5 s at a 40 ms trip
		m.note((int64_t)t, (uint32_t)(t + 40));
	}
	for (int i = 0; i < 600; i++, t += TICK) { // then 10 s at 90 ms: the old floor ages out
		lag = m.note((int64_t)t, (uint32_t)(t + 90));
	}
	CHECK(lag <= 1);
	printf("  the floor follows a lasting change of route: ok\n");
}

// --- ArrivalMeter ---

static uint32_t feed_arrivals(ArrivalMeter &m, uint32_t from, uint32_t ms, uint32_t spacing) {
	for (uint32_t t = from; t < from + ms; t += spacing) {
		m.note(t);
	}
	return m.pps();
}

static void test_arrivals_read_their_rate() {
	ArrivalMeter m;
	CHECK(feed_arrivals(m, 1000, 3000, 25) == 40);
	ArrivalMeter early;
	CHECK(feed_arrivals(early, 1000, ArrivalMeter::WINDOW_MS - 50, 25) == 0); // no window closed yet
	printf("  arrivals read their rate once a window closes: ok\n");
}

static void test_arrivals_pause_starts_over() {
	ArrivalMeter m;
	feed_arrivals(m, 1000, 3000, 25);
	CHECK(feed_arrivals(m, 9000, 3000, 50) == 20); // the 5 s gap isn't a slow link
	printf("  a pause in arrivals starts the count over: ok\n");
}

// --- RateAuto ---

// Reports at 60 a second for ms; returns the rate at the end.
static int feed_rate(RateAuto &c, uint32_t from, uint32_t ms, double lag, double delivered, double loss = 0.0) {
	for (uint32_t t = from; t < from + ms; t += 16) {
		c.note(lag, delivered, loss, t);
	}
	return c.rate();
}

// Past the warmup, calm.
static RateAuto warm_rate() {
	RateAuto c;
	feed_rate(c, 0, RateAuto::WARMUP_MS + 16, 5.0, 20000.0);
	return c;
}

// Long enough at 300 ms of lag to read as a queue: it crosses HIGH_MS within a few reports.
static const uint32_t QUEUEING = RateAuto::QUEUE_MS + 100;

static void test_rate_join_burst_is_not_judged() {
	RateAuto c;
	CHECK(feed_rate(c, 0, RateAuto::WARMUP_MS - 100, 500.0, 3000.0) == 0);
	printf("  the join's burst is not judged: ok\n");
}

static void test_rate_no_limit_while_it_keeps_up() {
	RateAuto c = warm_rate();
	CHECK(feed_rate(c, 6000, 10000, 5.0, 20000.0) == 0);
	printf("  no limit while the downlink keeps up: ok\n");
}

static void test_rate_lag_shorter_than_a_queue_sets_no_limit() {
	RateAuto c = warm_rate();
	CHECK(feed_rate(c, 6000, RateAuto::QUEUE_MS / 2, 300.0, 6000.0) == 0);
	CHECK(feed_rate(c, 6000 + RateAuto::QUEUE_MS / 2, 10000, 5.0, 6000.0) == 0);
	printf("  lag that doesn't last sets no limit: ok\n");
}

static void test_rate_first_limit_is_under_what_arrives() {
	RateAuto c = warm_rate();
	CHECK(feed_rate(c, 6000, QUEUEING, 300.0, 6000.0) == (int)(6000.0 * RateAuto::START));
	printf("  the first limit is just under what arrives: ok\n");
}

static void test_rate_cuts_while_queueing_down_to_the_floor() {
	RateAuto c = warm_rate();
	const int first = feed_rate(c, 6000, QUEUEING, 300.0, 6000.0); // cuts ~7025, held to ~9025
	CHECK(feed_rate(c, 7100, 1000, 300.0, 6000.0) == first); // held while the queue drains
	CHECK(feed_rate(c, 8100, 2000, 300.0, 6000.0) < first);  // still queueing as the hold ends
	CHECK(feed_rate(c, 10100, 60000, 300.0, 6000.0) == MIN_RATE);
	printf("  cuts while it queues, never under the floor: ok\n");
}

static void test_rate_climbs_while_calm_up_to_the_ceiling() {
	RateAuto c = warm_rate();
	const int limited = feed_rate(c, 6000, QUEUEING, 300.0, 6000.0);
	// Calm is counted from the cut (~7025), so the first climb lands ~10025 and the next ~13025.
	const int r = feed_rate(c, 7100, RateAuto::CALM_MS + 1100, 5.0, 3000.0);
	CHECK(r > limited && r <= (int)(limited * RateAuto::CLIMB) + 1);
	CHECK(feed_rate(c, 15000, 600000, 5.0, 3000.0) == RateAuto::MAX_RATE);
	printf("  climbs a step per calm spell, up to the ceiling: ok\n");
}

// A queue full from the first snapshot reads as no lag (it IS the floor), but its overflow shows as
// lost snapshots: that alone sets a limit while there is none.
static void test_rate_overflow_without_lag_still_limits() {
	RateAuto c = warm_rate();
	CHECK(feed_rate(c, 6000, 1000, 10.0, 6000.0, 0.8) == (int)(6000.0 * RateAuto::START));
	printf("  overflow with no visible lag still limits: ok\n");
}

// Once limited, missing snapshots may be the limit's own chokes: not a reason to cut further.
static void test_rate_choke_loss_is_not_congestion() {
	RateAuto c = warm_rate();
	const int limited = feed_rate(c, 6000, QUEUEING, 300.0, 6000.0);
	CHECK(feed_rate(c, 7100, RateAuto::CALM_MS + 1100, 5.0, 3000.0, 0.7) > limited);
	printf("  a limited client's chokes are not congestion: ok\n");
}

// --- CmdRateAuto ---

static const double CEILING = 60.0;

static double feed_cmd(CmdRateAuto &c, uint32_t from, uint32_t ms, double lag, double arriving = 0.0) {
	for (uint32_t t = from; t < from + ms; t += 16) {
		c.note(lag, arriving, CEILING, t);
	}
	return c.rate();
}

static CmdRateAuto cmd_at_ceiling() {
	CmdRateAuto c;
	feed_cmd(c, 1000, 10000, 5.0);
	return c;
}

static bool near(double a, double b) { return std::fabs(a - b) < 0.01; }

static void test_cmd_starts_low_and_climbs_quickly() {
	CmdRateAuto c;
	CHECK(c.note(0.0, 0.0, CEILING, 1000) == CmdRateAuto::START_HZ);
	const int steps = (int)((CEILING - CmdRateAuto::START_HZ) / CmdRateAuto::STEP);
	CHECK(feed_cmd(c, 1000, (steps + 1) * CmdRateAuto::SLOW_START_MS + 100, 5.0) == CEILING);
	CHECK(feed_cmd(c, 20000, 20000, 5.0) == CEILING);
	printf("  starts low, climbs quickly to the ceiling and stays: ok\n");
}

static void test_cmd_after_a_cut_it_climbs_slowly() {
	CmdRateAuto c;
	const double cut = feed_cmd(c, 1000, 1000, 300.0);
	CHECK(feed_cmd(c, 2000, 1800, 5.0) == cut); // slow start would step as the hold ends (~3050)
	printf("  after a cut, no slow-start steps: ok\n");
}

static void test_cmd_sustained_queueing_cuts_and_holds() {
	CmdRateAuto c = cmd_at_ceiling();
	double r = feed_cmd(c, 11000, 1000, 300.0);
	CHECK(r < CEILING && r >= CEILING * CmdRateAuto::CUT - 0.01);
	r = feed_cmd(c, 12000, 1000, 300.0); // the cut came ~50 ms in; its hold runs to ~13050
	CHECK(r >= CEILING * CmdRateAuto::CUT - 0.01);
	CHECK(feed_cmd(c, 13000, 20000, 300.0) == CmdRateAuto::MIN_HZ);
	printf("  sustained queueing cuts, holds, and stops at the floor: ok\n");
}

static void test_cmd_single_spike_does_not_cut() {
	CmdRateAuto c = cmd_at_ceiling();
	c.note(400.0, 0.0, CEILING, 11000);
	CHECK(feed_cmd(c, 11016, 2000, 5.0) == CEILING);
	printf("  a single spike doesn't cut: ok\n");
}

static void test_cmd_climbs_back_a_step_at_a_time() {
	CmdRateAuto c;
	const double floor_rate = feed_cmd(c, 1000, 20000, 300.0);
	const double r = feed_cmd(c, 21000, CmdRateAuto::CALM_MS + 1500, 5.0);
	CHECK(r > floor_rate && r <= floor_rate + CmdRateAuto::STEP);
	CHECK(feed_cmd(c, 30000, 60000, 5.0) == CEILING);
	printf("  climbs back a step per calm spell: ok\n");
}

static void test_cmd_cut_drains_under_the_arrival_rate() {
	// A modem uplink carrying ~44 of our packets a second, 400 ms behind: sending at 80% of that
	// drains it within the hold.
	CmdRateAuto c = cmd_at_ceiling();
	CHECK(near(feed_cmd(c, 11000, 1000, 400.0, 44.0), 44.0 * 0.8));
	CmdRateAuto full = cmd_at_ceiling();
	CHECK(near(feed_cmd(full, 11000, 1000, 1000.0, 44.0), 44.0 * CmdRateAuto::DRAIN_MIN));
	CmdRateAuto small = cmd_at_ceiling();
	CHECK(near(feed_cmd(small, 11000, 1000, 90.0, 55.0), 55.0 * CmdRateAuto::DRAIN_MAX));
	printf("  a cut goes under the arrival rate by the backlog: ok\n");
}

static void test_cmd_arrivals_above_the_rate_cut_from_the_rate() {
	// Packets bunched by jitter can count above what we send: the cut is still from our own rate.
	CmdRateAuto c = cmd_at_ceiling();
	const double before = feed_cmd(c, 11000, 1000, 1000.0, 40.0);
	CHECK(near(feed_cmd(c, 12000, 3000, 400.0, 50.0), before * 0.8));
	printf("  arrivals above our rate cut from our rate: ok\n");
}

// --- DownlinkMeter ---

static void test_downlink_samples_running_totals() {
	DownlinkMeter d;
	d.sample(50000, 100, 0, 1000); // the session so far: only a baseline
	CHECK(d.delivered_bps() == 0.0);
	d.sample(53000, 130, 10, 2000);
	CHECK(d.delivered_bps() == 3000.0);
	d.sample(1000, 5, 0, 3000); // totals went down: a new session, so no rate from this one
	CHECK(d.delivered_bps() == 3000.0);
	d.sample(4000, 35, 0, 4000);
	CHECK(d.delivered_bps() == 3000.0);
	printf("  delivered bytes come from running totals: ok\n");
}

static void test_downlink_limits_a_queueing_link() {
	DownlinkMeter d;
	uint32_t server = 900000; // the server's clock, far from ours
	uint64_t bytes = 0;
	uint32_t now = 1000;
	int extra = 0;
	for (; now < 1000 + RateAuto::WARMUP_MS + 3000; now += 33, server += 33) {
		if (d.sample_due(now)) {
			d.sample(bytes, 0, 0, now);
		}
		bytes += 100; // ~3000 B/s arriving
		if (now > 1000 + RateAuto::WARMUP_MS - 2000) {
			extra += 5; // and a queue building in front of it
		}
		d.note(server, now + extra);
	}
	CHECK(d.lag_ms() > 300);
	CHECK(d.rate() > 0 && d.rate() <= 3100 * RateAuto::START);
	printf("  a queueing downlink gets a limit under what arrives (%d B/s): ok\n", d.rate());
}

// A client frame stall: snapshots wait in the socket and are all read when it polls again, so they
// arrive late together. A burst held up on the link looks the same. Either clears within a few
// frames; neither is a queue, and neither may set a limit.
static void test_downlink_stall_is_not_a_queue() {
	for (const uint32_t stall : { 150u, 300u, 450u }) {
		DownlinkMeter d;
		uint32_t server = 900000;
		uint64_t bytes = 0;
		auto deliver = [&](uint32_t p_at) {
			if (d.sample_due(p_at)) {
				d.sample(bytes, 0, 0, p_at);
			}
			bytes += 60;
			d.note(server, p_at);
			server += 16;
		};
		uint32_t now = 1000;
		for (; now < 1000 + RateAuto::WARMUP_MS + 2000; now += 16) {
			deliver(now);
		}
		const uint32_t resume = now + stall;
		for (; now < resume; now += 16) {
			deliver(resume);
		}
		for (; now < resume + 10000; now += 16) {
			deliver(now);
		}
		CHECK(d.rate() == 0);
	}
	printf("  a stall or a late burst sets no limit: ok\n");
}

int main() {
	printf("auto_rate:\n");
	test_queue_steady_stream_reads_no_lag();
	test_queue_building_reads_its_extra_wait();
	test_queue_pause_is_not_lag();
	test_queue_floor_follows_a_new_trip();
	test_arrivals_read_their_rate();
	test_arrivals_pause_starts_over();
	test_rate_join_burst_is_not_judged();
	test_rate_no_limit_while_it_keeps_up();
	test_rate_lag_shorter_than_a_queue_sets_no_limit();
	test_rate_first_limit_is_under_what_arrives();
	test_rate_cuts_while_queueing_down_to_the_floor();
	test_rate_climbs_while_calm_up_to_the_ceiling();
	test_rate_overflow_without_lag_still_limits();
	test_rate_choke_loss_is_not_congestion();
	test_cmd_starts_low_and_climbs_quickly();
	test_cmd_after_a_cut_it_climbs_slowly();
	test_cmd_sustained_queueing_cuts_and_holds();
	test_cmd_single_spike_does_not_cut();
	test_cmd_climbs_back_a_step_at_a_time();
	test_cmd_cut_drains_under_the_arrival_rate();
	test_cmd_arrivals_above_the_rate_cut_from_the_rate();
	test_downlink_samples_running_totals();
	test_downlink_limits_a_queueing_link();
	test_downlink_stall_is_not_a_queue();
	printf("auto_rate: %d checks passed\n", checks);
	return 0;
}
