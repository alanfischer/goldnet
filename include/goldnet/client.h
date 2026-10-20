#pragma once
//
// The client half of the snapshot protocol: reconstruct each delta against the baseline frame the
// server diffed it against, apply spawns / despawns / leaves / changed slots through the host, and
// ack. Engine-agnostic, like Server.
//
// The uplink. Everything the client sends rides its uplink packets: its commands and messages
// (uplink(), see uplink.h), the snapshot ack, what it has of the server's channel, and the rate and
// update rate it asks for (set_rate, set_update_rate). Send one whenever uplink_due() says so:
//
//   if (client.uplink_due(now)) { Writer w; client.write_uplink(now, w); /* send w */ }
//
// A fresh ack makes a packet due on its own, unless commands are flowing: then it rides the next
// command packet, which comes within the command interval (set_command_rate).
//
// The rate and the command rate can each be picked automatically instead (auto_rate.h): the rate
// from how far behind the downlink arrives, the command rate from the server's report of the uplink.
//
// The host passed to apply() provides:
//   void protocol_mismatch()                       a different build is talking to us (once per session)
//   void server_time(uint32_t ms)                  every in-order snapshot's header clock, before
//                                                  any of its values are applied
//   void message(const uint8_t *data, size_t size, bool reliable)   one netchan message (netchan.h)
//   bool spawn(uint32_t net_id, uint32_t spawner_net_id, const uint8_t *data, size_t size)
//        create it; false = can't yet: the frame isn't acked, so the spawn is offered again
//   void despawn(uint32_t net_id)                  only for an id whose spawn() returned true
//   void relevance_lost(uint32_t net_id)
//   bool registered(uint32_t net_id)               does an entity with this id exist here?
//   int  begin_entity(uint32_t net_id, uint32_t mask, bool &r_defer)
//        slot count of a live target, or -1 if none; r_defer = known but not ready (warmup)
//   bool set_slot(int slot, const V &v)            apply to the current target; true if applied
//   void end_entity(bool applied)                  the current target is done
//   V    decode(Reader &r, uint32_t ref_time_ms)   one tagged value
//
// Messages are delivered from any snapshot newer than every one before it, committed or not:
// the reliable ones before the spawns, the datagram's after the entity deltas and before the
// despawns, so an event can name an entity the same snapshot creates or removes. QuakeWorld ran its
// datagram after the entities; it held only temp entities, which need no entity to land on.
//
#include "goldnet/auto_rate.h"
#include "goldnet/buffer.h"
#include "goldnet/codec.h"
#include "goldnet/netchan.h"
#include "goldnet/protocol.h"
#include "goldnet/uplink.h"

#include <cmath>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace goldnet {

template <class V>
class Client {
public:
	// What the last apply() saw, for a host's debug print.
	struct ApplyInfo {
		uint16_t seq = 0;
		uint16_t base_seq = 0;
		int spawns = 0;
		int despawns = 0;
		int changed = 0;
		bool committed = false;
		uint16_t ack = 0;         // the snapshot ack it queued for the uplink; 0 = none
		uint16_t command_ack = 0; // the header's: the newest command the server has; 0 = none
	};

	// Snapshots that arrived, and seqs skipped on the way to them, this session. The server
	// spends a seq only on a snapshot it sends, so a skipped one was lost (or is late, and is taken
	// back off when it lands). Counted before any check that could still drop the snapshot.
	struct ArrivalStats {
		uint32_t received = 0;
		uint32_t lost = 0;
	};
	const ArrivalStats &arrival_stats() const { return arrivals_; }

	// A fresh session: old baselines are meaningless.
	void reset() {
		arrivals_ = ArrivalStats();
		arrived_last_ = 0;
		for (int i = 0; i < RING; i++) {
			frames_[i].clear();
			frame_seq_[i] = 0;
		}
		spawned_.clear();
		last_seq_ = 0;
		has_ = false;
		defer_streak_ = 0;
		chan_ = ChannelReceiver();
		uplink_.reset();
		ack_ = 0;
		news_ = false;
		received_bytes_ = 0;
		downlink_ = DownlinkMeter();
		cmd_auto_ = CmdRateAuto();
		uplink_lag_ms_ = 0;
		uplink_pps_ = 0;
		update_rate_granted_ = -1;
		apply_link();
	}

	// p_now: the host's clock, the same one it writes its uplink packets by.
	template <class Host>
	ApplyInfo apply(const uint8_t *p_data, size_t p_size, Host &p_host, uint32_t p_now);

	// The commands and messages this client sends (uplink.h). Its rate, update rate and interval are
	// set from the ones below.
	CommandSender &uplink() { return uplink_; }
	const CommandSender &uplink() const { return uplink_; }

	// --- The link (GoldSrc rate, cl_updaterate, cl_cmdrate). Settings outlive reset(). ---

	// The bytes/sec the server may send us; 0 = no limit. While auto_rate is on, picked from how far
	// behind the downlink arrives instead (RateAuto).
	void set_rate(uint32_t p_bps) {
		rate_ = p_bps;
		apply_link();
	}
	uint32_t rate() const { return rate_; }
	void set_auto_rate(bool p_on) {
		auto_rate_ = p_on;
		apply_link();
	}
	bool auto_rate() const { return auto_rate_; }

	// The snapshots/sec we ask for; 0 = as many as the server sends.
	void set_update_rate(uint32_t p_hz) {
		update_rate_ = p_hz;
		apply_link();
	}
	uint32_t update_rate() const { return update_rate_; }
	// What the server granted, from its last header; -1 = no word yet this session.
	int update_rate_granted() const { return update_rate_granted_; }

	// Uplink packets/sec; 0 = whenever there is something to send. While auto_command_rate is on,
	// picked from the server's report of our uplink instead (CmdRateAuto), up to p_max_hz, the rate
	// commands are made; at that, whenever there is something to send.
	void set_command_rate(double p_hz) {
		command_rate_ = p_hz > 0.0 ? p_hz : 0.0;
		apply_link();
	}
	double command_rate() const { return command_rate_; }
	void set_auto_command_rate(bool p_on, double p_max_hz) {
		auto_command_rate_ = p_on;
		command_rate_max_ = p_max_hz > 0.0 ? p_max_hz : 0.0;
		apply_link();
	}
	bool auto_command_rate() const { return auto_command_rate_; }

	// Wire bytes each snapshot costs beyond its payload (headers below the protocol), for measuring
	// what the downlink delivers.
	void set_packet_overhead(int p_bytes) { packet_overhead_ = p_bytes > 0 ? p_bytes : 0; }

	// How far behind its best the downlink arrives, ms, and what it delivers, bytes/sec on the wire.
	uint32_t downlink_lag_ms() const { return downlink_.lag_ms(); }
	double delivered_bps() const { return downlink_.delivered_bps(); }
	// The server's last report of our uplink: how far behind its best it arrives, ms, and its packets
	// arriving per second.
	uint32_t uplink_lag_ms() const { return uplink_lag_ms_; }
	uint32_t uplink_pps() const { return uplink_pps_; }

	// Whether an uplink packet should go now: commands or messages due, or news the server is waiting on.
	bool uplink_due(uint32_t p_now) const {
		if (uplink_.due(p_now)) {
			return true;
		}
		return news_ && uplink_.idle_ms(p_now) >= uplink_.interval_ms() + ACK_WAIT_SLACK_MS;
	}

	// Build the next uplink packet.
	void write_uplink(uint32_t p_now, Writer &r_out) {
		uplink_.write(p_now, ack_, chan_.ack_source(), r_out);
		news_ = false;
	}

private:
	// A frame is { net_id -> its slot values, in slot order }.
	typedef std::unordered_map<uint32_t, std::vector<V>> Frame;

	// Receive history — a mirror of the server's ring, so a delta can be reconstructed against any
	// recent baseline the server might diff against.
	Frame frames_[RING];
	uint16_t frame_seq_[RING] = {};
	uint16_t last_seq_ = 0;
	bool has_ = false;
	// Spawns the host accepted. Spawns and despawns are resent until acked, so the same one can
	// arrive many times; this is what lets the host see each exactly once.
	std::unordered_set<uint32_t> spawned_;
	bool warned_mismatch_ = false;
	// Consecutive frames deferred because a KNOWN entity wasn't ready yet. Capped so an entity that
	// never resolves can't stall the stream forever; after the cap we accept the frame and move on.
	uint32_t defer_streak_ = 0;
	ArrivalStats arrivals_;
	uint16_t arrived_last_ = 0; // the newest seq seen; 0 = none yet
	ChannelReceiver chan_;
	CommandSender uplink_;
	uint16_t ack_ = 0;    // the snapshot ack the next uplink packet carries; 0 = none
	bool news_ = false;   // a snapshot arrived since the last uplink packet: an ack, or channel news
	// Commands count as flowing while one was pushed within the command interval plus this.
	static const uint32_t ACK_WAIT_SLACK_MS = 100;
	void count_arrival(uint16_t p_seq);
	void queue_ack(uint16_t p_seq, ApplyInfo &r_info) {
		ack_ = p_seq;
		news_ = true;
		r_info.ack = p_seq;
	}
	static const uint32_t MAX_DEFER_STREAK = RING; // ~1s at 30 Hz before giving up on a warmup entity

	uint32_t rate_ = 0;
	bool auto_rate_ = false;
	uint32_t update_rate_ = 0;
	int update_rate_granted_ = -1;
	double command_rate_ = 0.0;
	bool auto_command_rate_ = false;
	double command_rate_max_ = 0.0;
	int packet_overhead_ = 0;
	uint64_t received_bytes_ = 0; // on the wire, this session
	DownlinkMeter downlink_;
	CmdRateAuto cmd_auto_;
	uint32_t uplink_lag_ms_ = 0;
	uint32_t uplink_pps_ = 0;

	// The settings, or the automatic picks, onto the uplink.
	void apply_link() {
		uplink_.set_rate(auto_rate_ ? (uint32_t)downlink_.rate() : rate_);
		uplink_.set_update_rate(update_rate_);
		const double hz = auto_command_rate_ ? cmd_auto_.rate() : command_rate_;
		const bool every = hz <= 0.0 || (auto_command_rate_ && hz >= command_rate_max_);
		uplink_.set_interval_ms(every ? 0 : (uint32_t)std::lround(1000.0 / hz));
	}
	void note_header(uint32_t p_now, uint32_t p_server_time, uint32_t p_lag_ms, uint32_t p_pps, uint32_t p_granted);
};

template <class V>
template <class Host>
typename Client<V>::ApplyInfo Client<V>::apply(const uint8_t *p_data, size_t p_size, Host &p_host, uint32_t p_now) {
	ApplyInfo info;
	Reader r(p_data, p_size);
	received_bytes_ += p_size + (size_t)packet_overhead_;

	// Protocol guard: a mismatched build (different wire format) would misparse every field. Fail
	// loudly once and drop, rather than flooding the log with decode errors.
	if (r.get_u8() != WIRE_VERSION) {
		if (!warned_mismatch_) {
			warned_mismatch_ = true;
			p_host.protocol_mismatch();
		}
		return info;
	}

	const uint16_t seq = r.get_u16();
	const uint16_t base_seq = r.get_u16();
	const uint32_t server_time = r.get_u32();
	const uint16_t command_ack = r.get_u16();
	const uint32_t uplink_lag_ms = get_uvarint(&r);
	const uint32_t uplink_pps = get_uvarint(&r);
	const uint32_t update_rate = get_uvarint(&r);
	ChannelPart chan;
	chan.read(r);
	info.seq = seq;
	info.base_seq = base_seq;
	if (r.failed()) {
		return info;
	}
	count_arrival(seq);

	// Drop stale/reordered snapshots (unreliable transport). We only advance forward. A snapshot
	// newer than the last committed one but not the last received is stale too: GoldSrc drops any
	// packet older than the newest, and its messages must never play after a newer packet's.
	if ((has_ && !seq_newer(seq, last_seq_)) || !chan_.accept(seq)) {
		return info;
	}
	uplink_.on_channel_ack(chan.ack, p_now);
	news_ = true;
	std::vector<std::vector<uint8_t>> reliable;
	std::vector<std::vector<uint8_t>> unreliable;
	if (!chan.take(chan_, p_now, reliable, unreliable)) {
		return info;
	}
	auto deliver = [&p_host](std::vector<std::vector<uint8_t>> &p_msgs, bool p_reliable) {
		for (const std::vector<uint8_t> &m : p_msgs) {
			p_host.message(m.data(), m.size(), p_reliable);
		}
		p_msgs.clear();
	};

	// Surface the header clock: every snapshot carries the server's send-time, an always-on feed a
	// client can drive a clock estimator from. Emitted after the stale-seq drop (a reordered
	// snapshot's time is <= one already fed) but before the baseline-resolution drop below, so a
	// forward snapshot we can't reconstruct for state still contributes its valid clock sample.
	note_header(p_now, server_time, uplink_lag_ms, uplink_pps, update_rate);
	uplink_.on_ack(command_ack);
	info.command_ack = command_ack;
	p_host.server_time(server_time); // after note_header, so the host reads this header's link news

	// Resolve the baseline frame we reconstruct this delta against. base_seq 0 means full state. If
	// the server diffed against a frame we no longer hold, we can't reconstruct unchanged fields —
	// re-ack our last good frame so the server falls back to a baseline we have (eventually a full
	// frame), and drop this one.
	const Frame *base = nullptr;
	if (base_seq != 0) {
		const int bslot = base_seq & (RING - 1);
		if (frame_seq_[bslot] == base_seq) {
			base = &frames_[bslot];
		} else {
			deliver(reliable, true);
			deliver(unreliable, false);
			if (has_) {
				queue_ack(last_seq_, info);
			}
			return info;
		}
	}

	deliver(reliable, true);

	// Spawns first: create the entities so they register before the entity deltas below
	// reference them.
	const int spawn_ct = r.get_u16();
	bool spawn_refused = false;
	for (int i = 0; i < spawn_ct && !r.failed(); i++) {
		const uint32_t nid = r.get_u32();
		const uint32_t spawner_nid = r.get_u32();
		const uint32_t len = r.get_u32();
		const uint8_t *data = r.get_data(len);
		if (data && !spawned_.count(nid)) {
			if (p_host.spawn(nid, spawner_nid, data, len)) {
				spawned_.insert(nid);
			} else {
				spawn_refused = true;
			}
		}
	}
	info.spawns = spawn_ct;
	// Despawns: collect now (wire order), free after the entity deltas are applied.
	const int despawn_ct = r.get_u16();
	std::vector<uint32_t> despawns;
	for (int i = 0; i < despawn_ct && !r.failed(); i++) {
		despawns.push_back(r.get_u32());
	}
	info.despawns = despawn_ct;
	// Relevance leaves: collect now (wire order), apply after the entity deltas.
	const int leave_ct = r.get_u16();
	std::vector<uint32_t> leaves;
	for (int i = 0; i < leave_ct && !r.failed(); i++) {
		leaves.push_back(r.get_u32());
	}

	const int count = r.get_u16();
	info.changed = count;

	// New frame = baseline carried forward, with this delta's changed fields applied.
	Frame frame;
	if (base) {
		frame = *base;
	}

	// Set if a KNOWN entity couldn't be applied because it isn't ready yet — the warmup window. We
	// must not ack this frame as delivered (see the defer block below), or the server folds the
	// entity into this peer's baseline and — for an idle entity whose state never changes again —
	// never resends it, stranding it hidden.
	bool defer_ack = false;

	for (int i = 0; i < count && !r.failed(); i++) {
		const uint32_t net_id = r.get_u32();
		const uint32_t mask = get_uvarint(&r);

		bool defer = false;
		const int n_slots = p_host.begin_entity(net_id, mask, defer);
		if (defer) {
			defer_ack = true;
		}

		// Start from this entity's baseline values, then overlay the changed slots. Values are read
		// for every set bit whether or not there is a target, so the stream stays aligned even for
		// unknown ids.
		auto cur = frame.find(net_id);
		std::vector<V> vals;
		if (cur != frame.end()) {
			vals = cur->second;
		}
		if (n_slots >= 0 && vals.size() != (size_t)n_slots) {
			vals.resize(n_slots);
		}
		bool applied = false;
		for (int s = 0; s < MAX_SLOTS; s++) {
			if (!(mask & (1u << s))) {
				continue;
			}
			V v = p_host.decode(r, server_time); // self-delimiting — always consume
			if ((size_t)s < vals.size()) {
				vals[s] = v;
			}
			if (n_slots >= 0 && s < n_slots && p_host.set_slot(s, v)) {
				applied = true;
			}
		}
		if (!vals.empty()) {
			frame[net_id] = std::move(vals);
		}
		if (n_slots >= 0) {
			p_host.end_entity(applied);
		}
	}

	deliver(unreliable, false);

	// A truncated or corrupt packet: whatever it applied stays applied (later snapshots carry the
	// same state again), but it is not a frame we can baseline against, so don't commit or ack it.
	if (r.failed()) {
		return info;
	}

	// Now free despawned entities (after the deltas so nothing applies to one we then free).
	for (uint32_t nid : despawns) {
		if (spawned_.erase(nid)) {
			p_host.despawn(nid);
		}
	}

	// Drop entities no longer registered here (e.g. after a map change / despawn) so the client
	// baseline can't diverge from the server's, which only tracks live entities.
	for (auto it = frame.begin(); it != frame.end();) {
		if (!p_host.registered(it->first)) {
			it = frame.erase(it);
		} else {
			++it;
		}
	}

	// Relevance leaves: drop from the reconstructed frame (so it matches the server's baseline,
	// which also dropped these) and tell the host to hide the entity.
	for (uint32_t nid : leaves) {
		frame.erase(nid);
		p_host.relevance_lost(nid);
	}

	// Warmup defer: a known entity wasn't ready, so we couldn't apply (reveal) it. Committing +
	// acking this seq would advance the server's baseline past the entity; an idle one that never
	// changes again would then never be resent — stranded hidden. Instead re-ack our last good
	// frame so the server keeps resending its full baseline against a frame we hold, and drop this
	// one uncommitted. Capped, and needs a prior good frame to fall back to; on the very first
	// frame we just accept it.
	//
	// A refused spawn holds the same way, for the same reason: acking a frame that carries it tells
	// the server it was delivered, and it's never sent again. That one holds even on the first
	// frame, by not acking at all, since the server then keeps sending full state with the spawn.
	if ((spawn_refused || (defer_ack && has_)) && defer_streak_ < MAX_DEFER_STREAK) {
		defer_streak_++;
		if (has_) {
			queue_ack(last_seq_, info);
		}
		return info;
	}
	defer_streak_ = 0;

	// Commit this frame to the ring and ack it (newest wins, unreliable).
	const int slot = seq & (RING - 1);
	frames_[slot] = std::move(frame);
	frame_seq_[slot] = seq;
	last_seq_ = seq;
	has_ = true;
	queue_ack(seq, info);
	info.committed = true;
	return info;
}

// An in-order header's link news: the downlink's lag and what arrived, the server's report of the
// uplink, and the update rate it granted. The automatic picks follow from them.
template <class V>
void Client<V>::note_header(uint32_t p_now, uint32_t p_server_time, uint32_t p_lag_ms, uint32_t p_pps,
		uint32_t p_granted) {
	if (downlink_.sample_due(p_now)) {
		downlink_.sample(p_now, received_bytes_, arrivals_.received, arrivals_.lost);
	}
	downlink_.note(p_now, p_server_time);
	uplink_lag_ms_ = p_lag_ms;
	uplink_pps_ = p_pps;
	update_rate_granted_ = (int)p_granted;
	if (auto_command_rate_) {
		cmd_auto_.note(p_now, (double)p_lag_ms, (double)p_pps, command_rate_max_);
	}
	if (auto_rate_ || auto_command_rate_) {
		apply_link();
	}
}

template <class V>
void Client<V>::count_arrival(uint16_t p_seq) {
	arrivals_.received++;
	if (arrived_last_ == 0) {
		arrived_last_ = p_seq;
	} else if (seq_newer(p_seq, arrived_last_)) {
		uint16_t step = (uint16_t)(p_seq - arrived_last_);
		if (p_seq < arrived_last_) {
			step--; // the counter skipped 0 on the way round
		}
		arrivals_.lost += step - 1u;
		arrived_last_ = p_seq;
	} else if (arrivals_.lost > 0) {
		arrivals_.lost--; // late, not lost
	}
}

} // namespace goldnet
