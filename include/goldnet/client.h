#pragma once
//
// The client half of the snapshot protocol: reconstruct each delta against the baseline frame the
// server diffed it against, apply spawns / despawns / leaves / changed slots through the host, and
// ack. Engine-agnostic, like Server.
//
// The uplink. Everything the client sends rides its uplink packets: its commands and messages
// (push_command and queue_reliable / queue_unreliable, see uplink.h), the snapshot ack, what it has of the server's channel, and the rate and
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
// The host passed to apply() provides the following. Deriving it from ClientHost (at the end of this
// file) leaves only entity() to write; the rest default to a game that doesn't use them.
//
//   EntityResult entity(EntityUpdate<V> &u)
//        one entity's changed slots (EntityUpdate): decode each from u.reader(), as the server host's
//        encode wrote it, hand it to u.set(), and apply it. ENTITY_SKIP an entity the host doesn't have (its
//        slots are stepped over), or one sent whole() whose kinds_check says it would read the slots
//        differently; ENTITY_NOT_READY one it has but can't take values for yet (warmup), which holds the ack.
//   void protocol_mismatch()                       a different build is talking to us (once per session)
//   void server_time(uint32_t ms)                  every in-order snapshot's header clock, before
//                                                  any of its values are applied
//   void message(const uint8_t *data, size_t size, bool reliable)   one netchan message (netchan.h)
//   bool spawn(uint32_t net_id, uint32_t spawner_net_id, const uint8_t *data, size_t size)
//        create it; false = can't yet: the frame isn't acked, so the spawn is offered again
//   void despawn(uint32_t net_id)                  only for an id whose spawn() returned true
//   void relevance_lost(uint32_t net_id)
//
// The client keeps each entity's values at recent frames, to apply the next delta to. A despawn drops
// the spawn's own entity from them; a host that does away with any other entity it had (a map change,
// say) calls forget() with its id.
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

// The slots set in a mask, lowest first: for (int s : SlotMask{ mask }).
struct SlotMask {
	uint32_t bits = 0;

	struct iterator {
		uint32_t rest;
		int operator*() const {
			int s = 0;
			while (!(rest & (1u << s))) {
				s++;
			}
			return s;
		}
		iterator &operator++() {
			rest &= rest - 1;
			return *this;
		}
		bool operator!=(const iterator &p_other) const { return rest != p_other.rest; }
	};
	iterator begin() const { return { bits }; }
	iterator end() const { return { 0 }; }
};

// What the host's entity() made of an EntityUpdate.
enum EntityResult : uint8_t {
	ENTITY_APPLIED,   // decoded and applied, each slot handed to EntityUpdate::set
	ENTITY_SKIP,      // not an entity the host has, or one it would read differently: its slots are stepped over
	ENTITY_NOT_READY, // the host has it but can't take values yet: the snapshot isn't acked, so it comes again
};

template <class V>
class Client;

// What the host's entity() gets: one entity's changed slots in a snapshot.
template <class V>
class EntityUpdate {
public:
	uint32_t net_id() const { return net_id_; }
	// Every slot, none against a base: the first the client sees of it, or a resend in full.
	bool whole() const { return whole_; }
	// With whole(): the server host's summary of how it codes the slots (EntityLayout::kinds_check).
	uint8_t kinds_check() const { return kinds_check_; }
	// The snapshot's header clock, ms, for a slot coded against it.
	uint32_t server_time() const { return server_time_; }
	// The changed slots, lowest first; each one's bytes are next in reader().
	SlotMask changed() const { return { mask_ }; }
	Reader &reader() { return reader_; }
	// The value the client holds for a slot at the snapshot's baseline, which a slot coded as a change
	// is a change from; null when whole().
	const V *base(int p_slot) const {
		return base_ && (size_t)p_slot < base_->size() ? &(*base_)[p_slot] : nullptr;
	}
	// The slot's decoded value, which the next snapshot's base() gives back. Every changed slot must
	// be set, or the snapshot isn't one to ack.
	void set(int p_slot, const V &p_value) {
		if (p_slot < 0 || p_slot >= MAX_SLOTS) {
			return;
		}
		if ((size_t)p_slot >= vals_.size()) {
			vals_.resize(p_slot + 1);
		}
		vals_[p_slot] = p_value;
		set_ |= 1u << p_slot;
	}

private:
	friend class Client<V>;
	EntityUpdate(uint32_t p_net_id, bool p_whole, uint8_t p_kinds_check, uint32_t p_server_time, uint32_t p_mask,
			const uint8_t *p_data, size_t p_size, const std::vector<V> *p_base) :
			net_id_(p_net_id), whole_(p_whole), kinds_check_(p_kinds_check), server_time_(p_server_time),
			mask_(p_mask), reader_(p_data, p_size), base_(p_whole ? nullptr : p_base) {
		if (base_) {
			vals_ = *base_;
		}
	}

	uint32_t net_id_;
	bool whole_;
	uint8_t kinds_check_;
	uint32_t server_time_;
	uint32_t mask_;
	Reader reader_;
	const std::vector<V> *base_;
	std::vector<V> vals_; // the base's values, with the slots set over them
	uint32_t set_ = 0;
};

template <class V>
class Client {
public:
	// What apply() made of a snapshot: why it wasn't taken, if it wasn't, and what it saw, for a
	// host's debug print.
	struct ApplyInfo {
		Status status = STATUS_OK;
		uint16_t seq = 0;
		uint16_t base_seq = 0;
		int spawns = 0;
		int despawns = 0;
		int changed = 0;
		bool committed = false;
		uint16_t ack = 0;         // the snapshot ack it queued for the uplink; 0 = none
		uint16_t command_ack = 0; // the header's: the newest command the server has; 0 = none
	};

	// The link, as this end sees it. Totals are this session's.
	struct Stats {
		// Snapshots that arrived, and seqs skipped on the way to them. The server spends a seq only
		// on a snapshot it sends, so a skipped one was lost (or is late, and is taken back off when
		// it lands). Counted before any check that could still drop the snapshot.
		uint32_t snapshots_received = 0;
		uint32_t snapshots_lost = 0;
		// How far behind its best the downlink arrives, ms, and what it delivers, bytes/sec on the wire.
		uint32_t downlink_lag_ms = 0;
		double delivered_bps = 0.0;
		// The snapshots/sec the server granted, from its last header; -1 = no word yet.
		int update_rate_granted = -1;
		// What the uplink goes by now, the setting or the automatic pick: the rate asked of the
		// server, bytes/sec (0 = no limit), and the gap between command packets, ms (0 = whenever
		// there is something to send).
		uint32_t rate = 0;
		uint32_t command_interval_ms = 0;
		// The server's last report of our uplink: how far behind its best it arrives, ms, and its
		// packets arriving per second.
		uint32_t uplink_lag_ms = 0;
		uint32_t uplink_pps = 0;
		// Commands not acked yet, ones given up on (they fell out of COMMAND_RING), and the round trip.
		uint32_t commands_pending = 0;
		uint32_t commands_dropped = 0;
		uint32_t command_rtt_ms = 0;
		// Reliable messages queued to the server, and how many of those it has. Both in queue order.
		uint64_t messages_queued = 0;
		uint64_t messages_delivered = 0;
		// The server isn't taking our reliable data: GoldSrc dropped the connection.
		bool overflowed = false;
	};
	Stats stats() const {
		Stats st;
		st.snapshots_received = arrivals_.received;
		st.snapshots_lost = arrivals_.lost;
		st.downlink_lag_ms = downlink_.lag_ms();
		st.delivered_bps = downlink_.delivered_bps();
		st.update_rate_granted = update_rate_granted_;
		st.rate = uplink_.rate();
		st.command_interval_ms = uplink_.interval_ms();
		st.uplink_lag_ms = uplink_lag_ms_;
		st.uplink_pps = uplink_pps_;
		st.commands_pending = (uint32_t)uplink_.pending();
		st.commands_dropped = uplink_.dropped();
		st.command_rtt_ms = uplink_.rtt_ms();
		st.messages_queued = uplink_.reliable_queued();
		st.messages_delivered = uplink_.reliable_delivered();
		st.overflowed = uplink_.overflowed();
		return st;
	}

	// The host no longer has this entity, by any way but its despawn: the client stops holding its
	// values, so a later delta for it isn't applied to something new at the same id. Safe from
	// inside the host's callbacks.
	void forget(uint32_t p_net_id) {
		for (int i = 0; i < RING; i++) {
			frames_[i].erase(p_net_id);
		}
		forgotten_.push_back(p_net_id);
	}

	// A fresh session: old baselines are meaningless.
	void reset() {
		arrivals_ = Arrivals();
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

	// --- Commands and messages (uplink.h, netchan.h) ---

	// A command stream's field kinds, in order; the server's host must name the same. A stream with
	// none set is one FIELD_BYTES field. Set it before the stream's first command.
	void set_command_schema(uint32_t p_stream, const Schema &p_schema) { uplink_.set_schema(p_stream, p_schema); }
	const Schema &command_schema(uint32_t p_stream) const { return uplink_.schema(p_stream); }
	// Queue one command, one field per schema entry (missing ones are zero, extra ones dropped), or
	// one byte string for a stream with no schema set. Returns its seq.
	uint16_t push_command(uint32_t p_stream, Command p_cmd, uint32_t p_now) {
		return uplink_.push(p_stream, std::move(p_cmd), p_now);
	}
	uint16_t push_command(uint32_t p_stream, const uint8_t *p_data, size_t p_size, uint32_t p_now) {
		return uplink_.push(p_stream, p_data, p_size, p_now);
	}

	void queue_reliable(const uint8_t *p_data, size_t p_size) { uplink_.queue_reliable(p_data, p_size); }
	// False if the datagram is full and the message was dropped.
	bool queue_unreliable(const uint8_t *p_data, size_t p_size) { return uplink_.queue_unreliable(p_data, p_size); }

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
	// Ids forget() was given during the apply() underway, for the frame it's building.
	std::vector<uint32_t> forgotten_;
	bool warned_mismatch_ = false;
	// Consecutive frames deferred because a KNOWN entity wasn't ready yet. Capped so an entity that
	// never resolves can't stall the stream forever; after the cap we accept the frame and move on.
	uint32_t defer_streak_ = 0;
	struct Arrivals {
		uint32_t received = 0;
		uint32_t lost = 0;
	};
	Arrivals arrivals_;
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
	forgotten_.clear(); // any before this apply are out of the frames already

	// Protocol guard: a mismatched build (different wire format) would misparse every field. Fail
	// loudly once and drop, rather than flooding the log with decode errors.
	const uint8_t version = r.get_u8();
	if (r.failed()) {
		info.status = STATUS_MALFORMED;
		return info;
	}
	if (version != WIRE_VERSION) {
		if (!warned_mismatch_) {
			warned_mismatch_ = true;
			p_host.protocol_mismatch();
		}
		info.status = STATUS_MISMATCH;
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
		info.status = STATUS_MALFORMED;
		return info;
	}
	count_arrival(seq);

	// Drop stale/reordered snapshots (unreliable transport). We only advance forward. A snapshot
	// newer than the last committed one but not the last received is stale too: GoldSrc drops any
	// packet older than the newest, and its messages must never play after a newer packet's.
	if ((has_ && !seq_newer(seq, last_seq_)) || !chan_.accept(seq)) {
		info.status = STATUS_STALE;
		return info;
	}
	uplink_.on_channel_ack(chan.ack, p_now);
	news_ = true;
	std::vector<std::vector<uint8_t>> reliable;
	std::vector<std::vector<uint8_t>> unreliable;
	if (!chan.take(chan_, p_now, reliable, unreliable)) {
		info.status = STATUS_MALFORMED;
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
			info.status = STATUS_NO_BASELINE;
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
		const uint32_t len_whole = get_uvarint(&r);
		const uint32_t len = len_whole >> 1;
		const bool whole = (len_whole & 1) != 0;
		const uint8_t kinds_check = whole ? r.get_u8() : 0;
		const uint8_t *data = r.get_data(len);
		if (!data) {
			break;
		}

		// Start from this entity's baseline values, then overlay the changed slots. A change can only
		// be applied to the value it is a change from, so a delta for an entity missing from the
		// baseline frame can't be applied at all: the host is offered no slots, and can only say
		// whether it has the entity.
		auto cur = frame.find(net_id);
		const bool held = whole || cur != frame.end();
		EntityUpdate<V> u(net_id, whole, kinds_check, server_time, held ? mask : 0, data, len,
				cur != frame.end() ? &cur->second : nullptr);
		const EntityResult res = p_host.entity(u);
		if (res == ENTITY_NOT_READY) {
			defer_ack = true;
		}
		if (res != ENTITY_APPLIED) {
			continue;
		}
		if (!held) {
			// Known here, but not held at the baseline the server thinks it is: hold the ack, so the
			// server keeps diffing against a frame we have, until the cap gives up.
			defer_ack = true;
		} else if (u.reader().failed() || u.set_ != mask) {
			defer_ack = true; // the host read the slots differently than they were written: not a frame to ack
		} else {
			frame[net_id] = std::move(u.vals_);
		}
	}

	deliver(unreliable, false);

	// A truncated or corrupt packet: whatever it applied stays applied (later snapshots carry the
	// same state again), but it is not a frame we can baseline against, so don't commit or ack it.
	if (r.failed()) {
		info.status = STATUS_MALFORMED;
		return info;
	}

	// Now free despawned entities (after the deltas so nothing applies to one we then free), and stop
	// holding their values, as the server no longer does.
	for (uint32_t nid : despawns) {
		if (spawned_.erase(nid)) {
			p_host.despawn(nid);
		}
		frame.erase(nid);
	}
	// And any the host did away with meanwhile (forget()).
	for (uint32_t nid : forgotten_) {
		frame.erase(nid);
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
	// one uncommitted. Capped. On the very first frame there's no good frame to re-ack, so it holds
	// by not acking at all, and the server keeps sending full state: acking would leave the server
	// sure we hold the entity, and every later delta would be a change from a value we never got.
	//
	// A refused spawn holds the same way, for the same reason: acking a frame that carries it tells
	// the server it was delivered, and it's never sent again.
	if ((spawn_refused || defer_ack) && defer_streak_ < MAX_DEFER_STREAK) {
		defer_streak_++;
		if (has_) {
			queue_ack(last_seq_, info);
		}
		info.status = STATUS_HELD;
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

// A base for the host Client::apply calls back into: derive from it and define entity(), which it
// leaves pure. Everything else has a default that suits a game not using the feature, so define only
// what you use, marked override (see ServerHost).
template <class V>
struct ClientHost {
	virtual ~ClientHost() = default;

	virtual EntityResult entity(EntityUpdate<V> &u) = 0;

	// A spawn is taken, and creates nothing: a game with no runtime entities never gets one.
	virtual bool spawn(uint32_t, uint32_t, const uint8_t *, size_t) { return true; }
	virtual void despawn(uint32_t) {}
	virtual void relevance_lost(uint32_t) {}
	virtual void server_time(uint32_t) {}
	virtual void message(const uint8_t *, size_t, bool) {}
	virtual void protocol_mismatch() {}
};

} // namespace goldnet
