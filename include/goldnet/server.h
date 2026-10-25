#pragma once
//
// The server half of the snapshot protocol: per-entity change tracking, per-peer delta against
// the last acked frame, reliable-until-acked spawns and despawns, derived PVS leaves, and the
// priority-ordered packet budget. Engine-agnostic: the host supplies the values, the visibility
// answers, the value encoding, and the send.
//
// V is the host's value type. It must be default-constructible, copyable and comparable with !=;
// the server never looks inside one, it only compares and stores them and hands them back
// to the host to encode. X is optional per-entity storage for the host (cached property lookups, say),
// carried on the entity so reaching it costs no second lookup.
//
// Peers. add_peer() starts serving one and remove_peer() stops; tick() sends to the peers added, and
// receive_uplink() and the netchan take only theirs. A packet from any other peer is dropped. Add
// and remove peers between ticks, never from inside the host's callbacks.
//
// Entities. add_entity(key, net_id, layout) starts replicating one, and the layout says how its slots
// go (EntityLayout); remove_entity(key) stops.
//
// The host. tick() and receive_uplink() call back into an object the game passes them, with the methods
// listed below. Deriving it from ServerHost (at the end of this file) leaves only read, encode and send
// to write; the rest default to a game that doesn't use them.
//
// The host passed to tick() provides:
//   bool live(Entity &e)                        is the entity replicating this tick? (may refresh e.user)
//   void read(Entity &e, bool polled_only)      read the slot values, calling stamp() for each; when
//                                               polled_only, skip the slots e.pushed() names
//   bool visible(const Entity &e, int32_t peer) per-peer visibility of a live entity
//   bool spawn_visible(uint64_t key, int32_t peer)  same, for a spawn record's visibility keys
//   void encode(Writer &w, const Entity &e, int slot, const V *base, uint32_t now_ms)
//        slot's current value (e.value(slot)), as its change from base, the value the peer holds,
//        or whole when base is null (always null for a slot the layout doesn't step). Client::apply's
//        host decodes it the same way.
//   void send(int32_t peer, const Writer &packet)
//
// An entity's changed slots go as a mask, a flag saying whether they're whole, and their bytes
// behind a length, so a client that can't decode them (an id it doesn't know) can step over them.
// They're whole when the peer doesn't hold the entity at its baseline, or a changed stepped slot's
// value at that baseline has left its history (SLOT_HISTORY); a whole entity also carries the
// layout's kinds_check, so a client whose host would read its slots differently can refuse them.
//
// Spawns. An entity the client already has (one loaded with the map, at the same net_id on both
// ends) needs only add_entity. A runtime entity, which the client has no other way to know about,
// is two registrations under one net_id, and the host makes both:
//
//   add_spawn(net_id, record)          tells each peer to create it; record.data is the host's recipe
//   add_entity(key, net_id, layout)    streams its slot state, like any other entity
//
// and when it's destroyed, despawn(net_id) and remove_entity(key). Calling both on the same tick
// is fine: a snapshot carries its spawns ahead of its entity deltas. A spawn reaches a peer the
// first time the entity is visible to it (see SpawnRecord::vis_keys), goes in that snapshot and
// the RELIABLE_BACKUP after it, then again whenever it is overdue, until the peer acks a snapshot
// carrying it; the despawn reaches only peers that got it, the same way. Until a spawn is proven
// delivered, the entity at its net_id and the entities in its vis_keys don't count as held by the
// peer, so each snapshot sends them whole: a client that missed the spawn discards their deltas,
// and acking those must not leave the server believing it has them.
//
// On the client, Client::apply() hands each spawn to the host's spawn() before it reads the
// deltas, so by the time spawn() returns, entity() must know the entity. A delta for an id the host
// doesn't know is discarded and the frame still acked, and an idle entity's state is then never sent
// again. A host that can't create it yet returns false from spawn(), and one that has it but can't
// take values yet returns ENTITY_NOT_READY from entity(); either way the client withholds the ack and the
// server keeps offering it (Client::apply has the details).
//
// The uplink. receive_uplink() takes each packet a peer sends: its commands and messages go to the
// host, and the server takes the rest, the acks and what the client asks for. The host passed to it
// provides:
//   void protocol_mismatch(int32_t peer)            a different build is talking to us (once per peer)
//   const Schema *command_schema(int32_t peer, uint32_t stream_id)   the stream's (uplink.h), or null
//   void command(int32_t peer, uint32_t stream_id, const Command &cmd)
//   void message(int32_t peer, const uint8_t *data, size_t size, bool reliable)
//
// Messages. Each peer's snapshots carry its end of a GoldSrc netchan (netchan.h): queue_reliable and
// queue_unreliable put the host's messages in the peer's next snapshots, ahead of the entities in the
// packet budget. The acks for both directions ride the uplink and the snapshot header.
//
#include "goldnet/buffer.h"
#include "goldnet/codec.h"
#include "goldnet/netchan.h"
#include "goldnet/protocol.h"
#include "goldnet/uplink.h"

#include <algorithm>
#include <cmath>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace goldnet {

struct NoUserData {};

// How an entity's slots replicate, as add_entity takes it. Fixed for the entity's life; adding it
// again replaces it.
struct EntityLayout {
	int slots = 0;
	// Per slot: the host codes it against the peer's baseline value (encode's base), so the server
	// keeps its recent values. Empty means none is.
	std::vector<uint8_t> stepped;
	// Per-slot push declaration: listing a slot is the host's promise to mark_dirty() every
	// write to it; every slot NOT listed is read every tick regardless of the mark. That
	// inversion is the point: an undeclared slot cannot go stale, so forgetting to mark one
	// costs a read instead of silently desyncing it for every peer. Empty means all polled; one
	// that doesn't line up with the slots declares nothing.
	std::vector<uint8_t> push;
	// Peer-invariant importance weight for priority-ordered overflow (see PeerRing::stale_since
	// and SAFE_PACKET_BYTES).
	float priority = 1.0f;
	// The host's summary of how it codes this entity's slots (see encode), sent with every whole
	// entity so a client can tell it would read them differently.
	uint8_t kinds_check = 0;
};

template <class V, class X = NoUserData>
class Server {
public:
	class Entity {
	public:
		uint64_t key() const { return key_; }       // the host's identity for it (stable for the entity's life)
		uint32_t net_id() const { return net_id_; } // its identity on the wire, identical on server and client
		int slots() const { return layout_.slots; }
		// Slot s's latest value, as read() last stamped it.
		const V &value(int p_slot) const { return vals_[p_slot]; }
		// The layout declares slot s pushed, so a polled-only read() skips it: the host promised to
		// mark_dirty() a write to it, and nobody did.
		bool pushed(int p_slot) const { return (size_t)p_slot < layout_.push.size() && layout_.push[p_slot]; }

		X user;

	private:
		friend class Server;

		// The layout codes slot s against the peer's baseline value.
		bool stepped(int p_slot) const { return (size_t)p_slot < layout_.stepped.size() && layout_.stepped[p_slot]; }

		uint64_t key_ = 0;
		uint32_t net_id_ = 0;
		EntityLayout layout_;
		// Push-based change notification. Starts true so an entity is read once when it registers —
		// that first read is what gives vals_ its contents, which every later full-state send (a new
		// peer, a PVS re-entry) is served from.
		bool dirty_ = true;
		// True when at least one slot is undeclared and so must be read on a tick where the entity
		// was not marked. Entities that declare every slot skip the poll.
		bool has_polled_ = true;
		// Change tracking, stamped ONCE per snapshot tick. The values an entity holds are the same
		// for every peer, so "did slot s change" is answered once here rather than re-derived per
		// peer. vals_ is the latest read; slot_ctr_[s] is the snapshot counter at which slot s last
		// changed. A peer needs slot s iff slot_ctr_[s] > its baseline's counter.
		std::vector<V> vals_;
		std::vector<uint32_t> slot_ctr_;
		// Each stepped slot's recent changes as (snapshot counter, value), in a ring: what a peer holds
		// at its baseline is the newest one at or before the baseline's counter.
		struct History {
			std::vector<std::pair<uint32_t, V>> ring;
			size_t next = 0; // where the next change goes; once full, the oldest
		};
		std::vector<History> history_;
	};

	// A runtime entity's creation record, replayed to each peer the first time the entity is
	// visible to it. spawner_net_id names what creates it on the client (0 = none) and goes on the
	// wire with data, which is opaque to the protocol. node_key is the host's own; the server never
	// reads it. vis_keys are the keys to ask spawn_visible() about, and the entities beyond the one
	// at the spawn's net_id that the spawn creates.
	struct SpawnRecord {
		uint32_t spawner_net_id = 0;
		uint64_t node_key = 0;
		std::vector<uint8_t> data;
		// Empty ⇒ no visibility info ⇒ send to everyone (fail open).
		std::vector<uint64_t> vis_keys;
	};

	// --- Entities ---

	// Start replicating an entity: p_key is the host's identity for it, p_net_id its identity on the
	// wire (the same on both ends), p_layout how its slots go. A key already added is replaced.
	Entity &add_entity(uint64_t p_key, uint32_t p_net_id, EntityLayout p_layout) {
		auto it = index_.find(p_key);
		Entity *e;
		if (it != index_.end()) {
			e = &ents_[it->second];
			*e = Entity();
		} else {
			index_[p_key] = ents_.size();
			e = &ents_.emplace_back();
		}
		e->key_ = p_key;
		e->net_id_ = p_net_id;
		EntityLayout &l = e->layout_;
		l = std::move(p_layout);
		l.slots = std::max(l.slots, 0);
		if (l.push.size() != (size_t)l.slots) {
			l.push.clear(); // a declaration that doesn't line up with the slots declares nothing
		}
		e->has_polled_ = l.push.empty() || std::find(l.push.begin(), l.push.end(), 0) != l.push.end();
		e->vals_.assign(l.slots, V());
		// Newer than any frame sent so far, so a peer that held an entity at this net_id before (a key
		// added again) is sent every slot, even one that reads as the default.
		e->slot_ctr_.assign(l.slots, ctr_ + 1);
		e->history_.assign(l.slots, {});
		return *e;
	}

	// Order-preserving, so the packet order of everything else is unaffected.
	bool remove_entity(uint64_t p_key) {
		auto it = index_.find(p_key);
		if (it == index_.end()) {
			return false;
		}
		const size_t idx = it->second;
		index_.erase(it);
		ents_.erase(ents_.begin() + idx);
		for (auto &kv : index_) {
			if (kv.second > idx) {
				kv.second--;
			}
		}
		return true;
	}

	Entity *find(uint64_t p_key) {
		auto it = index_.find(p_key);
		return it == index_.end() ? nullptr : &ents_[it->second];
	}

	const std::vector<Entity> &entities() const { return ents_; }

	// The next tick reads this entity's declared (pushed) slots too. False if it isn't registered.
	bool mark_dirty(uint64_t p_key) {
		Entity *e = find(p_key);
		if (!e) {
			return false;
		}
		e->dirty_ = true;
		return true;
	}

	// Called by the host's read(): record slot s's current value, stamping it if it moved.
	void stamp(Entity &p_e, int p_slot, const V &p_value) {
		if (p_value != p_e.vals_[p_slot]) {
			p_e.vals_[p_slot] = p_value;
			p_e.slot_ctr_[p_slot] = ctr_;
			if (p_e.stepped(p_slot)) {
				typename Entity::History &h = p_e.history_[p_slot];
				if (h.ring.size() < (size_t)SLOT_HISTORY) {
					h.ring.emplace_back(ctr_, p_value);
				} else {
					h.ring[h.next] = { ctr_, p_value };
				}
				h.next = (h.next + 1) % SLOT_HISTORY;
			}
		}
	}

	// --- Spawns ---

	// A net_id may be reused once its entity is despawned. A peer still owed the old one's despawn
	// gets it first, and the new spawn only once that despawn is acked: the client reads a snapshot's
	// spawns before its despawns, so the two together would replace nothing and then free the new one.
	void add_spawn(uint32_t p_net_id, SpawnRecord p_rec) { spawns_[p_net_id] = std::move(p_rec); }

	const std::unordered_map<uint32_t, SpawnRecord> &spawns() const { return spawns_; }

	// A spawned entity was destroyed: it becomes a despawn owed to every peer that RECEIVED the
	// spawn. With lazy spawning, peers that never saw the entity never got the spawn, so they're
	// owed nothing — scope the despawn to peers holding it (delivered) or mid-receiving it.
	void despawn(uint32_t p_net_id) {
		if (spawns_.erase(p_net_id) == 0) {
			return;
		}
		std::unordered_set<int32_t> needers;
		for (auto &kv : rings_) {
			PeerRing &pr = kv.second;
			if (pr.spawn_acked.count(p_net_id) || pr.spawn_wait.count(p_net_id)) {
				needers.insert(kv.first);
			}
			pr.spawn_wait.erase(p_net_id);  // stop resending the (now void) spawn
			pr.spawn_acked.erase(p_net_id); // and forget delivery, so a reused id re-spawns cleanly
			pr.stale_since.erase(p_net_id); // and stop tracking overflow-priority for a dead entity
		}
		if (!needers.empty()) { // joining any peers still owed an earlier use of the id
			despawn_pending_[p_net_id].insert(needers.begin(), needers.end());
		}
	}

	// --- Peers ---

	// Start serving p_peer: tick() sends it snapshots from the next tick on, and its uplink packets and
	// messages are taken. A no-op for a peer already added.
	void add_peer(int32_t p_peer) {
		if (rings_.try_emplace(p_peer).second) {
			peers_.push_back(p_peer);
		}
	}

	// Stop serving p_peer and forget everything about it. A no-op for a peer not added.
	void remove_peer(int32_t p_peer) {
		if (rings_.erase(p_peer) == 0) {
			return;
		}
		peers_.erase(std::find(peers_.begin(), peers_.end(), p_peer));
		uplink_.remove_peer(p_peer);
		// Drop the peer from any despawn needer-set so those despawns can still retire.
		for (auto it = despawn_pending_.begin(); it != despawn_pending_.end();) {
			it->second.erase(p_peer);
			if (it->second.empty()) {
				it = despawn_pending_.erase(it);
			} else {
				++it;
			}
		}
	}

	bool has_peer(int32_t p_peer) const { return rings_.count(p_peer) > 0; }
	// The peers added, in the order they were, which is the order tick() serves them in.
	const std::vector<int32_t> &peers() const { return peers_; }

	// A peer's link, as the server sees it. All zeros for a peer not added.
	struct PeerStats {
		// GoldSrc rate: the most bytes/sec the peer's link takes, as it asks in its uplink (at least
		// MIN_RATE; 0 = unlimited). See charge().
		uint32_t rate = 0;
		// GoldSrc cl_updaterate: the snapshots/sec the peer was granted at its last tick, as its headers
		// tell it; 0 = every tick, at a rate the server doesn't know. A peer asks for a number in its
		// uplink (0 = as many as the server sends) and is granted that, under the snapshot rate and
		// under what its rate can carry in idle snapshots, and at least MIN_UPDATE_RATE, without
		// slowing anyone else down.
		uint32_t update_rate = 0;
		uint32_t chokes = 0; // snapshots withheld by its rate so far (GoldSrc's choke count)
		// How far behind its best the peer's uplink arrives, ms, and its packets arriving per second,
		// as its snapshot headers report them to its CmdRateAuto.
		uint32_t uplink_lag_ms = 0;
		uint32_t uplink_pps = 0;
		uint32_t commands_skipped = 0; // commands the peer gave up on before they arrived
		// Reliable messages queued to the peer, and how many of those it has. Both in queue order.
		uint64_t messages_queued = 0;
		uint64_t messages_delivered = 0;
		uint32_t datagrams_dropped = 0; // unreliable messages dropped for not fitting
		// The peer isn't taking its reliable data: GoldSrc dropped the client.
		bool overflowed = false;
	};
	PeerStats peer_stats(int32_t p_peer) const {
		PeerStats st;
		auto it = rings_.find(p_peer);
		if (it == rings_.end()) {
			return st;
		}
		const PeerRing &pr = it->second;
		st.rate = pr.rate_bps;
		st.update_rate = pr.update_rate;
		st.chokes = pr.chokes;
		st.uplink_lag_ms = pr.uplink_lag_ms;
		st.uplink_pps = pr.uplink_pps;
		st.commands_skipped = uplink_.skipped(p_peer);
		st.messages_queued = pr.rel.queued();
		st.messages_delivered = pr.rel.delivered();
		st.datagrams_dropped = pr.dgram.dropped();
		st.overflowed = pr.rel.overflowed();
		return st;
	}

	// The rate tick() is called at, snapshots/sec: the ceiling on every grant, and the tick a slower
	// peer's cadence is counted in. 0 = unknown.
	void set_snapshot_rate(double p_hz) { snapshot_rate_ = p_hz > 0.0 ? p_hz : 0.0; }
	double snapshot_rate() const { return snapshot_rate_; }

	// Wire bytes a snapshot costs beyond its payload (headers below the protocol), charged per packet.
	void set_packet_overhead(int p_bytes) { packet_overhead_ = p_bytes > 0 ? p_bytes : 0; }
	// GoldSrc choke. Every send to a rated peer — snapshots here, and whatever else the host reports
	// through this — pushes its clear time out by bytes / rate. While that's in the future the peer is
	// choked: it gets no snapshot this tick, and the next one it does get deltas against its acked
	// baseline, so it carries the latest state rather than a backlog. A slow link then gets fewer
	// snapshots instead of a queue that delays every one of them. A no-op for an unrated peer.
	void charge(int32_t p_peer, uint32_t p_now, size_t p_bytes) {
		auto it = rings_.find(p_peer);
		if (it == rings_.end() || it->second.rate_bps == 0) {
			return;
		}
		PeerRing &pr = it->second;
		const double start = pr.clear_at_ms > (double)p_now ? pr.clear_at_ms : (double)p_now;
		pr.clear_at_ms = start + (double)p_bytes * 1000.0 / (double)pr.rate_bps;
	}

	// --- The uplink (uplink.h) ---

	// One uplink packet from p_peer: its commands and messages go to p_host, its snapshot and channel
	// acks to this server, and the peer's next header gets the command ack, the uplink ack and the
	// uplink report. The rate and update rate it asks for apply from here on. False, with the reason,
	// if the packet was malformed, from a mismatched build, or from a peer not added (which is
	// dropped untouched).
	template <class Host>
	Result receive_uplink(int32_t p_peer, const uint8_t *p_data, size_t p_size, Host &p_host, uint32_t p_now);

	// A command stream's field kinds, as the client's Client::set_command_schema names them. The
	// uplink reads a stream set here by it, and asks the host's command_schema() for any other.
	void set_command_schema(uint32_t p_stream, const Schema &p_schema) { schemas_[p_stream] = p_schema; }

	// --- Messages (netchan.h) ---

	// False, and nothing queued, for a peer not added.
	bool queue_reliable(int32_t p_peer, const uint8_t *p_data, size_t p_size) {
		auto it = rings_.find(p_peer);
		if (it == rings_.end()) {
			return false;
		}
		it->second.rel.queue(p_data, p_size);
		return true;
	}
	// False if the peer's datagram is full and the message was dropped, or the peer isn't added.
	bool queue_unreliable(int32_t p_peer, const uint8_t *p_data, size_t p_size) {
		auto it = rings_.find(p_peer);
		return it != rings_.end() && it->second.dgram.queue(p_data, p_size);
	}

	// Opt-in relevance events: the server sends leave markers for entities that drop out of a
	// peer's PVS. Off by default so the protocol stays pure state replication.
	void set_relevance_events(bool p_enabled) { relevance_events_ = p_enabled; }
	bool relevance_events() const { return relevance_events_; }

	// Build and send this tick's snapshot to every peer that is due one.
	template <class Host>
	void tick(uint32_t p_now, Host &p_host);

private:
	// A stepped slot's value as of counter p_ctr, or null if that has left its history.
	static const V *value_at(const Entity &p_e, int p_slot, uint32_t p_ctr) {
		const typename Entity::History &h = p_e.history_[p_slot];
		const size_t n = h.ring.size();
		const size_t oldest = n < (size_t)SLOT_HISTORY ? 0 : h.next;
		if (n == 0 || h.ring[oldest].first > p_ctr) {
			return nullptr;
		}
		for (size_t i = 1; i <= n; i++) {
			const std::pair<uint32_t, V> &entry = h.ring[(oldest + n - i) % n];
			if (entry.first <= p_ctr) {
				return &entry.second;
			}
		}
		return nullptr;
	}

	// A client acked the newest frame it received. Advances that peer's delta baseline.
	void on_ack(int32_t p_peer, uint16_t p_seq, uint32_t p_now) {
		auto rit = rings_.find(p_peer);
		if (rit == rings_.end()) {
			return;
		}
		PeerRing &pr = rit->second;
		if (!pr.has_ack || seq_newer(p_seq, pr.last_acked)) {
			pr.last_acked = p_seq;
			pr.has_ack = true;
			// This frame is now what the peer holds — the set every PVS removal is derived against.
			// Copied rather than read from the ring at send time so it survives the frame aging out
			// under heavy loss: a stale-but-real `held` re-sends a removal the peer may already have
			// (harmless, the client just hides an entity twice), while an empty one would lose it.
			const int hslot = p_seq & (RING - 1);
			if (pr.frame_seq[hslot] == p_seq && p_seq != 0) {
				pr.held = pr.frames[hslot];
				pr.left_unacked.clear(); // the acked frame already excludes them
				// Only the first ack of a frame is timed: the client repeats it until a newer frame.
				const uint32_t sample = p_now - pr.frame_ms[hslot];
				pr.srtt_ms = pr.srtt_ms == 0 ? sample : (pr.srtt_ms * 7 + sample) / 8;
			}
		}
		// Retire spawn/despawn records this ack proves delivered.
		std::vector<uint32_t> retired;
		retire_proven(pr.spawn_wait, p_seq, retired);
		for (uint32_t nid : retired) {
			pr.spawn_acked.insert(nid); // durably remember delivery (spawn source record persists)
		}
		retired.clear();
		retire_proven(pr.despawn_wait, p_seq, retired);
		for (uint32_t nid : retired) {
			auto dit = despawn_pending_.find(nid);
			if (dit != despawn_pending_.end()) {
				dit->second.erase(p_peer);
				if (dit->second.empty()) {
					despawn_pending_.erase(dit); // all peers have it — done
				}
			}
		}
	}

	// From the peer's uplink: what it has received of the server's channel.
	void on_channel_ack(int32_t p_peer, const ChannelAck &p_ack) {
		auto it = rings_.find(p_peer);
		if (it != rings_.end()) {
			it->second.rel.on_ack(p_ack);
		}
	}

	// What the peer HAD at each frame, as net_ids only — values live once, on the entity, and
	// slot_ctr answers "did this change since the peer's baseline" directly. All a frame has to
	// remember is WHICH entities the peer had, so a re-entering entity still gets a full state
	// rather than a delta against something it never received.
	typedef std::unordered_set<uint32_t> IdSet;

	// Per-peer send history. Each new frame deltas against the frame the peer last acked; the
	// ring lets a promoted ack become the next baseline and lets a lost ack self-heal (we keep
	// diffing against the same acked frame). frame_seq[] guards against a stale (aged-out) slot.
	struct PeerRing {
		IdSet frames[RING];
		uint16_t frame_seq[RING] = {}; // 0 = empty
		// The monotonic snapshot counter each frame was sent at. The wire seq stays per-peer
		// (16-bit, its own space), while change tracking needs a counter that never aliases: a slot
		// untouched for 32768 ticks would otherwise compare as newly changed.
		uint32_t frame_ctr[RING] = {};
		uint32_t frame_ms[RING] = {}; // when each frame went, to time its ack
		uint32_t srtt_ms = 0;         // smoothed snapshot-to-ack round trip; 0 = none timed yet
		uint16_t next_seq = 1; // 0 is reserved for "no baseline / full state"
		uint16_t last_acked = 0;
		bool has_ack = false;
		// Reliable-until-acked spawn/despawn delivery: net_id -> the snapshots that carried it,
		// until an ack of one of them proves it arrived (see owe_include / retire_proven).
		std::unordered_map<uint32_t, Owed> spawn_wait;
		std::unordered_map<uint32_t, Owed> despawn_wait;
		// Spawns already acked by this peer. Unlike despawns (one-shot — the record is dropped once
		// delivered), a spawn's source record lives as long as the entity, so "absent from
		// spawn_wait" can't mean "delivered" — it would re-arm the spawn every frame. This set is the
		// durable "peer already has this node" marker; a net_id here is never re-sent until the
		// entity despawns (which clears it).
		IdSet spawn_acked;
		// Per-peer PVS relevance, GoldSrc-style: removals are DERIVED from the delta baseline rather
		// than queued. `held` is what this peer's last acked frame says it holds; anything in there
		// that the peer can no longer see gets a remove marker in the next snapshot. A lost snapshot
		// isn't acked, so `held` doesn't move and the same diff recomputes the same markers next
		// tick — self-correcting, with no retry queue to leak or starve. Empty until the first ack,
		// which is also why a joining peer gets no leave burst: it holds nothing yet.
		IdSet held;
		// Removals written since this peer's last ack. The peer has been told to drop these, but
		// `held` won't reflect it until the frame carrying them is acked — so a re-entry inside that
		// window must ship a full baseline, not a slot delta against a frame the peer no longer
		// matches. Soft state: losing it only costs a redundant full baseline.
		IdSet left_unacked;
		uint32_t update_rate_asked = 0; // from the uplink; see PeerStats::update_rate
		uint32_t update_rate = 0;       // granted, rounded, as the header carries it
		// When this peer was last considered for a send, for the cadence check. Advances even on a
		// tick that decides to send nothing.
		uint32_t last_sent_ms = 0;
		// When a snapshot was last actually EMITTED to this peer. Gates the clock keepalive, which
		// has to know how long the peer has really been without a header.
		uint32_t last_packet_ms = 0;
		// Priority-ordered overflow. net_id -> the snapshot counter this entity FIRST had an unsent
		// pending change for THIS peer; cleared once it's actually written. The wait, weighted by
		// EntityLayout::priority, scores entities when more changed this tick than fit in the budget — the
		// most-overdue, highest-weighted entity goes first. Only a successful write resets it, not
		// merely a later tick touching it again.
		std::unordered_map<uint32_t, uint32_t> stale_since;
		// The newest command delivered with none missing before it, for the next header; 0 = none.
		uint16_t cmd_ack = 0;
		uint16_t cmd_ack_sent = 0; // what the last snapshot told the peer, so a new one earns a send
		bool cmd_news = false;     // CommandReceiver::take_news: earns a snapshot though the ack hasn't moved
		uint32_t uplink_lag_ms = 0; // the uplink report, for the next header
		uint32_t uplink_pps = 0;
		uint32_t rate_bps = 0;     // see PeerStats::rate; 0 = unlimited
		double clear_at_ms = 0.0;  // when what's been sent to this peer has cleared its rate
		uint32_t chokes = 0;
		ReliableSender rel;
		DatagramSender dgram;
		// What the server has received of the peer's channel, for its next header. A new block bit or
		// fragment earns a snapshot of its own: it is what lets the peer send on.
		ChannelAckSource uplink;
		ChannelAck uplink_sent;  // what the last snapshot told the peer
	};

	struct Candidate {
		size_t idx;
		uint32_t net_id;
		uint32_t mask;
		float score;
		bool holdable; // false while the entity's spawn isn't proven delivered to this peer
		bool whole;    // its slots go whole rather than as their change from the peer's baseline
	};

	std::vector<Entity> ents_;
	std::unordered_map<uint64_t, size_t> index_;
	std::unordered_map<uint32_t, SpawnRecord> spawns_;
	// Despawns awaiting delivery: the peers that still need each one (reliable-until-acked); erased
	// once every such peer acks a frame carrying it.
	std::unordered_map<uint32_t, std::unordered_set<int32_t>> despawn_pending_;
	std::unordered_map<int32_t, PeerRing> rings_;
	std::vector<int32_t> peers_; // rings_'s keys, in the order add_peer() added them
	CommandReceiver uplink_;
	std::unordered_map<uint32_t, Schema> schemas_; // set_command_schema's
	// Monotonic snapshot counter, bumped once per tick that sends anything. It stamps
	// Entity::slot_ctr_ and PeerRing::frame_ctr so change tests are a single integer compare. Unlike
	// the 16-bit wire seq it never wraps in any realistic session (2^32 ticks at 60 Hz is ~2 years).
	uint32_t ctr_ = 0;
	int packet_overhead_ = 0;
	double snapshot_rate_ = 0.0;
	bool relevance_events_ = false;
	std::vector<size_t> live_; // scratch: entities live this tick
	std::unordered_set<uint64_t> unproven_; // scratch: entity keys whose spawn this peer may lack
	std::unordered_set<uint32_t> unproven_ids_; // scratch: the same, by the spawn's own net_id

	// See PeerStats::update_rate. 0 = every tick.
	double grant(const PeerRing &p_pr) const {
		double hz = (double)p_pr.update_rate_asked;
		if (p_pr.rate_bps > 0) {
			const double cap = p_pr.rate_bps * UPDATE_RATE_RATE_SHARE / (double)(IDLE_SNAPSHOT_BYTES + packet_overhead_);
			hz = hz > 0.0 ? std::min(hz, cap) : cap;
		}
		if (snapshot_rate_ > 0.0 && (hz <= 0.0 || hz > snapshot_rate_)) {
			hz = snapshot_rate_;
		}
		if (hz <= 0.0) {
			return 0.0;
		}
		hz = std::max(hz, (double)MIN_UPDATE_RATE);
		if (snapshot_rate_ > 0.0) {
			// Served every nth tick, so that is what it gets.
			double n = std::ceil(snapshot_rate_ / hz - 1e-9);
			n = std::max(1.0, std::min(n, std::floor(snapshot_rate_ / (double)MIN_UPDATE_RATE)));
			hz = snapshot_rate_ / n;
		}
		return hz;
	}
};

template <class V, class X>
template <class Host>
Result Server<V, X>::receive_uplink(int32_t p_peer, const uint8_t *p_data, size_t p_size, Host &p_host,
		uint32_t p_now) {
	auto rit = rings_.find(p_peer);
	if (rit == rings_.end()) {
		return { STATUS_UNKNOWN_PEER };
	}
	struct Router {
		Server &server;
		Host &host;
		uint32_t now;
		void protocol_mismatch(int32_t p) { host.protocol_mismatch(p); }
		void snapshot_ack(int32_t p, uint16_t seq) { server.on_ack(p, seq, now); }
		void channel_ack(int32_t p, const ChannelAck &a) { server.on_channel_ack(p, a); }
		const Schema *command_schema(int32_t p, uint32_t stream) {
			auto it = server.schemas_.find(stream);
			return it != server.schemas_.end() ? &it->second : host.command_schema(p, stream);
		}
		void command(int32_t p, uint32_t stream, const Command &c) { host.command(p, stream, c); }
		void message(int32_t p, const uint8_t *d, size_t n, bool reliable) { host.message(p, d, n, reliable); }
	};
	Router router{ *this, p_host, p_now };
	const Result res = uplink_.receive(p_peer, p_data, p_size, router, p_now);
	PeerRing &pr = rit->second;
	pr.cmd_ack = uplink_.ack(p_peer);
	pr.cmd_news = pr.cmd_news || uplink_.take_news(p_peer);
	pr.uplink = uplink_.channel_ack(p_peer);
	pr.uplink_lag_ms = uplink_.uplink_lag_ms(p_peer);
	pr.uplink_pps = uplink_.uplink_pps(p_peer);
	const uint32_t rate = uplink_.rate(p_peer);
	pr.rate_bps = rate > 0 && rate < MIN_RATE ? MIN_RATE : rate;
	pr.update_rate_asked = uplink_.update_rate(p_peer);
	return res;
}

template <class V, class X>
template <class Host>
void Server<V, X>::tick(uint32_t p_now, Host &p_host) {
	// Resolve which peers are due BEFORE the per-entity state read below: on a tick where every
	// peer is throttled (all of them on a slower cadence than the server's) this costs nothing.
	// last_sent_ms == 0 always sends — a peer's first snapshot must not wait out an interval.
	// A slower peer is due half a tick early, so clock jitter can't push it a whole tick late.
	const double slack_ms = snapshot_rate_ > 0.0 ? 500.0 / snapshot_rate_ : 0.0;
	std::vector<int32_t> due;
	for (int32_t peer : peers_) {
		PeerRing &pr = rings_[peer];
		const double hz = grant(pr);
		pr.update_rate = (uint32_t)std::lround(hz);
		const bool gated = hz > 0.0 && !(snapshot_rate_ > 0.0 && hz >= snapshot_rate_);
		if (gated && pr.last_sent_ms != 0 && (double)(p_now - pr.last_sent_ms) + slack_ms < 1000.0 / hz) {
			continue;
		}
		if (pr.rate_bps > 0 && pr.clear_at_ms > (double)p_now) {
			pr.chokes++;
			continue;
		}
		due.push_back(peer);
	}
	if (due.empty()) {
		return;
	}

	// Read every entity's current state ONCE per tick — the values are the server's authoritative
	// state, identical for all peers. Only the delta mask (vs. each peer's baseline) and the
	// visibility filter are peer-specific, so those stay in the per-peer loop.
	ctr_++;
	live_.clear();
	for (size_t i = 0; i < ents_.size(); i++) {
		Entity &e = ents_[i];
		if (!p_host.live(e)) {
			continue;
		}
		// A marked entity is read whole. A clean one keeps the values and stamps it already has for
		// every slot it declared, which is all any peer — including one being sent a full baseline —
		// needs from it. Undeclared slots on an unmarked entity are read anyway: that is what keeps a
		// missed mark_dirty from being a silent, permanent desync.
		if (e.dirty_) {
			p_host.read(e, false);
			e.dirty_ = false;
		} else if (e.has_polled_) {
			p_host.read(e, true);
		}
		live_.push_back(i);
	}

	std::vector<Candidate> candidates;
	Writer vals; // one entity's slot bytes, reused so its capacity outlives the entity
	for (int32_t peer : due) {
		PeerRing &pr = rings_[peer];
		pr.last_sent_ms = p_now;

		// Baseline = the frame this peer last acked, if we still hold it. Otherwise send a full
		// frame (baseline seq 0) — first send, or an ack aged out of the ring under heavy loss.
		uint16_t base_seq = 0;
		const IdSet *base = nullptr;
		uint32_t base_ctr = 0;
		if (pr.has_ack) {
			const int bslot = pr.last_acked & (RING - 1);
			if (pr.frame_seq[bslot] == pr.last_acked && pr.last_acked != 0) {
				base_seq = pr.last_acked;
				base = &pr.frames[bslot];
				base_ctr = pr.frame_ctr[bslot];
			}
		}

		// Taken for good only once the snapshot goes (below), so a gap in the seqs a client
		// receives is a lost snapshot, never a skipped one.
		const uint16_t seq = pr.next_seq;

		// The reliable block first: it has the packet before anything else. One written here always
		// goes, since it makes the snapshot non-empty.
		std::vector<uint8_t> block;
		const uint8_t block_flags = pr.rel.write(seq, block);
		const ChannelAck up = pr.uplink.at(p_now);

		// Spawns owed to this peer, reliable-until-acked.
		Writer spawn_body;
		uint16_t spawn_ct = 0;
		const uint32_t overdue = overdue_ms(pr.srtt_ms);
		unproven_.clear();
		unproven_ids_.clear();
		for (const auto &kv : spawns_) {
			if (pr.spawn_acked.count(kv.first)) {
				continue; // peer already has this node — don't re-arm the spawn every frame
			}
			unproven_.insert(kv.second.vis_keys.begin(), kv.second.vis_keys.end());
			unproven_ids_.insert(kv.first);
			// Lazy spawn (GoldSrc): defer sending the spawn to this peer until the entity is visible
			// to it — created client-side the first time it enters your PVS. Once delivered it
			// PERSISTS (spawn_acked keeps it; despawn only on server-free), so there's no
			// create/destroy churn as it moves in and out of view.
			if (!kv.second.vis_keys.empty()) {
				bool visible = false;
				for (uint64_t vk : kv.second.vis_keys) {
					if (p_host.spawn_visible(vk, peer)) {
						visible = true;
						break;
					}
				}
				if (!visible) {
					continue; // not in this peer's PVS yet — defer the spawn
				}
			}
			auto dit = despawn_pending_.find(kv.first);
			if (dit != despawn_pending_.end() && dit->second.count(peer)) {
				continue; // the id's last use isn't proven gone from this peer yet (add_spawn)
			}
			// on_ack is the sole populator of spawn_acked, the moment an ack proves a carrier arrived.
			if (!owe_include(pr.spawn_wait, kv.first, seq, p_now, overdue, RELIABLE_BACKUP)) {
				continue;
			}
			spawn_body.put_u32(kv.first);
			spawn_body.put_u32(kv.second.spawner_net_id);
			spawn_body.put_u32((uint32_t)kv.second.data.size());
			spawn_body.put_data(kv.second.data.data(), kv.second.data.size());
			spawn_ct++;
		}
		Writer despawn_body;
		uint16_t despawn_ct = 0;
		for (auto &kv : despawn_pending_) {
			if (!kv.second.count(peer)) {
				continue;
			}
			if (!owe_include(pr.despawn_wait, kv.first, seq, p_now, overdue, RELIABLE_BACKUP)) {
				continue; // not due again yet; on_ack drops the peer once it's proven delivered
			}
			despawn_body.put_u32(kv.first);
			despawn_ct++;
		}

		// Per-peer entity delta: filter by visibility, store the visible subset as this peer's next
		// baseline, and emit only the changed slots.
		//
		// Two sets, not one. `frame` is visibility this tick — every entity the peer can currently
		// see, used for PVS-leave detection, regardless of whether its bytes fit on the wire.
		// `included` is the peer's actual reconstructed state after this packet — only entities
		// whose current value the peer truly has (unchanged-so-nothing-to-send, or successfully
		// written this tick). It becomes the baseline future deltas compare against, so a changed
		// entity left out by the byte budget must NOT land in it: if it did, an ack of this seq
		// would push base_ctr past that entity's slot_ctr and the change would never be resent. Left
		// out of `included`, the same compare catches it again next tick — free retry.
		IdSet frame;
		IdSet included;
		Writer body;
		uint16_t changed = 0;

		// Pass 1: visibility + mask for every entity. Unchanged entities are settled immediately;
		// changed ones become overflow candidates, scored for pass 2.
		candidates.clear();
		// Set when any candidate's score departs from the flat 1.0 default. While it stays false
		// every score is identical and sorting is a no-op, so pass 2 skips it.
		bool needs_sort = false;
		for (size_t idx : live_) {
			const Entity &e = ents_[idx];
			if (!p_host.visible(e, peer)) {
				continue;
			}
			const uint32_t net_id = e.net_id_;
			frame.insert(net_id);

			// Did this peer hold this entity at its baseline? If not (first sight, or it left and
			// re-entered the peer's PVS) it needs every slot. Otherwise a slot changed iff it was
			// stamped after the baseline's counter — no value comparison here. left_unacked: we've
			// already told this peer to drop the entity, but its acked baseline still lists it — a
			// slot delta against that baseline would leave the client hidden with nothing to re-show
			// it (an idle entity re-entering changes no slot at all).
			uint32_t mask = 0;
			bool whole = false;
			const int n_vals = (int)e.vals_.size();
			const bool holdable = !unproven_.count(e.key_) && !unproven_ids_.count(net_id);
			if (!base || !base->count(net_id) || pr.left_unacked.count(net_id)) {
				mask = n_vals >= MAX_SLOTS ? 0xFFFFFFFFu : ((1u << n_vals) - 1u); // new → all slots
				whole = true;
			} else {
				// Every slot that changed since the baseline goes, even one back at the baseline's value:
				// the client may have applied a newer frame since. One whose baseline value has left
				// its history sends the entity whole.
				for (int s = 0; s < n_vals; s++) {
					if (e.slot_ctr_[s] > base_ctr) {
						mask |= (1u << s);
						whole = whole || (e.stepped(s) && !value_at(e, s, base_ctr));
					}
				}
			}
			if (mask == 0) {
				if (holdable) {
					included.insert(net_id); // peer's existing value is already correct
				}
				pr.stale_since.erase(net_id); // no longer owed anything — clear any old overflow mark
				continue;
			}
			// Score = how many ticks this entity has been waiting for THIS peer, weighted by its
			// priority. Only read here — pass 2 commits a wait-start or clears one.
			auto wit = pr.stale_since.find(net_id);
			const uint32_t wait_ticks = wit != pr.stale_since.end() ? ctr_ - wit->second : 0;
			Candidate c;
			c.idx = idx;
			c.net_id = net_id;
			c.mask = mask;
			c.score = (float)(wait_ticks + 1) * e.layout_.priority;
			c.whole = whole;
			c.holdable = holdable;
			if (c.score != 1.0f) {
				needs_sort = true;
			}
			candidates.push_back(c);
		}

		// Relevance leaves (opt-in), derived GoldSrc-style from the peer's acked baseline: every
		// entity that baseline holds and this frame can't see gets a remove marker. Nothing is
		// queued — a lost snapshot goes unacked, `held` stays put, and the next diff produces the
		// same markers. Enters need no event: an entering entity re-appears with a full baseline.
		// Built before pass 2 because the entity budget is whatever the packet has left once every
		// other section is built.
		Writer leave_body;
		uint16_t leave_ct = 0;
		if (relevance_events_) {
			std::vector<uint32_t> leaving;
			std::vector<uint32_t> carried;
			derive_leaves(pr.held, frame, MAX_LEAVES_PER_SNAPSHOT, leaving, carried);
			for (uint32_t id : leaving) {
				leave_body.put_u32(id);
				leave_ct++;
				// Told to drop it, so it leaves this frame's baseline too — which is what makes a
				// later re-entry a clean all-slots delta. Until the peer acks that, left_unacked
				// covers the window where its own baseline still disagrees.
				pr.left_unacked.insert(id);
			}
			// Held back by the cap: the peer still holds these, so they stay in the baseline and the
			// next diff picks them up again. Dropping them here would lose the removal entirely.
			for (uint32_t id : carried) {
				included.insert(id);
			}
		}

		// Pass 2: highest score first — the most-overdue, highest-weighted entity wins the budget.
		// Stable, so equal scores keep registration order.
		if (needs_sort) {
			std::stable_sort(candidates.begin(), candidates.end(),
					[](const Candidate &a, const Candidate &b) { return a.score > b.score; });
		}

		// What's left of a safe UDP payload after the header and every section already built —
		// measured, not estimated.
		int peer_budget = SAFE_PACKET_BYTES - SNAPSHOT_HEADER_BYTES - (int)spawn_body.size()
				- (int)despawn_body.size() - (int)leave_body.size();
		peer_budget -= (int)block.size() + (up.fragments ? 2 : 0);
		// Then the datagram, all of it or none, and the entities get the rest.
		std::vector<uint8_t> datagram;
		pr.dgram.take(peer_budget, datagram);
		peer_budget -= (int)datagram.size();

		for (size_t ci = 0; ci < candidates.size(); ci++) {
			const Candidate &c = candidates[ci];
			// Past budget, leave this entity out (see `included` above for why that's safe) and mark
			// it waiting so it outranks everything else next tick if it's still unlucky then.
			//
			// `ci > 0` makes the highest-scoring candidate unconditional, which guarantees forward
			// progress: a budget of zero or less (a spawn section that spent
			// the whole MTU) would otherwise starve replication completely and silently. One entity of
			// overrun beats a permanent stall, and it is self-limiting: the sections that crowded the
			// budget out are reliable-until-acked, so they retire and the budget reopens.
			if (ci > 0 && (int)body.size() >= peer_budget) {
				if (!pr.stale_since.count(c.net_id)) {
					pr.stale_since[c.net_id] = ctr_;
				}
				continue;
			}
			const Entity &e = ents_[c.idx];
			vals.clear();
			for (size_t s = 0; s < e.vals_.size(); s++) {
				if (c.mask & (1u << s)) {
					const bool from_base = !c.whole && e.stepped((int)s);
					p_host.encode(vals, e, (int)s, from_base ? value_at(e, (int)s, base_ctr) : nullptr, p_now);
				}
			}
			body.put_u32(c.net_id);
			put_uvarint(&body, c.mask); // most entities set only the low few bits — 1 byte, not 4
			put_uvarint(&body, (uint32_t)(vals.size() << 1) | (c.whole ? 1u : 0u));
			if (c.whole) {
				body.put_u8(e.layout_.kinds_check);
			}
			body.put_data(vals);
			if (c.holdable) {
				included.insert(c.net_id);
			}
			pr.stale_since.erase(c.net_id); // delivered (pending ack) — no longer owed
			changed++;
		}

		// Nothing to report: skip the header-only packet — except that the header carries
		// server_time, the client's only clock feed, so "nothing to report" only earns a skip while
		// that feed is still fresh. last_packet_ms == 0 is a peer never sent anything — its first
		// snapshot must go out regardless, or the clock never starts. Nothing above recorded `seq`
		// unless something went in the packet, so the next snapshot reuses it.
		// A command ack the peer hasn't been told is news too: it is what stops the peer resending.
		if (spawn_ct == 0 && despawn_ct == 0 && leave_ct == 0 && changed == 0 && pr.cmd_ack == pr.cmd_ack_sent && !pr.cmd_news
				&& block_flags == 0 && datagram.empty() && up.bit == pr.uplink_sent.bit
				&& (!up.fragments || up.next_fragment == pr.uplink_sent.next_fragment)
				&& pr.last_packet_ms != 0 && p_now - pr.last_packet_ms < CLOCK_KEEPALIVE_MS) {
			continue;
		}
		pr.last_packet_ms = p_now;
		pr.cmd_ack_sent = pr.cmd_ack;
		pr.cmd_news = false;
		pr.uplink_sent = up;
		pr.next_seq++;
		if (pr.next_seq == 0) {
			pr.next_seq = 1; // 0 is reserved for "no baseline"
		}

		const int slot = seq & (RING - 1);
		pr.frames[slot] = std::move(included);
		pr.frame_seq[slot] = seq;
		pr.frame_ctr[slot] = ctr_;
		pr.frame_ms[slot] = p_now;

		Writer packet;
		packet.put_u8(WIRE_VERSION);
		packet.put_u16(seq);
		packet.put_u16(base_seq);
		packet.put_u32(p_now);
		packet.put_u16(pr.cmd_ack);
		put_uvarint(&packet, pr.uplink_lag_ms);
		put_uvarint(&packet, pr.uplink_pps);
		put_uvarint(&packet, pr.update_rate);
		put_channel(packet, up, block_flags, block, datagram);
		packet.put_u16(spawn_ct);
		packet.put_data(spawn_body);
		packet.put_u16(despawn_ct);
		packet.put_data(despawn_body);
		packet.put_u16(leave_ct);
		packet.put_data(leave_body);
		packet.put_u16(changed);
		packet.put_data(body);
		p_host.send(peer, packet);
		charge(peer, p_now, packet.size() + (size_t)packet_overhead_);
	}
}

// A base for the host Server::tick and Server::receive_uplink call back into, one object for both:
// derive from it and define read, encode and send, which it leaves pure. Everything else has
// a default that suits a game not using the feature, so define only what you use, marked override
// so a misspelt name fails to compile rather than quietly keeping the default. A host needn't derive
// from it at all: the server calls whatever the object it's handed has, and this only supplies the rest.
template <class V, class X = NoUserData>
struct ServerHost {
	typedef typename Server<V, X>::Entity Entity;

	virtual ~ServerHost() = default;

	virtual void read(Entity &e, bool polled_only) = 0;
	virtual void encode(Writer &w, const Entity &e, int slot, const V *base, uint32_t now_ms) = 0;
	virtual void send(int32_t peer, const Writer &packet) = 0;

	// Every entity added replicates, to every peer.
	virtual bool live(Entity &) { return true; }
	virtual bool visible(const Entity &, int32_t) { return true; }
	virtual bool spawn_visible(uint64_t, int32_t) { return true; }

	// The uplink: no commands or messages taken, and a stream with no schema set on the server
	// (Server::set_command_schema) is one FIELD_BYTES field, as it is on the client.
	virtual void protocol_mismatch(int32_t) {}
	virtual const Schema *command_schema(int32_t, uint32_t) {
		static const Schema bytes_only{ FIELD_BYTES };
		return &bytes_only;
	}
	virtual void command(int32_t, uint32_t, const Command &) {}
	virtual void message(int32_t, const uint8_t *, size_t, bool) {}
};

} // namespace goldnet
