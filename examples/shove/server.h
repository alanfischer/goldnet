#pragma once
//
// The shove server: authoritative hop world, one goldnet Server streaming it.
//
// A client's commands run the moment they arrive, one move each, the way Quake ran usercmds, so
// the state a snapshot carries for a player is exactly the state after the command its header
// acks. That is what the client's reconciliation leans on. The blocks and bots step on the server's
// own 60 Hz tick.
//
#include "net.h"
#include "world.h"

#include "goldnet/netsim.h"
#include "goldnet/server.h"
#include "goldnet/uplink.h"

#include <algorithm>
#include <map>

namespace shove {

static const int MAX_PLAYERS = 8;
static const uint32_t FIRST_RUNTIME_ID = 1000;
static const uint32_t ARENA_BLOCK_ID = 100; // map-static blocks: 100, 101, ...
static const int MAX_BLOCKS = 48;
static const uint32_t HIT_CREDIT_MS = 5000;

// Entity kinds, the first byte of a spawn record.
static const uint8_t KIND_PLAYER = 1;
static const uint8_t KIND_BLOCK = 2;

// Player slots.
enum {
	PS_X, PS_Y, PS_Z, PS_VX, PS_VY, PS_VZ, PS_YAW, PS_PITCH, PS_ALIVE, PS_SCORE, PS_AMMO, PS_AMMO_TICKS,
	PS_COOLDOWN, PLAYER_SLOTS
};
// Block slots.
enum { BS_X, BS_Y, BS_Z, BS_QX, BS_QY, BS_QZ, BS_QW, BLOCK_SLOTS };

inline const std::vector<uint8_t> &player_quant() {
	static const std::vector<uint8_t> q = { TAG_FLOAT, TAG_FLOAT, TAG_FLOAT, TAG_HALF, TAG_HALF, TAG_HALF,
		TAG_ANGLE16, TAG_ANGLE16, QUANT_AUTO, QUANT_AUTO, QUANT_AUTO, QUANT_AUTO, QUANT_AUTO };
	return q;
}

inline const std::vector<uint8_t> &block_quant() {
	static const std::vector<uint8_t> q = { TAG_HALF, TAG_HALF, TAG_HALF, TAG_HALF, TAG_HALF, TAG_HALF, TAG_HALF };
	return q;
}

inline EntityLayout player_layout() {
	EntityLayout l;
	l.slots = PLAYER_SLOTS;
	l.priority = 4.0f; // players before blocks when a packet overflows
	return l;
}

inline EntityLayout block_layout() {
	EntityLayout l;
	l.slots = BLOCK_SLOTS;
	return l;
}

struct ServerApp : goldnet::ServerHost<Value> {
	struct Outgoing {
		std::string to;
		std::vector<uint8_t> bytes;
	};
	typedef goldnet::Server<Value> GServer;

	struct SPlayer : Player {
		int32_t peer = 0; // 0 = bot
		uint8_t color = 0;
		int64_t score = 0;
		uint32_t respawn_at = 0;
		uint32_t last_hitter = 0;
		uint32_t last_hit_ms = 0;
		uint32_t commands = 0; // since the last report
		// Bots only.
		uint32_t bot_think_ms = 0;
		float bot_strafe = 1.0f;
	};
	struct SBlock : Block {
		vec3 prev_vel;
		bool arena = false;
		vec3 home;
	};
	struct Peer {
		std::string addr;
		uint32_t last_heard = 0;
		uint32_t player = 0;
	};

	std::unique_ptr<Wire> wire;
	Sim sim;
	GServer server;
	NetSim<Outgoing> netsim;
	std::map<uint32_t, SPlayer> players;
	std::map<uint32_t, SBlock> blocks;
	std::map<int32_t, Peer> peers;
	std::map<std::string, int32_t> by_addr;
	int32_t next_peer = 1;
	uint32_t next_net_id = FIRST_RUNTIME_ID;
	uint32_t start_ms = 0;
	uint32_t ticks = 0;
	uint32_t next_tick = 0;
	uint32_t next_report = 0;
	size_t bytes_out = 0;
	int bots_wanted = 0;
	bool verbose = true;

	// --- goldnet's host interface (ServerHost): everything is visible to everyone, and the one
	// command stream is a byte string, so the defaults cover the rest ---

	void read(GServer::Entity &e, bool) override {
		auto pit = players.find(e.net_id());
		if (pit != players.end()) {
			const SPlayer &p = pit->second;
			const vec3 &pos = p.body->get_position();
			const vec3 &vel = p.body->get_velocity();
			const float v[] = { pos.x, pos.y, pos.z, vel.x, vel.y, vel.z, p.yaw, p.pitch };
			for (int s = 0; s < 8; s++) {
				server.stamp(e, s, Value(v[s]));
			}
			server.stamp(e, PS_ALIVE, Value((int64_t)p.alive));
			server.stamp(e, PS_SCORE, Value(p.score));
			server.stamp(e, PS_AMMO, Value((int64_t)p.ammo));
			server.stamp(e, PS_AMMO_TICKS, Value((int64_t)p.ammo_ticks));
			server.stamp(e, PS_COOLDOWN, Value((int64_t)p.cooldown));
			return;
		}
		const SBlock &b = blocks[e.net_id()];
		const vec3 &pos = b.body->get_position();
		const quat &q = b.body->get_orientation_quat();
		const float v[] = { pos.x, pos.y, pos.z, q.x, q.y, q.z, q.w };
		for (int s = 0; s < BLOCK_SLOTS; s++) {
			server.stamp(e, s, Value(v[s]));
		}
	}
	// Every value whole and self-tagged: simple, and right against any base.
	void encode(Writer &w, const GServer::Entity &e, int slot, const Value *, uint32_t) override {
		encode_value(w, e.value(slot), (players.count(e.net_id()) ? player_quant() : block_quant())[slot]);
	}
	void send(int32_t peer, const Writer &packet) override {
		Outgoing o{ peers[peer].addr, std::vector<uint8_t>(packet.data(), packet.data() + packet.size()) };
		bytes_out += o.bytes.size();
		if (netsim.loss_roll()) {
			return;
		}
		const uint32_t now = now_ms();
		const int delay = netsim.schedule(now, peer, o.bytes.size(), true);
		if (delay == 0) {
			transmit(o);
		} else if (delay > 0) {
			netsim.enqueue(now, peer, delay, std::move(o));
		}
	}

	void protocol_mismatch(int32_t peer) override {
		fprintf(stderr, "[server] peer %d speaks a different version\n", peer);
	}
	void command(int32_t peer, uint32_t, const goldnet::Command &c) override {
		auto it = players.find(peers[peer].player);
		Input in;
		if (it == players.end() || !read_input(c[0].bytes.data(), c[0].bytes.size(), in)) {
			return;
		}
		it->second.commands++;
		run_move(it->second, in);
	}

	// --- the game ---

	void run_move(SPlayer &p, const Input &in) {
		if (move_player(sim, p, in) && blocks.size() < MAX_BLOCKS) {
			vec3 pos, vel;
			throw_start(p, pos, vel);
			spawn_block(p, in.throw_tick, pos, vel);
		}
	}

	void transmit(const Outgoing &o) {
		wire->send(o.to, o.bytes.data(), o.bytes.size());
	}

	void broadcast_reliable(const uint8_t *data, size_t size) {
		for (auto &kv : peers) {
			server.queue_reliable(kv.first, data, size);
		}
	}

	uint8_t free_color() const {
		for (uint8_t c = 0; c < MAX_PLAYERS; c++) {
			bool taken = false;
			for (auto &kv : players) {
				taken |= kv.second.color == c;
			}
			if (!taken) {
				return c;
			}
		}
		return 0;
	}

	uint32_t add_player(int32_t peer) {
		const uint32_t id = next_net_id++;
		SPlayer &p = players[id];
		p.net_id = id;
		p.peer = peer;
		p.color = free_color();
		p.body = make_player_body();
		p.body->set_user_data((void *)(uintptr_t)id);
		p.body->set_collision_filter([this, id](Solid *other) { return !young_block_of(other, id); });
		sim.add_solid(p.body);
		respawn(p);

		GServer::SpawnRecord rec;
		rec.data = { KIND_PLAYER, p.color, (uint8_t)(peer == 0) };
		server.add_spawn(id, rec);
		server.add_entity(id, id, player_layout());
		return id;
	}

	void remove_player(uint32_t id) {
		sim.remove_solid(players[id].body);
		players.erase(id);
		server.remove_entity(id);
		server.despawn(id);
	}

	bool young_block_of(Solid *s, uint32_t owner) {
		auto it = blocks.find((uint32_t)(uintptr_t)s->get_user_data());
		return it != blocks.end() && it->second.owner == owner && it->second.age < OWNER_GRACE_TICKS;
	}

	void respawn(SPlayer &p) {
		const std::vector<vec3> &spots = spawn_spots();
		// The spot farthest from everyone else.
		vec3 best = spots[0];
		float best_d = -1.0f;
		for (const vec3 &s : spots) {
			float d = 1e9f;
			for (auto &kv : players) {
				if (kv.first != p.net_id && kv.second.alive) {
					d = std::min(d, hop::length(kv.second.body->get_position() - s));
				}
			}
			if (d > best_d) {
				best_d = d;
				best = s;
			}
		}
		p.body->set_position(best);
		p.body->set_velocity(vec3());
		p.yaw = atan2f(-best.y, -best.x);
		p.pitch = 0.0f;
		p.alive = true;
		p.ammo = MAX_AMMO;
		p.ammo_ticks = 0;
		p.cooldown = 0;
		p.last_hitter = 0;
	}

	void spawn_block(const SPlayer &owner, uint16_t throw_tick, const vec3 &pos, const vec3 &vel) {
		const uint32_t id = next_net_id++;
		SBlock &b = blocks[id];
		b.net_id = id;
		b.owner = owner.net_id;
		b.body = make_block_body();
		b.body->set_position(pos);
		b.body->set_velocity(vel);
		b.body->set_angular_velocity(vec3(vel.y, -vel.x, 0.0f) * 0.4f); // tumbling forward
		hook_block(b);
		b.body->activate();

		GServer::SpawnRecord rec;
		rec.data = { KIND_BLOCK, owner.color, 0, 0, 0, 0, (uint8_t)throw_tick, (uint8_t)(throw_tick >> 8) };
		put_u32_le(&rec.data[2], owner.net_id);
		server.add_spawn(id, rec);
		server.add_entity(id, id, block_layout());
	}

	void hook_block(SBlock &b) {
		const uint32_t id = b.net_id;
		b.body->set_user_data((void *)(uintptr_t)id);
		b.body->set_collision_callback([this, id](const hop::collision<float> &c) { on_block_hit(id, c); });
		b.body->set_collision_filter([this, id](Solid *other) {
			const SBlock &b = blocks[id];
			return !(b.age < OWNER_GRACE_TICKS && (uint32_t)(uintptr_t)other->get_user_data() == b.owner && b.owner);
		});
		sim.add_solid(b.body);
	}

	void on_block_hit(uint32_t block_id, const hop::collision<float> &c) {
		SBlock &b = blocks[block_id];
		if (!c.collidee) {
			return;
		}
		auto pit = players.find((uint32_t)(uintptr_t)c.collidee->get_user_data());
		const vec3 v = b.prev_vel;
		const float speed = hop::length(v);
		if (pit == players.end() || speed < HIT_SPEED || !pit->second.alive) {
			return;
		}
		SPlayer &p = pit->second;
		vec3 push(v.x, v.y, 0.0f);
		const float h = hop::length(push);
		push = h > 0.01f ? push * (KNOCKBACK * fminf(speed / THROW_SPEED, 1.2f) / h) : vec3();
		p.body->set_velocity(p.body->get_velocity() + push + vec3(0.0f, 0.0f, KNOCK_UP));
		b.prev_vel = vec3(); // one shove per throw
		if (b.owner && b.owner != p.net_id) {
			p.last_hitter = b.owner;
			p.last_hit_ms = now_ms();
		}
	}

	void add_arena_blocks() {
		uint32_t id = ARENA_BLOCK_ID;
		for (const vec3 &spot : arena_block_spots()) {
			SBlock &b = blocks[id];
			b.net_id = id;
			b.arena = true;
			b.home = spot;
			b.age = OWNER_GRACE_TICKS;
			b.body = make_block_body();
			b.body->set_position(spot);
			hook_block(b);
			server.add_entity(id, id, block_layout()); // no spawn: the client has these from the start
			id++;
		}
	}

	void kill(SPlayer &p, uint32_t now) {
		p.alive = false;
		p.respawn_at = now + (uint32_t)(RESPAWN_S * 1000);
		p.body->set_position(vec3(0.0f, 0.0f, -100.0f));
		p.body->set_velocity(vec3());
		uint32_t shover = 0;
		if (p.last_hitter && now - p.last_hit_ms < HIT_CREDIT_MS && players.count(p.last_hitter)) {
			shover = p.last_hitter;
			players[shover].score++;
		}
		uint8_t msg[9] = { MSG_KILL };
		put_u32_le(msg + 1, shover);
		put_u32_le(msg + 5, p.net_id);
		broadcast_reliable(msg, sizeof(msg));
	}

	// A bot is a player whose commands the server writes: run at the nearest player, keep away from
	// the edge, and lob blocks with a ballistic lead.
	Input bot_input(SPlayer &b, uint32_t now) {
		Input in;
		in.yaw = b.yaw;
		in.pitch = b.pitch;
		const vec3 me = b.body->get_position();
		const SPlayer *target = nullptr;
		float best = 1e9f;
		for (auto &kv : players) {
			const SPlayer &o = kv.second;
			if (o.net_id != b.net_id && o.alive) {
				const float d = hop::length(o.body->get_position() - me);
				if (d < best) {
					best = d;
					target = &o;
				}
			}
		}
		if (now >= b.bot_think_ms) {
			b.bot_strafe = (rand() & 1) ? 1.0f : -1.0f;
			b.bot_think_ms = now + 600 + rand() % 900;
		}
		if (target) {
			const vec3 tp = target->body->get_position();
			const vec3 tv = target->body->get_velocity();
			const float g = GRAVITY;
			const float v = THROW_SPEED;
			// Lead: where the target will be when a block gets there, refined twice.
			vec3 aim = tp;
			float pitch = 0.0f;
			for (int k = 0; k < 2; k++) {
				const vec3 d = aim - me;
				const float dh = sqrtf(d.x * d.x + d.y * d.y);
				const float dz = d.z + 0.6f;
				const float disc = v * v * v * v - g * (g * dh * dh + 2.0f * dz * v * v);
				pitch = disc > 0.0f ? atanf((v * v - sqrtf(disc)) / (g * dh)) : 0.6f;
				const float t = dh / (v * cosf(pitch));
				aim = tp + tv * t;
			}
			const vec3 d = aim - me;
			in.yaw = atan2f(d.y, d.x) + ((rand() % 100) - 50) * 0.0012f;
			in.pitch = pitch + ((rand() % 100) - 50) * 0.001f;
			if (best > 7.0f) {
				in.buttons |= B_FORWARD;
			} else if (best < 4.0f) {
				in.buttons |= B_BACK;
			}
			in.buttons |= b.bot_strafe > 0 ? B_RIGHT : B_LEFT;
			if (best < 16.0f && rand() % 40 == 0) {
				in.buttons |= B_THROW;
			}
		}
		// Too close to the edge: head for the middle instead.
		if (fabsf(me.x) > ARENA_HALF - 2.0f || fabsf(me.y) > ARENA_HALF - 2.0f) {
			in.yaw = atan2f(-me.y, -me.x);
			in.buttons = (uint8_t)((in.buttons & B_THROW) | B_FORWARD);
		}
		if (rand() % 120 == 0) {
			in.buttons |= B_JUMP;
		}
		return quantize(in);
	}

	void game_tick(uint32_t now) {
		for (auto &kv : players) {
			SPlayer &p = kv.second;
			if (p.peer == 0) {
				run_move(p, bot_input(p, now));
			}
		}
		for (auto &kv : blocks) {
			kv.second.prev_vel = kv.second.body->get_velocity();
			kv.second.age++;
		}
		sim.update(TICK, U_BLOCK);

		std::vector<uint32_t> gone;
		for (auto &kv : blocks) {
			SBlock &b = kv.second;
			const bool fell = b.body->get_position().z < FALL_Z;
			if (b.arena) {
				if (fell) { // back home, as good as new
					b.body->set_position(b.home);
					b.body->set_velocity(vec3());
					b.body->set_angular_velocity(vec3());
					b.body->set_orientation_from_quat(quat());
				}
			} else if (fell || b.age > BLOCK_LIFE_TICKS) {
				gone.push_back(kv.first);
			}
		}
		for (uint32_t id : gone) {
			sim.remove_solid(blocks[id].body);
			blocks.erase(id);
			server.remove_entity(id);
			server.despawn(id);
		}

		for (auto &kv : players) {
			SPlayer &p = kv.second;
			if (p.alive && p.body->get_position().z < FALL_Z) {
				kill(p, now);
			} else if (!p.alive && (int32_t)(now - p.respawn_at) >= 0) {
				respawn(p);
			}
		}

		int bots = 0;
		uint32_t a_bot = 0;
		for (auto &kv : players) {
			if (kv.second.peer == 0) {
				bots++;
				a_bot = kv.first;
			}
		}
		if (bots < bots_wanted && (int)players.size() < MAX_PLAYERS) {
			add_player(0);
		} else if (bots > 0 && (bots > bots_wanted || (int)players.size() > MAX_PLAYERS)) {
			remove_player(a_bot);
		}
	}

	void receive(uint32_t now) {
		std::vector<uint8_t> buf;
		std::string from;
		while (wire->receive(buf, from)) {
			if (buf.empty()) {
				continue;
			}
			auto it = by_addr.find(from);
			if (buf[0] == 'H' && it == by_addr.end()) {
				join(from, now);
			} else if (buf[0] == 'D' && it != by_addr.end()) {
				drop_peer(it->second, "left");
			} else if (buf[0] == 'U' && it != by_addr.end()) {
				const int32_t peer = it->second;
				peers[peer].last_heard = now;
				server.receive_uplink(peer, buf.data() + 1, buf.size() - 1, *this, now_ms());
			}
		}
	}

	void join(const std::string &from, uint32_t now) {
		int humans = 0;
		for (auto &kv : players) {
			humans += kv.second.peer != 0;
		}
		if (humans >= MAX_PLAYERS) {
			return; // full: no answer, and the client keeps knocking
		}
		const int32_t peer = next_peer++;
		Peer &p = peers[peer];
		p.addr = from;
		p.last_heard = now;
		by_addr[from] = peer;
		server.add_peer(peer);
		// Make room by retiring a bot if the arena is full.
		if ((int)players.size() >= MAX_PLAYERS) {
			for (auto &kv : players) {
				if (kv.second.peer == 0) {
					remove_player(kv.first);
					break;
				}
			}
		}
		p.player = add_player(peer);
		uint8_t msg[5] = { MSG_YOU };
		put_u32_le(msg + 1, p.player);
		server.queue_reliable(peer, msg, sizeof(msg));
		if (verbose) {
			printf("[server] peer %d joined from %s\n", peer, from.c_str());
		}
	}

	void drop_peer(int32_t peer, const char *why) {
		if (verbose) {
			printf("[server] peer %d %s\n", peer, why);
		}
		if (players.count(peers[peer].player)) {
			remove_player(peers[peer].player);
		}
		by_addr.erase(peers[peer].addr);
		peers.erase(peer);
		server.remove_peer(peer);
		netsim.forget_link(peer);
	}

	void start(std::unique_ptr<Wire> w, int bots) {
		wire = std::move(w);
		bots_wanted = bots;
		build_arena(sim);
		server.set_packet_overhead(wire->overhead());
		server.set_snapshot_rate(TICK_HZ);
		add_arena_blocks();
		start_ms = next_tick = now_ms();
		next_report = next_tick + 1000;
		if (verbose) {
			printf("[server] up with %d bots\n", bots);
		}
	}

	// Everything due by now. Returns when the next tick is.
	uint32_t poll() {
		uint32_t now = now_ms();
		receive(now);
		netsim.pump(now, [&](Outgoing &o) { transmit(o); });
		int caught_up = 0;
		while ((int32_t)(now - next_tick) >= 0 && caught_up++ < 5) {
			game_tick(now);
			for (auto it = peers.begin(); it != peers.end();) {
				const int32_t id = it->first;
				++it;
				if (now - peers[id].last_heard > TIMEOUT_MS) {
					drop_peer(id, "timed out");
				}
			}
			server.tick(now, *this);
			next_tick = start_ms + (uint32_t)((uint64_t)++ticks * 1000 / TICK_HZ);
		}
		if ((int32_t)(now - next_tick) > 100) { // far behind (a debugger, a stall): don't spin to catch up
			start_ms = next_tick = now;
			ticks = 0;
		}
		if (verbose && (int32_t)(now - next_report) >= 0) {
			printf("[server] %zu peers, %zu players, %zu blocks, %.1f KB/s out\n", peers.size(), players.size(),
					blocks.size(), bytes_out / 1024.0);
			for (auto &kv : peers) {
				const SPlayer *p = players.count(kv.second.player) ? &players[kv.second.player] : nullptr;
				const GServer::PeerStats st = server.peer_stats(kv.first);
				printf("[server]   peer %d: %u commands/s, update rate %u, %u given up on\n", kv.first,
						p ? p->commands : 0, st.update_rate, st.commands_skipped);
			}
			for (auto &kv : players) {
				kv.second.commands = 0;
			}
			bytes_out = 0;
			next_report += 1000;
		}
		return next_tick;
	}

	int run() {
		for (;;) {
			const uint32_t next = poll();
			wire->wait((int)(int32_t)(next - now_ms()));
		}
	}
};

} // namespace shove
