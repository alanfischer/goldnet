#pragma once
//
// The shove client, without the window: the session, goldnet's snapshot stream and uplink, and
// the two halves of hiding latency.
//
// Prediction. Your own player runs ahead of the server on the same move code. Every command goes
// into a history as it's made; when a snapshot lands, the player is reset to the server's state,
// which is the state after the command the header acks, and the commands after that one are run
// again. What the replay moved the player by is kept as an error offset that decays, so a
// correction slides instead of snapping. Your throws are predicted too: the block appears and
// flies at once, and when the server's spawn for it arrives (named by the command's tick) the
// prediction hands over to it.
//
// Interpolation. Everything else is drawn a little in the past, between the two snapshots that
// bracket the render time, so it moves smoothly whatever the snapshot rate and however the
// packets bunch up.
//
#include "net.h"
#include "server.h"
#include "world.h"

#include "goldnet/client.h"
#include "goldnet/netsim.h"
#include "goldnet/uplink.h"

#include <deque>
#include <map>
#include <string>

namespace shove {

inline quat yaw_quat(float yaw) { return quat(0.0f, 0.0f, sinf(yaw * 0.5f), cosf(yaw * 0.5f)); }

inline quat normalized(const quat &q) {
	const float l = sqrtf(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
	return l > 0.0f ? quat(q.x / l, q.y / l, q.z / l, q.w / l) : quat();
}

inline quat nlerp(const quat &a, const quat &b, float t) {
	const float d = a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
	const float s = d < 0.0f ? -1.0f : 1.0f; // the short way round
	return normalized(quat(a.x + (b.x * s - a.x) * t, a.y + (b.y * s - a.y) * t, a.z + (b.z * s - a.z) * t,
			a.w + (b.w * s - a.w) * t));
}

inline vec3 lerp(const vec3 &a, const vec3 &b, float t) { return a + (b - a) * t; }

// Snapshots of one entity, by server time.
struct Interp {
	struct Sample {
		uint32_t t;
		vec3 pos;
		quat rot;
	};
	std::deque<Sample> samples;

	void push(uint32_t t, const vec3 &pos, const quat &rot) {
		if (!samples.empty() && (int32_t)(t - samples.back().t) <= 0) {
			return;
		}
		samples.push_back({ t, pos, rot });
		while (samples.size() > 40) {
			samples.pop_front();
		}
	}

	bool sample(double t, vec3 &pos, quat &rot) const {
		if (samples.empty()) {
			return false;
		}
		const Sample &first = samples.front();
		if (t <= (double)first.t || samples.size() == 1) {
			pos = first.pos;
			rot = first.rot;
			return true;
		}
		for (size_t i = 1; i < samples.size(); i++) {
			const Sample &a = samples[i - 1];
			const Sample &b = samples[i];
			if (t <= (double)b.t) {
				if (hop::length(b.pos - a.pos) > 4.0f) { // a teleport: don't draw the trip
					pos = b.pos;
					rot = b.rot;
					return true;
				}
				const float f = (float)((t - a.t) / (double)(b.t - a.t));
				pos = lerp(a.pos, b.pos, f);
				rot = nlerp(a.rot, b.rot, f);
				return true;
			}
		}
		pos = samples.back().pos; // past the newest: hold, don't guess
		rot = samples.back().rot;
		return true;
	}
};

struct ClientCore {
	struct Ent {
		uint8_t kind = 0;
		uint8_t color = 0;
		bool bot = false;
		uint32_t owner = 0;     // blocks: the thrower
		uint16_t throw_tick = 0; // blocks: the command that threw it
		std::vector<Value> vals;
		bool has_state = false;
		Interp interp;
		SolidPtr proxy; // stands in for it in the prediction world
		// A block that took over from a prediction: drawn offset toward where the prediction was,
		// and the offset decays.
		SolidPtr adopt_from;
		vec3 offset;
	};
	struct Pending {
		uint16_t seq;
		Input in;
	};
	struct Predicted {
		SolidPtr body;
		uint32_t born_ms;
	};
	struct Kill {
		uint32_t shover, victim;
		uint32_t at_ms;
	};

	int fd = -1;
	sockaddr_in server_addr = {};
	Client<Value> client;
	NetSim<std::vector<uint8_t>> up_sim, down_sim; // both legs of this client's link

	Sim sim; // the prediction world: the arena, your player, stand-ins for everything else
	std::map<uint32_t, Ent> ents;
	uint32_t my_id = 0;
	Player me;
	bool me_ready = false;
	vec3 me_prev, me_cur; // the last two ticks, for drawing between them
	vec3 me_error;        // what corrections moved the drawn player by, decaying
	std::deque<Pending> history;
	uint16_t acked = 0; // the newest command the server has run
	std::map<uint16_t, Predicted> predicted; // by the throwing command's tick
	uint16_t tick_count = 0;
	std::deque<Kill> kills;

	bool predict = true;
	bool interpolate = true;
	uint32_t interp_ms = 100;

	// The clock: local ms minus server ms, tracked toward the smallest seen.
	double clock_offset = 0.0;
	bool clock_set = false;
	uint32_t server_ms = 0;
	bool fresh = false;

	uint32_t last_heard = 0;
	uint32_t last_poll = 0;
	uint32_t next_hello = 0;
	bool connected = false;

	// Per-second stats.
	struct Stats {
		size_t bytes_in = 0, bytes_out = 0;
		int snapshots = 0;
		float error_sum = 0.0f, error_max = 0.0f;
		int corrections = 0, replayed = 0;
	};
	// Over the whole session.
	int throws_predicted = 0, throws_adopted = 0, throws_unclaimed = 0;
	Stats cur, last;
	uint32_t next_stats = 0;

	// --- goldnet's host interface (Client::apply) ---

	void protocol_mismatch() { fprintf(stderr, "[client] the server speaks a different version\n"); }
	void server_time(uint32_t ms) {
		server_ms = ms;
		fresh = true;
		const double off = (double)now_ms() - (double)ms;
		if (!clock_set || off < clock_offset) {
			clock_offset = off; // a faster packet: that's closer to the true offset
			clock_set = true;
		} else {
			clock_offset += (off - clock_offset) * 0.01; // drift and route changes, slowly
		}
	}
	void message(const uint8_t *data, size_t size, bool) {
		if (size >= 5 && data[0] == MSG_YOU) {
			my_id = get_u32_le(data + 1);
			auto it = ents.find(my_id);
			if (it != ents.end() && it->second.proxy) {
				sim.remove_solid(it->second.proxy);
				it->second.proxy.reset();
			}
		} else if (size >= 9 && data[0] == MSG_KILL) {
			kills.push_back({ get_u32_le(data + 1), get_u32_le(data + 5), now_ms() });
			while (kills.size() > 5) {
				kills.pop_front();
			}
		}
	}
	bool spawn(uint32_t net_id, uint32_t, const uint8_t *data, size_t size) {
		if (size < 1) {
			return true;
		}
		Ent &e = ents[net_id];
		e = Ent();
		e.kind = data[0];
		if (e.kind == KIND_PLAYER && size >= 3) {
			e.color = data[1];
			e.bot = data[2] != 0;
			e.vals.resize(PLAYER_SLOTS);
			if (net_id != my_id) {
				e.proxy = make_proxy(make_player_body());
			}
		} else if (e.kind == KIND_BLOCK && size >= 8) {
			e.color = data[1];
			e.owner = get_u32_le(data + 2);
			e.throw_tick = (uint16_t)(data[6] | (data[7] << 8));
			e.vals.resize(BLOCK_SLOTS);
			e.proxy = make_proxy(make_block_body());
			e.proxy->set_user_data((void *)(uintptr_t)net_id);
			auto it = predicted.find(e.throw_tick);
			if (e.owner == my_id && it != predicted.end()) {
				e.adopt_from = it->second.body;
				predicted.erase(it);
				throws_adopted++;
			}
		}
		return true;
	}
	void despawn(uint32_t net_id) {
		auto it = ents.find(net_id);
		if (it == ents.end()) {
			return;
		}
		if (it->second.proxy) {
			sim.remove_solid(it->second.proxy);
		}
		if (it->second.adopt_from) {
			sim.remove_solid(it->second.adopt_from);
		}
		ents.erase(it);
	}
	void relevance_lost(uint32_t) {}
	bool registered(uint32_t net_id) { return ents.count(net_id) > 0; }
	uint32_t cur_id = 0;
	int begin_entity(uint32_t net_id, uint32_t, bool &) {
		auto it = ents.find(net_id);
		if (it == ents.end()) {
			return -1;
		}
		cur_id = net_id;
		return (int)it->second.vals.size();
	}
	bool set_slot(int slot, const Value &v) {
		ents[cur_id].vals[slot] = v;
		return true;
	}
	void end_entity(bool applied) {
		if (applied) {
			ents[cur_id].has_state = true;
		}
	}
	Value decode(Reader &r, uint32_t) { return decode_value(r); }

	// --- the session ---

	SolidPtr make_proxy(SolidPtr s) {
		s->set_infinite_mass();
		s->set_scope(U_NEVER);
		s->set_position(vec3(0.0f, 0.0f, -100.0f));
		sim.add_solid(s);
		return s;
	}

	void add_arena_blocks() {
		uint32_t id = ARENA_BLOCK_ID;
		for (size_t i = 0; i < arena_block_spots().size(); i++) {
			Ent &e = ents[id++];
			e.kind = KIND_BLOCK;
			e.vals.resize(BLOCK_SLOTS);
			e.proxy = make_proxy(make_block_body());
		}
	}

	void new_session() {
		client.reset();
		client.set_update_rate(30);
		client.set_rate(40000);
		acked = 0;
		for (auto &kv : ents) {
			if (kv.second.proxy) {
				sim.remove_solid(kv.second.proxy);
			}
			if (kv.second.adopt_from) {
				sim.remove_solid(kv.second.adopt_from);
			}
		}
		ents.clear();
		for (auto &kv : predicted) {
			sim.remove_solid(kv.second.body);
		}
		predicted.clear();
		history.clear();
		my_id = 0;
		me_ready = false;
		me.alive = false;
		clock_set = false;
		add_arena_blocks();
		const uint8_t hello = 'H';
		sendto(fd, &hello, 1, 0, (sockaddr *)&server_addr, sizeof(server_addr));
	}

	bool start(const char *host, uint16_t port) {
		if (!resolve(host, port, server_addr)) {
			fprintf(stderr, "[client] can't resolve %s\n", host);
			return false;
		}
		fd = open_socket(0);
		build_arena(sim);
		me.body = make_player_body();
		me.body->set_position(vec3(0.0f, 0.0f, -100.0f));
		me.body->set_collision_filter([this](Solid *other) { return !is_young_own_block(other); });
		sim.add_solid(me.body);
		new_session();
		next_hello = now_ms() + 500;
		next_stats = now_ms() + 1000;
		return true;
	}

	void leave() {
		const uint8_t bye = 'D';
		sendto(fd, &bye, 1, 0, (sockaddr *)&server_addr, sizeof(server_addr));
	}

	// A block of ours young enough to pass through us, as it does on the server.
	bool young_throw(uint16_t throw_tick) const {
		return (uint16_t)(tick_count - throw_tick) < OWNER_GRACE_TICKS + history.size();
	}
	bool is_young_own_block(Solid *s) {
		for (auto &kv : predicted) {
			if (kv.second.body.get() == s) {
				return young_throw(kv.first);
			}
		}
		auto it = ents.find((uint32_t)(uintptr_t)s->get_user_data());
		return it != ents.end() && it->second.kind == KIND_BLOCK && it->second.owner == my_id && my_id &&
				young_throw(it->second.throw_tick);
	}

	void transmit(const std::vector<uint8_t> &pkt) {
		cur.bytes_out += pkt.size();
		sendto(fd, pkt.data(), pkt.size(), 0, (sockaddr *)&server_addr, sizeof(server_addr));
	}

	// A packet through one leg of the simulated link: dropped, delayed, or on its way now.
	template <class F>
	void through(NetSim<std::vector<uint8_t>> &s, std::vector<uint8_t> &&pkt, F &&now_fn) {
		if (s.loss_roll()) {
			return;
		}
		const uint32_t now = now_ms();
		const int delay = s.schedule(now, 0, pkt.size(), true);
		if (delay == 0) {
			now_fn(pkt);
		} else if (delay > 0) {
			s.enqueue(now, 0, delay, std::move(pkt));
		}
	}

	void handle_snapshot(const std::vector<uint8_t> &pkt) {
		if (!connected) {
			printf("[client] connected\n");
		}
		connected = true;
		last_heard = now_ms();
		cur.bytes_in += pkt.size();
		cur.snapshots++;
		fresh = false;
		const auto info = client.apply(pkt.data(), pkt.size(), *this, now_ms());
		if (info.command_ack != 0) {
			acked = info.command_ack;
		}
		if (fresh && info.committed) {
			on_snapshot();
		}
	}

	// Network in and out. Call every frame.
	void poll() {
		uint8_t buf[1500];
		ssize_t n;
		while ((n = recvfrom(fd, buf, sizeof(buf), MSG_DONTWAIT, nullptr, nullptr)) > 0) {
			through(down_sim, std::vector<uint8_t>(buf, buf + n), [&](std::vector<uint8_t> &p) { handle_snapshot(p); });
		}
		const uint32_t now = now_ms();
		down_sim.pump(now, [&](std::vector<uint8_t> &p) { handle_snapshot(p); });
		up_sim.pump(now, [&](std::vector<uint8_t> &p) { transmit(p); });
		if (!connected && (int32_t)(now - next_hello) >= 0) {
			new_session(); // no server yet: keep knocking
			next_hello = now + 500;
		} else if (connected && now - last_heard > TIMEOUT_MS) {
			printf("[client] the server went quiet; starting over\n");
			connected = false;
		}
		if ((int32_t)(now - next_stats) >= 0) {
			last = cur;
			cur = Stats();
			next_stats += 1000;
			if ((int32_t)(now - next_stats) > 0) {
				next_stats = now + 1000;
			}
		}
		const float decay = expf(-(float)(now - last_poll) / 100.0f); // toward no correction offset
		last_poll = now;
		me_error = me_error * decay;
		for (auto &kv : ents) {
			kv.second.offset = kv.second.offset * decay;
		}
	}

	static vec3 vals_pos(const std::vector<Value> &v) { return vec3(as_float(v[0]), as_float(v[1]), as_float(v[2])); }

	static quat vals_rot(const Ent &e) {
		if (e.kind == KIND_PLAYER) {
			return yaw_quat(as_float(e.vals[PS_YAW]));
		}
		return normalized(quat(as_float(e.vals[BS_QX]), as_float(e.vals[BS_QY]), as_float(e.vals[BS_QZ]),
				as_float(e.vals[BS_QW])));
	}

	static bool alive(const Ent &e) { return e.kind != KIND_PLAYER || as_int(e.vals[PS_ALIVE]) != 0; }

	void on_snapshot() {
		for (auto &kv : ents) {
			Ent &e = kv.second;
			if (!e.has_state) {
				continue;
			}
			const vec3 pos = vals_pos(e.vals);
			const quat rot = vals_rot(e);
			e.interp.push(server_ms, pos, rot);
			if (e.proxy) { // the prediction collides with the newest state there is
				e.proxy->set_position(alive(e) ? pos : vec3(0.0f, 0.0f, -100.0f));
				if (e.kind == KIND_BLOCK) {
					e.proxy->set_orientation_from_quat(rot);
				}
			}
		}
		reconcile();
	}

	void reconcile() {
		auto it = ents.find(my_id);
		if (it == ents.end() || !it->second.has_state) {
			return;
		}
		const std::vector<Value> &v = it->second.vals;
		const vec3 before = me.body->get_position();
		me.alive = as_int(v[PS_ALIVE]) != 0;
		me.ammo = (int)as_int(v[PS_AMMO]);
		me.ammo_ticks = (int)as_int(v[PS_AMMO_TICKS]);
		me.cooldown = (int)as_int(v[PS_COOLDOWN]);
		me.body->set_position(vals_pos(v));
		me.body->set_velocity(vec3(as_float(v[PS_VX]), as_float(v[PS_VY]), as_float(v[PS_VZ])));
		while (!history.empty() && seq_le(history.front().seq, acked)) {
			history.pop_front();
		}
		if (predict && me.alive) {
			for (const Pending &p : history) {
				move_player(sim, me, p.in);
				cur.replayed++;
			}
		}
		const vec3 after = me.body->get_position();
		const vec3 moved = before - after;
		const float err = hop::length(moved);
		if (!me_ready || err > 3.0f || !predict) {
			me_ready = true;
			me_prev = me_cur = after;
			me_error = vec3();
			return;
		}
		if (err > 0.001f) {
			cur.corrections++;
			cur.error_sum += err;
			cur.error_max = fmaxf(cur.error_max, err);
		}
		me_error = me_error + moved;
		me_prev = me_prev - moved;
		me_cur = after;
	}

	// One command: run it here, queue it for the server, send if due.
	void tick(Input in) {
		const uint32_t now = now_ms();
		in.throw_tick = tick_count++;
		in = quantize(in);
		uint8_t bytes[INPUT_BYTES];
		write_input(in, bytes);
		const uint16_t seq = client.uplink().push(CMD_STREAM, bytes, sizeof(bytes), now);
		history.push_back({ seq, in });
		while (history.size() > COMMAND_RING) {
			history.pop_front();
		}

		if (predict && me_ready) {
			if (move_player(sim, me, in)) {
				vec3 pos, vel;
				throw_start(me, pos, vel);
				spawn_prediction(in.throw_tick, pos, vel);
			}
			me_prev = me_cur;
			me_cur = me.body->get_position();
		} else {
			me.yaw = in.yaw;
			me.pitch = in.pitch;
		}
		sim.update(TICK, U_BLOCK); // the predicted blocks
		for (auto it = predicted.begin(); it != predicted.end();) {
			if (now - it->second.born_ms > 2000) { // the server never threw it
				throws_unclaimed++;
				sim.remove_solid(it->second.body);
				it = predicted.erase(it);
			} else {
				++it;
			}
		}

		if (connected && client.uplink_due(now)) {
			Writer w;
			w.put_u8('U');
			client.write_uplink(now, w);
			through(up_sim, std::vector<uint8_t>(w.data(), w.data() + w.size()),
					[&](std::vector<uint8_t> &p) { transmit(p); });
		}
	}

	void spawn_prediction(uint16_t throw_tick, const vec3 &pos, const vec3 &vel) {
		throws_predicted++;
		Predicted &p = predicted[throw_tick];
		p.body = make_block_body();
		p.body->set_position(pos);
		p.body->set_velocity(vel);
		p.body->set_angular_velocity(vec3(vel.y, -vel.x, 0.0f) * 0.4f);
		p.body->set_collision_filter(
				[this, throw_tick](Solid *other) { return !(other == me.body.get() && young_throw(throw_tick)); });
		p.born_ms = now_ms();
		p.body->activate();
		sim.add_solid(p.body);
	}

	// --- what the view draws ---

	double render_time() const { return (double)now_ms() - clock_offset - (interpolate ? interp_ms : 0); }

	// Where to draw an entity, or false if there's nothing to draw yet. p_alpha: how far the
	// frame is between the last two prediction ticks.
	bool placement(uint32_t net_id, Ent &e, float alpha, vec3 &pos, quat &rot) {
		if (net_id == my_id && predict && me_ready) {
			pos = lerp(me_prev, me_cur, alpha) + me_error;
			rot = yaw_quat(me.yaw);
			return me.alive;
		}
		if (!e.has_state || !alive(e)) {
			return false;
		}
		if (interpolate) {
			if (!e.interp.sample(render_time(), pos, rot)) {
				return false;
			}
		} else {
			pos = vals_pos(e.vals);
			rot = vals_rot(e);
		}
		if (e.adopt_from) { // hand a predicted throw over without a jump
			e.offset = e.adopt_from->get_position() - pos;
			sim.remove_solid(e.adopt_from);
			e.adopt_from.reset();
		}
		pos = pos + e.offset;
		return true;
	}

	float ping_ms() const { return (float)client.uplink().rtt_ms(); }
};

} // namespace shove
