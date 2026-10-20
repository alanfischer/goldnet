#pragma once
//
// The game rules both ends run: the arena, a player's move for one command, and the blocks. The
// server runs them for real; the client runs the same move for its own player ahead of the server
// (prediction) and replays it from each snapshot (reconciliation). z is up, units are metres.
//
#include <hop/hop.h>

#include <cmath>
#include <cstdint>
#include <memory>
#include <vector>

namespace shove {

typedef hop::vec3<float> vec3;
typedef hop::quat<float> quat;
typedef hop::solid<float> Solid;
typedef std::shared_ptr<Solid> SolidPtr;
typedef hop::simulator<float> Sim;

static const int TICK_HZ = 60;
static const float TICK = 1.0f / TICK_HZ;

static const float GRAVITY = 20.0f;
static const float ARENA_HALF = 10.0f;
static const float FALL_Z = -8.0f; // below this you're out
static const float RESPAWN_S = 2.0f;

static const float PLAYER_HALF = 0.4f;
static const float PLAYER_HEIGHT = 1.8f;
static const float EYE_HEIGHT = 1.5f;
static const float MAX_SPEED = 7.0f;
static const float ACCEL = 10.0f;
static const float AIR_ACCEL = 2.0f;
static const float FRICTION = 6.0f;
static const float STOP_SPEED = 2.0f;
static const float JUMP_SPEED = 6.5f;

static const float BLOCK_HALF = 0.3f;
static const float THROW_SPEED = 18.0f;
static const int MAX_AMMO = 3;
static const int AMMO_TICKS = 30;     // one block back every half second
static const int THROW_COOLDOWN = 12; // between throws
static const int BLOCK_LIFE_TICKS = 8 * TICK_HZ;
static const int OWNER_GRACE_TICKS = 15; // a fresh block passes through its thrower
static const float KNOCKBACK = 9.0f;
static const float KNOCK_UP = 4.0f;
static const float HIT_SPEED = 4.0f; // slower blocks just bump

// Update scopes: which solids a sim.update() steps.
static const int U_PLAYER = 1;
static const int U_BLOCK = 2;
static const int U_NEVER = 4; // the arena, and the client's stand-ins for other entities

enum Buttons : uint8_t {
	B_FORWARD = 1,
	B_BACK = 2,
	B_LEFT = 4,
	B_RIGHT = 8,
	B_JUMP = 16,
	B_THROW = 32,
};

// One command: what the player did for one tick.
struct Input {
	uint8_t buttons = 0;
	float yaw = 0.0f;
	float pitch = 0.0f;
	uint16_t throw_tick = 0; // the client's tick, so a throw can find its predicted block again
};

static const size_t INPUT_BYTES = 7;

inline void write_input(const Input &in, uint8_t *out) {
	const uint16_t yaw = (uint16_t)lroundf(fmodf(in.yaw + 4 * M_PI, 2 * M_PI) / (2 * M_PI) * 65536.0f);
	const int16_t pitch = (int16_t)lroundf(in.pitch / (M_PI / 2) * 32767.0f);
	out[0] = in.buttons;
	out[1] = (uint8_t)yaw;
	out[2] = (uint8_t)(yaw >> 8);
	out[3] = (uint8_t)pitch;
	out[4] = (uint8_t)((uint16_t)pitch >> 8);
	out[5] = (uint8_t)in.throw_tick;
	out[6] = (uint8_t)(in.throw_tick >> 8);
}

inline bool read_input(const uint8_t *data, size_t size, Input &in) {
	if (size < INPUT_BYTES) {
		return false;
	}
	in.buttons = data[0];
	in.yaw = (float)(uint16_t)(data[1] | (data[2] << 8)) / 65536.0f * 2 * M_PI;
	in.pitch = (float)(int16_t)(data[3] | (data[4] << 8)) / 32767.0f * (M_PI / 2);
	in.throw_tick = (uint16_t)(data[5] | (data[6] << 8));
	return true;
}

// The quantized form, so the client predicts with exactly what the server will run.
inline Input quantize(const Input &in) {
	uint8_t b[INPUT_BYTES];
	write_input(in, b);
	Input out;
	read_input(b, INPUT_BYTES, out);
	return out;
}

inline vec3 aim_dir(float yaw, float pitch) {
	return vec3(cosf(yaw) * cosf(pitch), sinf(yaw) * cosf(pitch), sinf(pitch));
}

// A player's state: everything a move reads and writes, so a snapshot of it is enough to replay
// from. The position and velocity live in the solid.
struct Player {
	uint32_t net_id = 0;
	SolidPtr body;
	bool alive = false;
	int ammo = MAX_AMMO;
	int ammo_ticks = 0;
	int cooldown = 0;
	float yaw = 0.0f, pitch = 0.0f;
};

struct Block {
	uint32_t net_id = 0;
	uint32_t owner = 0; // the thrower's net_id; 0 for the arena's own
	int age = 0;
	SolidPtr body;
};

inline void set_channels(Solid &s) {
	s.set_collision_scope(1);
	s.set_collide_with_scope(1);
}

inline SolidPtr add_static_box(Sim &sim, const vec3 &mins, const vec3 &maxs) {
	auto s = std::make_shared<Solid>();
	s->set_infinite_mass();
	s->set_scope(U_NEVER);
	s->set_coefficient_of_gravity(0.0f);
	s->set_coefficient_of_restitution(0.0f);
	// hop averages the two bodies' friction. The arena has none, so a player on it gets only the
	// move's own friction, which the client can replay exactly; a block brings its own.
	s->set_coefficient_of_static_friction(0.0f);
	s->set_coefficient_of_dynamic_friction(0.0f);
	s->add_shape(std::make_shared<hop::shape<float>>(hop::aa_box<float>(mins, maxs)));
	set_channels(*s);
	sim.add_solid(s);
	return s;
}

struct Box {
	vec3 mins, maxs;
};

// The arena: a floating platform with a dais in the middle and four crates for cover. No walls,
// which is the point.
inline std::vector<Box> arena_boxes() {
	const float h = ARENA_HALF;
	return {
		{ vec3(-h, -h, -1.0f), vec3(h, h, 0.0f) },
		{ vec3(-2.5f, -2.5f, 0.0f), vec3(2.5f, 2.5f, 0.5f) },
		{ vec3(-7.0f, -7.0f, 0.0f), vec3(-5.6f, -5.6f, 1.2f) },
		{ vec3(5.6f, -7.0f, 0.0f), vec3(7.0f, -5.6f, 1.2f) },
		{ vec3(-7.0f, 5.6f, 0.0f), vec3(-5.6f, 7.0f, 1.2f) },
		{ vec3(5.6f, 5.6f, 0.0f), vec3(7.0f, 7.0f, 1.2f) },
	};
}

inline void build_arena(Sim &sim) {
	sim.set_gravity(vec3(0.0f, 0.0f, -GRAVITY));
	sim.set_default_contact_mode(hop::contact_mode::sweep_slide);
	for (const Box &b : arena_boxes()) {
		add_static_box(sim, b.mins, b.maxs);
	}
}

// Where the arena's own loose blocks sit: map-static entities, which both ends know about
// without a spawn.
inline std::vector<vec3> arena_block_spots() {
	std::vector<vec3> spots;
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			spots.push_back(vec3(-0.7f + i * 0.7f, -0.7f + j * 0.7f, 0.5f + BLOCK_HALF));
		}
	}
	spots.push_back(vec3(-6.3f, -6.3f, 1.2f + BLOCK_HALF));
	spots.push_back(vec3(6.3f, -6.3f, 1.2f + BLOCK_HALF));
	spots.push_back(vec3(-6.3f, 6.3f, 1.2f + BLOCK_HALF));
	spots.push_back(vec3(6.3f, 6.3f, 1.2f + BLOCK_HALF));
	return spots;
}

inline const std::vector<vec3> &spawn_spots() {
	static const std::vector<vec3> spots = {
		vec3(-8, 0, 0.05f), vec3(8, 0, 0.05f), vec3(0, -8, 0.05f), vec3(0, 8, 0.05f),
		vec3(-4, -8, 0.05f), vec3(4, 8, 0.05f), vec3(8, -4, 0.05f), vec3(-8, 4, 0.05f),
	};
	return spots;
}

inline SolidPtr make_player_body() {
	auto s = std::make_shared<Solid>();
	s->set_mass(1.0f);
	s->set_scope(U_PLAYER);
	s->set_contact_mode(hop::contact_mode::sweep_slide);
	s->set_coefficient_of_restitution(0.0f);
	s->set_coefficient_of_static_friction(0.0f); // the move does its own friction
	s->set_coefficient_of_dynamic_friction(0.0f);
	s->set_stay_active(true);
	s->add_shape(std::make_shared<hop::shape<float>>(hop::aa_box<float>(
			vec3(-PLAYER_HALF, -PLAYER_HALF, 0.0f), vec3(PLAYER_HALF, PLAYER_HALF, PLAYER_HEIGHT))));
	set_channels(*s);
	return s;
}

inline SolidPtr make_block_body() {
	auto s = std::make_shared<Solid>();
	s->set_mass(1.0f);
	s->set_scope(U_BLOCK);
	s->set_contact_mode(hop::contact_mode::speculative);
	const float i = (2.0f * BLOCK_HALF) * (2.0f * BLOCK_HALF) / 6.0f; // m s² / 6
	s->set_inertia(vec3(i, i, i));
	s->set_coefficient_of_restitution(0.2f);
	s->set_coefficient_of_static_friction(1.2f); // 0.6 against the arena
	s->set_coefficient_of_dynamic_friction(0.9f);
	s->set_coefficient_of_angular_damping(0.2f);
	s->add_shape(std::make_shared<hop::shape<float>>(
			hop::aa_box<float>(vec3(-BLOCK_HALF, -BLOCK_HALF, -BLOCK_HALF), vec3(BLOCK_HALF, BLOCK_HALF, BLOCK_HALF))));
	set_channels(*s);
	return s;
}

// Where a throw starts and how fast it goes.
inline void throw_start(const Player &p, vec3 &pos, vec3 &vel) {
	const vec3 dir = aim_dir(p.yaw, p.pitch);
	const vec3 eye = p.body->get_position() + vec3(0.0f, 0.0f, EYE_HEIGHT - 0.2f);
	pos = eye + vec3(dir.x, dir.y, 0.0f) * (PLAYER_HALF + BLOCK_HALF + 0.15f);
	vel = dir * THROW_SPEED + vec3(0.0f, 0.0f, 2.0f) + p.body->get_velocity() * 0.5f;
}

inline bool on_ground(Sim &sim, Solid &body) {
	if (body.get_velocity().z > 0.5f) {
		return false;
	}
	hop::collision<float> c;
	hop::segment<float> probe;
	probe.set_start_dir(body.get_position(), vec3(0.0f, 0.0f, -0.05f));
	sim.trace_solid(c, &body, probe, 1);
	return c.time < 1.0f && c.normal.z > 0.7f;
}

inline void accelerate(vec3 &vel, const vec3 &wishdir, float wishspeed, float accel) {
	const float cur = vel.x * wishdir.x + vel.y * wishdir.y;
	const float add = wishspeed - cur;
	if (add <= 0.0f) {
		return;
	}
	const float a = fminf(accel * TICK * wishspeed, add);
	vel.x += a * wishdir.x;
	vel.y += a * wishdir.y;
}

// One command's worth of movement, Quake-style: friction and acceleration on the ground, a little
// control in the air, gravity from the sim. Returns true if this command throws a block, which
// the caller creates (the server for real, the client as a prediction).
inline bool move_player(Sim &sim, Player &p, const Input &in) {
	p.yaw = in.yaw;
	p.pitch = in.pitch;
	if (!p.alive) {
		return false;
	}
	Solid &body = *p.body;
	vec3 vel = body.get_velocity();
	const bool ground = on_ground(sim, body);

	const vec3 fwd(cosf(in.yaw), sinf(in.yaw), 0.0f);
	const vec3 right(sinf(in.yaw), -cosf(in.yaw), 0.0f);
	float fmove = 0.0f, smove = 0.0f;
	fmove += (in.buttons & B_FORWARD) ? 1.0f : 0.0f;
	fmove -= (in.buttons & B_BACK) ? 1.0f : 0.0f;
	smove += (in.buttons & B_RIGHT) ? 1.0f : 0.0f;
	smove -= (in.buttons & B_LEFT) ? 1.0f : 0.0f;
	vec3 wish = fwd * fmove + right * smove;
	const float wlen = sqrtf(wish.x * wish.x + wish.y * wish.y);
	const float wishspeed = wlen > 0.0f ? MAX_SPEED : 0.0f;
	if (wlen > 0.0f) {
		wish = wish * (1.0f / wlen);
	}

	if (ground) {
		const float speed = sqrtf(vel.x * vel.x + vel.y * vel.y);
		if (speed > 0.0f) {
			const float drop = fmaxf(speed, STOP_SPEED) * FRICTION * TICK;
			const float scale = fmaxf(speed - drop, 0.0f) / speed;
			vel.x *= scale;
			vel.y *= scale;
		}
		accelerate(vel, wish, wishspeed, ACCEL);
		if (in.buttons & B_JUMP) {
			vel.z = JUMP_SPEED;
		}
	} else {
		accelerate(vel, wish, wishspeed, AIR_ACCEL);
	}
	body.set_velocity(vel);
	sim.update(TICK, 0, &body);

	if (p.ammo < MAX_AMMO && ++p.ammo_ticks >= AMMO_TICKS) {
		p.ammo++;
		p.ammo_ticks = 0;
	}
	if (p.cooldown > 0) {
		p.cooldown--;
	}
	if ((in.buttons & B_THROW) && p.ammo > 0 && p.cooldown == 0) {
		if (p.ammo == MAX_AMMO) {
			p.ammo_ticks = 0;
		}
		p.ammo--;
		p.cooldown = THROW_COOLDOWN;
		return true;
	}
	return false;
}

} // namespace shove
