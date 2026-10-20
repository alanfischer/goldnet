#pragma once
//
// The window: raylib input into commands, and the world drawn from what ClientCore says.
//
#include "client.h"
#include "server.h"

#include <raylib.h>
#include <raymath.h>

#include <string>

namespace shove {

static const char *const COLOR_NAMES[MAX_PLAYERS] = { "Red", "Blue", "Green", "Yellow", "Purple", "Orange", "Cyan",
	"Pink" };
static const Color PALETTE[MAX_PLAYERS] = {
	{ 230, 70, 70, 255 }, { 70, 130, 240, 255 }, { 80, 200, 110, 255 }, { 240, 210, 70, 255 },
	{ 170, 100, 230, 255 }, { 245, 140, 50, 255 }, { 70, 210, 220, 255 }, { 240, 120, 190, 255 },
};
static const Color SKY = { 14, 16, 24, 255 };

static const char *LIT_VS = R"(#version 330
in vec3 vertexPosition;
in vec3 vertexNormal;
uniform mat4 mvp;
uniform mat4 matModel;
uniform mat4 matNormal;
out vec3 fragPos;
out vec3 fragNormal;
void main() {
	fragPos = vec3(matModel * vec4(vertexPosition, 1.0));
	fragNormal = normalize(vec3(matNormal * vec4(vertexNormal, 0.0)));
	gl_Position = mvp * vec4(vertexPosition, 1.0);
}
)";

static const char *LIT_FS = R"(#version 330
in vec3 fragPos;
in vec3 fragNormal;
uniform vec4 colDiffuse;
uniform vec3 viewPos;
uniform vec3 fogColor;
out vec4 finalColor;
void main() {
	vec3 n = normalize(fragNormal);
	float sun = max(dot(n, normalize(vec3(0.45, 0.3, 1.0))), 0.0);
	float fill = max(dot(n, normalize(vec3(-0.6, -0.4, 0.2))), 0.0);
	vec3 c = colDiffuse.rgb * (0.32 + 0.6 * sun + 0.15 * fill);
	float fog = clamp((length(fragPos - viewPos) - 25.0) / 50.0, 0.0, 1.0);
	finalColor = vec4(mix(c, fogColor, fog), colDiffuse.a);
}
)";

inline Color lighten(Color c, float t) {
	return { (unsigned char)(c.r + (255 - c.r) * t), (unsigned char)(c.g + (255 - c.g) * t),
		(unsigned char)(c.b + (255 - c.b) * t), c.a };
}

inline Vector3 rl(const vec3 &v) { return { v.x, v.y, v.z }; }

struct View {
	ClientCore &core;
	ServerApp *local; // the in-process server, for host mode
	std::string screenshot;
	int quit_after = 0;

	Shader lit = {};
	Mesh cube = {};
	Material mat = {};
	int loc_view_pos = -1;
	float yaw = 0.0f, pitch = 0.0f;
	bool mouse_captured = true;
	bool show_ghosts = false;
	bool was_alive = false;
	int link_latency = 0, link_loss = 0;
	float acc = 0.0f;
	double t = 0.0;

	View(ClientCore &c, ServerApp *l) : core(c), local(l) {
		link_latency = core.up_sim.latency_max_ms() * 2;
		link_loss = core.up_sim.loss_percent();
	}

	void set_link() {
		for (NetSim<std::vector<uint8_t>> *s : { &core.up_sim, &core.down_sim }) {
			s->set_latency_min_ms(link_latency / 2);
			s->set_latency_max_ms(link_latency / 2);
			s->set_loss_percent(link_loss);
		}
	}

	void box(const vec3 &center, const vec3 &size, const quat &q, Color c) {
		const Matrix m = MatrixMultiply(MatrixMultiply(MatrixScale(size.x, size.y, size.z),
												QuaternionToMatrix({ q.x, q.y, q.z, q.w })),
				MatrixTranslate(center.x, center.y, center.z));
		mat.maps[MATERIAL_MAP_DIFFUSE].color = c;
		DrawMesh(cube, mat, m);
	}

	// The top of the arena under (x, y), or false over the void.
	static bool ground_under(const vec3 &p, float &z) {
		bool found = false;
		for (const Box &b : arena_boxes()) {
			if (p.x >= b.mins.x && p.x <= b.maxs.x && p.y >= b.mins.y && p.y <= b.maxs.y && b.maxs.z <= p.z + 0.05f &&
					(!found || b.maxs.z > z)) {
				z = b.maxs.z;
				found = true;
			}
		}
		return found;
	}

	void shadow(const vec3 &p, float size) {
		float z;
		if (!ground_under(p, z)) {
			return;
		}
		const float h = p.z - z;
		const float s = size * fmaxf(0.4f, 1.0f - h * 0.08f);
		box(vec3(p.x, p.y, z + 0.006f), vec3(s, s, 0.01f), quat(), { 0, 0, 0, (unsigned char)(110 * fmaxf(0.2f, 1.0f - h * 0.1f)) });
	}

	Input sample_input() {
		Input in;
		in.yaw = yaw;
		in.pitch = pitch;
		in.buttons |= IsKeyDown(KEY_W) ? B_FORWARD : 0;
		in.buttons |= IsKeyDown(KEY_S) ? B_BACK : 0;
		in.buttons |= IsKeyDown(KEY_A) ? B_LEFT : 0;
		in.buttons |= IsKeyDown(KEY_D) ? B_RIGHT : 0;
		in.buttons |= IsKeyDown(KEY_SPACE) ? B_JUMP : 0;
		in.buttons |= (mouse_captured && IsMouseButtonDown(MOUSE_BUTTON_LEFT)) || IsKeyDown(KEY_E) ? B_THROW : 0;
		return in;
	}

	void handle_keys() {
		if (IsKeyPressed(KEY_TAB)) {
			mouse_captured = !mouse_captured;
			mouse_captured ? DisableCursor() : EnableCursor();
		}
		if (!mouse_captured && IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
			mouse_captured = true;
			DisableCursor();
		}
		if (IsKeyPressed(KEY_P)) {
			core.predict = !core.predict;
		}
		if (IsKeyPressed(KEY_I)) {
			core.interpolate = !core.interpolate;
		}
		if (IsKeyPressed(KEY_G)) {
			show_ghosts = !show_ghosts;
		}
		if (IsKeyPressed(KEY_RIGHT_BRACKET)) {
			link_latency = std::min(link_latency + 50, 500);
			set_link();
		}
		if (IsKeyPressed(KEY_LEFT_BRACKET)) {
			link_latency = std::max(link_latency - 50, 0);
			set_link();
		}
		if (IsKeyPressed(KEY_EQUAL)) {
			link_loss = std::min(link_loss + 5, 50);
			set_link();
		}
		if (IsKeyPressed(KEY_MINUS)) {
			link_loss = std::max(link_loss - 5, 0);
			set_link();
		}
		if (IsKeyPressed(KEY_U)) {
			static const uint32_t rates[] = { 10, 20, 30, 60 };
			uint32_t next = rates[0];
			for (int i = 0; i < 4; i++) {
				if (rates[i] == core.commands.update_rate()) {
					next = rates[(i + 1) % 4];
				}
			}
			core.commands.set_update_rate(next);
		}
		if (mouse_captured) {
			const Vector2 d = GetMouseDelta();
			yaw -= d.x * 0.0025f;
			pitch = Clamp(pitch - d.y * 0.0025f, -1.2f, 1.3f);
		}
	}

	Camera3D camera(const vec3 &me, bool following) {
		Camera3D cam = {};
		cam.up = { 0.0f, 0.0f, 1.0f };
		cam.fovy = 70.0f;
		cam.projection = CAMERA_PERSPECTIVE;
		if (!following) {
			const float a = (float)t * 0.12f;
			cam.position = { 24.0f * cosf(a), 24.0f * sinf(a), 14.0f };
			cam.target = { 0.0f, 0.0f, 0.0f };
			return cam;
		}
		const vec3 dir = aim_dir(yaw, pitch);
		const vec3 right(sinf(yaw), -cosf(yaw), 0.0f);
		const vec3 eye = me + vec3(0.0f, 0.0f, EYE_HEIGHT) + right * 0.7f;
		cam.position = rl(eye - dir * 4.0f + vec3(0.0f, 0.0f, 0.5f));
		cam.target = rl(eye + dir * 8.0f);
		return cam;
	}

	void draw_player(const vec3 &p, float y, Color c) {
		const quat q = yaw_quat(y);
		box(p + vec3(0.0f, 0.0f, PLAYER_HEIGHT * 0.5f), vec3(PLAYER_HALF * 2, PLAYER_HALF * 2, PLAYER_HEIGHT), q, c);
		const vec3 fwd(cosf(y), sinf(y), 0.0f);
		box(p + fwd * (PLAYER_HALF + 0.02f) + vec3(0.0f, 0.0f, EYE_HEIGHT), vec3(0.12f, 0.6f, 0.18f), q,
				{ 25, 28, 40, 255 });
		shadow(p, PLAYER_HALF * 2.2f);
	}

	void draw_world(Camera3D &cam, float alpha) {
		const float vp[3] = { cam.position.x, cam.position.y, cam.position.z };
		SetShaderValue(lit, loc_view_pos, vp, SHADER_UNIFORM_VEC3);

		const std::vector<Box> boxes = arena_boxes();
		for (size_t i = 0; i < boxes.size(); i++) {
			const Box &b = boxes[i];
			const Color c = i == 0 ? Color{ 58, 66, 88, 255 } : i == 1 ? Color{ 76, 86, 112, 255 } : Color{ 128, 98, 66, 255 };
			box((b.mins + b.maxs) * 0.5f, b.maxs - b.mins, quat(), c);
		}
		// The edge: where you don't want to be.
		const float h = ARENA_HALF;
		const Color edge = { 240, 170, 60, 255 };
		box(vec3(0, h - 0.05f, 0.005f), vec3(2 * h, 0.1f, 0.02f), quat(), edge);
		box(vec3(0, -h + 0.05f, 0.005f), vec3(2 * h, 0.1f, 0.02f), quat(), edge);
		box(vec3(h - 0.05f, 0, 0.005f), vec3(0.1f, 2 * h, 0.02f), quat(), edge);
		box(vec3(-h + 0.05f, 0, 0.005f), vec3(0.1f, 2 * h, 0.02f), quat(), edge);

		const float bs = BLOCK_HALF * 2;
		for (auto &kv : core.ents) {
			ClientCore::Ent &e = kv.second;
			vec3 pos;
			quat rot;
			if (!core.placement(kv.first, e, alpha, pos, rot)) {
				continue;
			}
			if (e.kind == KIND_PLAYER) {
				const float py = kv.first == core.my_id ? yaw : as_float(e.vals[PS_YAW]);
				draw_player(pos, kv.first == core.my_id && !core.predict ? as_float(e.vals[PS_YAW]) : py,
						PALETTE[e.color % MAX_PLAYERS]);
			} else {
				const Color c = e.owner ? lighten(PALETTE[e.color % MAX_PLAYERS], 0.3f) : Color{ 200, 176, 128, 255 };
				box(pos, vec3(bs, bs, bs), rot, c);
				shadow(pos, bs * 1.2f);
			}
		}
		const auto mine = core.ents.find(core.my_id);
		const Color my_color = mine != core.ents.end() ? PALETTE[mine->second.color % MAX_PLAYERS] : WHITE;
		for (auto &kv : core.predicted) {
			const vec3 pos = kv.second.body->get_position();
			box(pos, vec3(bs, bs, bs), kv.second.body->get_orientation_quat(), lighten(my_color, 0.3f));
			shadow(pos, bs * 1.2f);
		}

		// Unlit overlays.
		for (int i = -(int)h; i <= (int)h; i += 2) {
			DrawLine3D({ (float)i, -h, 0.004f }, { (float)i, h, 0.004f }, { 255, 255, 255, 18 });
			DrawLine3D({ -h, (float)i, 0.004f }, { h, (float)i, 0.004f }, { 255, 255, 255, 18 });
		}
		if (show_ghosts) {
			// The newest snapshot, undelayed and unpredicted: what the server last said.
			for (auto &kv : core.ents) {
				ClientCore::Ent &e = kv.second;
				if (!e.has_state || !ClientCore::alive(e)) {
					continue;
				}
				const vec3 p = ClientCore::vals_pos(e.vals);
				if (e.kind == KIND_PLAYER) {
					DrawCubeWires(rl(p + vec3(0, 0, PLAYER_HEIGHT * 0.5f)), PLAYER_HALF * 2, PLAYER_HALF * 2,
							PLAYER_HEIGHT, kv.first == core.my_id ? WHITE : Color{ 255, 255, 255, 120 });
				} else {
					DrawCubeWires(rl(p), bs, bs, bs, { 255, 255, 255, 90 });
				}
			}
		}
	}

	void hud() {
		const int w = GetScreenWidth(), hgt = GetScreenHeight();
		const ClientCore::Stats &s = core.last;
		char line[256];
		int y = 12;
		auto text = [&](const char *str, int x, int yy, int size, Color c) { DrawText(str, x, yy, size, c); };
		DrawRectangle(8, 8, 520, 128, { 0, 0, 0, 120 });
		text("shove  -  goldnet + hop", 16, y, 20, RAYWHITE);
		y += 26;
		snprintf(line, sizeof(line), "ping %.0f ms   simulated link: +%d ms, %d%% loss  ([ ] and - =)", core.ping_ms(),
				link_latency, link_loss);
		text(line, 16, y, 16, LIGHTGRAY);
		y += 20;
		snprintf(line, sizeof(line), "snapshots %d/s (asked %u, U)   in %.1f KB/s   out %.1f KB/s", s.snapshots,
				core.commands.update_rate(), s.bytes_in / 1024.0, s.bytes_out / 1024.0);
		text(line, 16, y, 16, LIGHTGRAY);
		y += 20;
		snprintf(line, sizeof(line), "prediction %s (P)   interpolation %s, %u ms (I)   ghosts %s (G)",
				core.predict ? "ON" : "OFF", core.interpolate ? "ON" : "OFF", core.interp_ms, show_ghosts ? "ON" : "OFF");
		text(line, 16, y, 16, core.predict ? LIGHTGRAY : ORANGE);
		y += 20;
		snprintf(line, sizeof(line), "corrections %d/s, mean %.2f m, max %.2f m   %zu commands in flight",
				s.corrections, s.corrections ? s.error_sum / s.corrections : 0.0f, s.error_max, core.history.size());
		text(line, 16, y, 16, LIGHTGRAY);

		// Scores.
		int sy = 12;
		DrawRectangle(w - 200, 8, 192, 24 + 22 * (int)std::count_if(core.ents.begin(), core.ents.end(),
															[](auto &kv) { return kv.second.kind == KIND_PLAYER; }),
				{ 0, 0, 0, 120 });
		text("score", w - 190, sy, 18, RAYWHITE);
		sy += 24;
		for (auto &kv : core.ents) {
			const ClientCore::Ent &e = kv.second;
			if (e.kind != KIND_PLAYER) {
				continue;
			}
			DrawRectangle(w - 190, sy + 2, 14, 14, PALETTE[e.color % MAX_PLAYERS]);
			snprintf(line, sizeof(line), "%s%s%s", COLOR_NAMES[e.color % MAX_PLAYERS], e.bot ? " (bot)" : "",
					kv.first == core.my_id ? "  < you" : "");
			text(line, w - 170, sy, 16, RAYWHITE);
			snprintf(line, sizeof(line), "%lld", e.has_state ? (long long)as_int(e.vals[PS_SCORE]) : 0LL);
			text(line, w - 30, sy, 16, RAYWHITE);
			sy += 22;
		}
		sy += 10;
		for (const ClientCore::Kill &k : core.kills) {
			if (now_ms() - k.at_ms > 5000) {
				continue;
			}
			auto name = [&](uint32_t id) -> const char * {
				auto it = core.ents.find(id);
				return it == core.ents.end() ? "someone" : COLOR_NAMES[it->second.color % MAX_PLAYERS];
			};
			if (k.shover) {
				snprintf(line, sizeof(line), "%s shoved %s", name(k.shover), name(k.victim));
			} else {
				snprintf(line, sizeof(line), "%s fell off", name(k.victim));
			}
			text(line, w - 190, sy, 16, { 255, 220, 150, 255 });
			sy += 20;
		}

		if (!core.connected) {
			text("connecting...", w / 2 - 70, hgt / 2 - 10, 24, RAYWHITE);
		} else if (core.me_ready && !core.me.alive) {
			text("SHOVED", w / 2 - 60, hgt / 2 - 40, 40, { 255, 120, 90, 255 });
		} else if (core.me_ready) {
			DrawLine(w / 2 - 8, hgt / 2, w / 2 + 8, hgt / 2, RAYWHITE);
			DrawLine(w / 2, hgt / 2 - 8, w / 2, hgt / 2 + 8, RAYWHITE);
			for (int i = 0; i < MAX_AMMO; i++) {
				const Rectangle r = { (float)(w / 2 - 50 + i * 36), (float)(hgt - 50), 28, 28 };
				if (i < core.me.ammo) {
					DrawRectangleRec(r, { 200, 176, 128, 255 });
				}
				DrawRectangleLinesEx(r, 2, { 200, 176, 128, 255 });
			}
		}
		text("WASD move   Space jump   mouse aim   click/E throw   Tab mouse   Esc quit", 12, hgt - 24, 16,
				{ 200, 200, 200, 160 });
	}

	void run() {
		SetConfigFlags(FLAG_MSAA_4X_HINT | FLAG_WINDOW_RESIZABLE | FLAG_VSYNC_HINT);
		InitWindow(1280, 720, "shove - goldnet + hop");
		SetTargetFPS(144);
		lit = LoadShaderFromMemory(LIT_VS, LIT_FS);
		loc_view_pos = GetShaderLocation(lit, "viewPos");
		const float fog[3] = { SKY.r / 255.0f, SKY.g / 255.0f, SKY.b / 255.0f };
		SetShaderValue(lit, GetShaderLocation(lit, "fogColor"), fog, SHADER_UNIFORM_VEC3);
		cube = GenMeshCube(1.0f, 1.0f, 1.0f);
		mat = LoadMaterialDefault();
		mat.shader = lit;
		if (screenshot.empty()) {
			DisableCursor();
		} else {
			mouse_captured = false;
		}

		int frames = 0;
		while (!WindowShouldClose()) {
			const float dt = fminf(GetFrameTime(), 0.25f);
			t += dt;
			if (local) {
				local->poll();
			}
			core.poll();
			handle_keys();
			if (core.me.alive && !was_alive && core.me_ready) {
				yaw = core.me.yaw; // respawned: face the way the server put us
				pitch = 0.0f;
			}
			was_alive = core.me.alive && core.me_ready;
			acc += dt;
			int n = 0;
			while (acc >= TICK && n++ < 8) {
				core.tick(sample_input());
				acc -= TICK;
			}
			if (acc > TICK) {
				acc = 0.0f;
			}

			vec3 me_pos;
			quat me_rot;
			bool following = false;
			auto mine = core.ents.find(core.my_id);
			if (mine != core.ents.end()) {
				following = core.placement(core.my_id, mine->second, acc / TICK, me_pos, me_rot);
			}
			Camera3D cam = camera(me_pos, following);

			BeginDrawing();
			ClearBackground(SKY);
			BeginMode3D(cam);
			draw_world(cam, acc / TICK);
			EndMode3D();
			hud();
			EndDrawing();

			if (quit_after > 0 && ++frames >= quit_after) {
				if (!screenshot.empty()) {
					Image img = LoadImageFromScreen();
					ExportImage(img, screenshot.c_str());
					UnloadImage(img);
				}
				break;
			}
		}
		UnloadShader(lit);
		CloseWindow();
	}
};

} // namespace shove
