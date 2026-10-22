#pragma once
//
// shove: goldnet + hop + raylib. Throw blocks, knock everyone else off the platform.
//
//   shove host   [--port N] [--bots N] [--latency MS] [--loss PCT]   a server and a window on it
//   shove server [--port N] [--bots N]                               a server, no window
//   shove client [--host H] [--port N] [--latency MS] [--loss PCT]   a window on someone's server
//   shove bench  [--clients N] [--seconds S] [--bots N] [--latency MS] [--loss PCT]
//                a server and scripted clients in one process, no window; prints how well the
//                prediction held up
//
// --latency and --loss are this client's link, applied each way. run() takes the wires, so a host
// with another transport reuses all of this.
//
#include "client.h"
#include "server.h"
#include "view.h"

#include <string>

namespace shove {

struct Options {
	std::string mode;
	std::string host = "127.0.0.1";
	uint16_t port = DEFAULT_PORT;
	int bots = -1;
	int latency = 0;
	int loss = 0;
	int clients = 2;
	int seconds = 20;
	std::string screenshot;
	int frames = 0;
};

inline void usage(const Wires &wires) {
	fprintf(stderr,
			"usage: shove host   [--port N] [--bots N] [--latency MS] [--loss PCT]\n"
			"       shove server [--port N] [--bots N]\n"
			"       shove client [--host H] [--port N] [--latency MS] [--loss PCT]\n"
			"       shove bench  [--clients N] [--seconds S] [--bots N] [--latency MS] [--loss PCT]\n%s",
			wires.usage());
	exit(2);
}

inline void set_link(ClientCore &c, int latency, int loss) {
	for (NetSim<std::vector<uint8_t>> *s : { &c.up_sim, &c.down_sim }) {
		s->set_latency_min_ms(latency / 2);
		s->set_latency_max_ms(latency / 2);
		s->set_loss_percent(loss);
	}
}

// A scripted player for the bench: run in loops, jump now and then, throw every second.
inline Input script_input(ClientCore &c, int i, uint32_t t) {
	Input in;
	const vec3 p = c.me.body->get_position();
	const float phase = t * 0.0008f + i * 1.7f;
	in.yaw = phase;
	in.pitch = 0.15f;
	in.buttons = B_FORWARD | ((t / 700 + i) % 2 ? B_LEFT : B_RIGHT);
	if (fabsf(p.x) > ARENA_HALF - 3.0f || fabsf(p.y) > ARENA_HALF - 3.0f) {
		in.yaw = atan2f(-p.y, -p.x);
		in.buttons = B_FORWARD;
	}
	if ((t / 16) % 130 == (uint32_t)i * 7) {
		in.buttons |= B_JUMP;
	}
	if ((t / 16) % 60 == (uint32_t)i * 11 % 60) {
		in.buttons |= B_THROW;
	}
	return in;
}

inline int bench(const Options &o, Wires &wires) {
	static ServerApp server;
	server.verbose = false;
	server.start(wires.server(o.port, false), o.bots < 0 ? 2 : o.bots);
	std::vector<ClientCore *> clients;
	for (int i = 0; i < o.clients; i++) {
		ClientCore *c = new ClientCore();
		set_link(*c, o.latency, o.loss);
		std::unique_ptr<Wire> w = wires.client("127.0.0.1", o.port);
		if (!w) {
			fprintf(stderr, "bench: no wire to the server\n");
			return 1;
		}
		c->start(std::move(w));
		clients.push_back(c);
	}
	const uint32_t start = now_ms();
	uint32_t next_tick = start, ticks = 0, next_print = start + 1000;
	float err_sum = 0.0f, err_max = 0.0f;
	int corrections = 0, seconds = 0;
	size_t in_bytes = 0, out_bytes = 0;
	printf("bench: %d clients, %d bots, latency %d ms, loss %d%%, %d s\n", o.clients, server.bots_wanted, o.latency,
			o.loss, o.seconds);
	while (now_ms() - start < (uint32_t)o.seconds * 1000) {
		server.poll();
		const uint32_t now = now_ms();
		for (ClientCore *c : clients) {
			c->poll();
		}
		while ((int32_t)(now - next_tick) >= 0) {
			for (size_t i = 0; i < clients.size(); i++) {
				clients[i]->tick(script_input(*clients[i], (int)i, now - start));
			}
			next_tick = start + (uint32_t)((uint64_t)++ticks * 1000 / TICK_HZ);
		}
		if ((int32_t)(now - next_print) >= 0) {
			next_print += 1000;
			ClientCore &c = *clients[0];
			const ClientCore::Stats &s = c.last;
			printf("  t=%2ds  client 0: %2d snapshots/s  in %5.1f KB/s  out %4.1f KB/s  ping %3.0f ms  "
				   "%2d corrections (mean %.3f, max %.3f m)  replayed %d\n",
					++seconds, s.snapshots, s.bytes_in / 1024.0, s.bytes_out / 1024.0, c.ping_ms(), s.corrections,
					s.corrections ? s.error_sum / s.corrections : 0.0f, s.error_max, s.replayed);
			for (ClientCore *cl : clients) {
				err_sum += cl->last.error_sum;
				err_max = fmaxf(err_max, cl->last.error_max);
				corrections += cl->last.corrections;
				in_bytes += cl->last.bytes_in;
				out_bytes += cl->last.bytes_out;
			}
		}
		server.wire->wait(1);
	}
	int predicted = 0, adopted = 0, unclaimed = 0, connected = 0;
	for (ClientCore *c : clients) {
		predicted += c->throws_predicted;
		adopted += c->throws_adopted;
		unclaimed += c->throws_unclaimed;
		connected += c->connected;
		c->leave();
	}
	const float per = (float)(seconds * clients.size());
	printf("bench done: %d/%zu connected; per client %.1f KB/s in, %.1f KB/s out; %.1f corrections/s, mean %.3f m, "
		   "max %.3f m; throws %d predicted, %d claimed by the server, %d never claimed\n",
			connected, clients.size(), in_bytes / 1024.0 / per, out_bytes / 1024.0 / per, corrections / per,
			corrections ? err_sum / corrections : 0.0f, err_max, predicted, adopted, unclaimed);
	return connected == (int)clients.size() ? 0 : 1;
}

inline int run(int argc, char **argv, Wires &wires) {
	if (argc < 2) {
		usage(wires);
	}
	setvbuf(stdout, nullptr, _IOLBF, 0);
	Options o;
	o.mode = argv[1];
	for (int i = 2; i < argc; i++) {
		const std::string a = argv[i];
		if (i + 1 >= argc) {
			usage(wires);
		}
		const char *v = argv[++i];
		if (a == "--port") {
			o.port = (uint16_t)atoi(v);
		} else if (a == "--host") {
			o.host = v;
		} else if (a == "--bots") {
			o.bots = atoi(v);
		} else if (a == "--latency") {
			o.latency = atoi(v);
		} else if (a == "--loss") {
			o.loss = atoi(v);
		} else if (a == "--clients") {
			o.clients = atoi(v);
		} else if (a == "--seconds") {
			o.seconds = atoi(v);
		} else if (a == "--screenshot") { // save the frame after --frames frames, then quit
			o.screenshot = v;
		} else if (a == "--frames") {
			o.frames = atoi(v);
		} else if (!wires.option(a, v)) {
			usage(wires);
		}
	}
	if (o.mode == "server") {
		static ServerApp server;
		server.start(wires.server(o.port, true), o.bots < 0 ? 3 : o.bots);
		return server.run();
	}
	if (o.mode == "bench") {
		return bench(o, wires);
	}
	if (o.mode != "host" && o.mode != "client") {
		usage(wires);
	}
	static ServerApp server;
	ServerApp *local = nullptr;
	if (o.mode == "host") {
		server.verbose = false;
		server.start(wires.server(o.port, true), o.bots < 0 ? 3 : o.bots);
		local = &server;
	}
	static ClientCore core;
	set_link(core, o.latency, o.loss);
	const std::string host = o.mode == "host" ? "127.0.0.1" : o.host;
	std::unique_ptr<Wire> w = wires.client(host, o.port);
	if (!w) {
		fprintf(stderr, "[client] can't reach %s\n", host.c_str());
		return 1;
	}
	core.start(std::move(w));
	View view(core, local);
	view.screenshot = o.screenshot;
	view.quit_after = o.frames;
	view.run();
	core.leave();
	return 0;
}

} // namespace shove
