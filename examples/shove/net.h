#pragma once
//
// What the shove server and client share about the wire: the value type goldnet carries, its
// encoding, the wire (UDP unless a host supplies another), and the framing around goldnet's packets.
//
// Client -> server datagrams are 'H' (hello: start a session), 'U' + an uplink packet, or 'D'
// (leaving). Server -> client datagrams are goldnet snapshots, unframed. Connection management is
// the host's job, not goldnet's, so this is all of it: a session is an address that said hello,
// and it ends with a 'D' or after TIMEOUT_MS of silence.
//
#include "goldnet/buffer.h"
#include "goldnet/codec.h"
#include "goldnet/protocol.h"

#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <map>
#include <memory>
#include <string>
#include <variant>
#include <vector>

namespace shove {

using namespace goldnet;

typedef std::variant<int64_t, float> Value;

static const uint16_t DEFAULT_PORT = 27510;
static const uint32_t TIMEOUT_MS = 3000;
static const uint32_t CMD_STREAM = 1;

// Reliable messages, server -> client.
static const uint8_t MSG_YOU = 'Y';  // u32 net_id: the player entity that is yours
static const uint8_t MSG_KILL = 'K'; // u32 shover net_id (0 = fell on their own), u32 victim net_id

inline uint32_t now_ms() {
	static const auto start = std::chrono::steady_clock::now();
	return 1 + (uint32_t)std::chrono::duration_cast<std::chrono::milliseconds>(
					   std::chrono::steady_clock::now() - start)
					   .count();
}

inline float as_float(const Value &v) {
	return std::holds_alternative<float>(v) ? std::get<float>(v) : (float)std::get<int64_t>(v);
}

inline int64_t as_int(const Value &v) {
	return std::holds_alternative<int64_t>(v) ? std::get<int64_t>(v) : (int64_t)std::get<float>(v);
}

inline void encode_value(Writer &w, const Value &v, uint8_t quant) {
	if (std::holds_alternative<int64_t>(v)) {
		w.put_u8(TAG_INT);
		put_varint(&w, std::get<int64_t>(v));
		return;
	}
	const float f = std::get<float>(v);
	switch (quant) {
		case TAG_HALF:
			w.put_u8(TAG_HALF);
			w.put_half(f);
			break;
		case TAG_ANGLE16:
			w.put_u8(TAG_ANGLE16);
			put_angle16(&w, f);
			break;
		default:
			w.put_u8(TAG_FLOAT);
			w.put_float(f);
	}
}

inline Value decode_value(Reader &r) {
	switch (r.get_u8()) {
		case TAG_INT:
			return get_varint(&r);
		case TAG_FLOAT:
			return r.get_float();
		case TAG_HALF:
			return r.get_half();
		case TAG_ANGLE16: {
			const float a = get_angle16(&r); // [0, TAU): fold back to (-PI, PI]
			return a > TAU / 2 ? a - TAU : a;
		}
		default:
			r.get_data((size_t)-1); // a tag this demo never sends: fail the packet
			return Value();
	}
}

inline void put_u32_le(uint8_t *p, uint32_t v) {
	p[0] = (uint8_t)v;
	p[1] = (uint8_t)(v >> 8);
	p[2] = (uint8_t)(v >> 16);
	p[3] = (uint8_t)(v >> 24);
}

inline uint32_t get_u32_le(const uint8_t *p) {
	return p[0] | (p[1] << 8) | (p[2] << 16) | ((uint32_t)p[3] << 24);
}

// The datagram wire under shove. A server's wire names each sender with an address string; a
// client's wire has one far end, the server, and ignores addresses.
struct Wire {
	virtual ~Wire() {}
	// One waiting datagram, or false when none is.
	virtual bool receive(std::vector<uint8_t> &pkt, std::string &from) = 0;
	virtual void send(const std::string &to, const uint8_t *data, size_t size) = 0;
	// Until a datagram may be waiting, or `ms` has passed.
	virtual void wait(int ms) = 0;
	// What each datagram costs below goldnet's packet, for its budget.
	virtual uint32_t overhead() const = 0;
	// A server's wire: another server on it should be the one, so this one should stand down.
	virtual bool outranked() const { return false; }
};

class UdpWire : public Wire {
public:
	// A server's wire on `port`, or a client's on any port when `server` is given.
	explicit UdpWire(uint16_t port, const sockaddr_in *server = nullptr) {
		fd = socket(AF_INET, SOCK_DGRAM, 0);
		if (fd < 0) {
			perror("socket");
			exit(1);
		}
		sockaddr_in addr = {};
		addr.sin_family = AF_INET;
		addr.sin_addr.s_addr = htonl(INADDR_ANY);
		addr.sin_port = htons(port);
		if (bind(fd, (sockaddr *)&addr, sizeof(addr)) < 0) {
			perror("bind");
			exit(1);
		}
		if (server) {
			far = *server;
			client = true;
		}
	}
	~UdpWire() override { close(fd); }

	// A client's wire to `host`, or null when it doesn't resolve.
	static std::unique_ptr<Wire> to(const char *host, uint16_t port) {
		addrinfo hints = {}, *res = nullptr;
		hints.ai_family = AF_INET;
		hints.ai_socktype = SOCK_DGRAM;
		if (getaddrinfo(host, nullptr, &hints, &res) != 0 || !res) {
			return nullptr;
		}
		sockaddr_in server = *(sockaddr_in *)res->ai_addr;
		server.sin_port = htons(port);
		freeaddrinfo(res);
		return std::make_unique<UdpWire>(0, &server);
	}

	bool receive(std::vector<uint8_t> &pkt, std::string &from) override {
		uint8_t buf[1500];
		sockaddr_in addr;
		socklen_t len = sizeof(addr);
		const ssize_t n = recvfrom(fd, buf, sizeof(buf), MSG_DONTWAIT, (sockaddr *)&addr, &len);
		if (n <= 0) {
			return false;
		}
		pkt.assign(buf, buf + n);
		if (!client) {
			char key[64];
			snprintf(key, sizeof(key), "%s:%d", inet_ntoa(addr.sin_addr), ntohs(addr.sin_port));
			from = key;
			senders[from] = addr;
		}
		return true;
	}

	void send(const std::string &to, const uint8_t *data, size_t size) override {
		const sockaddr_in *addr = &far;
		if (!client) {
			auto it = senders.find(to);
			if (it == senders.end()) {
				return;
			}
			addr = &it->second;
		}
		sendto(fd, data, size, 0, (const sockaddr *)addr, sizeof(*addr));
	}

	void wait(int ms) override {
		pollfd p = { fd, POLLIN, 0 };
		poll(&p, 1, ms < 0 ? 0 : ms);
	}

	uint32_t overhead() const override { return 28; } // IPv4 + UDP

private:
	int fd = -1;
	bool client = false;
	sockaddr_in far = {};
	std::map<std::string, sockaddr_in> senders; // grows by one entry per address ever heard from
};

// Makes main()'s wires: UDP here, another transport in a host that overrides these.
struct Wires {
	virtual ~Wires() {}
	// An option main() doesn't know, with its value; false if this doesn't know it either.
	virtual bool option(const std::string &, const char *) { return false; }
	// Usage lines for those options.
	virtual const char *usage() const { return ""; }
	// `remote`: whether clients in other processes may join, which bench mode never has.
	virtual std::unique_ptr<Wire> server(uint16_t port, bool remote) {
		(void)remote;
		return std::make_unique<UdpWire>(port);
	}
	// Null when `host` can't be reached.
	virtual std::unique_ptr<Wire> client(const std::string &host, uint16_t port) {
		return UdpWire::to(host.c_str(), port);
	}
};

} // namespace shove
