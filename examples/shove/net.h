#pragma once
//
// What the shove server and client share about the wire: the value type goldnet carries, its
// encoding, the UDP socket, and the framing around goldnet's packets.
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
#include <string>
#include <variant>

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

inline int open_socket(uint16_t port) {
	const int fd = socket(AF_INET, SOCK_DGRAM, 0);
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
	return fd;
}

inline bool resolve(const char *host, uint16_t port, sockaddr_in &out) {
	addrinfo hints = {}, *res = nullptr;
	hints.ai_family = AF_INET;
	hints.ai_socktype = SOCK_DGRAM;
	if (getaddrinfo(host, nullptr, &hints, &res) != 0 || !res) {
		return false;
	}
	out = *(sockaddr_in *)res->ai_addr;
	out.sin_port = htons(port);
	freeaddrinfo(res);
	return true;
}

inline void wait_readable(int fd, int ms) {
	pollfd p = { fd, POLLIN, 0 };
	poll(&p, 1, ms < 0 ? 0 : ms);
}

inline std::string addr_key(const sockaddr_in &a) {
	char buf[64];
	snprintf(buf, sizeof(buf), "%s:%d", inet_ntoa(a.sin_addr), ntohs(a.sin_port));
	return buf;
}

} // namespace shove
