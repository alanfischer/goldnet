#pragma once
//
// The examples' value coding: a 1-byte tag, then the payload, so a decoder needs no hint (it reads
// the tag and knows the width, and can step over a value for an entity it doesn't know). Only the
// sender chooses a lossy tag, from the slot's hint. goldnet itself never looks inside a value; this
// is one way a host can code them, the same tags goldnet-godot uses for its tagged values.
//
#include <cstdint>

enum ValueTag : uint8_t {
	TAG_OPAQUE = 0,     // u32 length + bytes in the host engine's own encoding
	TAG_FLOAT = 1,      // f32
	TAG_INT = 2,        // zig-zag varint
	TAG_VEC3 = 3,       // 3x f32
	TAG_BOOL = 4,       // u8
	TAG_ANGLE16 = 5,    // float radians -> u16 over [0, TAU)
	TAG_HALF = 6,       // float -> IEEE binary16
	TAG_VEC3_HALF = 7,  // vec3 -> 3x binary16
	TAG_ANGLE8 = 8,     // float radians -> u8 over [0, TAU) — GoldSrc-classic angle precision
	TAG_TIME_DELTA = 9, // int ms timestamp -> zig-zag varint of (value - this packet's header time)
};

// Per-slot quantization hint meaning "none — pick the tag from the value's runtime type".
static const uint8_t QUANT_AUTO = 255;
