# goldnet

GoldSrc/Quake-style netcode as a header-only C++17 library: an unreliable snapshot stream,
delta-compressed against the last frame each client acked, with Quake's command uplink and
GoldSrc's netchan riding the same packets. No engine and no dependency beyond the standard
library. You bring the sockets and the game state, and goldnet decides what goes in each packet.

Engine adapters:

- [goldnet-godot](https://github.com/alanfischer/goldnet-godot): goldnet behind Godot 4's
  `MultiplayerAPI`, serving stock `MultiplayerSynchronizer` / `MultiplayerSpawner` / `@rpc`

## Why

Reliable, ordered replication is simple and correct, but on a lossy link the retransmits pile up
until the connection times out. GoldSrc takes the opposite approach. It sends an **unreliable**
snapshot every tick, delta-compressed against the last frame the client *acked*. A lost packet is
never resent: the next snapshot carries whatever has changed since the acked frame, so the stream
recovers by itself. The result is steady bandwidth and connections that survive loss where
reliable replication drops clients.

## How it works

**Templated on your engine.** `goldnet::Server<V, X>` and `goldnet::Client<V>` are templates over
your value type `V` (default-constructible, copyable, comparable with `!=`) and optional
per-entity storage `X`. They reach your engine only through a host object you pass in: read a
slot, test visibility, encode a value, send bytes. Derive your host from `goldnet::ServerHost` or
`goldnet::ClientHost` and you write only the few methods every game needs; the rest default to a game
that doesn't use that feature (see **Using it**). goldnet never looks inside a value; it compares
them, stores them and hands them back to you to encode.

**Entities are slots.** An entity is a `net_id` and a fixed list of slots (position x, y, z, a
yaw, a score, ...). Every tick the server reads each live entity's slots once, records which ones
changed, and then for each peer that's due a snapshot writes only the slots that changed since
the frame that peer last acked. An entity with nothing changed costs nothing, and a snapshot with
nothing in it isn't sent (apart from a header-only keepalive every 250 ms, so the client's clock
feed never goes quiet).

**Spawns.** An entity both ends already know about (one that loads with the map) only needs
`add_entity`. A runtime entity also gets `add_spawn` with your own recipe bytes, which goes to
each peer reliable-until-acked, ahead of the entity deltas in the same packet. Despawns go only
to peers that received the spawn, the same way.

**The packet budget.** The entity section gets whatever a safe UDP payload (1200 bytes) has left,
so a joining client's first full frame or a burst of changes can't overrun the MTU. Anything that
doesn't fit is offered again next tick, ordered by how long it has waited for that peer times an
optional per-entity priority, so nothing starves.

**Commands, the other way.** A client's input (Quake's usercmds) is a stream where every element
matters, so it can't converge the way state does. `CommandSender` / `CommandReceiver` deliver each
command once and in order, the way Quake did, without a reliable channel underneath: every uplink
packet carries the commands not sent yet, two backups, and any unacked one older than a round
trip and a half. Each command is a row of typed fields, delta-coded field by field against the one
before it the way GoldSrc's `delta.lst` coded a usercmd. The server's
command ack rides each snapshot header, and the client's snapshot ack rides each uplink packet.

**The netchan.** Both directions also carry a GoldSrc netchan: a reliable stream (one block in
flight, resent only when an ack shows it lost, with large messages fragmented) and an unreliable
datagram. Use it for events: state converges, so a value that goes A→B→A between two acked frames
is only ever seen as A.

**Rates.** Per-peer `rate` (bytes/sec, GoldSrc's choke) and `update_rate` (snapshots/sec,
`cl_updaterate`), requested by the client over the uplink and granted by the server, and the
client's command rate (uplink packets/sec, `cl_cmdrate`). `Client::set_auto_rate` and
`set_auto_command_rate` pick the rate and the command rate from how far behind each direction
arrives (`auto_rate.h`).

**NetSim.** `netsim.h` simulates loss, latency, latency spikes and a slow link on any send path,
seeded so a whole session can be replayed.

The byte-level wire format is in [`docs/protocol.md`](docs/protocol.md).

## Using it

Add `include/` to your include path, or `add_subdirectory(goldnet)` and link the `goldnet`
target. Tests and examples only build when goldnet is the top-level project.

Every call that takes the clock takes it last: ms from any clock that counts up, starting wherever it
likes (0 included). A `Writer` to fill comes first, and a host or callback just before the clock.

Server:

```cpp
goldnet::Server<MyValue> server;
server.set_packet_overhead(28);     // IPv4 + UDP, for rate budgets
server.set_snapshot_rate(60);       // how often you call tick()

server.add_entity(net_id, layout);  // starts replicating (EntityLayout: its slots); remove_entity(net_id)
server.add_spawn(net_id, record);   // a runtime entity clients must create; despawn(net_id)
server.mark_dirty(net_id);          // a pushed slot was written (EntityLayout::push)

server.add_peer(peer);              // on connect; remove_peer(peer) on disconnect
server.tick(host, now_ms);          // every tick: reads state, sends each due peer a snapshot
server.peer_stats(peer);            // its rate, chokes, uplink report and netchan counts
server.silent_ms(peer, now_ms);     // how long since it last sent anything: time a quiet one out

server.set_command_schema(stream_id, schema);  // a command stream's fields, as the client names them
server.receive_uplink(peer, bytes, size, host, now_ms);  // each uplink packet from a peer
// host gets command() and message() in order; the server takes the acks, rate and update rate
```

The server's host, one object for `tick()` and `receive_uplink()`:

```cpp
struct MyServerHost : goldnet::ServerHost<MyValue> {
    void read(Entity &e, bool polled_only) override;  // server.stamp(e, slot, value) for each slot
    void encode(goldnet::Writer &w, const Entity &e, int slot, const MyValue *base, uint32_t now) override;
    void send(int32_t peer, const goldnet::Writer &packet) override;
    // As you need them: live, visible, spawn_visible, command, message, command_schema, protocol_mismatch
};
```

Client:

```cpp
goldnet::Client<MyValue> client;
client.apply(bytes, size, host, now_ms);  // each snapshot; .status says why one wasn't taken
client.forget(net_id);  // an entity the host did away with, other than by its despawn
// client.reset() for a new session

client.push_command(stream_id, bytes, size, now_ms);  // one command; returns its seq
client.set_rate(25000);           // what to ask the server for; or set_auto_rate(true)
if (client.uplink_due(now_ms)) {  // commands, messages, or an ack the server is waiting on
    goldnet::Writer w;
    client.write_uplink(w, now_ms);  // send w to the server
}
client.stats();                   // arrivals, lag, the rates in force, commands and messages
client.silent_ms(now_ms);         // how long since a snapshot arrived: time a quiet server out
```

The client's host:

```cpp
struct MyClientHost : goldnet::ClientHost<MyValue> {
    goldnet::EntityResult entity(goldnet::EntityUpdate<MyValue> &u) override {
        Thing *t = find(u.net_id());
        if (!t) return goldnet::ENTITY_SKIP;       // not one we have: its bytes are stepped over
        for (int s : u.changed()) {                // each changed slot, in order
            MyValue v = decode(u.reader(), s, u.base(s));  // as your encode wrote it
            u.set(s, v);                           // the next delta's base
            t->apply(s, v);
        }
        return goldnet::ENTITY_APPLIED;
    }
    // As you need them: spawn, despawn, relevance_lost, server_time, message, protocol_mismatch
};
```

Mark each method `override`, so a misspelt one fails to compile instead of quietly leaving the
default in place. The bases are optional: the server and client call whatever the host object they're
handed has, so a host that defines every method needn't derive from anything. The contract for each
method is at the top of `server.h` and `client.h`.

Connection management (saying hello, deciding who is a peer, when to give up on one) is up to you;
the server serves only the peers you add, and `silent_ms` on each end says how long the other has
been quiet. Both examples show a
minimal one over plain UDP.

## Examples

`examples/demo_terminal.cpp` (macOS / Linux): a server bouncing balls that spawn and despawn, and
clients that draw them in the terminal and send it a command every tick over the uplink, with
NetSim loss and latency on either end.

```bash
cmake -S . -B build && cmake --build build
build/examples/goldnet_demo server --loss 20 --latency 60
build/examples/goldnet_demo client --loss 20 --latency 60   # another terminal; as many as you like
```

## Testing

```bash
cmake -S . -B build
cmake --build build
ctest --test-dir build --output-on-failure
```

No engine, and it runs in a few seconds. The tests build with UndefinedBehaviorSanitizer and
recovery off, and that is load-bearing: see the comment in `tests/CMakeLists.txt`.

- `test_codec`: the varint codecs (including malformed over-long streams, which are reachable from
  the wire), the angle16/angle8 quantizers (wraparound, non-finite input), the half-float codec,
  the bounds-checked `Reader`, and the seeded loss PRNG.
- `test_seq`: uint16 sequence comparison across rollover and the reliable-until-acked
  bookkeeping.
- `test_protocol`: a real `Server` feeding real `Client`s over a fake link that drops snapshots or
  acks: deltas against the acked baseline, recovery after a lost snapshot or ack, aging into a full
  frame, spawns, despawns scoped to holders, PVS leaves, the packet budget and its priority order,
  per-peer cadence, pushed slots, the clock keepalive, warmup defer, and reordered, truncated or
  mismatched packets. `TestServerHost` and `TestClientHost` there are a complete minimal host pair.
- `test_uplink`: command delivery, order, backups, resends and give-ups.
- `test_netchan`: the reliable block, fragments and the datagram.
- `test_auto_rate`: the meters, and the controllers that pick a client's rate and command rate.
- `test_netsim`: config clamping, seeded replay, the latency range and loss rate, the delay pipe,
  the spike cycle, and the link cap.

## License

MIT. See [LICENSE](LICENSE).
