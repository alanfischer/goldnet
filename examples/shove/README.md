# shove (work in progress)

A small multiplayer game on goldnet, [hop](https://github.com/alanfischer/hop) physics and
raylib, built to show prediction working over goldnet's uplink. Its rules are a placeholder
(throw blocks, knock the other players off a floating platform) and are expected to change.

```bash
cmake -S . -B build-shove -DGOLDNET_BUILD_SHOVE=ON    # add -DHOP_DIR=<path> to use a local hop
cmake --build build-shove --target shove
build-shove/examples/shove host --bots 3 --latency 150 --loss 5
```

Modes: `host` (a server and a window on it), `server` (no window), `client --host H`, and
`bench`, which runs a server and scripted clients in one process and prints how well the
prediction held up.

What it does:

- **Your player is predicted.** It runs the same move code as the server, ahead of it. Each
  snapshot resets it to the server's state after the command the header acks, and the commands
  since are replayed; whatever the replay moved it by is drawn as an offset that decays.
- **Your throws are predicted.** The block flies at once; the server's spawn names the command
  that threw it, and the prediction hands over to it.
- **Everything else is interpolated**, 100 ms behind the server's clock as read from snapshot
  headers.
- The client simulates its own link both ways (`[` `]` latency, `-` `=` loss in the window).

One hop detail that matters for prediction: hop averages the two bodies' friction, so the arena
is frictionless and the blocks bring their own. With friction on the floor, the contact solver
added friction to the player that depended on contact state the client can't rebuild after a
reset, and the replay drifted by about a centimetre per snapshot.

Known gaps: the rendering hasn't been looked at yet, a few percent of predicted throws are never
claimed by the server, and knockback from hits isn't predicted.
