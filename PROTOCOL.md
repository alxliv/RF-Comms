# Wanderer Base — Laptop Text Protocol

A simple, human-readable, line-based protocol for talking to the **Base** Pico2
over USB CDC. It is the only protocol on the USB link. The Base translates
between this text protocol and the binary nRF24 protocol used over the air to
the **Wanderer** tactical Pico2.

```
  Laptop  ──── text (this protocol) ────  Base Pico2  ──── binary (nRF24) ────  Wanderer Pico2
 (human or                                (translator /                         (tactical FSM,
  program)                                 gateway)                              motors, sensors)
```

The Base never sends binary packets to the laptop. Every byte on the USB link
is printable text terminated by newlines.

---

## 1. Design principles

- **One encoding on the wire.** Text down, text up. No binary framing, no
  sync-byte demultiplexing. A reader does "read until `\n`, parse the line."
- **Resynchronizable by construction.** A corrupt or partial line is discarded
  at the next newline. There is no framing state to lose.
- **Forward-compatible.** A parser that does not recognize a line's class or a
  field simply skips it. New message classes and new fields can be added without
  breaking existing clients.
- **Thin, mechanical translation.** Each text command maps to exactly one binary
  command; each binary reply/telemetry maps to one text line. No protocol
  semantics live in the text layer. `protocol.h` remains the single source of
  truth for what commands exist.
- **Human-first, machine-friendly.** Typeable by hand in a serial terminal,
  parseable by a script without a custom framer.

---

## 2. Framing

- A **message is one line**, terminated by `\n` (line feed).
- A leading `\r` is tolerated, so `\r\n` terminators work too.
- Maximum line length is bounded (see §7). Over-length input is discarded to the
  next newline and reported as an error.
- Encoding is 7-bit ASCII.

**Direction is encoded by shape, not by a sigil on both sides:**

- **Laptop → Base** lines are **bare commands**: `verb [arg ...]`.
- **Base → Laptop** lines are **sigil-prefixed**: `<sigil>payload`.

Only the Base output carries a sigil, because only the laptop receives multiple
asynchronous message classes and needs to tell them apart. The Base receives
nothing but commands, so it needs no disambiguation — and bare verbs are nicer
to type. As a side effect, in any captured log every sigil line is something the
Base said and every bare line is something the laptop sent, so direction is
visible at a glance.

---

## 3. Message classes (Base → Laptop)

| Sigil | Class       | Meaning                                                        | Solicited?         |
|:-----:|-------------|---------------------------------------------------------------|--------------------|
| `=`   | result      | Base's immediate ack of your command line (`=ok` / `=err`)    | reply to your line |
| `>`   | data reply  | Answer to a query, often forwarded from the Wanderer          | reply to a query   |
| `#`   | telemetry   | The periodic heartbeat stream                                 | async              |
| `!`   | event       | State change, link up/down, fault                             | async              |
| `*`   | log         | Free-form human text; machine clients ignore it               | async              |

### The two-level acknowledgment: `=` vs `>`

These are different things and must not be conflated:

- **`=` is syntactic.** "I parsed and queued this." Emitted by the Base
  *immediately*, locally, before anything goes over the air.
- **`>` is the result.** The answer to a query. For anything forwarded to the
  Wanderer it arrives a poll **later**, not in the same instant.

So `ver` produces `=ok ver` right away, then `>ver fw=0.1` shortly after. A
purely local command such as `tlm 5` produces only `=ok ...`, because nothing
crosses the RF link.

---

## 4. Commands (Laptop → Base)

| Verb   | Args                  | Maps to       | Scope                          |
|--------|-----------------------|---------------|--------------------------------|
| `arm`  | —                     | `CMD_ARM`     | forwarded to Wanderer          |
| `stop` | —                     | `CMD_STOP`    | forwarded to Wanderer          |
| `move` | `vL vR` (mm/s, int16) | `CMD_MOVE`    | forwarded to Wanderer          |
| `ver`  | —                     | `CMD_GETVER`  | forwarded to Wanderer          |
| `stat` | —                     | `CMD_GETSTAT` | forwarded to Wanderer          |
| `tlm`  | `on` \| `off` \| `<hz>` | —           | Base-local (forwarding control)|
| `rf`   | — \| `on` \| `off`    | `CMD_GETPA`   | Base-local stats; also queries Wanderer PA |
| `setbpa`| `<0-3>`              | —             | Base-local (set *Base* PA level) |
| `setwpa`| `<0-3>`              | `CMD_SETPA`   | Set *Wanderer* PA; `>pa=` confirms |
| `ping` | —                     | —             | Base-local (is the *Base* up)  |
| `help` | —                     | —             | Base-local                     |

- Verbs are **case-insensitive**.
- Arguments are **positional**, whitespace-separated.
- This table *is* the translator: one verb ↔ one binary command type.

---

## 5. Format rules

- **Commands:** `verb [arg ...]` — terse, positional.
- **Base output:** `<sigil>keyword [key=value ...]` — extensible.

The style asymmetry is deliberate. Positional args keep hand-typed commands
short. `key=value` output means adding a field later (e.g. `batt=`) does not
break a parser that does not know it.

**Value conventions**

- Integers: decimal, optionally signed (`-120`).
- Booleans: `0` / `1`.
- FSM state: the **name** (`SAFE`, `ACTIVE`, `FALLBACK`, `FAULT`), for
  readability, not the raw enum number.

**Parser robustness (both directions)**

- Unknown sigil → skip the line.
- Unknown `key=` → ignore that field, keep the rest.
- Unparseable line → skip to next newline.
- Blank lines are ignored.
- On **input**, `*`-prefixed and blank lines are also ignored, so a command
  script may contain `*` comments and spacing.

---

## 6. Field reference

Current telemetry carries only what the firmware actually produces today
(`sequence`, `tactical_state`, `flags`). Battery, odometry, and velocity fields
slot in later as additional `key=value` pairs without a protocol revision.

### `#` telemetry

| Key      | Type   | Meaning                                              |
|----------|--------|------------------------------------------------------|
| `seq`    | uint8  | Telemetry sequence counter (wraps 0–255); gap = loss |
| `state`  | name   | Wanderer FSM state                                   |
| `armed`  | bool   | Motors enabled (FSM in `ACTIVE` or `FALLBACK`)       |
| `moving` | bool   | Motors enabled **and** a target velocity is non-zero |

`armed` and `moving` are the decoded `flags` byte. The raw byte may be carried
as `flags=<n>` alongside them if a client wants it; the decoded form is
preferred on this channel because readability is the point.

### `>` data replies

| Reply  | Fields                              | Source            |
|--------|-------------------------------------|-------------------|
| `>ver` | `fw=<major>.<minor>`                | Wanderer firmware |
| `>stat`| `armed=<0/1> moving=<0/1> vL=<int> vR=<int>` | Wanderer  |
| `>rf`  | `link=<up/down> sent=<n> ok=<n> lost=<n> arc=<0-15> rpd=<0/1> chan=<n> pa_base=<0-3>` | Base (local) |
| `>pa`  | `=<0-3>` (i.e. `>pa=2`)             | Wanderer firmware |

`>rf` is the nRF24 link readout, for range tests. The base polls the Wanderer
at ~100 Hz, so `sent`/`ok`/`lost` are the delivery counts (hardware ACKs) over
the interval **since the previous `>rf` report** — instantaneous link quality,
not a lifetime total. `arc` is the last packet's auto-retransmit count (0–15, a
rough signal-margin proxy) and `rpd` is the received-power detector (`1` = last
received signal ≥ −64 dBm). `pa_base` is the base's own PA level (0 = MIN ..
3 = MAX). `rf on` repeats the report once a second so the link can be watched
dynamically while walking out the range; `rf off` stops it.

`>rf` reports only the **base** PA, which the base reads directly. The
**Wanderer's** PA is on the remote board, so each `rf` report also issues a
`CMD_GETPA` query; the Wanderer's answer arrives a poll later as a separate
`>pa=<0-3>` line. A query, not telemetry: PA rarely changes, so it is fetched on
demand rather than carried in every heartbeat.

PA is settable from either end. `setbpa <0-3>` sets the **base** radio's PA
locally; the `=ok setbpa pa=<n>` ack echoes the level read back from the radio.
`setwpa <0-3>` forwards `CMD_SETPA` to the **Wanderer**, which applies it and
replies `>pa=<n>` with the level actually in effect — so "try to set" is
confirmed by read-back. An out-of-range Wanderer value leaves its PA untouched
and the `>pa=` reply shows it unchanged. (`setwpa` only changes the Wanderer's
transmit power, which the base sees as ACK-payload signal strength; to also
match the base's own power, set `setbpa` to taste.)

### `!` events

| Event       | Form                              | Meaning                       |
|-------------|-----------------------------------|-------------------------------|
| state change| `!state from=<name> to=<name>`    | FSM transition observed       |
| link down   | `!link down`                      | Base stopped hearing Wanderer |
| link up     | `!link up`                        | Base reacquired the Wanderer  |

### `=` results

| Form                         | Meaning                                  |
|------------------------------|------------------------------------------|
| `=ok <verb> [echoed fields]` | Command parsed and accepted/forwarded    |
| `=err <reason>`              | Command rejected (unknown / bad args)    |

---

## 7. Limits and errors

- **Line buffer is bounded.** If an input line exceeds the buffer, the Base
  discards bytes to the next newline and emits `=err line too long`.
- **Input is never trusted by length.** Tokens are validated before use, the
  same defensive instinct applied to the binary structs.
- Argument errors are specific where practical:
  - `=err unknown command: <verb>`
  - `=err move: expected 2 integers`
  - `=err tlm: expected on|off|<hz>`

---

## 8. Examples

### Individual messages

```
#seq=412 state=ACTIVE armed=1 moving=1
>ver fw=0.1
>stat armed=1 moving=1 vL=120 vR=-120
=ok move vL=120 vR=-120
=err unknown command: mve
=err move: expected 2 integers
!state from=ACTIVE to=FALLBACK
!link down
!link up
```

### A worked session

Bare lines are what the laptop sends; sigil lines are the Base replying.

```
arm
=ok arm
!state from=SAFE to=ACTIVE
move 150 150
=ok move vL=150 vR=150
tlm 5
=ok tlm rate=5 on=1
#seq=88 state=ACTIVE armed=1 moving=1
#seq=93 state=ACTIVE armed=1 moving=1
ver
=ok ver
>ver fw=0.1
stop
=ok stop
!state from=ACTIVE to=SAFE
```

---

## 9. Semantics worth pinning down

### `=ok` means *accepted*, not *executed*

The Base confirms it **parsed and forwarded** the command. Whether the Wanderer
actually acts depends on the Wanderer's own state — for example, a `move` is
ignored unless the Wanderer is in `ACTIVE` (or resuming from `FALLBACK`). That
outcome is visible in `#` telemetry and `>stat`, never in the `=` ack. The Base
cannot synchronously know the Wanderer's state, and must not block on RF
pretending otherwise.

### Telemetry subscription is *forwarding* control, not *poll* control

The Base keeps polling the Wanderer at its internal rate (≈100 Hz) regardless of
subscription. `tlm <hz>` only decimates what is **forwarded** to the laptop.

- Default is **off**, so a human attaching a terminal gets a clean console
  instead of a 100 Hz firehose.
- `tlm on` forwards every frame; `tlm <hz>` forwards at most that many per
  second; `tlm off` stops the `#` stream.
- `tlm off` silences only `#` telemetry. **`!` events still fire**, because the
  Base derives `!state` transitions by watching `tactical_state` change across
  the telemetry it is still receiving internally. You can run quiet and still be
  told the instant the vehicle drops into `FALLBACK`.

---

## 10. Reserved / optional extensions

Held in reserve — not part of the baseline, documented so the syntax is
pre-reserved:

- **Client tag echo.** A trailing `@<tag>` on a query, echoed back on the reply,
  to correlate multiple in-flight queries at the text layer:

  ```
  ver @7
  =ok ver @7
  >ver fw=0.1 @7
  ```

  The binary replies dropped their sequence echo, so this restores correlation
  purely in text without touching the RF protocol. Only worth implementing if a
  client will have several queries in flight at once; the human and
  simple-script cases do not need it.

- **Per-line checksum.** An optional `*XX` suffix (NMEA-style) for machine
  clients wanting integrity beyond what USB CDC already provides. Optional
  because the USB layer already error-checks.

---

## 11. Quick reference

```
Laptop -> Base (bare verbs):
  arm | stop | move <vL> <vR> | ver | stat | tlm on|off|<hz> | rf on|off
  setbpa <0-3> | setwpa <0-3> | ping | help

Base -> Laptop (sigil + payload):
  =  result      =ok <verb> ...        =err <reason>
  >  data reply  >ver fw=...            >stat armed=.. moving=.. vL=.. vR=..
                 >rf link=.. sent=.. ok=.. lost=.. arc=.. rpd=.. chan=.. pa_base=..
                 >pa=..  (Wanderer PA, follows an `rf` query a poll later)
  #  telemetry   #seq=.. state=.. armed=.. moving=..
  !  event       !state from=.. to=..  !link up|down
  *  log         *free text (ignored by machines)

States: SAFE | ACTIVE | FALLBACK | FAULT
Skip rule: unknown sigil/key/line -> discard, keep parsing.
```
