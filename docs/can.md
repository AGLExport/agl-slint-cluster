# CAN interface

The cluster can take its signals from a CAN bus instead of the built-in
simulation. It understands two sets of frames, which do not overlap, so a bus
may carry either or both:

- **The cluster's own frames** carry exactly the fields of the `Telemetry`
  struct (`rust/src/telemetry.rs`), split over six classic CAN frames with
  11-bit identifiers. The same layout is described for other tools in
  [`can/agl-slint-cluster.dbc`](../can/agl-slint-cluster.dbc), and encoded and
  decoded by `rust/src/can.rs`; keep the three in step.
- **The AGL virtual car (agl-vcar) messages** are the CAN database that
  Automotive Grade Linux's demos share, so the cluster can be driven by the
  same simulators and CAN devices as AGL's stock instrument cluster. They are
  decoded by `rust/src/vcar.rs`; see [AGL virtual car](#agl-virtual-car-agl-vcar)
  below.

## Quick start on a virtual bus

No hardware is needed. Set up a virtual interface, start the cluster reading
from it, and let `cansim` play the vehicle by putting the built-in drive cycle
on the bus:

```sh
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0

cd rust
CLUSTER_SOURCE=can:vcan0 cargo run &   # the cluster, listening
cargo run --bin cansim                 # the "vehicle", sending on vcan0
```

Now poke at it by hand with `cansend` from can-utils; each frame only updates
the fields it carries, so the rest of the display keeps showing what it last
saw:

```sh
cansend vcan0 104#15            # left indicator + low beam + parking brake
cansend vcan0 104#0002          # ..all off again, engine lamp red
cansend vcan0 100#1027          # 100.00 km/h (0x2710, little-endian)
cansend vcan0 101#1E            # battery 30 %
cansend vcan0 105#00486F6D65    # next turn: "Home"
```

A frame may be shorter than its message: only the fields it fully contains
are updated, which is what makes the one- and two-byte frames above work.

`candump vcan0` shows the traffic; with [cantools](https://cantools.readthedocs.io)
and the DBC it is decoded into signal names:

```sh
candump vcan0 | cantools decode ../can/agl-slint-cluster.dbc
```

`cansim` also sends the agl-vcar messages for the fields they cover (speed,
fuel, turn signals, gear, doors, seat belts, lamps), so AGL's own tools see a
plausible AGL vehicle on the same bus.

On a board, name the real interface: `CLUSTER_SOURCE=can` reads `can0`, and
`CLUSTER_SOURCE=can:can1` any other. The interface is reopened whenever it is
missing or goes down, so the cluster may start before the bus is up.

## Frame layout

All multi-byte fields are little-endian (Intel byte order, `@1` in the DBC).
Unlisted bytes are reserved and should be sent as zero. A frame may be shorter
than the message; fields it does not fully contain are left unchanged.

### 0x100 CLUSTER_MOTION

| Bytes | Field                | Type | Scale | Meaning                          |
|-------|----------------------|------|-------|----------------------------------|
| 0-1   | `speed_kmh`          | u16  | 0.01  | road speed, km/h                 |
| 2-3   | `power`              | i16  | 0.001 | -1.000 full regen .. +1.000 full power |
| 4     | `efficiency_percent` | u8   | 1     | 0..100 %                         |
| 5     | `gear`               | u8   |       | ASCII `P` `R` `N` `D`            |
| 6     | `ready`              | bit 0 |      | drivetrain ready                 |

### 0x101 CLUSTER_ENERGY

| Bytes | Field                | Type | Scale | Meaning                 |
|-------|----------------------|------|-------|-------------------------|
| 0     | `battery_level`      | u8   | 1     | 0..100 %                |
| 1     | `fuel_level`         | u8   | 1     | 0..100 %                |
| 2-3   | `range_km`           | u16  | 1     | remaining range, km     |
| 4-5   | `avg_efficiency_kwh` | u16  | 0.01  | km/kWh                  |
| 6     | `outside_temp_c`     | i8   | 1     | deg C                   |

### 0x102 CLUSTER_TRIP

| Bytes | Field          | Type | Scale | Meaning        |
|-------|----------------|------|-------|----------------|
| 0-3   | `odo_km`       | u32  | 0.1   | odometer, km   |
| 4-7   | `trip_seconds` | u32  | 1     | trip time, s   |

### 0x103 CLUSTER_ADAS

| Bytes | Field              | Type | Scale | Meaning                                  |
|-------|--------------------|------|-------|------------------------------------------|
| 0     | `cruise_speed_kmh` | u8   | 1     | cruise set speed, km/h                   |
| 1     | `cruise`           | u8   |       | 0 off, 1 set, 2 active                   |
| 2     | `lane`             | u8   |       | 0 off, 1 tracking, 2 warn, 3 depart      |
| 3     | flags              | bits |       | bit 0 `lta_on`, bit 1 `lca_on`           |
| 4-5   | `turn_distance_km` | u16  | 0.01  | distance to the next turn, km            |

### 0x104 CLUSTER_TELLTALES

| Bytes | Field          | Type | Meaning                                                                 |
|-------|----------------|------|-------------------------------------------------------------------------|
| 0     | flags          | bits | 0 `turn_left`, 1 `turn_right`, 2-3 `headlights` (0 off, 1 low, 2 high), 4 `parking_brake`, 5 `seatbelt`, 6 `door_open` |
| 1     | `engine`       | u8   | 0 off, 1 amber, 2 red                                                   |
| 2     | `oil`          | u8   | 0 off, 1 amber, 2 red                                                   |
| 3     | `battery_warn` | u8   | 0 off, 1 amber, 2 red                                                   |
| 4     | `abs`          | u8   | 0 off, 1 amber, 2 red                                                   |

The turn-signal bits are the *request*; the cluster does the blinking.

### 0x105 CLUSTER_STREET

The next-turn street name as UTF-8, up to 28 bytes, in chunks of 7:

| Bytes | Field   | Meaning                                  |
|-------|---------|------------------------------------------|
| 0     | `chunk` | 0..3, position of this chunk in the name |
| 1-7   | `text`  | up to 7 bytes of the name                |

Chunk 0 starts a new name, so a name of 7 bytes or fewer needs a single frame.
Trailing bytes may be omitted (shorter DLC) or padded with `0x00`.

## AGL virtual car (agl-vcar)

[agl-vcar](https://git.automotivelinux.org/src/agl-dbc/) ("AGL Virtual Car
1.0", `agl-vcar.dbc` and `doc/` in the `src/agl-dbc` repository, commit
`ad41d4910236`) is the CAN database Automotive Grade Linux's demos share: the
Kuksa CAN provider turns it into VSS signals, and AGL's own instrument cluster
reads it directly. The cluster decodes the agl-vcar messages that carry
something it shows, in addition to its own frames, so it can be dropped into
an AGL image and fed by the same simulators and CAN devices as the stock one.

The DBC is not bundled here (the agl-dbc repository carries only AGL's generic
layer licence text); the layout is implemented from AGL's published tables.
To decode a bus with cantools, fetch the DBC from AGL:

```sh
curl -O https://git.automotivelinux.org/src/agl-dbc/plain/agl-vcar.dbc
candump vcan0 | cantools decode agl-vcar.dbc
```

Messages and signals used. Byte order is as in the DBC: `@0` big-endian
(Motorola, start bit is the MSB), `@1` little-endian.

| ID (dec / hex) | Message            | Signal                | Bits      | `Telemetry`                                     |
|----------------|--------------------|-----------------------|-----------|-------------------------------------------------|
| 1001 / 0x3E9   | `Vehicle_Status_1` | `PT_VehicleSpeed`     | 7\|15@0, x0.015625 km/h | `speed_kmh`                                     |
| 985 / 0x3D9    | `Vehicle_Status_2` | `PT_FuelLevelPct`     | 8\|8@1, x0.392157 %     | `fuel_level`                                    |
| 986 / 0x3DA    | `Vehicle_Status_3` | `PT_HazardOn`, `PT_LeftTurnOn`, `PT_RightTurnOn` | 0\|1, 1\|1, 2\|1 @1 | `turn_left`, `turn_right` (hazard sets both) |
| 142 / 0x08E    | `BCM_Status`       | `FrontLeftDoor` .. `RearLeftDoor` | 4..7\|1@0, 1 = open | `door_open` (any door)                        |
|                |                    | `LeftSeatBelt`        | 12\|1@0, 1 = fastened | `seatbelt` (driver's belt unfastened)           |
| 180 / 0x0B4    | `EV_Powertrain`    | `EV_ShiftPosition`    | 12\|3@0: 1 P, 2 R, 3 N, 4 D, 5 B | `gear` (0, 6, 7 leave it unchanged)       |
|                |                    | `EV_MotorFaultLamp`   | 1\|1@0    | `engine` (amber)                                |
| 184 / 0x0B8    | `ESP`              | `AbsFaultLamp`        | 27\|1@0   | `abs` (red)                                     |
| 532 / 0x214    | `Transmission`     | `Gear`                | 7\|8@0, offset -1 | `gear`, as VSS `SelectedGear` (126 P, 127 D, 0 N, <0 R) |

Everything else the cluster shows (power, efficiency, charge, range, trip,
cruise, lane, navigation, headlights, parking brake, oil and 12 V lamps) has
no agl-vcar message; it comes from the cluster's own frames above, or from
[Kuksa](kuksa.md), where AGL's VSS overlay already maps `PT_VehicleSpeed`, the
turn signals and `Gear` from agl-vcar.

Some examples with `cansend`:

```sh
cansend vcan0 3E9#3200000000000000   # PT_VehicleSpeed 100 km/h (0x32 << 7 x 0.015625)
cansend vcan0 3DA#0100000000000000   # hazard lights
cansend vcan0 0B4#0010000000000000   # EV_ShiftPosition 4 = D
cansend vcan0 0B8#0000000800000000   # AbsFaultLamp
cansend vcan0 08E#1000000000000000   # front left door open (and the driver's belt unfastened)
```
