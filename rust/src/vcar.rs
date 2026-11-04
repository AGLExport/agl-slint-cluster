// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2026 Signal Slot Inc.

//! The AGL virtual car (agl-vcar) CAN signals: the subset of
//! Automotive Grade Linux's `agl-vcar.dbc` that carries something the cluster
//! shows, decoded into `Telemetry` and encoded from it for `cansim`.
//!
//! agl-vcar is the CAN database AGL's demos share. The Kuksa CAN provider
//! turns it into VSS signals, and AGL's own instrument cluster reads it
//! directly, so a cluster that understands these frames can be dropped into
//! an AGL image and driven by the same simulators and CAN devices as the
//! stock one. The specification is the `agl-vcar.dbc` and `doc/` in
//! <https://git.automotivelinux.org/src/agl-dbc/> (commit ad41d4910236,
//! "AGL Virtual Car 1.0"). The DBC itself is not bundled here: the agl-dbc
//! repository carries only AGL's generic layer licence text, so the layout is
//! re-implemented from the published tables and the DBC is fetched from AGL
//! when a tool needs it.
//!
//! These IDs do not overlap the cluster's own frames in `can.rs`; a bus may
//! carry both, and `can::decode` hands any ID it does not own to
//! [`decode`] here. agl-vcar covers speed, fuel, turn signals, gear, doors,
//! seat belts and the motor and ABS fault lamps; everything else the cluster
//! shows (range, charge, trip, cruise, lane, navigation, ...) has no agl-vcar
//! message and comes from the cluster's own frames or from Kuksa.
//!
//! Bit numbering follows the DBC: `@1` signals are little-endian (Intel) and
//! `@0` signals big-endian (Motorola), both with the DBC's start-bit
//! convention. The formulas in agl-dbc's `doc/vehicle_signal.md` are the
//! reference for the big-endian ones.

use crate::can::Message;
use crate::telemetry::{Telemetry, Warn};

/// `Vehicle_Status_1`: `PT_VehicleSpeed`.
pub const ID_VEHICLE_STATUS_1: u16 = 1001;
/// `Vehicle_Status_2`: `PT_FuelLevelPct`, `PT_EngineSpeed`, `PT_FuelLevelLow`.
pub const ID_VEHICLE_STATUS_2: u16 = 985;
/// `Vehicle_Status_3`: `PT_HazardOn`, `PT_LeftTurnOn`, `PT_RightTurnOn`.
pub const ID_VEHICLE_STATUS_3: u16 = 986;
/// `BCM_Status`: the four doors and the two front seat belts.
pub const ID_BCM_STATUS: u16 = 142;
/// `EV_Powertrain`: `EV_ShiftPosition`, `EV_MotorFaultLamp`.
pub const ID_EV_POWERTRAIN: u16 = 180;
/// `ESP`: `EspOffLamp`, `AbsFaultLamp`, `EspActLamp`.
pub const ID_ESP: u16 = 184;
/// `Transmission`: `Gear`, which the AGL VSS overlay maps to
/// `Vehicle.Powertrain.Transmission.SelectedGear`.
pub const ID_TRANSMISSION: u16 = 532;

/// Every message agl-vcar defines is 8 bytes long.
const DLC: usize = 8;

/// One DBC signal: start bit, length and byte order, as in the `SG_` line.
#[derive(Clone, Copy)]
struct Sig {
    start: u8,
    len: u8,
    big_endian: bool,
}

/// `@1`: little-endian (Intel). `start` is the LSB.
const fn le(start: u8, len: u8) -> Sig {
    Sig {
        start,
        len,
        big_endian: false,
    }
}

/// `@0`: big-endian (Motorola). `start` is the MSB, in DBC's sawtooth numbering.
const fn be(start: u8, len: u8) -> Sig {
    Sig {
        start,
        len,
        big_endian: true,
    }
}

impl Sig {
    /// Byte and bit of value bit `i` (0 = LSB) inside the frame.
    fn pos(&self, i: u8) -> (usize, u8) {
        let lin = if self.big_endian {
            // The MSB sits at `start`; the bits that follow run towards bit 0
            // of that byte, then on from bit 7 of the next. Mapping the start
            // bit to a "7 - bit" linear position makes that a plain increment.
            let msb = (self.start / 8) * 8 + (7 - self.start % 8);
            msb + (self.len - 1 - i)
        } else {
            self.start + i
        };
        let bit = lin % 8;
        (
            (lin / 8) as usize,
            if self.big_endian { 7 - bit } else { bit },
        )
    }

    /// The raw (unscaled) value, or `None` when the frame is too short to
    /// contain the whole signal.
    fn get(&self, d: &[u8]) -> Option<u64> {
        let mut v = 0u64;
        for i in 0..self.len {
            let (byte, bit) = self.pos(i);
            let b = *d.get(byte)?;
            v |= (((b >> bit) & 1) as u64) << i;
        }
        Some(v)
    }

    fn put(&self, d: &mut [u8; DLC], raw: u64) {
        for i in 0..self.len {
            let (byte, bit) = self.pos(i);
            if (raw >> i) & 1 != 0 {
                d[byte] |= 1 << bit;
            } else {
                d[byte] &= !(1 << bit);
            }
        }
    }
}

// Vehicle_Status_1 (1001)
const PT_VEHICLE_SPEED: Sig = be(7, 15); // x 0.015625 km/h
// Vehicle_Status_2 (985)
const PT_FUEL_LEVEL_PCT: Sig = le(8, 8); // x 0.392157 %
const PT_ENGINE_SPEED: Sig = be(23, 16); // x 0.25 rpm
const PT_FUEL_LEVEL_LOW: Sig = le(55, 1);
// Vehicle_Status_3 (986)
const PT_HAZARD_ON: Sig = le(0, 1);
const PT_LEFT_TURN_ON: Sig = le(1, 1);
const PT_RIGHT_TURN_ON: Sig = le(2, 1);
// BCM_Status (142); doors 1 = open, belts 1 = fastened
const FRONT_LEFT_DOOR: Sig = be(4, 1);
const FRONT_RIGHT_DOOR: Sig = be(5, 1);
const REAR_RIGHT_DOOR: Sig = be(6, 1);
const REAR_LEFT_DOOR: Sig = be(7, 1);
const LEFT_SEAT_BELT: Sig = be(12, 1);
const RIGHT_SEAT_BELT: Sig = be(13, 1);
// EV_Powertrain (180)
const EV_SHIFT_POSITION: Sig = be(12, 3); // 1 P, 2 R, 3 N, 4 D, 5 B
const EV_MOTOR_FAULT_LAMP: Sig = be(1, 1);
// ESP (184)
const ABS_FAULT_LAMP: Sig = be(27, 1);
// Transmission (532)
const GEAR: Sig = be(7, 8); // offset -1: VSS SelectedGear (126 P, 127 D, 0 N, <0 R)

const SPEED_SCALE: f32 = 0.015625;
const FUEL_SCALE: f32 = 0.392157;
const FUEL_LOW_BELOW: f32 = 0.1;

/// Apply one agl-vcar message to `t`. Returns `false` for an ID that is not an
/// agl-vcar message the cluster uses; `t` is then untouched. As in `can.rs`, a
/// signal is only updated when the frame is long enough to contain it.
pub fn decode(t: &mut Telemetry, id: u16, d: &[u8]) -> bool {
    match id {
        ID_VEHICLE_STATUS_1 => {
            if let Some(v) = PT_VEHICLE_SPEED.get(d) {
                t.speed_kmh = v as f32 * SPEED_SCALE;
            }
        }
        ID_VEHICLE_STATUS_2 => {
            // PT_EngineSpeed and PT_FuelLevelLow have no field in Telemetry.
            if let Some(v) = PT_FUEL_LEVEL_PCT.get(d) {
                t.fuel_level = (v as f32 * FUEL_SCALE / 100.0).min(1.0);
            }
        }
        ID_VEHICLE_STATUS_3 => {
            // Hazard lights are both indicators; the cluster does the blinking.
            if let (Some(h), Some(l), Some(r)) = (
                PT_HAZARD_ON.get(d),
                PT_LEFT_TURN_ON.get(d),
                PT_RIGHT_TURN_ON.get(d),
            ) {
                t.turn_left = h != 0 || l != 0;
                t.turn_right = h != 0 || r != 0;
            }
        }
        ID_BCM_STATUS => {
            if let (Some(fl), Some(fr), Some(rr), Some(rl)) = (
                FRONT_LEFT_DOOR.get(d),
                FRONT_RIGHT_DOOR.get(d),
                REAR_RIGHT_DOOR.get(d),
                REAR_LEFT_DOOR.get(d),
            ) {
                t.door_open = fl != 0 || fr != 0 || rr != 0 || rl != 0;
            }
            // The driver's belt (left: agl-vcar's virtual car is left-hand
            // drive). The passenger seat may be empty, so its belt is not a
            // warning.
            if let Some(b) = LEFT_SEAT_BELT.get(d) {
                t.seatbelt = b == 0;
            }
        }
        ID_EV_POWERTRAIN => {
            if let Some(v) = EV_SHIFT_POSITION.get(d) {
                // 0 is "invalid", 6 and 7 are reserved: leave the gear alone.
                if let Some(g) = shift_to_gear(v) {
                    t.gear = g;
                }
            }
            if let Some(v) = EV_MOTOR_FAULT_LAMP.get(d) {
                t.engine = if v != 0 { Warn::Amber } else { Warn::Off };
            }
        }
        ID_ESP => {
            if let Some(v) = ABS_FAULT_LAMP.get(d) {
                // Red, as the Kuksa source shows Vehicle.ADAS.ABS.IsError.
                t.abs = if v != 0 { Warn::Red } else { Warn::Off };
            }
        }
        ID_TRANSMISSION => {
            if let Some(v) = GEAR.get(d) {
                t.gear = match v as i64 - 1 {
                    126 => 'P',
                    127 => 'D',
                    0 => 'N',
                    g if g < 0 => 'R',
                    _ => 'D',
                };
            }
        }
        _ => return false,
    }
    true
}

fn shift_to_gear(v: u64) -> Option<char> {
    Some(match v {
        1 => 'P',
        2 => 'R',
        3 => 'N',
        4 => 'D',
        5 => 'B',
        _ => return None,
    })
}

fn gear_to_shift(gear: char) -> u64 {
    match gear {
        'P' => 1,
        'R' => 2,
        'N' => 3,
        'D' => 4,
        'B' => 5,
        _ => 0,
    }
}

/// Encode the agl-vcar messages for everything in `t` they can carry, for
/// senders such as `cansim`. Fields agl-vcar has no signal for are not sent;
/// `can::encode` covers those.
pub fn encode(t: &Telemetry) -> Vec<Message> {
    let mut out = Vec::with_capacity(7);
    let msg = |id: u16, d: [u8; DLC]| Message {
        id,
        data: d.to_vec(),
    };

    let mut d = [0u8; DLC];
    PT_VEHICLE_SPEED.put(&mut d, raw(t.speed_kmh / SPEED_SCALE, 15));
    out.push(msg(ID_VEHICLE_STATUS_1, d));

    let mut d = [0u8; DLC];
    PT_FUEL_LEVEL_PCT.put(&mut d, raw(t.fuel_level * 100.0 / FUEL_SCALE, 8));
    PT_ENGINE_SPEED.put(&mut d, 0);
    PT_FUEL_LEVEL_LOW.put(&mut d, (t.fuel_level < FUEL_LOW_BELOW) as u64);
    out.push(msg(ID_VEHICLE_STATUS_2, d));

    let mut d = [0u8; DLC];
    PT_HAZARD_ON.put(&mut d, (t.turn_left && t.turn_right) as u64);
    PT_LEFT_TURN_ON.put(&mut d, t.turn_left as u64);
    PT_RIGHT_TURN_ON.put(&mut d, t.turn_right as u64);
    out.push(msg(ID_VEHICLE_STATUS_3, d));

    let mut d = [0u8; DLC];
    FRONT_LEFT_DOOR.put(&mut d, t.door_open as u64);
    LEFT_SEAT_BELT.put(&mut d, (!t.seatbelt) as u64);
    RIGHT_SEAT_BELT.put(&mut d, 1);
    out.push(msg(ID_BCM_STATUS, d));

    let mut d = [0u8; DLC];
    EV_SHIFT_POSITION.put(&mut d, gear_to_shift(t.gear));
    EV_MOTOR_FAULT_LAMP.put(&mut d, (t.engine != Warn::Off) as u64);
    out.push(msg(ID_EV_POWERTRAIN, d));

    let mut d = [0u8; DLC];
    ABS_FAULT_LAMP.put(&mut d, (t.abs != Warn::Off) as u64);
    out.push(msg(ID_ESP, d));

    let mut d = [0u8; DLC];
    let selected: i64 = match t.gear {
        'P' => 126,
        'R' => -1,
        'N' => 0,
        _ => 127,
    };
    GEAR.put(&mut d, (selected + 1) as u64);
    out.push(msg(ID_TRANSMISSION, d));

    out
}

/// Round a scaled physical value to a raw field of `bits` bits, saturating.
fn raw(v: f32, bits: u32) -> u64 {
    let max = (1u64 << bits) - 1;
    (v.round().max(0.0) as u64).min(max)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn apply(t: &mut Telemetry, id: u16, d: &[u8]) {
        assert!(decode(t, id, d), "unrecognised: {id:#x} {d:02x?}");
    }

    /// The worked formulas in agl-dbc's doc/vehicle_signal.md, as bytes.
    #[test]
    fn matches_the_published_formulas() {
        let mut t = Telemetry::default();
        // VehicleSpeed = ((payload[0] * 128) + ((payload[1] / 2) & 0x7f)) * 0.015625
        apply(&mut t, ID_VEHICLE_STATUS_1, &[0x32, 0x00, 0, 0, 0, 0, 0, 0]);
        assert_eq!(t.speed_kmh, 100.0);
        apply(&mut t, ID_VEHICLE_STATUS_1, &[0x32, 0x02, 0, 0, 0, 0, 0, 0]);
        assert_eq!(t.speed_kmh, 100.015625);
        // FuelLevelPct = payload[1] * 0.392157
        apply(&mut t, ID_VEHICLE_STATUS_2, &[0, 0xff, 0, 0, 0, 0, 0, 0]);
        assert!((t.fuel_level - 1.0).abs() < 0.001);
        // EV_ShiftPosition = (payload[1] >> 2) & 0x07
        apply(&mut t, ID_EV_POWERTRAIN, &[0, 2 << 2, 0, 0, 0, 0, 0, 0]);
        assert_eq!(t.gear, 'R');
        // AbsFaultLamp = (payload[3] >> 3) & 0x01
        apply(&mut t, ID_ESP, &[0, 0, 0, 1 << 3, 0, 0, 0, 0]);
        assert_eq!(t.abs, Warn::Red);
        // FrontLeftDoor: byte 0 bit 4; LeftSeatBelt: byte 1 bit 4 (1 = fastened)
        apply(&mut t, ID_BCM_STATUS, &[1 << 4, 1 << 4, 0, 0, 0, 0, 0, 0]);
        assert!(t.door_open);
        assert!(!t.seatbelt);
        apply(&mut t, ID_BCM_STATUS, &[0; 8]);
        assert!(!t.door_open);
        assert!(t.seatbelt, "an unfastened driver's belt is the warning");
        // Turn signals: byte 0 bits 0 hazard, 1 left, 2 right
        apply(&mut t, ID_VEHICLE_STATUS_3, &[0b010, 0, 0, 0, 0, 0, 0, 0]);
        assert!(t.turn_left && !t.turn_right);
        apply(&mut t, ID_VEHICLE_STATUS_3, &[0b001, 0, 0, 0, 0, 0, 0, 0]);
        assert!(t.turn_left && t.turn_right);
        // Gear (532): raw - 1, VSS SelectedGear
        apply(&mut t, ID_TRANSMISSION, &[127, 0, 0, 0, 0, 0, 0, 0]);
        assert_eq!(t.gear, 'P');
        apply(&mut t, ID_TRANSMISSION, &[0, 0, 0, 0, 0, 0, 0, 0]);
        assert_eq!(t.gear, 'R');
    }

    #[test]
    fn roundtrip() {
        let t = Telemetry {
            speed_kmh: 123.45,
            fuel_level: 0.07,
            gear: 'R',
            turn_left: true,
            turn_right: false,
            engine: Warn::Amber,
            abs: Warn::Red,
            seatbelt: true,
            door_open: true,
            ..Telemetry::default()
        };
        let mut r = Telemetry::default();
        for m in encode(&t) {
            assert_eq!(m.data.len(), DLC);
            apply(&mut r, m.id, &m.data);
        }
        assert!((r.speed_kmh - t.speed_kmh).abs() < SPEED_SCALE);
        assert!((r.fuel_level - t.fuel_level).abs() < FUEL_SCALE / 100.0);
        assert_eq!(r.gear, t.gear);
        assert_eq!((r.turn_left, r.turn_right), (true, false));
        assert_eq!(r.engine, Warn::Amber);
        assert_eq!(r.abs, Warn::Red);
        assert!(r.seatbelt && r.door_open);
    }

    #[test]
    fn invalid_and_reserved_shift_positions_keep_the_gear() {
        let mut t = Telemetry {
            gear: 'N',
            ..Telemetry::default()
        };
        for v in [0u8, 6, 7] {
            apply(&mut t, ID_EV_POWERTRAIN, &[0, v << 2, 0, 0, 0, 0, 0, 0]);
            assert_eq!(t.gear, 'N');
        }
    }

    #[test]
    fn short_frames_leave_the_signals_they_cut() {
        let mut t = Telemetry {
            speed_kmh: 50.0,
            ..Telemetry::default()
        };
        // One byte cannot hold the 15-bit speed.
        apply(&mut t, ID_VEHICLE_STATUS_1, &[0x32]);
        assert_eq!(t.speed_kmh, 50.0);
        assert!(!decode(&mut t, 0x7ff, &[0; 8]));
    }
}
