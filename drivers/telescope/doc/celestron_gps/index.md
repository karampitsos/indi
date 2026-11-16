# Celestron GPS

## Device Overview

`indi_celestron_gps` controls Celestron mounts through the NexStar or StarSense hand controller.
It supports mounts such as the CPC, CGX, CGEM, AVX, Evolution and NexStar SE series, on
equatorial (GEM and wedge) and Alt-Az setups.

## Features

- GoTo, sync, park and unpark, abort
- Manual motion at the hand controller slew rates
- Tracking on and off, with Sidereal, Solar and Lunar rates on equatorial mounts
- **Custom tracking:** continuous signed rates on both motors (see below)
- Pulse guiding (Celestron AUX guiding on recent motor firmware)
- Time, date and location from the mount
- PEC recording and playback on supported mounts
- Focuser on the AUX bus (Celestron focus motor)
- Az/Alt reported in `HORIZONTAL_COORD`
- Simulation mode

Which features are offered depends on the hand controller firmware; the driver logs a warning
when the firmware is too old for one.

## Installation

The driver ships with INDI core and needs nothing extra. Connect the hand controller by USB or a
serial adapter and start it with:

```shell
indiserver -v indi_celestron_gps
```

In KStars/Ekos choose **Celestron › Celestron GPS** (or the entry for your hand controller).

## Configuration

- **Connection:** serial, 9600 baud by default (USB-serial adapters appear as `/dev/ttyUSB*`),
  or TCP for network adapters.
- **Mount Info › Track Mode** shows the Celestron track mode read from the hand controller:
  Alt-Az, EQ North, EQ South or RA and Dec (StarSense). Set it in the hand controller during
  alignment.
- **Options › Hibernate** makes Park hibernate the mount, so it can be woken up without a new
  alignment (firmware 4.22 and later).

## Custom tracking

Custom tracking drives both motors at continuous, signed rates. It is meant for clients that
compute the motion themselves, for example satellite trackers. Requires hand controller firmware
2.3 or later.

1. Set `TELESCOPE_TRACK_MODE` to `TRACK_CUSTOM`.
2. Set the rates in `TELESCOPE_TRACK_RATE`, in arcseconds per second.
3. Set `TELESCOPE_TRACK_STATE` to `TRACK_ON`.

The rates are **axis rates**, not sky rates:

| Element | Alt-Az mount | Equatorial mount |
|---|---|---|
| `TRACK_RATE_RA` | azimuth motor | RA motor |
| `TRACK_RATE_DE` | altitude motor | Dec motor |

The sign gives the motor direction. On an equatorial mount the RA motor does not add sidereal
tracking on top: about +15.04 arcsec/s on an EQ North mount (−15.04 on EQ South) tracks the stars.

The limit is ±16383 arcsec/s (about 4.55°/s) per axis, which is what the mount's variable-rate
command can carry.

While custom tracking is on, the mount's own tracking is off. Selecting Sidereal (or Solar or
Lunar on equatorial mounts) gives tracking back to the mount. On Alt-Az mounts only Sidereal and
Custom can be selected.

Rates can be changed while tracking. As for every INDI mount, the base driver refuses to reverse
the sign of a rate while tracking; switch tracking off, set the new rates and switch it on again.

Abort, GoTo and Park stop the custom rates. After a GoTo, tracking resumes at the custom rates
that are set.

## Simulation

Enable **Options › Simulation** before connecting. The simulated mount slews, tracks and parks,
and moves its axes at the commanded rates in custom tracking. **Options › Sim Track Mode**
chooses whether it behaves as an Alt-Az, EQ North or EQ South mount; it takes effect when
connecting.

## Usage & Tips

- If the driver can't connect, check that the hand controller is past its start-up questions
  and that no other program holds the port.
- For custom tracking on an Alt-Az mount, align the mount in the hand controller first, so
  `HORIZONTAL_COORD` reports a true azimuth and altitude.
- Enable **Debug** and the **Scope Verbose** level to log every command sent to the mount.
