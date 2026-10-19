# LX200 GPS

## Device Overview

`indi_lx200gps` controls Meade LX200 GPS and Autostar II mounts, and other controllers that
implement the LX200 GPS serial commands. It is built from the LX200 generic driver.

## Features

- GoTo, sync, park and unpark, abort
- Manual motion and pulse guiding
- Tracking on and off, with Sidereal, Solar and Lunar rates
- **Custom tracking:** continuous signed rates on both axes (see below)
- Alt/Az reported in `HORIZONTAL_COORD`, when the controller answers `:GA#` and `:GZ#`
- GPS, PEC and anti-backlash controls, focuser
- Simulation mode

## Installation

The driver ships with INDI core. Connect the mount by serial or USB-serial and start it with:

```shell
indiserver -v indi_lx200gps
```

In KStars/Ekos choose **Meade › LX200 GPS**.

## Tracking on and off

Tracking off switches the mount to the Land alignment (`:AL#`), in which it does not track.
Tracking on switches it back to the alignment shown in **Alignment** (`:AA#` or `:AP#`) and selects
the track rate. If the mount is in Land alignment, select AltAz or Polar first.

## Custom tracking

Custom tracking moves both axes at continuous, signed rates. It is meant for clients that compute
the motion themselves, for example satellite trackers.

1. Set `TELESCOPE_TRACK_MODE` to `TRACK_CUSTOM`.
2. Set the rates in `TELESCOPE_TRACK_RATE`, in arcseconds per second.
3. Set `TELESCOPE_TRACK_STATE` to `TRACK_ON`.

The rates are **axis rates**, not sky rates:

| Element | Positive rate on an Alt-Az mount | Positive rate on an equatorial mount |
|---|---|---|
| `TRACK_RATE_RA` | west move (`:Mw#`): azimuth increases | west move: with the sky |
| `TRACK_RATE_DE` | north move (`:Mn#`): altitude increases | north move: declination increases |

A rate of 0 stops that axis. The limits are ±16384 arcsec/s (about 4.55°/s); the mount's own
maximum slew rate may be lower.

While custom tracking is on, the mount's own tracking is off (Land alignment), so it does not add
to the rates. Selecting another track mode gives tracking back to the mount. Abort, Park, GoTo and
disconnecting stop the axes; after a GoTo, custom tracking resumes when the slew completes.

As in other INDI drivers, a rate can't change sign while tracking: switch tracking off, set the
new rates and switch tracking on again.

### Serial commands

| Action | Commands |
|---|---|
| Mount tracking off / on | `:AL#` / `:AA#` or `:AP#` |
| Axis rate, degrees per second | `:RAd.dddd#` (RA/Az axis), `:REd.dddd#` (Dec/Alt axis) |
| Move at that rate | `:Mw#` / `:Me#` (RA/Az axis), `:Mn#` / `:Ms#` (Dec/Alt axis) |
| Stop one direction / all | `:Qw#` `:Qe#` `:Qn#` `:Qs#` / `:Q#` |
| Position | `:GA#`, `:GZ#` (Alt/Az), `:GR#`, `:GD#` (RA/Dec) |

The rate is sent as an unsigned number with four decimals, for example `:RA0.1000#` for
360 arcsec/s. That is more precise than the `DD.D` in the Meade protocol; a controller that only
reads one decimal moves in steps of 0.1°/s. The move command is sent again with every rate, so a
controller may apply the new rate either immediately or at the next move command.

## Simulation

In simulation, custom tracking integrates the axis rates: on the azimuth and altitude axes in the
AltAz and Land alignments, and on the RA and Dec axes in the Polar alignment.
