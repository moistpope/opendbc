# Fisker Ocean Integration Status

This document tracks the current bring-up state of the Fisker Ocean integration.

## What Was Added

### 1. Longitudinal and lateral path enabled with placeholders

The interface now enables development-time control paths:

- safety mode switched to `allOutput` (temporary, development only)
- `dashcamOnly = False`
- `openpilotLongitudinalControl = True`
- `pcmCruise = False`
- placeholder long tuning and start/stop parameters

Why: this allows long/lat command generation and validation while real calibration is still being reverse engineered.

### 2. Placeholder ACC/cruise state and button mapping

`CarState` now includes provisional decode paths for:

- ACC availability/enabled from `ADAS_ACC.NEW_SIGNAL_1`
- cruise button events from `NEW_MSG_310.LEFT_STALK`

These are explicitly placeholders and should be refined with real drive captures.

### 3. Placeholder accel payload scaling

`CarController` now maps requested acceleration (`m/s^2`) into `ADAS_ACCEL_CONTROL.PAYLOAD` using a centered 12-bit placeholder transform:

- neutral payload: `2048`
- linear gain: `300 payload / (m/s^2)`
- payload clamp: `[0, 4095]`

This is not calibrated and is expected to need tuning.

### 4. Startup SecOC key recovery daemon

A new startup process `fisker_secoc_keyd` was added.

Behavior:

- runs onroad startup
- activates only when detected `CarParams.brand == "fisker"` and `secOcRequired == True`
- queries multiple radar modules (`MRR`, `CMRR_FL`, `CMRR_FR`, `CMRR_RL`, `CMRR_RR`)
- reads candidate DID(s) for SecOC material (currently `0xEFF5`)
- extracts 16-byte key candidates from responses
- verifies key agreement across multiple modules
- stores key only if at least two modules match exactly
- persists to both param store (`SecOCKey`) and `/cache/params/SecOCKey`

If modules disagree, no key is stored.

## Current Operational State

## Working now

- Fisker interface enters non-dashcam mode for development builds.
- Longitudinal and lateral command generation is active in the control stack.
- SecOC signing pipeline remains in place for steer/accel messages.
- Automatic SecOC key retrieval attempts happen at startup on Fisker.
- Lateral takeover emits the full pair the EPS requires: `ADAS_STEER_CONTROL` (0x1D0, SecOC) **and**
  `LKAS_STEER_AUTHORITY` (0x1C0, non-SecOC, CRC over bytes 1-7 xor-out 0x03, all validity fields
  set). Missing 0x1C0 makes the EPS raise `U12F786`/`U12F787` and ignore steering.
- ARC and SecOC message counters are seeded from the stock Hydra module and continued without a
  discontinuity at takeover (`secoc_mirror.py`): the full SecOC running counter is reconstructed the
  way the receiver does (past the 6-bit wire wrap), not read raw, so our MAC matches the EPS's
  expected freshness. Counters restart at 1 on each `GW_SECOC_SYNC` reset epoch.

## Placeholder / not production-safe yet

- `SAFETY_FISKER` (`opendbc/safety/modes/fisker.h`) enforces the lateral limits, but the limits
  themselves (steer torque cap/rates, RT delta, driver-torque allowance) are provisional and the
  driver-torque calibration is unverified.
- ACC state decode and stalk button mapping are provisional.
- accel payload conversion is placeholder-only.
- steering and longitudinal limits remain placeholder values.
- radar object tracking remains undecoded.
- the `0x1C0` payload is confirmed from capture: idle `b2=0x48 b3=0x10`, LKAS-active `b2=0x49 b3=0x51`
  (StsVld=`b2&0x01`, DrvrOvrdVld=`b3&0x01`, ReqVld=`b3&0x40`); the specific field-to-bit name mapping
  is a best guess but all three flip together to "valid".

## Next Calibration Priorities

1. Confirm ACC state bits in `ADAS_ACC` against live logs.
2. Calibrate `ADAS_ACCEL_CONTROL.PAYLOAD` vs measured acceleration.
3. Validate left-stalk cruise mapping and cancel behavior.
4. Tune the `SAFETY_FISKER` steering limits and driver-torque calibration against real data.
5. Decode radar tracks and remove `radarUnavailable` stub behavior.
