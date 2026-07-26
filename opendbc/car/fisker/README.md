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

## Placeholder / not production-safe yet

- `allOutput` safety mode is temporary and bypasses a brand-specific panda safety policy.
- ACC state decode and stalk button mapping are provisional.
- accel payload conversion is placeholder-only.
- steering and longitudinal limits remain placeholder values.
- radar object tracking remains undecoded.

## Next Calibration Priorities

1. Confirm ACC state bits in `ADAS_ACC` against live logs.
2. Calibrate `ADAS_ACCEL_CONTROL.PAYLOAD` vs measured acceleration.
3. Validate left-stalk cruise mapping and cancel behavior.
4. Replace `allOutput` with a real `SAFETY_FISKER` panda mode.
5. Decode radar tracks and remove `radarUnavailable` stub behavior.
