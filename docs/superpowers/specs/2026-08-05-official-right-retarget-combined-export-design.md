# Official Right-Retarget Combined Export Design

## Goal

Rename the incorrect numeric left-to-right mirror package with an `UNUSABLE_numeric_mirror` marker. Create a separate 200 Hz candidate combining mirrored arm targets with a right-hand trajectory produced by the official `Handedness.Right` retargeter from the original right-glove skeleton. This is offline data only and never commands hardware.

## Transform

- Rename the old NPZ and CSV without changing their bytes.
- Read the 200 Hz source arms, exchange them, then apply `(-1,+1,-1,+1,-1,+1,-1)`.
- Read `august_02_origin/session_20260802_174440_936.mcap`; feed its `/right_glove/hand_skeleton` directly to `RetargetSession.for_hand(WujiHand, Handedness.Right)`, with no wrist Y mirror.
- Clip only floating-point boundary excess to official right-MJCF control ranges, then linearly resample right-hand targets to the source 5 ms clock.

## Outputs and Validation

Write `recorded_hand_guarded_chop_200hz_official_right_retarget_candidate.npz` and CSV. Validate finite data, an equal 831-sample 5 ms clock, all official right-hand ranges, exact arm transform identities, and metadata identifying the package as offline candidate data rather than a physical command or safety certificate.
