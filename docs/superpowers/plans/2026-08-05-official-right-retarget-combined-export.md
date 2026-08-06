# Official Right-Retarget Combined Export Implementation Plan

**Goal:** Retire the bad numeric mirror and create an offline 200 Hz arm-plus-official-right-hand candidate.

### Task 1: Generate and validate local data

- [ ] Rename the old NPZ/CSV to `recorded_hand_guarded_chop_200hz_UNUSABLE_numeric_mirror.*`.
- [ ] Retarget the raw right skeleton with `Handedness.Right`, clip only model-boundary floating-point noise, interpolate to the source 5 ms clock, and combine it with the specified mirrored arm arrays.
- [ ] Write `recorded_hand_guarded_chop_200hz_official_right_retarget_candidate.npz/csv` with candidate/offline metadata.
- [ ] Verify 831 samples, exact 5 ms spacing, right MJCF ranges, and exact arm transform identities; do not connect hardware.
