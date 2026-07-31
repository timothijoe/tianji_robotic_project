# Wuji Hand model

This directory vendors the original left Wuji Hand MuJoCo model from the
`wuji-technology/wuji-description` project.

- Source in the local download:
  `/home/linux/august_folder/wuji-description/hand/body/`
- Variant: original Wuji Hand, left side
- Imported: 2026-07-31
- License: MIT; see `LICENSE`

`left_hand.xml` keeps the upstream body, joint, inertia, collision, actuator,
and ordering data. Its mesh path is adjusted for this repository and its
top-level simulation option is omitted so the parent chopping scene remains in
control of the solver and integrator.

Runtime simulation uses only the files in this directory. It does not require
the downloaded `wuji-description` or `wuji-mjlab` repositories.
