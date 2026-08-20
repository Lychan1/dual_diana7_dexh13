# Diana7 Dual DexH13

This repository contains the Diana7 dual-arm ROS description and its MuJoCo
MJCF model with the complete DH13 tactile layout.

## Repository layout

- `diana7_dual_dexh13_description/`: ROS 1 Catkin package, meshes, and the
  dual-arm URDF.
- `dexh13_mjcf/`: dual-arm MJCF, viewer/test scenes, generator, and the
  repository-local 1140-site DH13 layout reference.

The tactile model has 1140 taxel frames and 1140 touch sensors per hand, 2280
for the two-hand model. The old `*.npy` files were incomplete and are not part
of the generation path.

## Regenerate

Run from the repository root:

```bash
python3 diana7_dual_dexh13_description/scripts/add_paxini_taxels.py
python3 dexh13_mjcf/build_dual_mjcf.py
```

The generated MJCF can be opened with:

```bash
python3 -m mujoco.viewer --mjcf dexh13_mjcf/dual_diana7_dexh13.xml
```

## Known issue

The DH13 L-shaped flange mating pose in the dual-arm URDF is still under
review. The current tactile and MJCF work preserves that pose so the flange
connection can be corrected separately without mixing it with the taxel
conversion.
