# Diana7 Dual DexH13 Description

Self-contained ROS 1 Catkin description package for Diana7 with native left
and right DexH13 hands. The main model is
`urdf/dual_diana7_dexh13.urdf`; all referenced meshes are included locally.

The URDF includes the Paxini/DH13 tactile geometry and the complete 1140 fixed
taxel links per hand (2280 total). They are generated from the dense site layout
in the original DH13 MuJoCo hand MJCF by `scripts/add_paxini_taxels.py`; the
partial `config/paxini_taxel_positions/*.npy` files are not used. Each taxel is
a small visual sphere attached to the corresponding finger or thumb phalanx,
so ROS consumers can discover the frames directly from the URDF.

`config/tactile_layout.yaml` still contains 22 coarse patch-level channel
entries for compatibility with existing consumers; it is not the dense taxel
index.
