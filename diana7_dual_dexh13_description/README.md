# Diana7 Dual DexH13 Description

Self-contained ROS 1 Catkin description package for Diana7 with native left
and right DexH13 hands. The main model is
`urdf/dual_diana7_dexh13.urdf`; all referenced meshes are included locally.

The current package contains URDF semantics only. The 22 entries in
`config/tactile_layout.yaml` define tactile channel names and frames for a
later simulator integration; they are not active sensors.
