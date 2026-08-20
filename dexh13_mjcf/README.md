# DexH13 MuJoCo model

`dual_diana7_dexh13.xml` is the main MuJoCo MJCF model for the complete
Diana7 dual-arm system. It is generated from the tactile URDF and includes both
7-DOF arms, both DH13 hands, the supplied meshes, 46 hinge actuators, and 2280
dense Paxini taxel sites with matching MuJoCo touch sensors (1140 per hand).
The standalone viewer starts with gravity disabled and only the tactile shells
enabled for collision, which keeps the URDF zero pose stable and avoids mesh
self-collision. For a gravity-driven experiment, set
`m.opt.gravity[:] = (0, 0, -9.81)` after selecting a controlled initial pose.

Regenerate the taxelized URDF and then the dual-arm MJCF from the repository
root with:

```bash
python diana7_dual_dexh13_description/scripts/add_paxini_taxels.py \
  --source-mjcf dexh13_mjcf/reference/dh13_taxel_layout.xml
python dexh13_mjcf/build_dual_mjcf.py
```

Load it directly in the MuJoCo viewer:

```bash
python -m mujoco.viewer --mjcf dexh13_mjcf/dual_diana7_dexh13.xml
```

`hand.xml` remains a smaller self-contained MuJoCo MJCF model for the right
DexH13 hand and its patch-level press-test scenes. The full 1140-site layout is
vendored as `reference/dh13_taxel_layout.xml`, extracted from the original DH13
MJCF. Both models use the STL
meshes shipped in `../diana7_dual_dexh13_description/meshes`, so they do not
depend on the original `dexh13_raw` export directory.

The smaller `hand.xml` model exposes one patch-level tactile channel for each of the 11 right-hand
tactile pads:

- a box `site` named like `right_index_proximal_site`, sized from
  `config/tactile_layout.yaml`;
- a MuJoCo `<touch>` sensor named like `right_index_proximal_touch`.

This is a patch-level taxel model: each pad is one taxel/channel. It is not a
dense 4x4 or 16x16 taxel array. The existing `site_*_sensor_frame` sites are
kept as small frame markers for compatibility with the original hand model.

The scene files in this directory include `hand.xml`, for example:

```bash
python -m mujoco.viewer --mjcf dexh13_mjcf/hand.xml
```

The scene files can be opened in the same way when an object or contact test is
needed. Sensor values are available through the names in the `<sensor>` block,
or through their numeric indices in `data.sensordata`.
