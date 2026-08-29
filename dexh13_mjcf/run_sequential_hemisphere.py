#!/usr/bin/env python3
"""Run the right-hand sequential tactile contact experiment.

The experiment is headless by default; pass ``--viewer`` for a live MuJoCo
window that follows the IK motion and tactile stages.

The full dense model has many taxels per tactile link.  This driver reports
the requested 11 tactile links (patches) while retaining every raw right-hand
taxel value in the JSON output.
"""

from __future__ import annotations

import argparse
import json
import re
import time
from pathlib import Path

import mujoco
import numpy as np


PATCHES = (
    ("index_proximal", "index_ip", ("right_index_joint_1",), 1.10),
    ("index_middle", "index_mp", ("right_index_joint_2",), 1.10),
    ("index_tip", "index_dp", ("right_index_joint_3",), 1.15),
    ("middle_proximal", "middle_ip", ("right_middle_joint_1",), 1.10),
    ("middle_middle", "middle_mp", ("right_middle_joint_2",), 1.10),
    ("middle_tip", "middle_dp", ("right_middle_joint_3",), 1.15),
    ("ring_proximal", "ring_ip", ("right_ring_joint_1",), 1.10),
    ("ring_middle", "ring_mp", ("right_ring_joint_2",), 1.10),
    ("ring_tip", "ring_dp", ("right_ring_joint_3",), 1.15),
    ("thumb_pad", "thumb_pad", ("right_thumb_joint_2",), 1.05),
    ("thumb_tip", "thumb_tip", ("right_thumb_joint_3",), 1.15),
)
PATCH_BY_KEY = {row[1]: row[0] for row in PATCHES}
DEFAULT_ARM_FALLBACK = np.array([1.72, 1.44, 0.55, 2.81, 0.96, -2.22, 1.46], dtype=float)
SENSOR_RE = re.compile(
    r"^right_paxini_taxel_(index|middle|ring|thumb)_(ip|mp|dp|pad|tip)_site_p\d+_touch$"
)
TACTILE_GEOM_RE = re.compile(
    r"^right_paxini_taxel_(index|middle|ring|thumb)_(ip|mp|dp|pad|tip)_site_p\d+_geom$"
)


def name_id(model: mujoco.MjModel, obj: mujoco.mjtObj, name: str) -> int:
    idx = mujoco.mj_name2id(model, obj, name)
    if idx < 0:
        raise ValueError(f"MuJoCo object not found: {name}")
    return idx


def body_ik(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    body_id: int,
    target_pos: np.ndarray,
    target_mat: np.ndarray,
    arm_qpos: np.ndarray,
    arm_joint_ids: list[int],
    iterations: int = 300,
) -> tuple[np.ndarray, float]:
    """Damped least-squares IK for the palm, with position and orientation."""
    qadr = np.array([model.jnt_qposadr[j] for j in arm_joint_ids], dtype=int)
    dof = np.array([model.jnt_dofadr[j] for j in arm_joint_ids], dtype=int)
    data.qpos[qadr] = arm_qpos
    data.qvel[:] = 0
    mujoco.mj_forward(model, data)
    jacp = np.zeros((3, model.nv))
    jacr = np.zeros((3, model.nv))
    last_err = float("inf")
    for _ in range(iterations):
        mujoco.mj_forward(model, data)
        mujoco.mj_jacBody(model, data, jacp, jacr, body_id)
        pos_err = target_pos - data.xpos[body_id]
        rot_err = 0.5 * np.array(
            [
                target_mat[2, 1] * data.xmat[body_id, 6]
                - target_mat[1, 2] * data.xmat[body_id, 3],
                target_mat[0, 2] * data.xmat[body_id, 0]
                - target_mat[2, 0] * data.xmat[body_id, 6],
                target_mat[1, 0] * data.xmat[body_id, 3]
                - target_mat[0, 1] * data.xmat[body_id, 0],
            ]
        )
        # A direct matrix skew error is well behaved for this small rotation.
        current_mat = data.xmat[body_id].reshape(3, 3)
        rot_err = 0.5 * np.array(
            [
                (target_mat @ current_mat.T - current_mat @ target_mat.T)[2, 1],
                (target_mat @ current_mat.T - current_mat @ target_mat.T)[0, 2],
                (target_mat @ current_mat.T - current_mat @ target_mat.T)[1, 0],
            ]
        )
        err = np.concatenate((pos_err, 0.35 * rot_err))
        err_norm = float(np.linalg.norm(err))
        if err_norm < 1e-5:
            break
        j = np.vstack((jacp[:, dof], 0.35 * jacr[:, dof]))
        damping = 2e-3
        dq = j.T @ np.linalg.solve(j @ j.T + damping**2 * np.eye(6), err)
        dq = np.clip(dq, -0.08, 0.08)
        data.qpos[qadr] += dq
        for i, jid in enumerate(arm_joint_ids):
            lo, hi = model.jnt_range[jid]
            data.qpos[qadr[i]] = np.clip(data.qpos[qadr[i]], lo, hi)
        last_err = err_norm
    mujoco.mj_forward(model, data)
    return data.qpos[qadr].copy(), last_err


def sensor_groups(model: mujoco.MjModel) -> tuple[dict[str, np.ndarray], list[str], list[int]]:
    groups: dict[str, list[int]] = {row[1]: [] for row in PATCHES}
    names: list[str] = []
    ids: list[int] = []
    for sid in range(model.nsensor):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_SENSOR, sid) or ""
        match = SENSOR_RE.match(name)
        if match:
            key = f"{match.group(1)}_{match.group(2)}"
            groups.setdefault(key, []).append(sid)
            names.append(name)
            ids.append(sid)
    return {key: np.asarray(value, dtype=int) for key, value in groups.items()}, names, ids


def pose_residual(data: mujoco.MjData, body_id: int, target_pos: np.ndarray, target_mat: np.ndarray) -> float:
    current = data.xmat[body_id].reshape(3, 3)
    skew = target_mat @ current.T - current @ target_mat.T
    rot = 0.5 * np.array([skew[2, 1], skew[0, 2], skew[1, 0]])
    return float(np.linalg.norm(np.r_[target_pos - data.xpos[body_id], 0.35 * rot]))


def run(args: argparse.Namespace) -> dict:
    model = mujoco.MjModel.from_xml_path(str(args.scene))
    data = mujoco.MjData(model)
    viewer = None
    if args.viewer:
        # macOS needs this script to be launched with `mjpython` so GLFW can
        # run its application event loop in the main process.
        from mujoco import viewer as mujoco_viewer

        viewer = mujoco_viewer.launch_passive(model, data)
        viewer.cam.lookat[:] = (0.347, -0.40, 0.30)
        viewer.cam.distance = 1.75
        viewer.cam.azimuth = 120.0
        viewer.cam.elevation = -25.0
        viewer.sync()
    palm_id = name_id(model, mujoco.mjtObj.mjOBJ_BODY, "right_palm_link")
    arm_joints = [name_id(model, mujoco.mjtObj.mjOBJ_JOINT, f"right_arm_joint_{i}") for i in range(1, 8)]
    arm_qadr = np.array([model.jnt_qposadr[j] for j in arm_joints], dtype=int)
    arm_dof = np.array([model.jnt_dofadr[j] for j in arm_joints], dtype=int)
    joint_ids = {mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i): i for i in range(model.njnt)}
    act_for_joint: dict[str, int] = {}
    for aid in range(model.nu):
        aname = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, aid) or ""
        if aname.startswith("ctrl_right_"):
            jid = int(model.actuator_trnid[aid, 0])
            jname = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, jid)
            if jname:
                act_for_joint[jname] = aid
    groups, taxel_names, taxel_ids = sensor_groups(model)
    if len(taxel_ids) != 1140:
        raise RuntimeError(f"Expected 1140 right-hand taxels, found {len(taxel_ids)}")
    tactile_geoms: dict[str, list[int]] = {row[1]: [] for row in PATCHES}
    for gid in range(model.ngeom):
        gname = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, gid) or ""
        match = TACTILE_GEOM_RE.match(gname)
        if match:
            tactile_geoms.setdefault(f"{match.group(1)}_{match.group(2)}", []).append(gid)
    original_contype = model.geom_contype.copy()
    original_conaffinity = model.geom_conaffinity.copy()

    # The target makes palm local Y vertical and local Z point toward -Y.
    target_pos = np.array([args.palm_x, args.palm_y, args.palm_z], dtype=float)
    target_mat = np.array(((1.0, 0.0, 0.0), (0.0, 0.0, -1.0), (0.0, 1.0, 0.0)))
    open_q = data.qpos.copy()
    # The zero pose is close to a kinematic singularity.  For the default
    # front-platform target, solve again from a known reachable arm posture;
    # this is important after moving the experiment datum from x=0 to x=0.347.
    ik_seeds = [open_q[arm_qadr]]
    if np.allclose(target_pos, (0.247, -0.40, 0.35), atol=1e-9):
        ik_seeds.append(DEFAULT_ARM_FALLBACK.copy())
    candidate_errors = []
    for seed in ik_seeds:
        candidate_q, candidate_error = body_ik(
            model, data, palm_id, target_pos, target_mat, seed, arm_joints
        )
        candidate_errors.append((candidate_q, candidate_error))
    arm_target, ik_error = min(candidate_errors, key=lambda item: item[1])
    # Build one reachable wrist pose per patch.  The 20 mm hemisphere is much
    # smaller than the span of the 11 links, so each link gets its own precise
    # alignment while the hand orientation remains parallel to the cylinder.
    base_arm_target = arm_target.copy()
    data.qpos[arm_qadr] = base_arm_target
    mujoco.mj_forward(model, data)
    sphere_centre = np.array([0.347, -0.40, 0.25], dtype=float)
    # Put the taxel sphere centres at the hemisphere centre.  This gives a
    # small, deterministic overlap (the sensor radius is 0.7 mm) instead of
    # relying on a grazing surface contact while the arm settles.
    contact_point = sphere_centre.copy()
    stage_targets = {}
    stage_ik_errors = {}
    for patch_name, patch_key, joints, final_angle in PATCHES:
        # Measure the patch after its own closure angle, since phalange motion
        # can translate a taxel by several centimetres relative to the open hand.
        data.qpos[:] = open_q
        data.qpos[arm_qadr] = base_arm_target
        for joint in joints:
            data.qpos[model.jnt_qposadr[joint_ids[joint]]] = final_angle
        mujoco.mj_forward(model, data)
        sids = groups[patch_key]
        site_ids = [int(model.sensor_objid[sid]) for sid in sids]
        closed_patch_centre = np.mean(data.site_xpos[site_ids], axis=0)
        stage_pos = target_pos + contact_point - closed_patch_centre
        q_stage, e_stage = body_ik(
            model, data, palm_id, stage_pos, target_mat, base_arm_target, arm_joints
        )
        stage_targets[patch_name] = q_stage
        stage_ik_errors[patch_name] = e_stage
    # Reset after all IK solves so the physical run starts from the model pose.
    mujoco.mj_resetData(model, data)
    open_hand = {name: float(data.qpos[model.jnt_qposadr[jid]]) for name, jid in joint_ids.items() if name.startswith("right_") and "_arm_" not in name}
    for name, aid in act_for_joint.items():
        if name in open_hand:
            data.ctrl[aid] = open_hand[name]
    for i, aid in enumerate([act_for_joint[f"right_arm_joint_{j}"] for j in range(1, 8)]):
        data.ctrl[aid] = float(data.qpos[arm_qadr[i]])
    mujoco.mj_forward(model, data)

    threshold = float(args.threshold)
    stage_time = float(args.stage_time)
    dt = float(model.opt.timestep)
    stage_records = []
    stage_raw_values = []
    contact_seen: dict[tuple[str, str], float] = {}
    first_contact: dict[str, dict] = {}
    previous_q = data.qpos[arm_qadr].copy()

    def patch_stats(values: np.ndarray) -> dict[str, float | int]:
        return {"active_taxel_count": int(np.count_nonzero(values > threshold)), "peak_sensor_value": float(np.max(values)) if values.size else 0.0}

    def read_stats(enabled_key: str | None = None) -> tuple[dict[str, dict], np.ndarray]:
        raw = data.sensordata[np.asarray(taxel_ids, dtype=int)].copy()
        stats = {}
        if args.isolate_patches and enabled_key is not None:
            for i, sid in enumerate(taxel_ids):
                name = taxel_names[i]
                match = SENSOR_RE.match(name)
                if match and f"{match.group(1)}_{match.group(2)}" != enabled_key:
                    raw[i] = 0.0
        for key, sids in groups.items():
            # sensor IDs are global; map them back to the dense right-hand array.
            if args.isolate_patches and enabled_key is not None and key != enabled_key:
                values = np.zeros(0)
            elif len(sids):
                values = data.sensordata[sids]
            else:
                values = np.zeros(0)
            stats[key] = patch_stats(values)
        return stats, raw

    def command(t: float, arm_q: np.ndarray, active_targets: dict[str, float]) -> None:
        for i, j in enumerate(range(1, 8)):
            data.ctrl[act_for_joint[f"right_arm_joint_{j}"]] = float(arm_q[i])
        for name, aid in act_for_joint.items():
            if "_arm_" in name:
                continue
            target = open_hand.get(name, float(data.qpos[model.jnt_qposadr[joint_ids[name]]]))
            if name in active_targets:
                target = active_targets[name]
            data.ctrl[aid] = target

    sim_time = 0.0
    step_count = 0

    def step_simulation() -> None:
        mujoco.mj_step(model, data)
        if viewer is not None and viewer.is_running():
            viewer.sync()

    closure_targets: dict[str, float] = {}
    # Approach smoothly in arm joint space; fingers remain open.
    while sim_time < args.approach_time:
        alpha = min(1.0, sim_time / max(args.approach_time, dt))
        command(sim_time, (1 - alpha) * previous_q + alpha * arm_target, closure_targets)
        step_simulation()
        sim_time = float(data.time)
        step_count += 1
    previous_q = arm_target.copy()
    # Give the position actuators a short settling interval before closing.
    settle_end = sim_time + args.settle_time
    while sim_time < settle_end:
        command(sim_time, previous_q, closure_targets)
        step_simulation()
        sim_time = float(data.time)
        step_count += 1

    reposition_time = float(args.reposition_time)
    for stage_index, (patch_name, patch_key, joints, final_angle) in enumerate(PATCHES):
        stage_target = stage_targets[patch_name]
        # Open every link before each isolated trial and move the wrist to the
        # selected tactile patch.  This prevents a previously closed link from
        # masking the requested one.
        closure_targets: dict[str, float] = {}
        if args.isolate_patches:
            for key, geom_ids in tactile_geoms.items():
                enabled = key == patch_key
                for gid in geom_ids:
                    model.geom_contype[gid] = 1 if enabled else 0
                    model.geom_conaffinity[gid] = 1 if enabled else 0
        move_start = sim_time
        move_end = move_start + reposition_time
        while sim_time < move_end:
            alpha = min(1.0, (sim_time - move_start) / max(reposition_time, dt))
            command(sim_time, (1 - alpha) * previous_q + alpha * stage_target, closure_targets)
            step_simulation()
            sim_time = float(data.time)
            step_count += 1
        previous_q = stage_target.copy()
        if args.snap_stage:
            # Finish each slow approach at the analytically solved pose.  This
            # removes actuator tracking error from the millimetre-scale contact
            # measurement while preserving the visible/reproducible approach.
            data.qpos[arm_qadr] = stage_target
            data.qvel[arm_dof] = 0.0
            mujoco.mj_forward(model, data)
        stage_start = sim_time
        stage_end = stage_start + stage_time
        local_peak = {key: {"active_taxel_count": 0, "peak_sensor_value": 0.0} for key in groups}
        stage_first_contact = None
        unexpected = set()
        while sim_time < stage_end:
            alpha = min(1.0, (sim_time - stage_start) / max(stage_time, dt))
            active = {}
            for joint in joints:
                active[joint] = (1 - alpha) * open_hand[joint] + alpha * final_angle
            command(sim_time, previous_q, active)
            step_simulation()
            sim_time = float(data.time)
            step_count += 1
            stats, _ = read_stats(patch_key)
            for contact_index in range(data.ncon):
                contact = data.contact[contact_index]
                geom_names = tuple(
                    mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, int(g)) or str(g)
                    for g in (contact.geom1, contact.geom2)
                )
                if any("experiment_" in name for name in geom_names):
                    contact_seen[geom_names] = min(float(contact.dist), contact_seen.get(geom_names, float("inf")))
            for key, current in stats.items():
                peak = local_peak[key]
                peak["active_taxel_count"] = max(int(peak["active_taxel_count"]), int(current["active_taxel_count"]))
                peak["peak_sensor_value"] = max(float(peak["peak_sensor_value"]), float(current["peak_sensor_value"]))
                if current["active_taxel_count"] and key not in first_contact:
                    first_contact[key] = {
                        "first_contact_time_s": sim_time,
                        "active_taxel_count": int(current["active_taxel_count"]),
                        "peak_sensor_value": float(current["peak_sensor_value"]),
                        "stage": patch_name,
                    }
                if current["active_taxel_count"]:
                    if key == patch_key and stage_first_contact is None:
                        stage_first_contact = sim_time
                    elif key != patch_key:
                        unexpected.add(PATCH_BY_KEY.get(key, key))
        stage_records.append(
            {
                "stage": patch_name,
                "stage_index": stage_index,
                "reposition_time_s": reposition_time,
                "closure_time_s": stage_time,
                "target_arm_q": stage_target.tolist(),
                "ik_error": stage_ik_errors[patch_name],
                "target_patch": patch_name,
                "target_patch_first_contact_time_s": stage_first_contact,
                "unexpected_active_patches": sorted(unexpected),
                "patch_stats": local_peak,
            }
        )
        _, stage_raw = read_stats(patch_key)
        stage_raw_values.append({"stage": patch_name, "values": {name: float(value) for name, value in zip(taxel_names, stage_raw)}})

    model.geom_contype[:] = original_contype
    model.geom_conaffinity[:] = original_conaffinity
    mujoco.mj_forward(model, data)
    if viewer is not None:
        viewer.sync()

    final_stats, final_raw = read_stats()
    contacts = [{"geom1": pair[0], "geom2": pair[1], "distance_m": distance} for pair, distance in contact_seen.items()]
    output = {
        "scene": str(args.scene),
        "units": {"length": "m", "time": "s"},
        "geometry": {"cylinder_diameter_m": 0.10, "cylinder_length_m": 0.20, "hemisphere_radius_m": 0.020},
        "palm_target": {"position_m": target_pos.tolist(), "rotation_matrix": target_mat.tolist(), "ik_error": ik_error},
        "simulation": {"duration_s": sim_time, "steps": step_count, "timestep_s": dt, "threshold": threshold, "snap_stage_pose": bool(args.snap_stage), "isolate_target_patch": bool(args.isolate_patches)},
        "first_contact_by_patch": [
            {"patch": name, **first_contact[patch_key]}
            if patch_key in first_contact
            else {"patch": name, "first_contact_time_s": None, "active_taxel_count": 0, "peak_sensor_value": 0.0}
            for name, patch_key, _, _ in PATCHES
        ],
        "stage_records": stage_records,
        "stage_raw_right_taxel_values": stage_raw_values,
        "final_patch_stats": {PATCH_BY_KEY.get(key, key): value for key, value in final_stats.items()},
        "final_contacts_with_object": contacts,
        "raw_right_taxel_values": {name: float(value) for name, value in zip(taxel_names, final_raw)},
        "final_palm_position_m": data.xpos[palm_id].tolist(),
        "final_palm_rotation_matrix": data.xmat[palm_id].reshape(3, 3).tolist(),
    }
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(output, indent=2), encoding="utf-8")
    if viewer is not None:
        print("Simulation complete. Close the MuJoCo window to exit.")
        while viewer.is_running():
            viewer.sync()
            time.sleep(0.02)
        viewer.close()
    return output


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", type=Path, default=Path(__file__).with_name("sequential_hemisphere_scene.xml"))
    parser.add_argument("--out", type=Path, default=Path("/tmp/dexh13_sequential_hemisphere.json"))
    parser.add_argument("--threshold", type=float, default=1e-6)
    parser.add_argument("--approach-time", type=float, default=3.0)
    parser.add_argument("--settle-time", type=float, default=0.5)
    parser.add_argument("--reposition-time", type=float, default=1.5)
    parser.add_argument("--stage-time", type=float, default=1.5)
    parser.add_argument("--viewer", action="store_true",
                        help="show and continuously sync the MuJoCo Viewer while the experiment runs")
    parser.add_argument("--no-snap-stage", dest="snap_stage", action="store_false", default=True,
                        help="keep purely actuator-driven stage motion (less precise contact alignment)")
    parser.add_argument("--no-isolate-patches", dest="isolate_patches", action="store_false", default=True,
                        help="allow natural contact from neighboring tactile patches")
    parser.add_argument("--palm-x", type=float, default=0.247)
    parser.add_argument("--palm-y", type=float, default=-0.40)
    parser.add_argument("--palm-z", type=float, default=0.35)
    args = parser.parse_args()
    result = run(args)
    print(json.dumps({"out": str(args.out), "duration_s": result["simulation"]["duration_s"], "first_contact_by_patch": result["first_contact_by_patch"]}, indent=2))


if __name__ == "__main__":
    main()
