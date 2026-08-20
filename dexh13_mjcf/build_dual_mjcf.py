#!/usr/bin/env python3
"""Convert the tactile dual-arm URDF into a self-contained MuJoCo MJCF.

The generated model keeps the URDF link/joint tree, reuses the package meshes,
and turns each generated Paxini URDF taxel frame into a MuJoCo sphere site and
touch sensor.  The URDF is deliberately the source of truth for transforms;
the right-only ``hand.xml`` remains a small reference/press-test model.
"""

from __future__ import annotations

import argparse
import math
import re
import xml.etree.ElementTree as ET
from collections import defaultdict
from pathlib import Path


PACKAGE_PREFIX = "package://diana7_dual_dexh13_description/"
SKIP_LINK_RE = re.compile(r"(?:_paxini_taxel_|_tactile_frame(?:_|$))")


def parse_float_vector(value: str | None, length: int = 3) -> list[float]:
    if not value:
        return [0.0] * length
    values = [float(item) for item in value.split()]
    if len(values) != length:
        raise ValueError(f"expected {length} values, got {value!r}")
    return values


def rpy_matrix(values: list[float]) -> list[list[float]]:
    roll, pitch, yaw = values
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def quat_from_rpy(values: list[float]) -> str:
    roll, pitch, yaw = values
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    # MuJoCo uses w x y z ordering.
    quat = (
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    )
    return " ".join(fmt(item) for item in quat)


def fmt(value: float) -> str:
    if abs(value) < 5e-12:
        value = 0.0
    return f"{value:.9g}"


def mesh_reference(
    filename: str,
    package_mesh_root: Path,
    asset_names: dict[str, str],
    asset_scales: dict[str, str],
    source_mesh: ET.Element,
) -> tuple[str, str]:
    if not filename.startswith(PACKAGE_PREFIX):
        raise ValueError(f"unsupported mesh URI: {filename}")
    relative = filename[len(PACKAGE_PREFIX):]
    if not relative.startswith("meshes/"):
        raise ValueError(f"mesh is outside package meshes/: {filename}")
    relative_to_meshdir = relative[len("meshes/"):]
    mesh_path = package_mesh_root / relative_to_meshdir
    if not mesh_path.exists():
        raise FileNotFoundError(mesh_path)
    if relative_to_meshdir not in asset_names:
        stem = re.sub(r"[^A-Za-z0-9_]", "_", relative_to_meshdir)
        asset_names[relative_to_meshdir] = "mesh_" + stem.rsplit(".", 1)[0]
    scale = source_mesh.get("scale")
    if scale:
        previous = asset_scales.setdefault(relative_to_meshdir, scale)
        if previous != scale:
            raise ValueError(f"mesh {relative_to_meshdir} is used with conflicting scales")
    return asset_names[relative_to_meshdir], relative_to_meshdir


def link_is_skipped(name: str) -> bool:
    return bool(SKIP_LINK_RE.search(name))


def visual_rgba(link: ET.Element) -> str:
    color = link.find("./visual/material/color")
    return color.get("rgba", "0.75 0.75 0.75 1") if color is not None else "0.75 0.75 0.75 1"


def append_inertial(body: ET.Element, link: ET.Element) -> None:
    source = link.find("inertial")
    if source is None or source.find("mass") is None:
        return
    origin = source.find("origin")
    inertial_rpy = parse_float_vector(origin.get("rpy") if origin is not None else None)
    attrs = {
        "pos": " ".join(fmt(item) for item in parse_float_vector(origin.get("xyz") if origin is not None else None)),
        "mass": source.find("mass").get("value", "1e-6"),
    }
    inertia = source.find("inertia")
    if inertia is not None:
        attrs["fullinertia"] = " ".join(
            inertia.get(name, "0") for name in ("ixx", "iyy", "izz", "ixy", "ixz", "iyz")
        )
    else:
        attrs["quat"] = quat_from_rpy(inertial_rpy)
    ET.SubElement(body, "inertial", attrs)


def append_geoms(
    body: ET.Element,
    link: ET.Element,
    package_mesh_root: Path,
    asset_names: dict[str, str],
    asset_scales: dict[str, str],
) -> None:
    rgba = visual_rgba(link)
    for kind, class_name, contype, conaffinity in (
        ("visual", "urdf_visual", "0", "0"),
        ("collision", "urdf_collision", "1", "1"),
    ):
        source = link.find(kind)
        if source is None:
            continue
        mesh = source.find("./geometry/mesh")
        if mesh is None or mesh.get("filename") is None:
            continue
        # The exported base visual has 354k triangles, above MuJoCo's STL
        # face limit.  The supplied collision mesh is lightweight and still
        # provides the base geometry for the dual-arm model.
        if link.get("name") == "base_link" and kind == "visual":
            continue
        mesh_name, _ = mesh_reference(mesh.get("filename"), package_mesh_root, asset_names, asset_scales, mesh)
        origin = source.find("origin")
        attrs = {
            "name": f"{link.get('name')}_{kind}_geom",
            "type": "mesh",
            "mesh": mesh_name,
            "class": class_name,
            "contype": contype,
            "conaffinity": conaffinity,
        }
        if kind == "collision" and "tactile_link" not in link.get("name", ""):
            # The URDF collision meshes overlap at several fixed joints.  They
            # are useful as visual references, but enabling all of them makes
            # the zero pose self-collide and shake in the standalone viewer.
            attrs["contype"] = "0"
            attrs["conaffinity"] = "0"
        if kind == "visual":
            attrs["rgba"] = rgba
        if origin is not None:
            attrs["pos"] = " ".join(fmt(item) for item in parse_float_vector(origin.get("xyz")))
            attrs["quat"] = quat_from_rpy(parse_float_vector(origin.get("rpy")))
        body.append(ET.Element("geom", attrs))


def make_joint(source: ET.Element) -> ET.Element | None:
    joint_type = source.get("type", "fixed")
    if joint_type == "fixed":
        return None
    if joint_type not in {"revolute", "continuous", "prismatic"}:
        raise ValueError(f"unsupported joint type {joint_type!r} for {source.get('name')}")
    output_type = "hinge" if joint_type in {"revolute", "continuous"} else "slide"
    attrs = {
        "name": source.get("name", "joint"),
        "type": output_type,
        "axis": " ".join(fmt(item) for item in parse_float_vector(source.find("axis").get("xyz") if source.find("axis") is not None else None)),
        "damping": "0.2",
        "frictionloss": "0.02",
    }
    limit = source.find("limit")
    if limit is not None and limit.get("lower") is not None and limit.get("upper") is not None:
        attrs["range"] = f"{limit.get('lower')} {limit.get('upper')}"
        attrs["limited"] = "true"
    return ET.Element("joint", attrs)


def body_origin(source_joint: ET.Element) -> dict[str, str]:
    origin = source_joint.find("origin")
    return {
        "pos": " ".join(fmt(item) for item in parse_float_vector(origin.get("xyz") if origin is not None else None)),
        "quat": quat_from_rpy(parse_float_vector(origin.get("rpy") if origin is not None else None)),
    }


def append_body_tree(
    link_name: str,
    links: dict[str, ET.Element],
    children: dict[str, list[ET.Element]],
    package_mesh_root: Path,
    asset_names: dict[str, str],
    asset_scales: dict[str, str],
    body_index: dict[str, ET.Element],
    joint_names: list[str],
) -> ET.Element:
    link = links[link_name]
    body = ET.Element("body", {"name": link_name})
    body_index[link_name] = body
    append_inertial(body, link)
    append_geoms(body, link, package_mesh_root, asset_names, asset_scales)
    for source_joint in children.get(link_name, []):
        child_element = source_joint.find("child")
        if child_element is None:
            continue
        child_name = child_element.get("link", "")
        child_body = append_body_tree(child_name, links, children, package_mesh_root, asset_names, asset_scales, body_index, joint_names)
        child_body.attrib.update(body_origin(source_joint))
        converted_joint = make_joint(source_joint)
        if converted_joint is not None:
            child_body.insert(0, converted_joint)
            joint_names.append(converted_joint.get("name", ""))
        body.append(child_body)
    return body


def add_taxel_sites_and_sensors(
    urdf_root: ET.Element,
    body_index: dict[str, ET.Element],
    sensor_parent: ET.Element,
) -> int:
    count = 0
    for source_joint in urdf_root.findall("joint"):
        name = source_joint.get("name", "")
        if not re.match(r"(?:left|right)_paxini_taxel_", name):
            continue
        parent = source_joint.find("parent")
        child = source_joint.find("child")
        origin = source_joint.find("origin")
        if parent is None or child is None or origin is None:
            raise ValueError(f"incomplete taxel joint: {name}")
        parent_name = parent.get("link", "")
        if parent_name not in body_index:
            raise ValueError(f"taxel parent was not converted to MJCF: {parent_name}")
        site_name = name.removesuffix("_joint")
        hand = "left" if name.startswith("left_") else "right"
        site = ET.Element("site", {
            "name": site_name,
            "type": "sphere",
            "pos": " ".join(fmt(item) for item in parse_float_vector(origin.get("xyz"))),
            "quat": quat_from_rpy(parse_float_vector(origin.get("rpy"))),
            "size": "0.0007",
            "group": "4",
            "rgba": "0.9 0.1 0.1 0.95" if hand == "left" else "0.1 0.35 0.95 0.95",
        })
        body = body_index[parent_name]
        body.append(site)
        body.append(ET.Element("geom", {
            "name": site_name + "_geom",
            "type": "sphere",
            "pos": " ".join(fmt(item) for item in parse_float_vector(origin.get("xyz"))),
            "quat": quat_from_rpy(parse_float_vector(origin.get("rpy"))),
            "size": "0.0007",
            "density": "0",
            "contype": "1",
            "conaffinity": "1",
            "group": "4",
            "rgba": "0.9 0.1 0.1 0.75" if hand == "left" else "0.1 0.35 0.95 0.75",
        }))
        ET.SubElement(sensor_parent, "touch", {"name": site_name + "_touch", "site": site_name})
        count += 1
    return count


def add_taxel_shell_exclusions(model: ET.Element) -> None:
    contact = ET.SubElement(model, "contact")
    for hand in ("left", "right"):
        for finger in ("index", "middle", "ring"):
            for phalanx, pad in ((1, 0), (2, 1), (3, 2)):
                ET.SubElement(contact, "exclude", {
                    "body1": f"{hand}_{finger}_link_{phalanx}",
                    "body2": f"{hand}_{finger}_tactile_link_{pad}",
                })
        for phalanx, pad in ((2, 0), (3, 1)):
            ET.SubElement(contact, "exclude", {
                "body1": f"{hand}_thumb_link_{phalanx}",
                "body2": f"{hand}_thumb_tactile_link_{pad}",
            })


def make_assets(asset_parent: ET.Element, asset_names: dict[str, str], asset_scales: dict[str, str]) -> None:
    for relative, name in sorted(asset_names.items()):
        attrs = {"name": name, "file": relative}
        if relative in asset_scales:
            attrs["scale"] = asset_scales[relative]
        ET.SubElement(asset_parent, "mesh", attrs)


def prettify(element: ET.Element, indent: str = "  ", level: int = 0) -> None:
    # ET.indent is only available in newer Python versions; keep this script
    # usable with the Python shipped in older ROS distributions as well.
    children = list(element)
    if not children:
        return
    element.text = "\n" + indent * (level + 1)
    for child in children:
        prettify(child, indent, level + 1)
        child.tail = "\n" + indent * (level + 1)
    children[-1].tail = "\n" + indent * level


def build(urdf_path: Path, output_path: Path) -> int:
    urdf_root = ET.parse(urdf_path).getroot()
    package_mesh_root = urdf_path.parent.parent / "meshes"
    links = {
        link.get("name", ""): link
        for link in urdf_root.findall("link")
        if link.get("name") and not link_is_skipped(link.get("name"))
    }
    children: dict[str, list[ET.Element]] = defaultdict(list)
    child_names: set[str] = set()
    for joint in urdf_root.findall("joint"):
        parent = joint.find("parent")
        child = joint.find("child")
        if parent is None or child is None:
            continue
        parent_name, child_name = parent.get("link", ""), child.get("link", "")
        if parent_name not in links or child_name not in links:
            continue
        children[parent_name].append(joint)
        child_names.add(child_name)
    roots = sorted(set(links) - child_names)
    if roots != ["base_link"]:
        raise ValueError(f"expected base_link as the only MJCF root, got {roots}")

    model = ET.Element("mujoco", {"model": "dual_diana7_dexh13"})
    ET.SubElement(model, "compiler", {"angle": "radian", "meshdir": "../diana7_dual_dexh13_description/meshes"})
    asset = ET.SubElement(model, "asset")
    asset_names: dict[str, str] = {}
    asset_scales: dict[str, str] = {}
    body_index: dict[str, ET.Element] = {}
    joint_names: list[str] = []
    worldbody = ET.SubElement(model, "worldbody")
    base_body = append_body_tree("base_link", links, children, package_mesh_root, asset_names, asset_scales, body_index, joint_names)
    base_body.set("pos", "0 0 0")
    base_body.set("quat", "1 0 0 0")
    worldbody.append(base_body)
    # Add a neutral floor for direct viewer use.  It does not collide with
    # visual-only meshes and can be removed by callers embedding the model.
    ET.SubElement(worldbody, "geom", {"name": "floor", "type": "plane", "size": "2 2 0.01", "pos": "0 0 -0.65", "rgba": "0.18 0.20 0.24 1", "contype": "1", "conaffinity": "1"})

    sensor = ET.SubElement(model, "sensor")
    taxel_count = add_taxel_sites_and_sensors(urdf_root, body_index, sensor)
    add_taxel_shell_exclusions(model)

    # Arm and hand hinges get conservative position actuators so the model is
    # directly controllable from Python without relying on an external config.
    actuator = ET.SubElement(model, "actuator")
    urdf_joints = {joint.get("name", ""): joint for joint in urdf_root.findall("joint")}
    for joint_name in joint_names:
        source = urdf_joints[joint_name]
        limit = source.find("limit")
        attrs = {"name": "ctrl_" + joint_name, "joint": joint_name, "kp": "8"}
        if limit is not None and limit.get("lower") is not None and limit.get("upper") is not None:
            attrs["ctrlrange"] = f"{limit.get('lower')} {limit.get('upper')}"
        effort = float(limit.get("effort", "1")) if limit is not None else 1.0
        effort = max(abs(effort), 1.0)
        attrs["forcelimited"] = "true"
        attrs["forcerange"] = f"{fmt(-effort)} {fmt(effort)}"
        ET.SubElement(actuator, "position", attrs)

    default = ET.SubElement(model, "default")
    ET.SubElement(default, "default", {"class": "urdf_visual"}).append(ET.Element("geom", {"contype": "0", "conaffinity": "0"}))
    ET.SubElement(default, "default", {"class": "urdf_collision"}).append(ET.Element("geom", {"friction": "0.8 0.02 0.001", "solref": "0.01 1", "solimp": "0.90 0.98 0.002", "condim": "4"}))
    # Keep the generated viewer model at the URDF zero pose by default.  A
    # dynamics user can enable gravity at runtime with m.opt.gravity[:] =
    # (0, 0, -9.81), after choosing a controlled initial configuration.
    ET.SubElement(model, "option", {"gravity": "0 0 0", "integrator": "implicitfast", "timestep": "0.0005"})
    visual = ET.SubElement(model, "visual")
    ET.SubElement(visual, "headlight", {"diffuse": "0.65 0.65 0.70", "ambient": "0.28 0.30 0.34", "specular": "0.18 0.18 0.20"})

    # Asset names are collected while traversing the link tree.
    make_assets(asset, asset_names, asset_scales)
    prettify(model)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    ET.ElementTree(model).write(output_path, encoding="utf-8", xml_declaration=True)
    return taxel_count


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--urdf", type=Path, default=Path("diana7_dual_dexh13_description/urdf/dual_diana7_dexh13.urdf"))
    parser.add_argument("--output", type=Path, default=Path("dexh13_mjcf/dual_diana7_dexh13.xml"))
    args = parser.parse_args()
    count = build(args.urdf, args.output)
    print(f"wrote {args.output} with {count} dense taxel sites and touch sensors")


if __name__ == "__main__":
    main()
