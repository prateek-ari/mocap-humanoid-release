"""Robot model wrapper: joint table, limit sources, foot/end-effector bodies.

Split out of ``eval/metrics.py`` (still the public facade).
"""

from __future__ import annotations

import re
from dataclasses import dataclass, field
from pathlib import Path

import mujoco as mj

from mocap_g1.contact import foot_contact_geoms

from .io import DEFAULT_URDF, MECHANICAL_URDF, parse_urdf_limits, resolve_robot_xml

# Foot-body name patterns, tried in order; first pattern that yields exactly
# one left and one right body wins. Do NOT hardcode G1 names.
FOOT_PATTERNS = [r"ankle_roll", r"foot", r"ankle", r"toe"]

# End-effector patterns for self-collision exclusion (feet handled separately).
HAND_PATTERNS = r"wrist|hand|gripper"



# ---------------------------------------------------------------------------
# Robot model wrapper
# ---------------------------------------------------------------------------


@dataclass
class JointInfo:
    name: str
    jnt_id: int
    dof_col: int  # column in dof_pos
    limited: bool
    lo: float
    hi: float
    vel_limit: float | None = None
    pos_source: str = "mjcf"  # "urdf" once mechanical URDF limits are applied
    effort_limit: float | None = None  # N*m, from URDF <limit effort=...>
    dof_adr: int = -1  # qvel/qacc index of this joint's DOF


@dataclass
class RobotModel:
    robot: str
    xml_path: Path
    model: mj.MjModel = field(repr=False)
    data: mj.MjData = field(repr=False)
    joints: list[JointInfo] = field(default_factory=list)
    foot_bodies: dict[str, int] = field(default_factory=dict)  # side -> body id
    foot_geoms: dict[str, list[int]] = field(default_factory=dict)
    ee_bodies: set[int] = field(default_factory=set)
    notes: list[str] = field(default_factory=list)
    # Resolved limit-source files actually applied (None when unavailable),
    # for provenance in score reports.
    pos_limits_urdf: Path | None = None
    vel_limits_urdf: Path | None = None


def _body_children(model: mj.MjModel) -> dict[int, list[int]]:
    kids: dict[int, list[int]] = {i: [] for i in range(model.nbody)}
    for b in range(1, model.nbody):
        kids[model.body_parentid[b]].append(b)
    return kids


def _subtree(model: mj.MjModel, root: int) -> set[int]:
    kids = _body_children(model)
    out, stack = set(), [root]
    while stack:
        b = stack.pop()
        out.add(b)
        stack.extend(kids[b])
    return out


def _body_depth(model: mj.MjModel, b: int) -> int:
    d = 0
    while b != 0:
        b = model.body_parentid[b]
        d += 1
    return d


def resolve_foot_bodies(model: mj.MjModel, override: list[str] | None = None
                        ) -> tuple[dict[str, int], list[str]]:
    """Find {left: body_id, right: body_id} for the feet by name pattern.

    Handles the G1 ``*_ankle_roll_link`` convention and other common
    foot/toe names. Bodies with geoms are preferred;
    ties broken by kinematic depth (deepest = most distal).
    """
    names = {mj.mj_id2name(model, mj.mjtObj.mjOBJ_BODY, b): b for b in range(1, model.nbody)}
    notes: list[str] = []
    if override:
        if len(override) != 2:
            raise ValueError("--foot-bodies needs exactly two names: left,right")
        out = {}
        for nm in override:
            if nm not in names:
                raise KeyError(f"foot body {nm!r} not in model")
            side = "left" if re.search(r"left|(^|_)l(_|$)", nm, re.I) else "right"
            out[side] = names[nm]
        if set(out) != {"left", "right"}:
            raise ValueError(f"could not classify {override} into left/right")
        return out, [f"foot bodies overridden: {override}"]

    for pat in FOOT_PATTERNS:
        cand = {nm: b for nm, b in names.items() if re.search(pat, nm, re.I)}
        sides: dict[str, list[tuple[str, int]]] = {"left": [], "right": []}
        for nm, b in cand.items():
            if re.search(r"left|(^|_)l(_|$)", nm, re.I):
                sides["left"].append((nm, b))
            elif re.search(r"right|(^|_)r(_|$)", nm, re.I):
                sides["right"].append((nm, b))
        if sides["left"] and sides["right"]:
            out = {}
            for side, lst in sides.items():
                # prefer bodies that own geoms; then deepest in the tree
                lst_sorted = sorted(
                    lst,
                    key=lambda t: (model.body_geomnum[t[1]] > 0, _body_depth(model, t[1])),
                    reverse=True,
                )
                out[side] = lst_sorted[0][1]
            notes.append(
                f"foot bodies via pattern {pat!r}: "
                + ", ".join(f"{side}={mj.mj_id2name(model, mj.mjtObj.mjOBJ_BODY, b)}"
                            for side, b in out.items())
            )
            return out, notes
    raise RuntimeError(
        f"No foot bodies found with patterns {FOOT_PATTERNS}; pass --foot-bodies"
    )


def dof_column_joint_names(rm: RobotModel) -> list[str]:
    """MJCF joint names in ``dof_pos`` column order (qpos[7:] address order)."""
    return [ji.name for ji in sorted(rm.joints, key=lambda ji: ji.dof_col)]


def verify_dof_joint_order(rm: RobotModel, joint_names: list[str] | None) -> bool:
    """Check a pkl's declared ``joint_names`` against the loaded MJCF.

    Returns True when the names match column-for-column, False when the pkl
    declares none (LEGACY pkl; the caller must flag the order as unverified).
    Raises ``ValueError`` on any mismatch: applying limits and FK to the
    wrong columns would produce a confidently wrong score, so it is refused.
    """
    if joint_names is None:
        return False
    expected = dof_column_joint_names(rm)
    if list(joint_names) != expected:
        if len(joint_names) != len(expected):
            detail = f"{len(joint_names)} names vs {len(expected)} MJCF hinges"
        else:
            index = next(i for i, (a, b) in enumerate(zip(joint_names, expected)) if a != b)
            detail = (f"first mismatch at column {index}: pkl {joint_names[index]!r} vs "
                      f"MJCF {expected[index]!r}")
        raise ValueError(
            f"pkl joint_names do not match the hinge order of {rm.xml_path.name} ({detail}); "
            "refusing to score dof_pos columns against the wrong joints")
    return True


def build_robot(robot: str, mjcf_override: str | None = None,
                urdf: str | None = None,
                foot_bodies: list[str] | None = None,
                pos_urdf: str | None = None) -> RobotModel:
    xml_path = resolve_robot_xml(robot, mjcf_override)
    model = mj.MjModel.from_xml_path(str(xml_path))
    data = mj.MjData(model)
    rm = RobotModel(robot=robot, xml_path=xml_path, model=model, data=data)

    # Validate: single free root joint at qpos[0:7].
    if model.njnt == 0 or model.jnt_type[0] != mj.mjtJoint.mjJNT_FREE or model.jnt_qposadr[0] != 0:
        raise RuntimeError(f"{xml_path}: expected a single free root joint at qpos[0:7]")
    if any(model.jnt_type[j] == mj.mjtJoint.mjJNT_FREE for j in range(1, model.njnt)):
        raise RuntimeError(f"{xml_path}: multiple free joints unsupported")

    # Joint table (hinge/slide only), mapped to dof_pos columns.
    for j in range(1, model.njnt):
        # Compare as ints: newer MuJoCo enums do not compare equal to numpy ints
        # inside tuple membership tests.
        if int(model.jnt_type[j]) not in (int(mj.mjtJoint.mjJNT_HINGE), int(mj.mjtJoint.mjJNT_SLIDE)):
            raise RuntimeError(f"{xml_path}: ball joints unsupported by this harness")
        rm.joints.append(JointInfo(
            name=mj.mj_id2name(model, mj.mjtObj.mjOBJ_JOINT, j),
            jnt_id=j,
            dof_col=int(model.jnt_qposadr[j]) - 7,
            limited=bool(model.jnt_limited[j]),
            lo=float(model.jnt_range[j, 0]),
            hi=float(model.jnt_range[j, 1]),
            dof_adr=int(model.jnt_dofadr[j]),
        ))

    # D1/D5 limits-source resolver.
    # POSITION limits: mechanical URDF (MJCF jnt_range can be IK-narrowed and
    # must not define feasibility). VELOCITY limits: DEFAULT_URDF, official
    # vendor URDF as of D5 (MJCF has none); LEGACY_COMPANION_URDF available
    # via --urdf for A/B comparison. Both sources are logged.
    pos_urdf_path = Path(pos_urdf) if pos_urdf else MECHANICAL_URDF.get(robot)
    if pos_urdf_path and Path(pos_urdf_path).is_file():
        plim = parse_urdf_limits(pos_urdf_path)
        rm.pos_limits_urdf = Path(pos_urdf_path).resolve()
        matched = replaced = 0
        for ji in rm.joints:
            ent = plim.get(ji.name)
            if ent and ent["lower"] is not None and ent["upper"] is not None:
                if ji.limited and (abs(ji.lo - ent["lower"]) > 1e-9
                                   or abs(ji.hi - ent["upper"]) > 1e-9):
                    replaced += 1
                ji.lo, ji.hi = ent["lower"], ent["upper"]
                ji.limited = True
                ji.pos_source = "urdf"
                matched += 1
            # EFFORT limits (2026-08-16) ride along on the same mechanical
            # URDF.  Populated silently -- rm.notes is deliberately NOT
            # touched, so existing output stays byte-identical; coverage is
            # reported inside the new ``torque_feasibility`` block instead.
            if ent and ent.get("effort") is not None:
                ji.effort_limit = float(ent["effort"])
        rm.notes.append(
            f"position limits: {matched}/{len(rm.joints)} joints from mechanical "
            f"URDF {pos_urdf_path} ({replaced} MJCF ranges replaced); unmatched "
            f"joints keep MJCF jnt_range")
        if matched == 0:
            rm.notes.append(
                "WARNING: no joint names matched the mechanical URDF; position "
                "limits fall back to MJCF jnt_range (possibly IK-narrowed)")
    else:
        rm.notes.append(
            "WARNING: no mechanical URDF for position limits (--pos-urdf); "
            "using MJCF jnt_range, which may be IK-narrowed")

    # Velocity limits from companion URDF (optional).
    urdf_path = Path(urdf) if urdf else DEFAULT_URDF.get(robot)
    if urdf_path and Path(urdf_path).is_file():
        vlim = parse_urdf_limits(urdf_path)
        rm.vel_limits_urdf = Path(urdf_path).resolve()
        matched = 0
        for ji in rm.joints:
            ent = vlim.get(ji.name)
            if ent and ent["velocity"] is not None:
                ji.vel_limit = ent["velocity"]
                matched += 1
        rm.notes.append(f"velocity limits: {matched}/{len(rm.joints)} joints matched from {urdf_path}")
        if matched == 0:
            rm.notes.append("WARNING: no joint names matched the URDF; velocity check skipped")
    else:
        rm.notes.append("no URDF with velocity limits available; velocity feasibility skipped")

    # Feet.
    rm.foot_bodies, notes = resolve_foot_bodies(model, foot_bodies)
    rm.notes.extend(notes)
    for side, b in rm.foot_bodies.items():
        # Shared selection (mocap_g1.contact): collision geoms, else visual.
        gids, visual_fallback = foot_contact_geoms(model, b)
        if gids and visual_fallback:
            rm.notes.append(f"{side} foot: no collision geoms; using visual geoms for height")
        if not gids:
            raise RuntimeError(f"{side} foot body has no geoms at all; cannot measure height")
        rm.foot_geoms[side] = gids

    # End-effector body set = foot subtrees + hand/wrist subtrees.
    ee = set()
    for b in rm.foot_bodies.values():
        ee |= _subtree(model, b)
    for b in range(1, model.nbody):
        if re.search(HAND_PATTERNS, mj.mj_id2name(model, mj.mjtObj.mjOBJ_BODY, b), re.I):
            ee |= _subtree(model, b)
    rm.ee_bodies = ee
    return rm


