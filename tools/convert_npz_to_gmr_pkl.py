#!/usr/bin/env python3
"""Convert a G1 NPZ recording into the exact GMR-pkl schema eval/score_motion.py
consumes.

This is a faithful format bridge, not a retarget: every value it writes is a
direct (possibly reordered) copy of a value already present in the source
NPZ.  It never modifies the source file.

Convention mapping (both sides verified by direct inspection, 2026-08-16)
---------------------------------------------------------------------------
NPZ side -- mocap_g1.window_audit.load_npz_motion accepts either the
canonical layout (``root_pos`` (T,3), ``root_rot`` (T,4), ``dof_pos``
(T,29), ``fps``) or the compact cache layout used throughout
tools/xsens + tools/control + mocap_g1.conditioning (``qpos`` (T,36) =
``root_pos[3] + root_rot[4] + dof_pos[29]``, plus ``fps`` and usually a
``metadata`` JSON scalar):

  - root_pos       meters, world frame -- IDENTICAL layout on both sides,
                   no conversion needed.
  - root_rot       unit quaternion, **wxyz** order. Every NPZ producer in
                   this repo uses wxyz: mocap_g1.conditioning.load_qpos_npz
                   fail-closes on any ``metadata.root_quaternion_order``
                   other than "wxyz", tools/xsens/loop_bvh_g1.py's cache
                   writer hardcodes it, and a live preview.npz inspected
                   under artifacts/xsens-offline/ confirms it in practice.
  - dof_pos        29 columns in mocap_g1.schema.G1_29DOF_JOINTS order.
  - fps            scalar (may be a non-integer float after
                   mocap_g1.conditioning's decimation, e.g. 60/4=15.0).

GMR pkl side -- eval/score_motion.py's ``metrics.load_gmr_pkl`` /
mocap_g1.dataset.load_canonical_motion (GMR's own scripts/bvh_to_robot.py
save format, per eval/metrics.py's module docstring "Input format"):

  - root_pos       (T,3) float64, meters -- same layout, no conversion.
  - root_rot       (T,4) float64, **xyzw** order (GMR's own convention).
                   Needs the index permutation [1, 2, 3, 0] applied to our
                   wxyz arrays.  The pkl also carries the explicit
                   ``root_quat_order: "xyzw"`` key that every loader now
                   requires (mocap_g1/gmr_pkl.py).
  - dof_pos        (T,D) float64, MJCF qpos[7:] order for the GMR robot
                   registered under ``--robot``. For "unitree_g1" the
                   $MOCAP_WS/GMR/assets/unitree_g1/g1_mocap_29dof.xml
                   <joint> order is identical to
                   mocap_g1.schema.G1_29DOF_JOINTS, so dof_pos needs NO
                   column reordering, only a dtype cast.
  - joint_names    list[str], the 29 hinge names in dof_pos column order
                   (docs/pkl-schema-note.md). This is what makes the
                   column order CHECKABLE: eval's run_fk_pass refuses to
                   score a pkl whose joint_names differ from the loaded
                   MJCF's hinge order, instead of silently applying limits
                   and FK to the wrong columns.
  - fps            plain Python ``int``. score_motion.py's loader does
                   ``int(d["fps"])``, which TRUNCATES toward zero rather
                   than rounding -- so a source fps of e.g. 59.97 would
                   silently become 59, not 60, if we deferred the cast to
                   score_motion.py. This converter therefore performs an
                   explicit ``round()`` itself (not a truncation) before
                   writing the pkl, and records a note whenever the source
                   fps was not already within FPS_ROUND_TOLERANCE_HZ of an
                   integer. Rounding rescales every velocity/acceleration
                   metric by fps_out/fps_in, so it is REFUSED (ConversionError,
                   exit 2) when that relative change exceeds
                   FPS_MAX_RELATIVE_ROUNDING_ERROR (0.1 %): 59.97 -> 60 is
                   accepted with a note, 60/7 = 8.571 -> 9 is refused.

So the only real data conversion is the quaternion reorder; everything else
is a dtype/shape normalization. See ``convert_motion`` below.
"""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import pickle
import sys
import tempfile

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))

import numpy as np  # noqa: E402

from mocap_g1.gmr_pkl import declare_gmr_root_quat_order, wxyz_to_xyzw  # noqa: E402
from mocap_g1.io_utils import (  # noqa: E402
    read_npz_metadata as _read_npz_metadata,
    sha256_file,
    write_json_atomic,
)
from mocap_g1.schema import G1_29DOF_JOINTS  # noqa: E402
from mocap_g1.window_audit import (  # noqa: E402
    MotionArrays,
    WindowAuditError,
    load_npz_motion,
)


#: v2 (2026-10-12): the pkl carries ``joint_names``; non-integer fps whose
#: rounding would move the timebase by more than 0.1 % is refused.
SCHEMA_VERSION = 2
FPS_ROUND_TOLERANCE_HZ = 1e-2
FPS_MAX_RELATIVE_ROUNDING_ERROR = 1e-3
JOINT_NAMES_KEY = "joint_names"


class ConversionError(RuntimeError):
    """Raised when the source NPZ cannot be faithfully converted."""


def read_npz_metadata(path: str | Path) -> dict:
    """Best-effort read of the optional ``metadata`` JSON scalar.

    Returns ``{}`` if absent or unreadable; callers that need the
    quaternion-order guarantee should treat that as "unknown, assume the
    repo-wide wxyz convention" (see ``convert_motion``).
    """
    return _read_npz_metadata(path)


def convert_motion(motion: MotionArrays, *, source_metadata: dict | None = None
                   ) -> tuple[dict, dict]:
    """Convert validated NPZ arrays into the exact GMR-pkl schema.

    Returns ``(pkl_dict, notes)``; ``notes`` documents exactly which
    conversions were applied (frames, fps rounding, reorder rationale) for
    the CLI's ``--report`` and for a driver's ledger.
    """
    metadata = source_metadata or {}
    declared_order = metadata.get("root_quaternion_order")
    if declared_order is not None and declared_order != "wxyz":
        raise ConversionError(
            f"unsupported root_quaternion_order {declared_order!r} in source "
            "metadata; every NPZ producer in this repo writes wxyz (see "
            "mocap_g1.conditioning.load_qpos_npz), and this converter only "
            "implements that convention -- see this module's docstring"
        )

    joint_names = list(G1_29DOF_JOINTS)
    if motion.dof_pos.ndim != 2 or motion.dof_pos.shape[1] != len(joint_names):
        raise ConversionError(
            f"dof_pos must have {len(joint_names)} columns in G1_29DOF_JOINTS order, "
            f"got shape {motion.dof_pos.shape}"
        )
    declared_names = metadata.get(JOINT_NAMES_KEY)
    if declared_names is not None and list(declared_names) != joint_names:
        raise ConversionError(
            "source metadata joint_names differ from mocap_g1.schema.G1_29DOF_JOINTS; "
            "this converter only implements the canonical column order"
        )

    root_rot_xyzw = wxyz_to_xyzw(motion.root_rot)  # GMR native order

    if not np.isfinite(motion.fps) or motion.fps <= 0.0:
        raise ConversionError(f"source fps must be finite and positive, got {motion.fps!r}")
    fps_rounded = int(round(motion.fps))
    relative_error = abs(fps_rounded - motion.fps) / motion.fps
    if fps_rounded < 1 or relative_error > FPS_MAX_RELATIVE_ROUNDING_ERROR:
        raise ConversionError(
            f"source fps {motion.fps!r} is not an integer and rounding it to "
            f"{fps_rounded} would rescale every velocity-based metric by "
            f"{100.0 * relative_error:.3g}% (> {100.0 * FPS_MAX_RELATIVE_ROUNDING_ERROR:g}%); "
            "the GMR pkl stores an integer fps, so resample to an integer rate first"
        )
    fps_note = None
    if abs(fps_rounded - motion.fps) > FPS_ROUND_TOLERANCE_HZ:
        fps_note = (
            f"source fps {motion.fps!r} is not within {FPS_ROUND_TOLERANCE_HZ} Hz "
            f"of an integer; rounded to {fps_rounded} for the GMR pkl int-fps "
            "convention (score_motion.py's load_gmr_pkl does int(d['fps']), which "
            "TRUNCATES -- this converter rounds instead so the stored value is "
            "already exact when score_motion.py re-casts it)"
        )

    dof_pos = np.ascontiguousarray(motion.dof_pos, dtype=np.float64)
    pkl = declare_gmr_root_quat_order({
        "fps": fps_rounded,
        "root_pos": np.ascontiguousarray(motion.root_pos, dtype=np.float64),
        "root_rot": np.ascontiguousarray(root_rot_xyzw, dtype=np.float64),
        "dof_pos": dof_pos,
        JOINT_NAMES_KEY: joint_names,
    })
    notes = {
        "source_input_layout": motion.input_layout,
        "source_fps": motion.fps,
        "output_fps": fps_rounded,
        "fps_rounding_note": fps_note,
        "quaternion_conversion": (
            "wxyz (NPZ/mocap_g1 convention) -> xyzw (GMR pkl convention), "
            "index permutation [1, 2, 3, 0]"
        ),
        "dof_pos_reorder": (
            "identity: dof_pos columns are mocap_g1.schema.G1_29DOF_JOINTS order, "
            "written to the pkl as joint_names and verified against the scoring "
            "MJCF's hinge order by eval's run_fk_pass (stage 7)"
        ),
        "joint_names": joint_names,
        "fps_relative_rounding_error": relative_error,
        "frames": int(dof_pos.shape[0]),
        "joint_count": int(dof_pos.shape[1]),
    }
    return pkl, notes


def _write_pkl_atomic(path: Path, payload: dict) -> None:
    if path.exists():
        raise ConversionError(f"refusing to overwrite existing output: {path}")
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary_name: str | None = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="wb", prefix=f".{path.name}.", suffix=".tmp",
            dir=path.parent, delete=False,
        ) as handle:
            temporary_name = handle.name
            pickle.dump(payload, handle, protocol=pickle.HIGHEST_PROTOCOL)
        os.replace(temporary_name, path)
    finally:
        if temporary_name is not None:
            Path(temporary_name).unlink(missing_ok=True)


def _write_json_atomic(path: Path, payload: dict) -> None:
    write_json_atomic(path, payload, exists_error=ConversionError)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "input", type=Path,
        help="source .npz recording (canonical root_pos/root_rot/dof_pos+fps, "
             "or the compact qpos+fps[+metadata] cache format)",
    )
    parser.add_argument("output", type=Path, help="new GMR-schema .pkl (refuses to overwrite)")
    parser.add_argument(
        "--report", type=Path,
        help="JSON conversion report (default: <output>.convert.json); refuses to overwrite",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    report_path = args.report or args.output.with_suffix(".convert.json")
    try:
        if args.input.resolve() == args.output.resolve():
            raise ConversionError("input and output paths must be different")
        if args.output.exists():
            raise ConversionError(f"refusing to overwrite existing output: {args.output}")
        if report_path.exists():
            raise ConversionError(f"refusing to overwrite existing report: {report_path}")

        source_sha256 = sha256_file(args.input)
        motion = load_npz_motion(args.input)
        metadata = read_npz_metadata(args.input)
        pkl, notes = convert_motion(motion, source_metadata=metadata)
        _write_pkl_atomic(args.output, pkl)
        try:
            report = {
                "schema_version": SCHEMA_VERSION,
                "status": "PASS",
                "reasons": [],
                "input": {"path": str(args.input.resolve()), "sha256": source_sha256},
                "output": {
                    "path": str(args.output.resolve()),
                    "sha256": sha256_file(args.output),
                },
                "conversion": notes,
            }
            _write_json_atomic(report_path, report)
        except Exception:
            # The pkl was just created by this invocation; removing it keeps
            # the CLI all-or-nothing and never leaves a report-less artifact.
            args.output.unlink(missing_ok=True)
            raise
    except (ConversionError, WindowAuditError, OSError, ValueError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2

    print(f"PASS: {notes['frames']} frames @ {notes['output_fps']} fps -> {args.output}")
    if notes["fps_rounding_note"]:
        print(f"note: {notes['fps_rounding_note']}")
    print(f"report: {report_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
