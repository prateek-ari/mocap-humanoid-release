"""Unitree G1 ``rt/arm_sdk`` bridge: LowCmd construction, CRC and LowState feedback.

This is the first robot-facing module in the repository.  Everything here is
testable without a G1 and without the Unitree SDK installed: the DDS backend is
imported lazily behind a guard, and :class:`FakeTransport` provides a fully
deterministic in-process substitute.

The module owns three responsibilities:

1. translating a retargeted waist/arm reference (``mocap_g1.schema`` ordering)
   into a ``unitree_hg.msg.dds_.LowCmd_``-shaped frame addressed by *Unitree
   motor index*;
2. driving the ``arm_sdk`` authority weight channel from
   :attr:`mocap_g1.safety.SafetyCommand.blend_weight`, so the existing envelope's
   soft-start ramp and its fail-closed ramp-down are what the robot actually
   sees; and
3. closing the loop the envelope docstring requires -- feeding
   :meth:`mocap_g1.safety.ArmSafetyEnvelope.reset` with *measured* ``LowState``
   joint positions on every arm/rearm, and failing closed when measured state
   goes stale.


Motor index map
---------------

The Unitree G1 29-DoF motor index order is **identical** to
``mocap_g1.schema.G1_29DOF_JOINTS``; the table below is asserted at import time
by :func:`_verify_motor_index_map` so the two can never silently diverge.

Source: ``G1JointIndex`` in ``example/g1/high_level/g1_arm5_sdk_dds_example.py``
and ``g1_arm7_sdk_dds_example.py`` (unitree_sdk2_python, ``master``), corroborated
by ``enum JointIndex`` in ``example/g1/high_level/g1_arm7_sdk_dds_example.cpp``
(unitree_sdk2, ``main``).

======  ===========================  ===========================================
Motor   ``schema.G1_29DOF_JOINTS``   Notes
======  ===========================  ===========================================
0       left_hip_pitch_joint         leg -- NOT commanded over ``rt/arm_sdk``
1       left_hip_roll_joint          leg
2       left_hip_yaw_joint           leg
3       left_knee_joint              leg
4       left_ankle_pitch_joint       leg (a.k.a. LeftAnkleB in ``mode_pr`` = AB)
5       left_ankle_roll_joint        leg (a.k.a. LeftAnkleA in ``mode_pr`` = AB)
6       right_hip_pitch_joint        leg
7       right_hip_roll_joint         leg
8       right_hip_yaw_joint          leg
9       right_knee_joint             leg
10      right_ankle_pitch_joint      leg (a.k.a. RightAnkleB)
11      right_ankle_roll_joint       leg (a.k.a. RightAnkleA)
12      waist_yaw_joint              arm_sdk joint  1/17
13      waist_roll_joint             arm_sdk joint  2/17 (invalid if waist locked)
14      waist_pitch_joint            arm_sdk joint  3/17 (invalid if waist locked)
15      left_shoulder_pitch_joint    arm_sdk joint  4/17
16      left_shoulder_roll_joint     arm_sdk joint  5/17
17      left_shoulder_yaw_joint      arm_sdk joint  6/17
18      left_elbow_joint             arm_sdk joint  7/17
19      left_wrist_roll_joint        arm_sdk joint  8/17
20      left_wrist_pitch_joint       arm_sdk joint  9/17 (invalid on 23-DoF G1)
21      left_wrist_yaw_joint         arm_sdk joint 10/17 (invalid on 23-DoF G1)
22      right_shoulder_pitch_joint   arm_sdk joint 11/17
23      right_shoulder_roll_joint    arm_sdk joint 12/17
24      right_shoulder_yaw_joint     arm_sdk joint 13/17
25      right_elbow_joint            arm_sdk joint 14/17
26      right_wrist_roll_joint       arm_sdk joint 15/17
27      right_wrist_pitch_joint      arm_sdk joint 16/17 (invalid on 23-DoF G1)
28      right_wrist_yaw_joint        arm_sdk joint 17/17 (invalid on 23-DoF G1)
29      --                           ``kNotUsedJoint``: arm_sdk authority WEIGHT
30..34  --                           ``kNotUsedJoint1..5``, left zero
======  ===========================  ===========================================

``schema.G1_ARM_SDK_JOINTS`` is ``G1_29DOF_JOINTS[12:]`` -- exactly the 17 motor
indices 12..28 that the upstream 7-DoF arm_sdk example commands, in the same
left-then-right order after the three waist joints.  The bridge therefore never
addresses a leg motor.

Motor slot 29 (``kNotUsedJoint``) is not a joint.  Its ``q`` field carries the
arm_sdk *authority weight* in ``[0, 1]``: ``1`` hands the waist/arm chain to this
publisher, ``0`` returns it to the on-board controller.  The upstream examples
ramp it up on entry and back down on exit; here that ramp is supplied by the
safety envelope's blend weight, so soft-start, stale-source ramp-down and
operator disable all come from one already-tested state machine.


CRC
---

``LowCmd_`` carries a trailing ``crc`` field.  The algorithm is a *non-standard*
MSB-first CRC-32 (poly ``0x04C11DB7``, init ``0xFFFFFFFF``) that additionally
XORs the polynomial whenever the current data bit is set -- see
:func:`crc32_core`.  (That extra XOR makes it equal to CRC-32/MPEG-2 over each
word's bytes, most significant first, which is how the table-driven port
computes it; the bit loop is kept in the tests as the reference.)  It runs over
the message packed with
:data:`HG_LOWCMD_PACK_FMT` (1004 bytes) reinterpreted as 251 little-endian
``uint32`` words, of which the **last word is dropped** because it is the ``crc``
field itself.

Sources: ``unitree_sdk2py/utils/crc.py`` (``CRC._crc_py``, ``CRC.__Trans``,
``CRC.__PackHGLowCmd``, ``self.__packFmtHGLowCmd``) in unitree_sdk2_python
``master``, cross-checked against ``crc32_core`` in
``include/unitree/dds_wrapper/common/crc.h`` in unitree_sdk2 ``main``.  The two
upstream implementations are bit-for-bit equivalent.

The known-answer vectors in ``tests/test_arm_bridge.py`` were produced by
executing upstream's own ``CRC._crc_py`` body (extracted verbatim from the
fetched ``crc.py``) -- they are *not* merely self-consistent with this port.
They have still never been checked against a byte capture from a real robot; see
below.


UNVERIFIED UNTIL HARDWARE
-------------------------

Nothing in this module has been exercised against a physical G1.  The following
are stated from published source and documentation only, and every one of them
must be confirmed on hardware day one before any command is allowed to move a
real joint:

* **Firmware mode / MotionSwitcher preconditions.** ``rt/arm_sdk`` is a *high
  level* seam layered on top of the on-board controller: the robot is expected to
  be in a mode where the waist/arm chain can be delegated. The upstream Python
  example imports ``MotionSwitcherClient`` but does not call it, so the exact
  required mode (and whether the running service must be released first via
  ``SetSilent`` / ``ReleaseMode``) is unconfirmed. This bridge deliberately does
  **not** switch modes; a human must place the robot in the correct state.
* **Topic names and QoS.** ``rt/arm_sdk`` (publish ``LowCmd_``) and ``rt/lowstate``
  (subscribe ``LowState_``) are taken from the examples. The examples construct
  channels with SDK defaults and a subscriber queue depth of 10; the underlying
  DDS reliability/durability/history settings have not been read off a live
  system, and the domain id / network interface binding is site specific.
* **CRC vectors.** Derived from upstream source, never compared against bytes
  captured off a G1. A firmware that packs ``LowCmd_`` differently would be
  rejected silently (the robot would simply ignore our commands).
* **``mode_machine``.** The low-level example learns this from
  ``LowState_.mode_machine`` and echoes it back; the arm_sdk examples leave it at
  0. This bridge echoes the observed value by default, which is the stricter
  choice, but whether ``rt/arm_sdk`` validates it is unconfirmed.
* **``mode_pr``.** ``0`` = PR (ankle pitch/roll parameterisation), ``1`` = AB.
  Only relevant to ankles, which this bridge never commands; left at 0.
* **Per-motor ``mode`` field.** The arm_sdk examples never set it (leaving 0),
  while the low-level example sets ``1`` to enable each motor. The default here
  matches the arm_sdk examples (0); :attr:`ArmBridgeConfig.motor_mode` exists to
  change it once the correct value is observed.
* **Gains.** ``kp = 60``, ``kd = 1.5`` are the example values for a G1 with bare
  arms. They are not validated for this robot's payload and must be re-tuned,
  starting lower, on hardware.
* **Sign conventions and zero offsets.** The retargeted reference is expressed in
  official-URDF joint coordinates. Whether the encoder zero and sign of every
  motor match that URDF is exactly what the physical zeroing procedure has to
  establish. Until it does, a correct-looking command stream can still drive the
  robot the wrong way, which is why :meth:`G1ArmBridge.arm` refuses to arm
  without measured state and the envelope's blend ramp is never bypassed.
* **Watchdog.** Whether the robot itself times out an ``rt/arm_sdk`` publisher
  that stops sending is unknown. This bridge assumes it does not, and fails
  closed on its own.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from collections import deque
from dataclasses import dataclass, field, replace
import math
from numbers import Integral, Real
import struct
import threading
from typing import Any, Callable, Iterable, Mapping, Sequence

from .safety import ArmSafetyEnvelope, SafetyCommand, SafetyConfig, SafetyError
from .schema import G1_29DOF_JOINTS, G1_ARM_SDK_JOINTS, JointLimit


__all__ = [
    "ARM_SDK_TOPIC",
    "LOWSTATE_TOPIC",
    "NUM_MOTOR_SLOTS",
    "WEIGHT_CHANNEL_INDEX",
    "G1_MOTOR_INDEX",
    "ARM_SDK_MOTOR_INDEX",
    "UPSTREAM_EXAMPLE_KP",
    "UPSTREAM_EXAMPLE_KD",
    "HG_LOWCMD_PACK_FMT",
    "HG_LOWCMD_PACKED_BYTES",
    "ArmBridgeConfig",
    "ArmBridgeError",
    "BridgeStep",
    "DdsTransport",
    "FakeTransport",
    "G1ArmBridge",
    "LowCmdFrame",
    "LowStateSnapshot",
    "MotorCommand",
    "Transport",
    "crc32_core",
    "lowcmd_crc",
    "lowcmd_crc_words",
    "pack_hg_lowcmd",
]


# ---------------------------------------------------------------------------
# Wire constants (see module docstring for provenance)
# ---------------------------------------------------------------------------

#: DDS topic the arm_sdk seam listens on; carries ``unitree_hg.msg.dds_.LowCmd_``.
ARM_SDK_TOPIC = "rt/arm_sdk"

#: DDS topic publishing ``unitree_hg.msg.dds_.LowState_``.
LOWSTATE_TOPIC = "rt/lowstate"

#: ``unitree_hg`` LowCmd_/LowState_ carry a fixed 35-element motor array.
NUM_MOTOR_SLOTS = 35

#: ``G1JointIndex.kNotUsedJoint``.  Its ``q`` is the arm_sdk authority weight.
WEIGHT_CHANNEL_INDEX = 29

#: Explicit schema-name -> Unitree motor index table.  See the module docstring
#: for the full annotated table and its source.  Verified against
#: ``G1_29DOF_JOINTS`` at import time.
G1_MOTOR_INDEX: dict[str, int] = {
    # ---- left leg (never commanded by this bridge) ----
    "left_hip_pitch_joint": 0,
    "left_hip_roll_joint": 1,
    "left_hip_yaw_joint": 2,
    "left_knee_joint": 3,
    "left_ankle_pitch_joint": 4,
    "left_ankle_roll_joint": 5,
    # ---- right leg (never commanded by this bridge) ----
    "right_hip_pitch_joint": 6,
    "right_hip_roll_joint": 7,
    "right_hip_yaw_joint": 8,
    "right_knee_joint": 9,
    "right_ankle_pitch_joint": 10,
    "right_ankle_roll_joint": 11,
    # ---- waist (arm_sdk) ----
    "waist_yaw_joint": 12,
    "waist_roll_joint": 13,
    "waist_pitch_joint": 14,
    # ---- left arm (arm_sdk) ----
    "left_shoulder_pitch_joint": 15,
    "left_shoulder_roll_joint": 16,
    "left_shoulder_yaw_joint": 17,
    "left_elbow_joint": 18,
    "left_wrist_roll_joint": 19,
    "left_wrist_pitch_joint": 20,
    "left_wrist_yaw_joint": 21,
    # ---- right arm (arm_sdk) ----
    "right_shoulder_pitch_joint": 22,
    "right_shoulder_roll_joint": 23,
    "right_shoulder_yaw_joint": 24,
    "right_elbow_joint": 25,
    "right_wrist_roll_joint": 26,
    "right_wrist_pitch_joint": 27,
    "right_wrist_yaw_joint": 28,
}

#: Gains used by the upstream ``g1_arm7_sdk_dds_example``.  They are the default
#: *and* the ceiling: :class:`ArmBridgeConfig` refuses larger gains unless
#: ``allow_gains_above_example`` is set explicitly.
UPSTREAM_EXAMPLE_KP = 60.0
UPSTREAM_EXAMPLE_KD = 1.5

#: Validation ceiling for :attr:`ArmBridgeConfig.tracking_error_limit_rad`, so
#: the tracking-error check cannot be disabled by configuring a huge limit.
MAX_TRACKING_ERROR_LIMIT_RAD = 1.0

#: Recent published commands retained for the tracking-error comparison.
_COMMAND_HISTORY_LENGTH = 64

#: The 17 motor indices this bridge is permitted to write, in schema order.
ARM_SDK_MOTOR_INDEX: dict[str, int] = {
    name: G1_MOTOR_INDEX[name] for name in G1_ARM_SDK_JOINTS
}


def _verify_motor_index_map() -> None:
    """Fail at import if the committed table drifts from the canonical order."""
    if len(G1_MOTOR_INDEX) != len(G1_29DOF_JOINTS):
        raise AssertionError("G1_MOTOR_INDEX must cover exactly the 29 named joints")
    for expected_index, name in enumerate(G1_29DOF_JOINTS):
        actual = G1_MOTOR_INDEX.get(name)
        if actual != expected_index:
            raise AssertionError(
                f"motor index map disagrees with schema order for {name}: "
                f"table={actual}, schema position={expected_index}"
            )
    if sorted(G1_MOTOR_INDEX.values()) != list(range(len(G1_29DOF_JOINTS))):
        raise AssertionError("motor indices must be a permutation of 0..28")
    if WEIGHT_CHANNEL_INDEX in G1_MOTOR_INDEX.values():
        raise AssertionError("the weight channel must not alias a joint motor")
    if not 0 <= WEIGHT_CHANNEL_INDEX < NUM_MOTOR_SLOTS:
        raise AssertionError("weight channel index outside the motor array")
    if tuple(ARM_SDK_MOTOR_INDEX.values()) != tuple(range(12, 29)):
        raise AssertionError("arm_sdk joints must be motor indices 12..28 in order")


_verify_motor_index_map()


class ArmBridgeError(RuntimeError):
    """Raised when the bridge cannot construct or publish a command safely."""


# ---------------------------------------------------------------------------
# CRC (bit-identical to upstream; see module docstring)
# ---------------------------------------------------------------------------

#: ``unitree_sdk2py.utils.crc.CRC.__packFmtHGLowCmd``.  4-byte aligned,
#: little-endian: ``mode_pr``, ``mode_machine``, 2 pad; then 35 x
#: (``mode`` + 3 pad + ``q``, ``dq``, ``tau``, ``kp``, ``kd`` + ``reserve``);
#: then ``reserve[4]`` and ``crc``.
HG_LOWCMD_PACK_FMT = "<2B2x" + "B3x5fI" * NUM_MOTOR_SLOTS + "5I"

#: Upstream documents this as 1004 bytes; asserted below.
HG_LOWCMD_PACKED_BYTES = struct.calcsize(HG_LOWCMD_PACK_FMT)
if HG_LOWCMD_PACKED_BYTES != 1004:
    raise AssertionError(
        f"HG LowCmd_ pack format must be 1004 bytes, got {HG_LOWCMD_PACKED_BYTES}"
    )

_CRC_POLYNOMIAL = 0x04C11DB7
_CRC_INIT = 0xFFFFFFFF
_UINT32_MASK = 0xFFFFFFFF


def _crc_table_entry(index: int) -> int:
    """Run the upstream bit loop over eight zero data bits from ``index << 24``.

    With no data bits set, the data-dependent XOR below never fires, so this is
    exactly the accumulator half of :func:`crc32_core`'s original bit loop.
    """
    crc = index << 24
    for _ in range(8):
        if crc & 0x80000000:
            crc = ((crc << 1) & _UINT32_MASK) ^ _CRC_POLYNOMIAL
        else:
            crc = (crc << 1) & _UINT32_MASK
    return crc


_CRC_TABLE: tuple[int, ...] = tuple(_crc_table_entry(index) for index in range(256))


def crc32_core(words: Iterable[int]) -> int:
    """Unitree's MSB-first CRC-32 over 32-bit words.

    Bit-identical to ``CRC._crc_py`` (unitree_sdk2_python) / ``crc32_core``
    (unitree_sdk2 ``crc.h``).  Upstream's loop looks non-standard: besides the
    usual shift-and-conditionally-XOR on the accumulator's MSB, it XORs the
    polynomial again whenever the current *data* bit is set.  The two XORs
    combine to one XOR when ``msb ^ data_bit`` is set, which is the textbook
    MSB-first update, so a byte-wise table over each word's bytes (most
    significant first) gives the same result with one lookup per byte instead
    of a branch per bit.  Do not "fix" the algorithm; ``tests/test_arm_bridge.py``
    checks this table against upstream's bit loop.
    """
    table = _CRC_TABLE
    crc = _CRC_INIT
    for word in words:
        current = int(word) & _UINT32_MASK
        crc = ((crc << 8) & _UINT32_MASK) ^ table[(crc >> 24) ^ (current >> 24)]
        crc = ((crc << 8) & _UINT32_MASK) ^ table[(crc >> 24) ^ ((current >> 16) & 0xFF)]
        crc = ((crc << 8) & _UINT32_MASK) ^ table[(crc >> 24) ^ ((current >> 8) & 0xFF)]
        crc = ((crc << 8) & _UINT32_MASK) ^ table[(crc >> 24) ^ (current & 0xFF)]
    return crc


def lowcmd_crc_words(packed: bytes) -> list[int]:
    """Reinterpret packed LowCmd bytes as little-endian words, dropping the CRC.

    Mirrors ``CRC.__Trans``: ``calcLen = (len >> 2) - 1``, so the trailing word
    -- the ``crc`` field itself -- is excluded from its own checksum.
    """
    if len(packed) % 4:
        raise ArmBridgeError("packed LowCmd length must be a multiple of 4 bytes")
    word_count = (len(packed) >> 2) - 1
    if word_count < 0:
        raise ArmBridgeError("packed LowCmd is too short to contain a CRC field")
    return list(struct.unpack_from(f"<{word_count}I", packed, 0))


# ---------------------------------------------------------------------------
# Message value objects
# ---------------------------------------------------------------------------


@dataclass
class MotorCommand:
    """One ``unitree_hg.msg.dds_.MotorCmd_`` slot.

    Field order matches the IDL: ``mode, q, dq, tau, kp, kd, reserve``.
    """

    mode: int = 0
    q: float = 0.0
    dq: float = 0.0
    tau: float = 0.0
    kp: float = 0.0
    kd: float = 0.0
    reserve: int = 0


@dataclass
class LowCmdFrame:
    """A ``unitree_hg.msg.dds_.LowCmd_`` as plain Python.

    Kept SDK-free so that construction, packing and CRC are testable with no
    cyclonedds installed.  :class:`DdsTransport` converts this into the real IDL
    object at publish time.
    """

    mode_pr: int = 0
    mode_machine: int = 0
    motor_cmd: list[MotorCommand] = field(
        default_factory=lambda: [MotorCommand() for _ in range(NUM_MOTOR_SLOTS)]
    )
    reserve: tuple[int, int, int, int] = (0, 0, 0, 0)
    crc: int = 0

    def __post_init__(self) -> None:
        if len(self.motor_cmd) != NUM_MOTOR_SLOTS:
            raise ArmBridgeError(
                f"LowCmd_ carries exactly {NUM_MOTOR_SLOTS} motor slots, "
                f"got {len(self.motor_cmd)}"
            )
        if len(self.reserve) != 4:
            raise ArmBridgeError("LowCmd_.reserve is a 4-element uint32 array")

    @property
    def weight(self) -> float:
        """The arm_sdk authority weight carried by motor slot 29's ``q``."""
        return self.motor_cmd[WEIGHT_CHANNEL_INDEX].q

    def joint_q(self, name: str) -> float:
        """``q`` commanded for a named schema joint."""
        return self.motor_cmd[G1_MOTOR_INDEX[name]].q

    def pack(self) -> bytes:
        """Serialise exactly as ``CRC.__PackHGLowCmd`` does."""
        return pack_hg_lowcmd(self)

    def with_crc(self) -> "LowCmdFrame":
        """Return a copy whose ``crc`` field is filled in."""
        self.crc = lowcmd_crc(self)
        return self


def pack_hg_lowcmd(frame: LowCmdFrame) -> bytes:
    """Pack a :class:`LowCmdFrame` into the 1004-byte CRC pre-image.

    Field order follows ``CRC.__PackHGLowCmd``: ``mode_pr``, ``mode_machine``,
    then per motor ``mode, q, dq, tau, kp, kd, reserve``, then ``reserve[4]`` and
    ``crc``.  Note ``tau`` precedes ``kp``/``kd`` -- the IDL field order, not the
    order the examples assign them in.
    """
    values: list[Any] = [int(frame.mode_pr) & 0xFF, int(frame.mode_machine) & 0xFF]
    for motor in frame.motor_cmd:
        values.extend(
            (
                int(motor.mode) & 0xFF,
                float(motor.q),
                float(motor.dq),
                float(motor.tau),
                float(motor.kp),
                float(motor.kd),
                int(motor.reserve) & _UINT32_MASK,
            )
        )
    values.extend(int(word) & _UINT32_MASK for word in frame.reserve)
    values.append(int(frame.crc) & _UINT32_MASK)
    try:
        return struct.pack(HG_LOWCMD_PACK_FMT, *values)
    except (struct.error, OverflowError) as exc:  # pragma: no cover - guard
        raise ArmBridgeError(f"cannot pack LowCmd_: {exc}") from exc


def lowcmd_crc(frame: LowCmdFrame) -> int:
    """CRC of a :class:`LowCmdFrame`, ignoring whatever its ``crc`` field holds."""
    return crc32_core(lowcmd_crc_words(pack_hg_lowcmd(frame)))


@dataclass(frozen=True)
class LowStateSnapshot:
    """Measured robot state, timestamped with *local* receive time.

    ``receive_time_s`` is deliberately the local monotonic arrival time rather
    than the robot's ``tick``: staleness must be measured against the clock the
    control loop runs on, so a frozen publisher is detectable even if its own
    timestamps keep advancing.
    """

    receive_time_s: float
    q: tuple[float, ...]
    dq: tuple[float, ...] = ()
    tau_est: tuple[float, ...] = ()
    mode_machine: int = 0
    mode_pr: int = 0
    tick: int = 0
    crc: int = 0

    def __post_init__(self) -> None:
        if len(self.q) != NUM_MOTOR_SLOTS:
            raise ArmBridgeError(
                f"LowState_ carries {NUM_MOTOR_SLOTS} motor states, got {len(self.q)}"
            )
        for value in self.q:
            if not math.isfinite(value):
                raise ArmBridgeError("measured LowState_ q contains a non-finite value")
        if not math.isfinite(self.receive_time_s):
            raise ArmBridgeError("LowState_ receive time must be finite")

    def arm_positions(self) -> dict[str, float]:
        """Measured positions for exactly the 17 arm_sdk joints, by schema name.

        This is the dictionary :meth:`ArmSafetyEnvelope.reset` requires; it is
        built from encoders, never from midpoint or neutral defaults.
        """
        return {name: float(self.q[index]) for name, index in ARM_SDK_MOTOR_INDEX.items()}

    @classmethod
    def from_arm_positions(
        cls,
        positions: Mapping[str, float],
        *,
        receive_time_s: float,
        mode_machine: int = 0,
        **kwargs: Any,
    ) -> "LowStateSnapshot":
        """Build a snapshot from named arm positions; other motors read zero.

        Test/simulation helper.  Real states come off the wire.
        """
        q = [0.0] * NUM_MOTOR_SLOTS
        for name, value in positions.items():
            if name not in ARM_SDK_MOTOR_INDEX:
                raise ArmBridgeError(f"{name} is not an arm_sdk joint")
            q[ARM_SDK_MOTOR_INDEX[name]] = float(value)
        return cls(
            receive_time_s=float(receive_time_s),
            q=tuple(q),
            mode_machine=int(mode_machine),
            **kwargs,
        )


# ---------------------------------------------------------------------------
# Transports
# ---------------------------------------------------------------------------


class Transport(ABC):
    """Publish ``LowCmd_`` frames and expose the newest ``LowState_``.

    Implementations must be safe to call from the control loop thread.  The
    bridge never blocks on a transport: :meth:`latest_state` returns whatever has
    arrived so far (possibly ``None``), and the bridge decides what staleness
    means.
    """

    @abstractmethod
    def publish(self, frame: LowCmdFrame) -> None:
        """Send one command frame.  ``frame.crc`` is already populated."""

    @abstractmethod
    def latest_state(self) -> LowStateSnapshot | None:
        """Newest received state, or ``None`` if nothing has arrived yet."""

    def close(self) -> None:
        """Release resources.  Default is a no-op."""


class FakeTransport(Transport):
    """Deterministic in-process transport for tests and dry runs.

    Records every published frame and lets the caller inject measured state.
    No sockets, no threads, no DDS.
    """

    def __init__(self, *, record_limit: int | None = None) -> None:
        self.published: list[LowCmdFrame] = []
        self.published_bytes: list[bytes] = []
        self._state: LowStateSnapshot | None = None
        self._record_limit = record_limit
        self.publish_count = 0
        self.closed = False

    def publish(self, frame: LowCmdFrame) -> None:
        if self.closed:
            raise ArmBridgeError("transport is closed")
        self.publish_count += 1
        if self._record_limit is not None and len(self.published) >= self._record_limit:
            return
        # Store a snapshot: the bridge is free to reuse its scratch frame.
        copy = replace(frame, motor_cmd=[replace(motor) for motor in frame.motor_cmd])
        self.published.append(copy)
        self.published_bytes.append(copy.pack())

    def latest_state(self) -> LowStateSnapshot | None:
        return self._state

    def feed_state(self, state: LowStateSnapshot) -> None:
        """Inject a measured state, as if one had arrived on ``rt/lowstate``."""
        self._state = state

    def feed_arm_positions(
        self,
        positions: Mapping[str, float],
        *,
        receive_time_s: float,
        mode_machine: int = 0,
    ) -> LowStateSnapshot:
        """Convenience wrapper around :meth:`LowStateSnapshot.from_arm_positions`."""
        state = LowStateSnapshot.from_arm_positions(
            positions, receive_time_s=receive_time_s, mode_machine=mode_machine
        )
        self.feed_state(state)
        return state

    def close(self) -> None:
        self.closed = True


_ROBOT_EXTRA_HINT = (
    "the Unitree SDK is not installed. Install the optional dependency group "
    "with `pip install -e '.[robot]'` (which pulls cyclonedds==0.10.2 and "
    "unitree_sdk2py). cyclonedds may additionally require CYCLONEDDS_HOME to "
    "point at a built Cyclone DDS 0.10.x tree; see the unitree_sdk2_python "
    "README. Until then use --transport fake."
)


class DdsTransport(Transport):
    """CycloneDDS / ``unitree_sdk2py`` backend for a real G1.

    The SDK is imported lazily inside :meth:`connect` so that importing this
    module -- and every test, CLI and simulator path in this repository -- works
    with no robot dependencies present.

    UNVERIFIED: never run against hardware.  See the module docstring.
    """

    def __init__(
        self,
        *,
        domain_id: int = 0,
        network_interface: str | None = None,
        arm_sdk_topic: str = ARM_SDK_TOPIC,
        lowstate_topic: str = LOWSTATE_TOPIC,
        state_queue_depth: int = 10,
    ) -> None:
        self.domain_id = int(domain_id)
        self.network_interface = network_interface
        self.arm_sdk_topic = arm_sdk_topic
        self.lowstate_topic = lowstate_topic
        self.state_queue_depth = int(state_queue_depth)
        self._lock = threading.Lock()
        self._state: LowStateSnapshot | None = None
        self._publisher: Any = None
        self._subscriber: Any = None
        self._lowcmd_type: Any = None
        self._motorcmd_default: Any = None
        self._clock: Callable[[], float] | None = None
        self._connected = False

    # -- SDK plumbing -------------------------------------------------------

    @staticmethod
    def _import_sdk() -> tuple[Any, Any, Any, Any, Any]:
        """Import unitree_sdk2py, or raise a message naming the ``robot`` extra."""
        try:
            from unitree_sdk2py.core.channel import (  # type: ignore[import-not-found]
                ChannelFactoryInitialize,
                ChannelPublisher,
                ChannelSubscriber,
            )
            from unitree_sdk2py.idl.default import (  # type: ignore[import-not-found]
                unitree_hg_msg_dds__LowCmd_,
            )
            from unitree_sdk2py.idl.unitree_hg.msg.dds_ import (  # type: ignore[import-not-found]
                LowCmd_,
                LowState_,
            )
        except ImportError as exc:
            raise ArmBridgeError(f"--transport dds requires unitree_sdk2py: {_ROBOT_EXTRA_HINT}") from exc
        return (
            ChannelFactoryInitialize,
            ChannelPublisher,
            ChannelSubscriber,
            (LowCmd_, LowState_),
            unitree_hg_msg_dds__LowCmd_,
        )

    def connect(self, *, clock: Any = None) -> "DdsTransport":
        """Initialise the DDS channel factory, publisher and subscriber.

        ``clock`` is a zero-argument callable returning the loop's monotonic
        time; state arrival is stamped with it so staleness is measured on the
        same clock the control loop uses.
        """
        if self._connected:
            return self
        import time as _time

        self._clock = clock or _time.monotonic
        try:
            (
                channel_factory_initialize,
                channel_publisher,
                channel_subscriber,
                (lowcmd_type, lowstate_type),
                lowcmd_default,
            ) = self._import_sdk()

            if self.network_interface:
                channel_factory_initialize(self.domain_id, self.network_interface)
            else:
                channel_factory_initialize(self.domain_id)

            self._lowcmd_type = lowcmd_type
            self._motorcmd_default = lowcmd_default

            self._publisher = channel_publisher(self.arm_sdk_topic, lowcmd_type)
            self._publisher.Init()

            self._subscriber = channel_subscriber(self.lowstate_topic, lowstate_type)
            self._subscriber.Init(self._on_low_state, self.state_queue_depth)
        except BaseException as exc:
            self.close()
            if isinstance(exc, ArmBridgeError) or not isinstance(exc, Exception):
                raise
            raise ArmBridgeError(
                f"failed to initialize DDS transport: {type(exc).__name__}: {exc}"
            ) from exc

        self._connected = True
        return self

    # -- Transport interface ------------------------------------------------

    def publish(self, frame: LowCmdFrame) -> None:
        if not self._connected:
            raise ArmBridgeError("DdsTransport.connect() must be called before publish()")
        self._publisher.Write(self._to_idl(frame))

    def latest_state(self) -> LowStateSnapshot | None:
        with self._lock:
            return self._state

    def close(self) -> None:
        for handle in (self._publisher, self._subscriber):
            close = getattr(handle, "Close", None)
            if callable(close):
                try:
                    close()
                except Exception:  # pragma: no cover - best-effort teardown
                    pass
        self._publisher = None
        self._subscriber = None
        self._connected = False

    # -- conversions --------------------------------------------------------

    def _to_idl(self, frame: LowCmdFrame) -> Any:
        """Copy a :class:`LowCmdFrame` into a real ``unitree_hg`` ``LowCmd_``."""
        message = self._motorcmd_default()
        message.mode_pr = int(frame.mode_pr)
        message.mode_machine = int(frame.mode_machine)
        for index, motor in enumerate(frame.motor_cmd):
            target = message.motor_cmd[index]
            target.mode = int(motor.mode)
            target.q = float(motor.q)
            target.dq = float(motor.dq)
            target.tau = float(motor.tau)
            target.kp = float(motor.kp)
            target.kd = float(motor.kd)
            target.reserve = int(motor.reserve)
        for index, word in enumerate(frame.reserve):
            message.reserve[index] = int(word)
        # Our own CRC, computed over our own packing, rather than the SDK's
        # helper: it keeps the checked-in known-answer vectors authoritative and
        # avoids a hard dependency on the SDK's compiled crc shared object.
        message.crc = int(frame.crc)
        return message

    def _on_low_state(self, message: Any) -> None:
        """``rt/lowstate`` callback.  Runs on an SDK thread."""
        now = float(self._clock()) if self._clock else 0.0
        try:
            snapshot = LowStateSnapshot(
                receive_time_s=now,
                q=tuple(float(state.q) for state in message.motor_state),
                dq=tuple(float(state.dq) for state in message.motor_state),
                tau_est=tuple(float(state.tau_est) for state in message.motor_state),
                mode_machine=int(message.mode_machine),
                mode_pr=int(message.mode_pr),
                tick=int(message.tick),
                crc=int(message.crc),
            )
        except (ArmBridgeError, TypeError, ValueError, AttributeError):
            # A malformed state must not kill the SDK callback thread; dropping
            # it lets the bridge's own staleness machinery fail closed.
            return
        with self._lock:
            self._state = snapshot


# ---------------------------------------------------------------------------
# Bridge
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class ArmBridgeConfig:
    """Publisher-side settings.  Safety limits live in :class:`SafetyConfig`."""

    #: Example values from the upstream arm_sdk examples.  UNVERIFIED for this
    #: robot's payload -- start lower on hardware.  These are also the ceiling
    #: enforced by :meth:`validate` unless ``allow_gains_above_example`` is set.
    kp: float = UPSTREAM_EXAMPLE_KP
    kd: float = UPSTREAM_EXAMPLE_KD

    #: Explicit opt-in required to exceed the upstream example gains.
    allow_gains_above_example: bool = False

    #: Feed-forward torque.  The examples publish 0 and so do we.
    tau_ff: float = 0.0

    #: Per-motor ``mode``.  arm_sdk examples leave this 0; the low-level example
    #: uses 1 to enable a motor.  UNVERIFIED which rt/arm_sdk expects.
    motor_mode: int = 0

    #: 0 = PR (ankle pitch/roll), 1 = AB.  Ankles are never commanded here.
    mode_pr: int = 0

    #: When set, publish this ``mode_machine`` instead of echoing ``LowState_``.
    mode_machine_override: int | None = None

    #: Measured-state age at which tracking is no longer trusted; the bridge
    #: ramps the authority weight down through the envelope.
    state_stale_timeout_s: float = 0.10

    #: Measured-state age at which the bridge latches and demands a rearm from
    #: fresh encoders.  Must be > ``state_stale_timeout_s`` so the stale
    #: ramp-down has a non-zero window.
    state_hard_timeout_s: float = 0.50

    #: Small allowance for a callback that timestamps a state just after the
    #: caller sampled ``now``. Larger future offsets indicate mixed clocks and
    #: must fail closed.
    state_future_tolerance_s: float = 0.005

    #: Per-joint ``|q_measured - q_commanded|`` that latches a safety fault.
    #: The measured state is compared with the command that was current when
    #: it was received, and only while that command's authority weight is at
    #: least ``tracking_error_min_weight``: at lower weights the on-board
    #: controller legitimately pulls the arm away from this publisher's
    #: command.  UNVERIFIED on hardware -- servo lag plus one control tick at
    #: the envelope's velocity limit must stay well inside this bound.
    tracking_error_limit_rad: float = 0.30
    tracking_error_min_weight: float = 0.9

    #: How many times the final weight-0 frame is published by :meth:`disarm`
    #: so a single lost sample cannot leave authority with this publisher.
    release_repeat_count: int = 5

    def validate(self) -> None:
        # Check order is part of the contract (the first failing check names
        # the error), so the checks stay in field order rather than grouped
        # by kind.  ``_finite_real`` / ``_integer`` refuse bool explicitly:
        # True/False would otherwise pass as 1/0.
        for name in ("kp", "kd"):
            gain = getattr(self, name)
            if not _finite_real(gain) or gain < 0:
                raise ArmBridgeError(f"{name} must be finite and non-negative")
        if not isinstance(self.allow_gains_above_example, bool):
            raise ArmBridgeError("allow_gains_above_example must be a bool")
        if not self.allow_gains_above_example and (
            self.kp > UPSTREAM_EXAMPLE_KP or self.kd > UPSTREAM_EXAMPLE_KD
        ):
            raise ArmBridgeError(
                f"kp={self.kp:g}/kd={self.kd:g} exceed the upstream example gains "
                f"(kp<={UPSTREAM_EXAMPLE_KP:g}, kd<={UPSTREAM_EXAMPLE_KD:g}); set "
                "allow_gains_above_example to exceed them deliberately"
            )
        if not _finite_real(self.tau_ff):
            raise ArmBridgeError("tau_ff must be finite")
        if not _integer(self.motor_mode) or not 0 <= self.motor_mode <= 0xFF:
            raise ArmBridgeError("motor_mode must be an integer fitting in a uint8")
        if not _integer(self.mode_pr) or self.mode_pr not in (0, 1):
            raise ArmBridgeError("mode_pr must be integer 0 (PR) or 1 (AB)")
        if self.mode_machine_override is not None and (
            not _integer(self.mode_machine_override)
            or not 0 <= self.mode_machine_override <= 0xFF
        ):
            raise ArmBridgeError(
                "mode_machine_override must be an integer fitting in a uint8"
            )
        stale, hard = self.state_stale_timeout_s, self.state_hard_timeout_s
        if not _finite_real(stale) or not _finite_real(hard) or not 0 < stale < hard:
            raise ArmBridgeError(
                "state timeouts must be finite and require "
                "0 < state_stale_timeout_s < state_hard_timeout_s"
            )
        future = self.state_future_tolerance_s
        if not _finite_real(future) or future < 0.0 or future >= stale:
            raise ArmBridgeError(
                "state_future_tolerance_s must be finite, non-negative, and smaller "
                "than state_stale_timeout_s"
            )
        limit = self.tracking_error_limit_rad
        if not _finite_real(limit) or not 0.0 < limit <= MAX_TRACKING_ERROR_LIMIT_RAD:
            raise ArmBridgeError(
                "tracking_error_limit_rad must be finite, positive, and at most "
                f"{MAX_TRACKING_ERROR_LIMIT_RAD:g} rad"
            )
        weight = self.tracking_error_min_weight
        if not _finite_real(weight) or not 0.0 < weight <= 1.0:
            raise ArmBridgeError(
                "tracking_error_min_weight must be greater than 0 and at most 1"
            )
        repeats = self.release_repeat_count
        if not _integer(repeats) or not 1 <= repeats <= 100:
            raise ArmBridgeError("release_repeat_count must be an integer in [1, 100]")


def _finite_real(value: object) -> bool:
    """A finite real number that is not a bool (``float()`` may raise on overflow)."""
    return (
        not isinstance(value, bool)
        and isinstance(value, Real)
        and math.isfinite(float(value))
    )


def _integer(value: object) -> bool:
    """An integral value that is not a bool."""
    return not isinstance(value, bool) and isinstance(value, Integral)


@dataclass(frozen=True)
class BridgeStep:
    """What one control tick produced."""

    frame: LowCmdFrame
    weight: float
    reason: str
    engaged: bool
    armed: bool
    rearm_required: bool
    state_age_s: float
    command: SafetyCommand | None = None

    @property
    def crc(self) -> int:
        return self.frame.crc


class G1ArmBridge:
    """Drive the G1 waist/arm chain over ``rt/arm_sdk`` behind the safety envelope.

    Ownership split, deliberately:

    * :class:`ArmSafetyEnvelope` decides *what* positions and what authority
      weight are permissible.  It is used unmodified.
    * This class decides *whether the loop is allowed to run at all* -- it owns
      the measured-state contract the envelope cannot check for itself: reset
      from encoders on arm, staleness of ``LowState_``, and latching a rearm
      requirement after a fault.
    * :class:`Transport` moves bytes.

    Fail-closed behaviour, in increasing severity:

    ``LowState_`` older than ``state_stale_timeout_s``
        The envelope is stepped with ``enable=False``.  Positions decelerate
        under the configured acceleration limit and the authority weight ramps
        toward 0 at ``blend_rate_per_s`` -- a controlled hand-back, not a
        discontinuity.

    ``LowState_`` older than ``state_hard_timeout_s``, a command source older
    than the envelope's ``hard_timeout_s``, or any :class:`SafetyError`
        The bridge latches ``rearm_required``, publishes weight 0 holding the
        last emitted positions, and refuses to track again until :meth:`arm` is
        called with fresh measured state.

    Measured position differs from the command by more than
    ``tracking_error_limit_rad`` while the weight is high
        Latched like a hard timeout (reason ``tracking_error``).

    Weight ramped to 0 while armed (operator disable, stale state)
        The envelope's stored pose is frozen while the on-board controller
        moves the arm, so before ramping up again the envelope is reset from
        fresh measured ``LowState_`` (one weight-0 ``reseeded`` tick).
    """

    def __init__(
        self,
        transport: Transport,
        limits: Mapping[str, JointLimit],
        *,
        config: ArmBridgeConfig | None = None,
        safety_config: SafetyConfig | None = None,
        envelope: ArmSafetyEnvelope | None = None,
    ) -> None:
        self.transport = transport
        self.config = config or ArmBridgeConfig()
        self.config.validate()
        self.safety_config = safety_config or SafetyConfig()
        self.envelope = envelope or ArmSafetyEnvelope(limits, self.safety_config)
        self.limits = {name: limits[name] for name in G1_ARM_SDK_JOINTS}

        self._armed = False
        self._rearm_required = False
        self._mode_machine = 0
        self._last_positions: dict[str, float] | None = None
        self._last_state: LowStateSnapshot | None = None
        self._arm_count = 0
        self._reset_positions: list[dict[str, float]] = []
        # Set once the envelope weight reaches 0 while armed; see step().
        self._reseed_required = False
        # (publish time, positions, weight) of recently *published* commands,
        # for the tracking-error check.  Unpublished (dry-run) commands are not
        # recorded: the robot cannot be expected to follow them.
        self._command_history: deque[tuple[float, dict[str, float], float]] = deque(
            maxlen=_COMMAND_HISTORY_LENGTH
        )

    # -- introspection ------------------------------------------------------

    @property
    def armed(self) -> bool:
        return self._armed and not self._rearm_required

    @property
    def rearm_required(self) -> bool:
        return self._rearm_required

    @property
    def arm_count(self) -> int:
        """How many times the envelope has been reset from measured state (arm or reseed)."""
        return self._arm_count

    @property
    def reset_positions(self) -> tuple[dict[str, float], ...]:
        """The measured position dicts handed to ``envelope.reset``, in order."""
        return tuple(dict(entry) for entry in self._reset_positions)

    def latest_state(self) -> LowStateSnapshot | None:
        """Newest ``LowState_`` snapshot seen by the transport."""
        return self.transport.latest_state()

    def state_age_s(self, now: float) -> float:
        """Signed age of the newest state; ``inf`` if none has arrived.

        A negative value is retained so callers can distinguish a small
        callback race from a gross clock-domain mismatch.
        """
        return _state_age(self.transport.latest_state(), now)

    # -- arming -------------------------------------------------------------

    def arm(self, *, now: float) -> LowStateSnapshot:
        """Reset the envelope from measured encoders and permit tracking.

        This is the hardware adapter the :class:`ArmSafetyEnvelope` docstring
        requires.  It refuses to arm from anything but a fresh ``LowState_``:
        never from midpoint defaults, never from the last command.
        """
        if self.armed:  # a reset now would drop the weight mid-motion; disarm first
            raise ArmBridgeError("cannot arm: already armed")
        now = float(now)
        if not math.isfinite(now):
            raise ArmBridgeError("cannot arm: control time must be finite")
        state = self.transport.latest_state()
        if state is None:
            raise ArmBridgeError(
                "cannot arm: no LowState_ received yet; the envelope must be "
                "reset from measured robot state"
            )
        self._reset_from_state(state, now=now, action="arm")

        self._armed = True
        self._rearm_required = False
        self._mode_machine = (
            int(self.config.mode_machine_override)
            if self.config.mode_machine_override is not None
            else int(state.mode_machine)
        )
        return state

    def _reset_from_state(
        self, state: LowStateSnapshot, *, now: float, action: str
    ) -> dict[str, float]:
        """Reset the envelope from a fresh ``LowState_``; shared by arm/reseed."""
        age = now - state.receive_time_s
        if not math.isfinite(age):
            raise ArmBridgeError(f"cannot {action}: LowState_ age is not finite")
        if age < -self.config.state_future_tolerance_s:
            raise ArmBridgeError(
                f"cannot {action}: newest LowState_ timestamp is {-age:.6g}s in the "
                "future; control and transport clocks do not match"
            )
        # Finite and at most the tolerance in the future: a callback race.
        age = max(0.0, age)
        if age >= self.config.state_stale_timeout_s:
            raise ArmBridgeError(
                f"cannot {action}: newest LowState_ is {age:.6g}s old "
                f"(limit {self.config.state_stale_timeout_s}s)"
            )
        measured = state.arm_positions()
        try:
            self.envelope.reset(measured, now=now)
        except SafetyError as exc:
            raise ArmBridgeError(f"cannot {action} from measured state: {exc}") from exc
        self._last_positions = dict(measured)
        self._last_state = state
        self._reseed_required = False
        self._command_history.clear()
        self._arm_count += 1
        self._reset_positions.append(dict(measured))
        return measured

    def disarm(
        self,
        *,
        now: float,
        publish: bool = True,
        repeat: int | None = None,
        between: Callable[[], None] | None = None,
    ) -> BridgeStep:
        """Hand authority back immediately: weight 0, holding last positions.

        The weight-0 frame is published ``repeat`` times (default
        ``config.release_repeat_count``) so one lost sample cannot leave the
        last non-zero weight in force.  ``between`` is called between repeats,
        e.g. to pace them one control period apart on a real transport.

        This is the terminal release.  A non-latching exit (normal end of
        replay, operator stop) should first ramp the weight down with
        ``step(..., enable=False)``; see ``tools/control/g1_arm_bridge.py``.
        """
        self._armed = False
        count = self.config.release_repeat_count if repeat is None else int(repeat)
        if count < 1:
            raise ArmBridgeError("disarm repeat count must be at least 1")
        released = self._release(now=now, reason="disarmed", publish=publish)
        if publish:
            for _ in range(count - 1):
                if between is not None:
                    between()
                self.transport.publish(released.frame)
        return released

    # -- control ------------------------------------------------------------

    def step(
        self,
        target_positions: Mapping[str, float],
        *,
        source_timestamp: float,
        now: float,
        enable: bool = True,
        publish: bool = True,
    ) -> BridgeStep:
        """Run one control tick and (by default) publish the resulting LowCmd."""
        now = float(now)
        # One snapshot per tick: freshness and the tracking-error check must
        # judge the same LowState_, not two reads a callback may separate.
        state = self.transport.latest_state()
        if state is not None:
            self._last_state = state
        if not math.isfinite(now):
            self._rearm_required = True
            return self._release(now=now, reason="control_clock_invalid", publish=publish)
        raw_age = _state_age(state, now)

        if raw_age < -self.config.state_future_tolerance_s:
            self._rearm_required = True
            return self._release(now=now, reason="state_clock_invalid", publish=publish)
        # ``now`` is finite and receive times are validated finite, so raw_age
        # is finite (or +inf with no state) and at most the tolerance in the
        # future here; only that callback race is clamped.
        age = max(0.0, raw_age)

        if not self._armed or self._rearm_required:
            reason = "rearm_required" if self._rearm_required else "disarmed"
            return self._release(now=now, reason=reason, publish=publish)

        if age >= self.config.state_hard_timeout_s:
            # Measured state has gone away entirely.  Anything we command now is
            # open loop against unknown encoders.
            self._rearm_required = True
            return self._release(now=now, reason="state_hard_timeout", publish=publish)

        state_fresh = age < self.config.state_stale_timeout_s
        effective_enable = bool(enable) and state_fresh

        if state_fresh and state is not None:
            fault = self._tracking_error(state)
            if fault is not None:
                # Latching and immediate: the arm is not where this publisher
                # commanded it, so continuing to hold authority is unsafe.
                self._rearm_required = True
                return self._release(
                    now=now, reason="tracking_error", publish=publish, detail=fault
                )

        if (
            effective_enable
            and self._reseed_required
            and not self.envelope.hard_timeout_latched
        ):
            # The weight reached 0 while armed, so the on-board controller has
            # owned the arm and the envelope's stored pose is stale.  Ramping
            # up from it would pull the arm back toward that frozen pose.
            # Reset from measured state and emit one weight-0 tick at the
            # measured pose; the ramp restarts from there on the next tick.
            assert state is not None  # state_fresh implies a state exists
            try:
                measured = self._reset_from_state(state, now=now, action="reseed")
            except ArmBridgeError as exc:
                self._rearm_required = True
                return self._release(
                    now=now, reason="safety_fault", publish=publish, detail=str(exc)
                )
            frame = self.build_lowcmd(
                measured, {name: 0.0 for name in G1_ARM_SDK_JOINTS}, weight=0.0
            )
            if publish:
                self.transport.publish(frame)
            return BridgeStep(
                frame=frame,
                weight=0.0,
                reason="reseeded",
                engaged=False,
                armed=self.armed,
                rearm_required=self._rearm_required,
                state_age_s=age,
                command=None,
            )

        try:
            command = self.envelope.step(
                target_positions,
                source_timestamp=float(source_timestamp),
                now=now,
                enable=effective_enable,
            )
        except SafetyError as exc:
            self._rearm_required = True
            return self._release(
                now=now, reason="safety_fault", publish=publish, detail=str(exc)
            )

        if command.reason == "hard_timeout":
            # The command source went away for longer than hard_timeout_s.  The
            # envelope stays latched until a measured-state reset, so the bridge
            # cannot track again without arm(); say so instead of reporting an
            # armed bridge that silently holds weight 0.
            self._rearm_required = True
            return self._release(now=now, reason="source_hard_timeout", publish=publish)

        self._last_positions = dict(command.positions)
        if command.blend_weight <= 0.0:
            self._reseed_required = True
        reason = "stale_state" if not state_fresh else command.reason
        frame = self.build_lowcmd(
            command.positions,
            command.velocities,
            weight=command.blend_weight,
        )
        if publish:
            self.transport.publish(frame)
            self._command_history.append(
                (now, dict(command.positions), float(frame.weight))
            )
        return BridgeStep(
            frame=frame,
            weight=frame.weight,
            reason=reason,
            engaged=command.engaged,
            armed=self.armed,
            rearm_required=self._rearm_required,
            state_age_s=age,
            command=command,
        )

    def _tracking_error(self, state: LowStateSnapshot) -> str | None:
        """Describe a tracking-error fault for ``state``, or return ``None``.

        ``state`` is compared with the newest published command at or before
        its receive time, so a state that is a tick or two old is not judged
        against a command it could not yet reflect.
        """
        reference = None
        horizon = state.receive_time_s + self.config.state_future_tolerance_s
        for entry in reversed(self._command_history):
            if entry[0] <= horizon:
                reference = entry
                break
        if reference is None:
            return None
        _, commanded, weight = reference
        if weight < self.config.tracking_error_min_weight:
            return None
        measured = state.arm_positions()
        worst_name = max(
            G1_ARM_SDK_JOINTS, key=lambda name: abs(measured[name] - commanded[name])
        )
        error = abs(measured[worst_name] - commanded[worst_name])
        if error <= self.config.tracking_error_limit_rad:
            return None
        return (
            f"{worst_name} measured-vs-commanded error {error:.6g} rad exceeds "
            f"{self.config.tracking_error_limit_rad:g} rad at weight {weight:.3f}"
        )

    def _release(
        self,
        *,
        now: float,
        reason: str,
        publish: bool,
        detail: str | None = None,
    ) -> BridgeStep:
        """Emit a weight-0 frame holding the last known-good positions.

        Weight 0 removes this publisher's authority; the held positions matter
        only for the instant before the on-board controller takes over, and are
        preferable to publishing zeros, which would read as "go to the zero
        pose".

        Latching faults (state or source hard timeout, clock mismatch, safety
        or tracking error) deliberately drop the weight to 0 in this one frame
        rather than ramping: each one means the measured state or the
        envelope's own state can no longer be trusted, so continuing to
        command any blend of positions would be open loop.  Handing back to
        the on-board controller immediately is the fail-safe.  Non-latching
        exits ramp down through ``step(enable=False)`` before :meth:`disarm`.
        """
        self._command_history.clear()
        positions = self._last_positions
        if positions is None:
            state = self._last_state or self.transport.latest_state()
            positions = state.arm_positions() if state is not None else None
        frame: LowCmdFrame | None = None
        if positions is not None:
            try:
                frame = self.build_lowcmd(
                    positions,
                    {name: 0.0 for name in G1_ARM_SDK_JOINTS},
                    weight=0.0,
                )
            except ArmBridgeError:
                # Unvalidated measured state (never armed) can lie outside the
                # URDF range.  Releasing must never fail, so fall through to
                # the inert frame below.
                frame = None
        if frame is None:
            # Nothing usable measured or commanded: publish a fully inert frame
            # with zero gains so no torque can be requested even if the weight
            # were misinterpreted.
            frame = self.build_lowcmd(
                {name: 0.0 for name in G1_ARM_SDK_JOINTS},
                {name: 0.0 for name in G1_ARM_SDK_JOINTS},
                weight=0.0,
                kp=0.0,
                kd=0.0,
            )
        if publish:
            self.transport.publish(frame)
        return BridgeStep(
            frame=frame,
            weight=0.0,
            reason=reason if detail is None else f"{reason}:{detail}",
            engaged=False,
            armed=self.armed,
            rearm_required=self._rearm_required,
            state_age_s=self.state_age_s(now),
            command=None,
        )

    # -- message construction ----------------------------------------------

    def build_lowcmd(
        self,
        positions: Mapping[str, float],
        velocities: Mapping[str, float] | None = None,
        *,
        weight: float,
        kp: float | None = None,
        kd: float | None = None,
    ) -> LowCmdFrame:
        """Build a CRC-stamped ``LowCmd_`` frame for the 17 arm_sdk joints.

        Only motor slots 12..28 (joints) and 29 (weight) are ever written; the
        twelve leg slots and ``kNotUsedJoint1..5`` stay at their zero defaults so
        a stray leg command cannot be produced by this code path.
        """
        weight_value = float(weight)
        if not math.isfinite(weight_value):
            raise ArmBridgeError("arm_sdk weight must be finite")
        weight_value = min(1.0, max(0.0, weight_value))

        gain_p = self.config.kp if kp is None else float(kp)
        gain_d = self.config.kd if kd is None else float(kd)

        frame = LowCmdFrame(
            mode_pr=int(self.config.mode_pr),
            mode_machine=int(self._mode_machine),
        )
        for name, index in ARM_SDK_MOTOR_INDEX.items():
            try:
                position = float(positions[name])
            except KeyError as exc:
                raise ArmBridgeError(f"missing commanded position for {name}") from exc
            if not math.isfinite(position):
                raise ArmBridgeError(f"commanded position for {name} is not finite")
            limit = self.limits[name]
            if not limit.lower <= position <= limit.upper:
                # The envelope should already guarantee this.  Refusing here
                # keeps a bug in an upstream stage from reaching the wire.
                raise ArmBridgeError(
                    f"commanded {name}={position} is outside the official URDF range "
                    f"[{limit.lower}, {limit.upper}]"
                )
            velocity = 0.0 if velocities is None else float(velocities.get(name, 0.0))
            if not math.isfinite(velocity):
                raise ArmBridgeError(f"commanded velocity for {name} is not finite")
            motor = frame.motor_cmd[index]
            motor.mode = int(self.config.motor_mode)
            motor.q = position
            motor.dq = velocity
            motor.tau = float(self.config.tau_ff)
            motor.kp = gain_p
            motor.kd = gain_d

        # The arm_sdk authority weight.  Motor slot 29 is not a joint: only its
        # ``q`` is meaningful, and gains stay zero.
        frame.motor_cmd[WEIGHT_CHANNEL_INDEX].q = weight_value

        return frame.with_crc()


def _state_age(state: LowStateSnapshot | None, now: float) -> float:
    """Signed age of ``state`` at ``now``; ``inf`` when no state has arrived."""
    if state is None:
        return math.inf
    return float(now) - state.receive_time_s


def describe_motor_map() -> list[tuple[int, str, bool]]:
    """``(motor_index, joint_name, is_arm_sdk_controllable)`` for all 29 joints."""
    return [
        (G1_MOTOR_INDEX[name], name, name in ARM_SDK_MOTOR_INDEX)
        for name in G1_29DOF_JOINTS
    ]


def arm_targets_from_qpos(frame: Sequence[float]) -> dict[str, float]:
    """Slice one 36-wide qpos row (7 root + 29 joints) into arm_sdk targets.

    Matches the convention already used by ``mocap_g1.fault_replay``:
    ``qpos[:, 7 + 12 : 7 + 29]``.
    """
    if len(frame) != 7 + len(G1_29DOF_JOINTS):
        raise ArmBridgeError(
            f"qpos row must be {7 + len(G1_29DOF_JOINTS)} wide, got {len(frame)}"
        )
    return {
        name: float(frame[7 + G1_MOTOR_INDEX[name]]) for name in G1_ARM_SDK_JOINTS
    }
