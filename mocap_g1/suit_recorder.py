"""Always-on raw recording of the incoming Xsens suit datagrams (plan 2026-11-16, B1 and B2).

The live tool records every UDP datagram it receives while it streams to the K1 live bridge
(``--robot-stream``), so a run can be read again later. It is raw evidence, apart from main's
``--record-session`` (which changes the loop for a bit-exact replay).

Threads: the receiver thread calls :meth:`SuitRecorder.tap` right after ``recvfrom``. The tap
only numbers and stamps the datagram and puts it on a bounded queue; a writer thread does all
disk work, so live input never waits for the disk.

Files: ``<root>/<capture id>/capture.json`` and ``chunk-000000.bin``, ``chunk-000001.bin``...
(a new chunk every 10 s). A chunk starts with :data:`HEADER` (magic, version, capture id,
wall-clock and monotonic time when the chunk was opened, chunk number), then records, all
little-endian: ``<u8 seq><f8 monotonic><4s ipv4><u2 port><u2 length><payload>``. ``seq`` counts
every datagram the tap saw, also the ones it could not queue, so a gap in ``seq`` proves a loss.

Bounds (also in ``capture.json`` and the runbook):

- The queue holds at most 4096 records. When it is full, or its oldest record is older than
  3 s, the tap stops queueing and counts, and the live tool prints
  ``SUIT RECORDING INCOMPLETE: the disk does not keep up`` (and a line when it recovers).
  The 3 s limit triggers the warning; it is not a maximum age of what can be lost.
- A crash of the live tool can lose up to 4096 queued records plus the record being written.
  A power loss or operating-system crash can also lose what the operating system had not yet
  written, and the last chunk may end in a partial record.
- A gap in ``seq`` proves a loss inside the recording when a later record survived. A lost
  end (suffix) has no later record to show it.
- The counts in ``capture.json`` (written at each chunk, at recovery and at shutdown) are extra
  information. They never make an export "complete"; only the records do.

Disk: about 30 MB per minute. With less than 10 GB free at start, or below 2 GB while
recording, the recorder does not record (or stops) and says so. It never deletes anything.
"""

from __future__ import annotations

from collections import deque
from collections.abc import Callable
from dataclasses import dataclass, field
import json
import os
from pathlib import Path
import secrets
import shutil
import socket
import struct
import sys
import threading
import time
from itertools import pairwise
from typing import Any, NamedTuple

MAGIC = b"SUITREC\0"
VERSION = 1
#: magic, version, capture id (UTF-8, NUL padded), wall-clock and monotonic time at open, chunk number
HEADER = struct.Struct("<8sH32sddI")
#: seq, monotonic receive time, source IPv4, source port, payload length (the payload follows)
RECORD = struct.Struct("<Qd4sHH")
CHUNK_S = 10.0
QUEUE_MAX = 4096
MAX_AGE_S = 3.0
START_FREE_GB = 10.0
RUN_FREE_GB = 2.0
MB_PER_MINUTE = 30
PRE_ROLL_S = 60.0
POST_ROLL_S = 5.0
EXPORT_WAIT_S = 10.0
INCOMPLETE = "SUIT RECORDING INCOMPLETE: the disk does not keep up"
BOUNDS = ("A crash of the live tool can lose up to 4096 queued records plus the record being "
          "written; the 3 s queue age only triggers the warning. A power loss or operating-system "
          "crash can also lose what the operating system had not yet written, and the last chunk "
          "may end in a partial record. A seq gap proves a loss when a later record survived; a "
          "lost end has no later record to show it. The counts are information only.")


def _stderr(text: str) -> None:
    print(text, file=sys.stderr, flush=True)


def _free_gb(path: Path) -> float:
    return shutil.disk_usage(path).free / 1e9


def write_json_atomic(path: Path, data: Any) -> None:
    partial = path.with_name(path.name + ".tmp")
    partial.write_text(json.dumps(data, indent=2, sort_keys=True), encoding="utf-8")
    os.replace(partial, path)


class SuitRecorder:
    """One capture folder per live tool run; see the module docstring."""

    def __init__(self, root: Path | str, *, session_id: int | None = None,
                 say: Callable[[str], None] = _stderr,
                 free_gb: Callable[[Path], float] = _free_gb,
                 write: Callable[[int, Any], int] = os.write,
                 clock: Callable[[], float] = time.monotonic,
                 wall: Callable[[], float] = time.time,
                 chunk_s: float = CHUNK_S, queue_max: int = QUEUE_MAX,
                 max_age_s: float = MAX_AGE_S) -> None:
        self.root = Path(root)
        self.session_id = session_id
        self.say, self.free_gb, self._write = say, free_gb, write
        self.clock, self.wall = clock, wall
        self.chunk_s, self.queue_max, self.max_age_s = chunk_s, queue_max, max_age_s
        #: "recording", or the operator text when it is off or incomplete (``STATUS``).
        self.state = "not started"
        self.folder: Path | None = None
        self.capture_id = ""
        self.seq = 0  # the next number the tap gives
        self.written = 0
        self.dropped = 0
        self.incomplete = False
        self._queue: deque[tuple[int, float, tuple[str, int], bytes]] = deque()
        self._wake = threading.Event()
        self._accepting = False
        self._overflow = False
        self._stopping = False
        self._counts_due = False
        self._thread: threading.Thread | None = None
        self._fd: int | None = None
        self._chunk = -1
        self._chunk_start = 0.0
        self._meta: dict[str, Any] = {}

    # -- control (live tool thread) --------------------------------------------------------

    def start(self) -> bool:
        """Create the capture folder and start the writer; False (and a printed reason) when
        the disk has too little space or the folder cannot be made."""
        try:
            self.root.mkdir(parents=True, exist_ok=True)
            free = self.free_gb(self.root)
            if free < START_FREE_GB:
                self._off(free)
                return False
            while True:  # exclusive: a second live tool never shares a folder
                self.capture_id = (time.strftime("%Y%m%d-%H%M%S", time.localtime(self.wall()))
                                   + f"-p{os.getpid()}-{secrets.token_hex(3)}")
                try:
                    (self.root / self.capture_id).mkdir()
                except FileExistsError:
                    continue
                break
            self.folder = self.root / self.capture_id
            self._meta = {
                "capture_id": self.capture_id, "pid": os.getpid(), "host": socket.gethostname(),
                "started_wall": self.wall(), "started_monotonic": self.clock(),
                "session_id": self.session_id, "chunk_seconds": self.chunk_s,
                "record": "<u8 seq><f8 monotonic><4s ipv4><u2 port><u2 length><payload>",
                "header": "<8s magic SUITREC\\0><u2 version><32s capture id><f8 wall><f8 monotonic>"
                          "<u4 chunk>",
                "bounds": BOUNDS}
            self._write_meta()
        except OSError as exc:
            self.state = f"SUIT RECORDING OFF: cannot create a capture folder in {self.root} ({exc})."
            self.say(self.state)
            return False
        self.state = "recording"
        self._accepting = True
        self._thread = threading.Thread(target=self._run, name="suit-recorder", daemon=True)
        self._thread.start()
        self.say(f"SUIT RECORDING to {self.folder} (about {MB_PER_MINUTE} MB per minute)")
        return True

    def close(self, timeout: float = 5.0) -> None:
        """Stop taking datagrams, write what is queued, then the final counts."""
        self._accepting = False
        self._stopping = True
        self._wake.set()
        if self._thread is not None:
            self._thread.join(timeout)
            if self._thread.is_alive():
                self.say(f"SUIT RECORDING: the writer did not finish in {timeout:g} s; "
                         f"{len(self._queue)} records were not written")

    # -- receiver thread -------------------------------------------------------------------

    def tap(self, data: bytes, source: tuple[str, int]) -> None:
        """Number, stamp and queue one datagram. Never touches the disk."""
        if not self._accepting:
            return
        now = self.clock()
        seq = self.seq
        self.seq += 1
        queue = self._queue
        if self._overflow:
            if queue:
                self.dropped += 1
                return
            self._overflow = False
            self._counts_due = True
            self.state = "recording"
            self.say(f"SUIT RECORDING recovered: recording again ({self.dropped} datagrams lost "
                     "so far; the gap is in the recording)")
        else:
            try:
                too_old = bool(queue) and now - queue[0][1] > self.max_age_s
            except IndexError:  # the writer emptied it meanwhile
                too_old = False
            if len(queue) >= self.queue_max or too_old:
                self._overflow = self.incomplete = True
                self.dropped += 1
                self.state = INCOMPLETE
                self.say(INCOMPLETE)
                self._wake.set()
                return
        queue.append((seq, now, source, data))
        self._wake.set()

    # -- writer thread ---------------------------------------------------------------------

    def _run(self) -> None:
        try:
            while True:
                self._wake.wait(0.5)
                self._wake.clear()
                while self._queue:
                    if not self._write_record(*self._queue[0]):
                        break
                    self._queue.popleft()
                if self._counts_due:
                    self._counts_due = False
                    self._write_meta()
                if self._stopping and (not self._queue or not self._accepting_records()):
                    break
        except OSError as exc:
            self._fail(exc)
        finally:
            if self._fd is not None:
                os.close(self._fd)
                self._fd = None
            try:
                self._write_meta()
            except OSError:
                pass

    def _accepting_records(self) -> bool:
        return self.state == "recording" or self.state == INCOMPLETE

    def _write_record(self, seq: int, stamp: float, source: tuple[str, int], data: bytes) -> bool:
        if not self._accepting_records():  # stopped for low disk: drop what is queued
            self.dropped += len(self._queue)
            self._queue.clear()
            return False
        if (self._fd is None or stamp - self._chunk_start >= self.chunk_s) and not self._roll(stamp):
            return False
        try:
            address = socket.inet_aton(source[0])
        except (OSError, TypeError, IndexError):
            address = b"\0\0\0\0"
        port = source[1] if len(source) > 1 and isinstance(source[1], int) else 0
        self._write_all(RECORD.pack(seq, stamp, address, port & 0xFFFF, len(data)) + bytes(data))
        self.written += 1
        return True

    def _roll(self, stamp: float) -> bool:
        if self._fd is not None:
            os.close(self._fd)
            self._fd = None
            free = self.free_gb(self.root)
            if free < RUN_FREE_GB:
                self._off(free)
                self._accepting = False
                self.dropped += len(self._queue)
                self._queue.clear()
                return False
        self._chunk += 1
        assert self.folder is not None
        path = self.folder / f"chunk-{self._chunk:06d}.bin"
        self._fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o644)
        self._chunk_start = stamp
        self._write_all(HEADER.pack(MAGIC, VERSION, self.capture_id.encode()[:32], self.wall(),
                                    self.clock(), self._chunk))
        self._write_meta()
        return True

    def _write_all(self, blob: bytes) -> None:
        """Hand the bytes to the operating system now; finish a short write or fail."""
        view = memoryview(blob)
        while view:
            count = self._write(self._fd, view)  # type: ignore[arg-type]
            if not count or count < 0:
                raise OSError("the disk accepted no bytes")
            view = view[count:]

    def _fail(self, exc: OSError) -> None:
        self._accepting = False
        self.incomplete = True
        self.dropped += len(self._queue)
        self._queue.clear()
        where = self.folder / f"chunk-{self._chunk:06d}.bin" if self.folder else self.root
        self.state = (f"SUIT RECORDING INCOMPLETE: cannot write {where} ({exc}). The run goes on "
                      "without suit recording.")
        self.say(self.state)

    def _off(self, free: float) -> None:
        self.state = (f"SUIT RECORDING OFF: only {free:.1f} GB free in {self.root}. Delete old "
                      "capture folders there.")
        self.say(self.state)

    def _write_meta(self) -> None:
        if self.folder is None:
            return
        write_json_atomic(self.folder / "capture.json", {
            **self._meta, "state": self.state, "updated_wall": self.wall(),
            "counts": {"seen": self.seq, "written": self.written, "dropped": self.dropped,
                       "incomplete": self.incomplete, "chunks": self._chunk + 1}})


# -- reading ------------------------------------------------------------------------------


class Record(NamedTuple):
    seq: int
    monotonic: float
    ip: str
    port: int
    payload: bytes


@dataclass
class Chunk:
    name: str
    header: dict[str, Any] | None
    records: list[Record] = field(default_factory=list)
    #: bytes after the last whole record (a partial record at a crash), and why reading stopped
    tail_bytes: int = 0
    error: str | None = None


def parse_chunk(data: bytes, name: str = "") -> Chunk:
    """Records of one chunk file; a truncated or unreadable end is reported, never raised."""
    if len(data) < HEADER.size:
        return Chunk(name, None, tail_bytes=len(data), error="no whole header")
    magic, version, capture, wall, mono, number = HEADER.unpack_from(data)
    if magic != MAGIC or version != VERSION:
        return Chunk(name, None, tail_bytes=len(data), error="not a suit recording chunk")
    chunk = Chunk(name, {"capture_id": capture.rstrip(b"\0").decode(errors="replace"),
                         "wall": wall, "monotonic": mono, "chunk": number})
    offset = HEADER.size
    while offset + RECORD.size <= len(data):
        seq, stamp, address, port, length = RECORD.unpack_from(data, offset)
        end = offset + RECORD.size + length
        if end > len(data):
            break
        last = chunk.records[-1] if chunk.records else None
        if last is not None and (seq <= last.seq or stamp < last.monotonic):
            chunk.error = f"unreadable from byte {offset}"
            break
        chunk.records.append(Record(seq, stamp, socket.inet_ntoa(address), port,
                                    data[offset + RECORD.size:end]))
        offset = end
    chunk.tail_bytes = len(data) - offset
    return chunk


def chunk_files(folder: Path) -> list[Path]:
    """The chunk files, from the directory (not from a manifest), in order."""
    return sorted(Path(folder).glob("chunk-*.bin"))


def read_capture(folder: Path) -> list[Chunk]:
    return [parse_chunk(path.read_bytes(), path.name) for path in chunk_files(folder)]


def _first_stamp(path: Path) -> float | None:
    with path.open("rb") as handle:
        head = handle.read(HEADER.size + RECORD.size)
    if len(head) < HEADER.size + RECORD.size or head[:len(MAGIC)] != MAGIC:
        return None
    return RECORD.unpack_from(head, HEADER.size)[1]


def _select(paths: list[Path], low: float, high: float) -> list[Path]:
    """Whole chunks whose time span (first record to the next chunk's first) meets
    ``low .. high``; an unreadable chunk next to a chosen one is chosen too."""
    stamps = [_first_stamp(path) for path in paths]
    chosen = [False] * len(paths)
    for index, start in enumerate(stamps):
        if start is None:
            continue
        end = next((s for s in stamps[index + 1:] if s is not None), float("inf"))
        chosen[index] = start <= high and end > low
    for index, start in enumerate(stamps):
        if start is None and ((index > 0 and chosen[index - 1])
                              or (index + 1 < len(paths) and chosen[index + 1])):
            chosen[index] = True
    return [path for path, keep in zip(paths, chosen) if keep]


def export(folder: Path, go: float, release: float | None, *,
           wait_s: float = EXPORT_WAIT_S, sleep: Callable[[float], None] = time.sleep,
           clock: Callable[[], float] = time.monotonic) -> tuple[dict[str, bytes], dict[str, Any]]:
    """Copies of the chunk files that overlap Go - 60 s .. release + 5 s (to the capture's end
    when the release is unknown), plus ``capture.json``, and the ``suit-recording.json`` report
    read from those copies. Waits up to ``wait_s`` for the writer when the end is not there yet."""
    folder = Path(folder)
    low = go - PRE_ROLL_S
    high = None if release is None else release + POST_ROLL_S
    deadline = clock() + wait_s
    while True:
        copies = {path.name: path.read_bytes()
                  for path in _select(chunk_files(folder), low, float("inf") if high is None else high)}
        chunks = [parse_chunk(data, name) for name, data in copies.items()]
        last = max((c.records[-1].monotonic for c in chunks if c.records), default=None)
        if high is None or (last is not None and last >= high) or clock() >= deadline:
            break
        sleep(0.5)
    try:
        meta_bytes = (folder / "capture.json").read_bytes()
        meta = json.loads(meta_bytes)
        copies["capture.json"] = meta_bytes
    except (OSError, ValueError):
        meta = {}
    report = assess(chunks, go=go, release=release, all_names=[p.name for p in chunk_files(folder)])
    report.update(capture_id=meta.get("capture_id", folder.name), folder=str(folder),
                  capture_state=meta.get("state"), counts=meta.get("counts"),
                  counts_note="information only; completeness comes from the records", bounds=BOUNDS)
    return copies, report


def assess(chunks: list[Chunk], *, go: float, release: float | None,
           all_names: list[str] | None = None) -> dict[str, Any]:
    """``complete`` (true, false or "unknown"), gaps and coverage of Go - 60 s .. release + 5 s."""
    low = go - PRE_ROLL_S
    high = None if release is None else release + POST_ROLL_S
    records = sorted((r for c in chunks for r in c.records), key=lambda r: r.seq)
    gaps: list[dict[str, Any]] = []
    for before, after in pairwise(records):
        if after.seq != before.seq + 1 and after.monotonic >= low and (
                high is None or before.monotonic <= high):
            gaps.append({"seq": [before.seq + 1, after.seq - 1],
                         "time": [before.monotonic, after.monotonic]})
    names = [c.name for c in chunks]
    unreadable = [{"file": c.name, "error": c.error or f"{c.tail_bytes} bytes after the last record"}
                  for c in chunks
                  if c.error or (c.tail_bytes and all_names and c.name != all_names[-1])]
    missing_chunks: list[str] = []
    if all_names and names:
        numbers = sorted(int(n[6:12]) for n in names if n[6:12].isdecimal())
        if numbers:
            missing_chunks = [f"chunk-{n:06d}.bin" for n in range(numbers[0], numbers[-1] + 1)
                              if f"chunk-{n:06d}.bin" not in all_names]
    report: dict[str, Any] = {
        "go_monotonic": go, "release_monotonic": release,
        "interval": [low, high], "gaps": gaps, "unreadable": unreadable,
        "missing_chunks": missing_chunks, "files": names, "records": len(records)}
    if not records:
        report.update(complete=False if high is not None else "unknown", coverage=None,
                      why="no readable record in the interval")
        return report
    first, last = records[0], records[-1]
    start_ok = first.monotonic <= low or first.seq == 0
    report["coverage"] = {"first": {"seq": first.seq, "monotonic": first.monotonic},
                          "last": {"seq": last.seq, "monotonic": last.monotonic},
                          "seconds_before_go": round(go - first.monotonic, 3),
                          "seconds_after_release": None if release is None
                          else round(last.monotonic - release, 3)}
    if first.monotonic > low:
        report["missing_pre_roll_s"] = round(first.monotonic - low, 3)
    if not start_ok:
        gaps.insert(0, {"seq": [None, first.seq - 1], "time": [low, first.monotonic]})
    if gaps or unreadable or missing_chunks:
        report["complete"] = False
    elif high is None:
        report["complete"] = "unknown"
        report["why"] = "the release time is unknown (interrupted attempt)"
    elif last.monotonic < high:
        report["complete"] = "unknown"
        report["why"] = "no record at or after release + 5 s: the end cannot be verified"
    else:
        report["complete"] = True
    return report
