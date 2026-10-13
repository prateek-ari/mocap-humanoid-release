"""Single supervised thread that runs runtime-adapter calls with deadlines.

The controller never waits on a runtime adapter without a deadline.  Every
adapter call runs on one dedicated daemon thread, so calls are serialized
exactly as they were under the controller lock and never run concurrently
with each other.  A call that overruns its deadline cannot be interrupted
from Python: the worker stays occupied ("wedged") until the adapter returns.
While it is wedged every later call fails fast with
:class:`RuntimeAdapterUnavailable` instead of queueing behind it, so a hung
adapter can delay a caller by at most that caller's own deadline.
"""

from __future__ import annotations

import logging
import threading
import time
from typing import Any, Callable


logger = logging.getLogger(__name__)

_UNSET = object()


class _PendingCall:
    __slots__ = ("label", "fn", "args", "result", "error", "done")

    def __init__(self, label: str, fn: Callable[..., Any], args: tuple[Any, ...]):
        self.label = label
        self.fn = fn
        self.args = args
        self.result: Any = _UNSET
        self.error: BaseException | None = None
        self.done = False


class AdapterWorker:
    """Run callables on one daemon thread with a per-call deadline.

    ``call`` raises ``timeout_error(label, timeout_s)`` when the call does not
    finish in time and ``unavailable_error(label, blocked_label)`` when an
    earlier call is still overdue.  Exceptions raised by the callable are
    re-raised unchanged in the caller's thread.
    """

    def __init__(
        self,
        *,
        timeout_error: Callable[[str, float], Exception],
        unavailable_error: Callable[[str, str], Exception],
        name: str = "mocap-g1-runtime-adapter",
    ) -> None:
        self._timeout_error = timeout_error
        self._unavailable_error = unavailable_error
        self._name = name
        self._cond = threading.Condition()
        self._queued: _PendingCall | None = None
        self._running: _PendingCall | None = None
        self._overdue_label: str | None = None
        self._thread: threading.Thread | None = None

    @property
    def blocked_call(self) -> str | None:
        """Label of the call that overran its deadline and has not returned."""

        with self._cond:
            return self._overdue_label

    def call(
        self, label: str, fn: Callable[..., Any], *args: Any, timeout_s: float
    ) -> Any:
        if threading.current_thread() is self._thread:
            # Re-entrant use from inside an adapter callback runs inline; the
            # outer call's deadline still bounds the caller.
            return fn(*args)
        deadline = time.monotonic() + float(timeout_s)
        with self._cond:
            while self._queued is not None or self._running is not None:
                if self._overdue_label is not None:
                    raise self._unavailable_error(label, self._overdue_label)
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    raise self._timeout_error(label, float(timeout_s))
                self._cond.wait(remaining)
            pending = _PendingCall(label, fn, args)
            self._queued = pending
            self._ensure_thread_locked()
            self._cond.notify_all()
            while not pending.done:
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    if self._queued is pending:
                        # Never started: withdraw it so a late start cannot
                        # act on a decision the caller has already abandoned.
                        self._queued = None
                        self._cond.notify_all()
                    else:
                        self._overdue_label = label
                        self._cond.notify_all()
                        logger.error(
                            "Runtime adapter call %s exceeded its %.3f s deadline",
                            label,
                            float(timeout_s),
                        )
                    raise self._timeout_error(label, float(timeout_s))
                self._cond.wait(remaining)
        if pending.error is not None:
            raise pending.error
        return pending.result

    def _ensure_thread_locked(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._thread = threading.Thread(target=self._run, name=self._name, daemon=True)
        self._thread.start()

    def _run(self) -> None:
        while True:
            with self._cond:
                while self._queued is None:
                    self._cond.wait()
                pending = self._queued
                self._queued = None
                self._running = pending
            try:
                pending.result = pending.fn(*pending.args)
            except BaseException as exc:  # re-raised in the caller's thread
                pending.error = exc
            with self._cond:
                pending.done = True
                self._running = None
                if self._overdue_label is not None:
                    logger.warning(
                        "Runtime adapter call %s returned after its deadline; "
                        "adapter calls are accepted again",
                        self._overdue_label,
                    )
                    self._overdue_label = None
                self._cond.notify_all()


__all__ = ["AdapterWorker"]
