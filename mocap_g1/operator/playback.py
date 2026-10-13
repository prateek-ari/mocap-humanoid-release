"""Validation and public metadata for read-only recorded-review playback."""

from __future__ import annotations

from pathlib import Path


class PlaybackConfigurationError(ValueError):
    """Raised when a configured review video is not a readable MP4 file."""


def resolve_playback_path(value: str | Path) -> Path:
    """Resolve one explicitly configured MP4 without accepting browser paths."""

    candidate = Path(value).expanduser()
    try:
        resolved = candidate.resolve(strict=True)
    except OSError as exc:
        raise PlaybackConfigurationError(
            f"playback file does not exist or cannot be resolved: {candidate}"
        ) from exc
    if resolved.suffix.lower() != ".mp4":
        raise PlaybackConfigurationError("playback file must use the .mp4 extension")
    if not resolved.is_file():
        raise PlaybackConfigurationError(f"playback path is not a regular file: {candidate}")
    try:
        with resolved.open("rb") as handle:
            header = handle.read(12)
    except OSError as exc:
        raise PlaybackConfigurationError(
            f"playback file is not readable: {candidate}"
        ) from exc
    if len(header) < 12 or header[4:8] != b"ftyp":
        raise PlaybackConfigurationError(
            "playback file is not a valid MP4: missing ftyp header"
        )
    return resolved


def playback_manifest(path: Path) -> dict[str, object]:
    """Return basename-only review metadata safe to expose to the browser."""

    return {
        "schema_version": 1,
        "available": True,
        "kind": "recorded_review_video",
        "presentation_mode": "recorded_playback",
        "filename": path.name,
        "bytes": path.stat().st_size,
        "video_url": "/api/v1/playback/video",
        "review_mode": "simulation_preview",
        "hardware_status": "blocked",
        "command_authority": False,
    }


__all__ = [
    "PlaybackConfigurationError",
    "playback_manifest",
    "resolve_playback_path",
]
