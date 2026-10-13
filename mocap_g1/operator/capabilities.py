"""Capability-manifest signing and fail-closed verification."""

from __future__ import annotations

from datetime import datetime, timezone
import hashlib
import hmac
import json
from typing import Any, Mapping

from .models import CapabilityLock


WHOLE_BODY_CAPABILITY = "whole_body_hardware"


def _canonical_payload(manifest: Mapping[str, Any]) -> bytes:
    unsigned = {key: value for key, value in manifest.items() if key != "signature"}
    return json.dumps(
        unsigned, sort_keys=True, separators=(",", ":"), ensure_ascii=True
    ).encode("utf-8")


def sign_manifest(manifest: Mapping[str, Any], secret: bytes | str) -> dict[str, Any]:
    """Return a copied manifest with a canonical HMAC-SHA256 signature."""

    key = secret.encode("utf-8") if isinstance(secret, str) else bytes(secret)
    if not key:
        raise ValueError("signing secret must not be empty")
    result = dict(manifest)
    result.pop("signature", None)
    result["signature"] = hmac.new(
        key, _canonical_payload(result), hashlib.sha256
    ).hexdigest()
    return result


class CapabilityManifestVerifier:
    """Verify a manifest and express every failure as a locked capability."""

    def __init__(self, secret: bytes | str | None) -> None:
        if isinstance(secret, str):
            secret = secret.encode("utf-8")
        self._secret = bytes(secret) if secret else None

    def verify(
        self,
        manifest: Mapping[str, Any] | None,
        *,
        now: datetime | None = None,
    ) -> CapabilityLock:
        if self._secret is None:
            return CapabilityLock(reason="Capability verification key is not configured")
        if not isinstance(manifest, Mapping):
            return CapabilityLock(reason="No capability manifest is installed")

        signature = manifest.get("signature")
        if not isinstance(signature, str):
            return CapabilityLock(reason="Capability manifest has no signature")
        expected = hmac.new(
            self._secret, _canonical_payload(manifest), hashlib.sha256
        ).hexdigest()
        if not hmac.compare_digest(signature, expected):
            return CapabilityLock(reason="Capability manifest signature is invalid")

        manifest_id = manifest.get("manifest_id")
        if not isinstance(manifest_id, str) or not manifest_id.strip():
            return CapabilityLock(reason="Capability manifest ID is missing")
        capabilities = manifest.get("capabilities")
        if not isinstance(capabilities, list) or not all(
            isinstance(value, str) for value in capabilities
        ):
            return CapabilityLock(reason="Capability manifest capabilities are invalid")
        expires_at = manifest.get("expires_at")
        if not isinstance(expires_at, str):
            return CapabilityLock(reason="Capability manifest expiry is missing")
        try:
            expiry = datetime.fromisoformat(expires_at.replace("Z", "+00:00"))
            if expiry.tzinfo is None:
                expiry = expiry.replace(tzinfo=timezone.utc)
        except ValueError:
            return CapabilityLock(reason="Capability manifest expiry is invalid")
        clock = now or datetime.now(timezone.utc)
        if clock.tzinfo is None:
            clock = clock.replace(tzinfo=timezone.utc)
        if expiry <= clock:
            return CapabilityLock(
                reason="Capability manifest has expired",
                manifest_id=manifest_id,
                expires_at=expires_at,
                capabilities=tuple(capabilities),
            )
        if WHOLE_BODY_CAPABILITY not in capabilities:
            return CapabilityLock(
                manifest_verified=True,
                reason="Whole-body hardware capability is not authorized",
                manifest_id=manifest_id,
                expires_at=expires_at,
                capabilities=tuple(capabilities),
            )
        return CapabilityLock(
            whole_body_unlocked=True,
            manifest_verified=True,
            reason="Verified capability manifest authorizes whole-body hardware control",
            manifest_id=manifest_id,
            expires_at=expires_at,
            capabilities=tuple(capabilities),
        )
