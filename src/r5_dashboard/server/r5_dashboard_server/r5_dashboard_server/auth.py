"""Password, session and login-throttle storage for the dashboard (spec NF-10).

Everything is kept in the robot's data directory. Files are written atomically and readable only by
the robot's user, so a power cut can't leave a half-written file (spec 6.2, 6.7).
"""

from __future__ import annotations

import hashlib
import hmac
import json
import os
import secrets
import tempfile
import time
from collections.abc import Callable
from dataclasses import dataclass, field
from pathlib import Path

# scrypt cost settings. n=2**15 takes roughly 0.1 s on a Raspberry Pi class computer, which is
# slow enough to make guessing expensive without making login feel slow.
_SCRYPT_N = 2**15
_SCRYPT_R = 8
_SCRYPT_P = 1
_SCRYPT_MAXMEM = 64 * 1024 * 1024

MIN_PASSWORD_LENGTH = 8


def write_private_file(path: Path, text: str) -> None:
    """Write text to path atomically, with permissions 0600."""
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, tmp = tempfile.mkstemp(dir=path.parent, prefix=f".{path.name}.")
    try:
        os.fchmod(fd, 0o600)
        with os.fdopen(fd, "w") as f:
            f.write(text)
            f.flush()
            os.fsync(f.fileno())
        os.replace(tmp, path)
    except BaseException:
        Path(tmp).unlink(missing_ok=True)
        raise


def _scrypt(password: str, salt: bytes, n: int, r: int, p: int) -> bytes:
    return hashlib.scrypt(
        password.encode("utf-8"), salt=salt, n=n, r=r, p=p, maxmem=_SCRYPT_MAXMEM, dklen=32
    )


class PasswordStore:
    """The single dashboard password, stored only as a salted scrypt hash."""

    def __init__(self, path: Path):
        self.path = path

    def is_set(self) -> bool:
        return self.path.is_file()

    def set(self, password: str) -> None:
        if len(password) < MIN_PASSWORD_LENGTH:
            raise ValueError(f"Password must be at least {MIN_PASSWORD_LENGTH} characters.")
        salt = secrets.token_bytes(16)
        record = {
            "algorithm": "scrypt",
            "n": _SCRYPT_N,
            "r": _SCRYPT_R,
            "p": _SCRYPT_P,
            "salt": salt.hex(),
            "hash": _scrypt(password, salt, _SCRYPT_N, _SCRYPT_R, _SCRYPT_P).hex(),
        }
        write_private_file(self.path, json.dumps(record, indent=2) + "\n")

    def verify(self, password: str) -> bool:
        try:
            record = json.loads(self.path.read_text())
        except (OSError, ValueError):
            return False
        if record.get("algorithm") != "scrypt":
            return False
        candidate = _scrypt(
            password, bytes.fromhex(record["salt"]), record["n"], record["r"], record["p"]
        )
        return hmac.compare_digest(candidate, bytes.fromhex(record["hash"]))


class SessionStore:
    """Login sessions. Only a hash of each token is stored, so the file can't be used to log in."""

    def __init__(self, path: Path, lifetime_s: float, clock: Callable[[], float] = time.time):
        self.path = path
        self.lifetime_s = lifetime_s
        self._clock = clock
        self._sessions: dict[str, float] = {}
        self._loaded_mtime: int | None = None
        self._load()

    @staticmethod
    def _key(token: str) -> str:
        return hashlib.sha256(token.encode("utf-8")).hexdigest()

    def _file_mtime(self) -> int | None:
        try:
            return self.path.stat().st_mtime_ns
        except OSError:
            return None

    def _load(self) -> None:
        self._loaded_mtime = self._file_mtime()
        try:
            data = json.loads(self.path.read_text())
            self._sessions = {str(k): float(v) for k, v in data.items()}
        except (OSError, ValueError, AttributeError):
            self._sessions = {}
        self._prune()

    def _reload_if_changed(self) -> None:
        # set_password rewrites the file to log everyone out while the server is running.
        if self._file_mtime() != self._loaded_mtime:
            self._load()

    def _save(self) -> None:
        write_private_file(self.path, json.dumps(self._sessions) + "\n")
        self._loaded_mtime = self._file_mtime()

    def _prune(self) -> None:
        now = self._clock()
        self._sessions = {k: v for k, v in self._sessions.items() if v > now}

    def create(self) -> str:
        token = secrets.token_urlsafe(32)
        self._reload_if_changed()
        self._prune()
        self._sessions[self._key(token)] = self._clock() + self.lifetime_s
        self._save()
        return token

    def is_valid(self, token: str | None) -> bool:
        if not token:
            return False
        self._reload_if_changed()
        expiry = self._sessions.get(self._key(token))
        return expiry is not None and expiry > self._clock()

    def revoke(self, token: str | None) -> None:
        if token and self._sessions.pop(self._key(token), None) is not None:
            self._save()

    def revoke_all(self) -> None:
        self._sessions = {}
        self._save()


@dataclass
class _Attempts:
    failures: int = 0
    locked_until: float = 0.0


@dataclass
class LoginThrottle:
    """Slows down password guessing: after max_failures wrong passwords, wait lockout_s."""

    max_failures: int = 5
    lockout_s: float = 60.0
    clock: Callable[[], float] = time.monotonic
    _by_client: dict[str, _Attempts] = field(default_factory=dict)

    def seconds_locked(self, client: str) -> float:
        attempts = self._by_client.get(client)
        if attempts is None:
            return 0.0
        return max(0.0, attempts.locked_until - self.clock())

    def record_failure(self, client: str) -> None:
        attempts = self._by_client.setdefault(client, _Attempts())
        attempts.failures += 1
        if attempts.failures >= self.max_failures:
            attempts.failures = 0
            attempts.locked_until = self.clock() + self.lockout_s

    def record_success(self, client: str) -> None:
        self._by_client.pop(client, None)
