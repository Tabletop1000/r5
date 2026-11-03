import json
import stat

import pytest

from r5_dashboard_server.auth import LoginThrottle, PasswordStore, SessionStore


class FakeClock:
    def __init__(self, now=1000.0):
        self.now = now

    def __call__(self):
        return self.now


def test_password_round_trip(tmp_path):
    store = PasswordStore(tmp_path / "pw.json")
    assert not store.is_set()
    store.set("correct horse")
    assert store.is_set()
    assert store.verify("correct horse")
    assert not store.verify("wrong horse")


def test_password_file_holds_no_plain_text_and_is_private(tmp_path):
    path = tmp_path / "pw.json"
    PasswordStore(path).set("correct horse")
    assert "correct horse" not in path.read_text()
    assert stat.S_IMODE(path.stat().st_mode) == 0o600


def test_short_password_rejected(tmp_path):
    with pytest.raises(ValueError):
        PasswordStore(tmp_path / "pw.json").set("short")


def test_verify_without_password_file_fails(tmp_path):
    assert not PasswordStore(tmp_path / "pw.json").verify("anything")


def test_sessions_expire_and_survive_restart(tmp_path):
    clock = FakeClock()
    path = tmp_path / "sessions.json"
    sessions = SessionStore(path, lifetime_s=100, clock=clock)
    token = sessions.create()
    assert sessions.is_valid(token)
    assert token not in path.read_text()

    reloaded = SessionStore(path, lifetime_s=100, clock=clock)
    assert reloaded.is_valid(token)

    clock.now += 101
    assert not reloaded.is_valid(token)


def test_session_revoke(tmp_path):
    sessions = SessionStore(tmp_path / "sessions.json", lifetime_s=100)
    token = sessions.create()
    sessions.revoke(token)
    assert not sessions.is_valid(token)
    assert not sessions.is_valid(None)
    assert not sessions.is_valid("made-up")


def test_running_store_sees_revoke_all_from_another_process(tmp_path):
    path = tmp_path / "sessions.json"
    server_view = SessionStore(path, lifetime_s=100)
    token = server_view.create()
    SessionStore(path, lifetime_s=0).revoke_all()  # what set_password does
    assert not server_view.is_valid(token)


def test_corrupt_session_file_is_treated_as_empty(tmp_path):
    path = tmp_path / "sessions.json"
    path.write_text("not json")
    assert not SessionStore(path, lifetime_s=100).is_valid("x")
    path.write_text(json.dumps([1, 2]))
    assert not SessionStore(path, lifetime_s=100).is_valid("x")


def test_throttle_locks_after_max_failures_then_unlocks():
    clock = FakeClock()
    throttle = LoginThrottle(max_failures=3, lockout_s=60, clock=clock)
    for _ in range(2):
        throttle.record_failure("a")
    assert throttle.seconds_locked("a") == 0
    throttle.record_failure("a")
    assert throttle.seconds_locked("a") == 60
    assert throttle.seconds_locked("b") == 0
    clock.now += 61
    assert throttle.seconds_locked("a") == 0


def test_throttle_success_resets_count():
    throttle = LoginThrottle(max_failures=2, lockout_s=60)
    throttle.record_failure("a")
    throttle.record_success("a")
    throttle.record_failure("a")
    assert throttle.seconds_locked("a") == 0
