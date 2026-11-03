"""Set or reset the dashboard password from a terminal on the robot (spec NF-10).

ros2 run r5_dashboard_server set_password [--data-dir DIR]

Setting a new password logs out every device.
"""

from __future__ import annotations

import argparse
import getpass
import sys
from pathlib import Path

from .auth import MIN_PASSWORD_LENGTH, PasswordStore, SessionStore
from .main import DEFAULT_DATA_DIR


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Set the R5 dashboard password")
    parser.add_argument("--data-dir", type=Path, default=DEFAULT_DATA_DIR)
    args = parser.parse_args(sys.argv[1:] if argv is None else argv)
    data_dir = args.data_dir.expanduser()

    password = getpass.getpass(f"New dashboard password (at least {MIN_PASSWORD_LENGTH} characters): ")
    if getpass.getpass("Repeat it: ") != password:
        print("The passwords don't match. Nothing was changed.", file=sys.stderr)
        return 1
    try:
        PasswordStore(data_dir / "dashboard_password.json").set(password)
    except ValueError as exc:
        print(exc, file=sys.stderr)
        return 1
    SessionStore(data_dir / "sessions.json", lifetime_s=0).revoke_all()
    print("Password set. All devices have been logged out.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
