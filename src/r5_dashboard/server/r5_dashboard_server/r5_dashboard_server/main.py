"""Start the dashboard server: ros2 run r5_dashboard_server server [options]."""

from __future__ import annotations

import argparse
import asyncio
import logging
import sys
from pathlib import Path

from aiohttp import web

from .app import make_app, make_redirect_app
from .auth import LoginThrottle, PasswordStore, SessionStore
from .tls import ensure_certificate, make_ssl_context

DEFAULT_DATA_DIR = Path.home() / ".r5"

log = logging.getLogger("r5_dashboard_server")


def default_web_root() -> Path | None:
    """The web app installed with this package, if it was built before colcon build."""
    try:
        from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
    except ImportError:
        return None
    try:
        root = Path(get_package_share_directory("r5_dashboard_server")) / "web"
    except PackageNotFoundError:
        return None
    return root if (root / "index.html").is_file() else None


def strip_ros_args(argv: list[str]) -> list[str]:
    """Drop the --ros-args section that ros2 launch adds to the command line."""
    if "--ros-args" not in argv:
        return argv
    start = argv.index("--ros-args")
    end = argv.index("--", start) if "--" in argv[start:] else len(argv)
    return argv[:start] + argv[end + 1 :]


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="R5 dashboard web server")
    parser.add_argument("--data-dir", type=Path, default=DEFAULT_DATA_DIR)
    parser.add_argument(
        "--web-root", type=Path, default=None, help="Folder holding the built web app"
    )
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--https-port", type=int, default=8443)
    parser.add_argument(
        "--http-port", type=int, default=8080, help="Redirects to HTTPS. 0 turns it off."
    )
    parser.add_argument("--rosbridge-url", default="ws://127.0.0.1:9090")
    parser.add_argument("--session-days", type=float, default=30.0)
    return parser.parse_args(strip_ros_args(argv))


async def serve(args: argparse.Namespace) -> None:
    data_dir: Path = args.data_dir.expanduser()
    web_root = args.web_root or default_web_root()
    if web_root is None:
        log.warning("No built web app found. Pass --web-root or rebuild after 'npm run build'.")

    passwords = PasswordStore(data_dir / "dashboard_password.json")
    if not passwords.is_set():
        log.warning(
            "No dashboard password set. Run 'ros2 run r5_dashboard_server set_password'."
        )
    sessions = SessionStore(data_dir / "sessions.json", lifetime_s=args.session_days * 86400)
    cert, key = ensure_certificate(data_dir / "tls")

    app = make_app(
        passwords=passwords,
        sessions=sessions,
        rosbridge_url=args.rosbridge_url,
        web_root=web_root,
        throttle=LoginThrottle(),
    )
    runner = web.AppRunner(app, access_log=None)
    await runner.setup()
    await web.TCPSite(
        runner, args.host, args.https_port, ssl_context=make_ssl_context(cert, key)
    ).start()
    log.info("Dashboard on https://<robot>:%d (web app: %s)", args.https_port, web_root)

    redirect_runner = None
    if args.http_port:
        redirect_runner = web.AppRunner(make_redirect_app(args.https_port), access_log=None)
        await redirect_runner.setup()
        await web.TCPSite(redirect_runner, args.host, args.http_port).start()

    try:
        await asyncio.Event().wait()
    finally:
        if redirect_runner:
            await redirect_runner.cleanup()
        await runner.cleanup()


def main(argv: list[str] | None = None) -> None:
    logging.basicConfig(level=logging.INFO, format="[%(levelname)s] [%(name)s] %(message)s")
    args = parse_args(sys.argv[1:] if argv is None else argv)
    try:
        asyncio.run(serve(args))
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
