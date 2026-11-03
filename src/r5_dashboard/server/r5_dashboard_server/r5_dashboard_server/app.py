"""The dashboard web server (spec D4, D7, NF-10).

It serves the built web app, handles login, and passes the browser's WebSocket through to
rosbridge, which only listens on localhost. Nothing reaches ROS without a valid session.
"""

from __future__ import annotations

import asyncio
import logging
from pathlib import Path
from urllib.parse import urlsplit

import aiohttp
from aiohttp import WSMsgType, web

from .auth import LoginThrottle, PasswordStore, SessionStore

log = logging.getLogger("r5_dashboard_server")

SESSION_COOKIE = "r5_session"

PASSWORDS = web.AppKey("passwords", PasswordStore)
SESSIONS = web.AppKey("sessions", SessionStore)
THROTTLE = web.AppKey("throttle", LoginThrottle)
ROSBRIDGE_URL = web.AppKey("rosbridge_url", str)
WEB_ROOT = web.AppKey("web_root", object)  # Path | None
HTTP_CLIENT = web.AppKey("http_client", aiohttp.ClientSession)

_SECURITY_HEADERS = {
    "X-Content-Type-Options": "nosniff",
    "X-Frame-Options": "DENY",
    "Referrer-Policy": "no-referrer",
    "Content-Security-Policy": (
        "default-src 'self'; connect-src 'self'; img-src 'self' data: blob:; "
        "style-src 'self' 'unsafe-inline'; frame-ancestors 'none'"
    ),
}


def _json_error(status: int, message: str, **extra) -> web.Response:
    return web.json_response({"error": message, **extra}, status=status)


def _session_token(request: web.Request) -> str | None:
    return request.cookies.get(SESSION_COOKIE)


def _is_authenticated(request: web.Request) -> bool:
    return request.app[SESSIONS].is_valid(_session_token(request))


def _same_origin(request: web.Request) -> bool:
    """Reject requests a page on another site makes on the user's behalf.

    Browsers always send Origin on WebSocket upgrades and cross-site POSTs. Requests without it
    (for example from command-line tools) still need the session cookie.
    """
    origin = request.headers.get("Origin")
    if origin is None:
        return True
    return urlsplit(origin).netloc.lower() == request.host.lower()


@web.middleware
async def _security_headers(request: web.Request, handler):
    response = await handler(request)
    for name, value in _SECURITY_HEADERS.items():
        response.headers.setdefault(name, value)
    return response


async def _get_session(request: web.Request) -> web.Response:
    return web.json_response(
        {
            "authenticated": _is_authenticated(request),
            "passwordSet": request.app[PASSWORDS].is_set(),
        }
    )


async def _login(request: web.Request) -> web.Response:
    if not _same_origin(request):
        return _json_error(403, "Request from another site refused.")
    passwords = request.app[PASSWORDS]
    if not passwords.is_set():
        return _json_error(
            503,
            "No dashboard password has been set on the robot. "
            "Run 'ros2 run r5_dashboard_server set_password' on the robot.",
        )

    client = request.remote or "unknown"
    throttle = request.app[THROTTLE]
    wait = throttle.seconds_locked(client)
    if wait > 0:
        return _json_error(
            429,
            f"Too many wrong passwords. Try again in {int(wait) + 1} seconds.",
            retryAfter=int(wait) + 1,
        )

    try:
        body = await request.json()
        password = body["password"]
        if not isinstance(password, str):
            raise TypeError
    except (ValueError, KeyError, TypeError):
        return _json_error(400, "Expected a JSON body with a password.")

    # scrypt is deliberately slow, so keep it off the event loop.
    ok = await asyncio.get_running_loop().run_in_executor(None, passwords.verify, password)
    if not ok:
        throttle.record_failure(client)
        log.warning("Failed dashboard login from %s", client)
        return _json_error(401, "Wrong password.")

    throttle.record_success(client)
    sessions = request.app[SESSIONS]
    token = sessions.create()
    log.info("Dashboard login from %s", client)
    response = web.json_response({"authenticated": True})
    response.set_cookie(
        SESSION_COOKIE,
        token,
        max_age=int(sessions.lifetime_s),
        path="/",
        secure=True,
        httponly=True,
        samesite="Strict",
    )
    return response


async def _logout(request: web.Request) -> web.Response:
    if not _same_origin(request):
        return _json_error(403, "Request from another site refused.")
    request.app[SESSIONS].revoke(_session_token(request))
    response = web.Response(status=204)
    response.del_cookie(SESSION_COOKIE, path="/")
    return response


async def _pump(source, sink) -> None:
    """Copy messages from one WebSocket to the other until source closes."""
    async for msg in source:
        if msg.type == WSMsgType.TEXT:
            await sink.send_str(msg.data)
        elif msg.type == WSMsgType.BINARY:
            await sink.send_bytes(msg.data)
        elif msg.type == WSMsgType.ERROR:
            break


async def _ros_websocket(request: web.Request) -> web.StreamResponse:
    if not _is_authenticated(request):
        return _json_error(401, "Not logged in.")
    if not _same_origin(request):
        return _json_error(403, "Request from another site refused.")

    try:
        upstream = await request.app[HTTP_CLIENT].ws_connect(
            request.app[ROSBRIDGE_URL], max_msg_size=0, heartbeat=10
        )
    except (aiohttp.ClientError, OSError) as exc:
        log.warning("Can't reach rosbridge at %s: %s", request.app[ROSBRIDGE_URL], exc)
        return _json_error(502, "The robot's ROS bridge isn't running.")

    client = web.WebSocketResponse(max_msg_size=0, heartbeat=10)
    await client.prepare(request)
    tasks = [
        asyncio.ensure_future(_pump(client, upstream)),
        asyncio.ensure_future(_pump(upstream, client)),
    ]
    try:
        await asyncio.wait(tasks, return_when=asyncio.FIRST_COMPLETED)
    finally:
        for task in tasks:
            task.cancel()
        await asyncio.gather(*tasks, return_exceptions=True)
        await upstream.close()
        await client.close()
    return client


async def _static(request: web.Request) -> web.StreamResponse:
    web_root: Path | None = request.app[WEB_ROOT]
    if web_root is None:
        return web.Response(
            status=503,
            text="The dashboard web app hasn't been built or installed. See README.md.",
        )
    relative = request.match_info["path"]
    root = web_root.resolve()
    target = (root / relative).resolve()
    if root not in target.parents and target != root:
        raise web.HTTPNotFound()
    if not target.is_file():
        # The app uses hash routes, so anything unknown gets the app itself.
        target = root / "index.html"
        if not target.is_file():
            raise web.HTTPNotFound()
    response = web.FileResponse(target)
    if relative.startswith("assets/"):
        # Vite puts a content hash in these names, so they never change.
        response.headers["Cache-Control"] = "public, max-age=31536000, immutable"
    else:
        response.headers["Cache-Control"] = "no-cache"
    return response


async def _http_client_ctx(app: web.Application):
    app[HTTP_CLIENT] = aiohttp.ClientSession()
    yield
    await app[HTTP_CLIENT].close()


def make_app(
    *,
    passwords: PasswordStore,
    sessions: SessionStore,
    rosbridge_url: str,
    web_root: Path | None,
    throttle: LoginThrottle | None = None,
) -> web.Application:
    app = web.Application(middlewares=[_security_headers], client_max_size=64 * 1024)
    app[PASSWORDS] = passwords
    app[SESSIONS] = sessions
    app[THROTTLE] = throttle or LoginThrottle()
    app[ROSBRIDGE_URL] = rosbridge_url
    app[WEB_ROOT] = web_root
    app.cleanup_ctx.append(_http_client_ctx)
    app.router.add_get("/api/session", _get_session)
    app.router.add_post("/api/login", _login)
    app.router.add_post("/api/logout", _logout)
    app.router.add_get("/ws", _ros_websocket)
    app.router.add_get("/{path:.*}", _static)
    return app


def make_redirect_app(https_port: int) -> web.Application:
    """Plain-HTTP site that sends every request to the HTTPS address."""

    async def redirect(request: web.Request) -> web.Response:
        host = request.host.rsplit(":", 1)[0] if not request.host.endswith("]") else request.host
        port = "" if https_port == 443 else f":{https_port}"
        raise web.HTTPMovedPermanently(f"https://{host}{port}{request.path_qs}")

    app = web.Application()
    app.router.add_route("*", "/{path:.*}", redirect)
    return app
