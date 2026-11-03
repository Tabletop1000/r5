import asyncio
import json
from http.cookies import SimpleCookie

from aiohttp import WSMsgType, web
from aiohttp.test_utils import AioHTTPTestCase, TestServer

from r5_dashboard_server.app import SESSION_COOKIE, make_app, make_redirect_app
from r5_dashboard_server.auth import LoginThrottle, PasswordStore, SessionStore

PASSWORD = "correct horse"


async def fake_rosbridge(request):
    """Echoes every message back, standing in for rosbridge."""
    ws = web.WebSocketResponse()
    await ws.prepare(request)
    async for msg in ws:
        if msg.type == WSMsgType.TEXT:
            await ws.send_str("echo:" + msg.data)
        elif msg.type == WSMsgType.BINARY:
            await ws.send_bytes(b"echo:" + msg.data)
    return ws


class DashboardAppTest(AioHTTPTestCase):
    async def asyncSetUp(self):
        import tempfile
        from pathlib import Path

        self._tmp = tempfile.TemporaryDirectory()
        self.tmp = Path(self._tmp.name)
        web_root = self.tmp / "web"
        (web_root / "assets").mkdir(parents=True)
        (web_root / "index.html").write_text("<html>app</html>")
        (web_root / "assets" / "app-123.js").write_text("console.log(1)")
        (self.tmp / "secret.txt").write_text("secret")

        rosbridge_app = web.Application()
        rosbridge_app.router.add_get("/", fake_rosbridge)
        self.rosbridge = TestServer(rosbridge_app)
        await self.rosbridge.start_server()

        self.passwords = PasswordStore(self.tmp / "pw.json")
        self.passwords.set(PASSWORD)
        self.web_root = web_root
        await super().asyncSetUp()

    async def asyncTearDown(self):
        await super().asyncTearDown()
        await self.rosbridge.close()
        self._tmp.cleanup()

    async def get_application(self):
        return make_app(
            passwords=self.passwords,
            sessions=SessionStore(self.tmp / "sessions.json", lifetime_s=3600),
            rosbridge_url=str(self.rosbridge.make_url("/")).replace("http", "ws", 1),
            web_root=self.web_root,
            throttle=LoginThrottle(max_failures=3, lockout_s=60),
        )

    async def login(self, password=PASSWORD):
        resp = await self.client.post("/api/login", json={"password": password})
        token = None
        if SESSION_COOKIE in resp.cookies:
            token = resp.cookies[SESSION_COOKIE].value
        return resp, token

    def auth_headers(self, token):
        return {"Cookie": f"{SESSION_COOKIE}={token}"}

    async def test_session_starts_logged_out(self):
        resp = await self.client.get("/api/session")
        assert resp.status == 200
        assert await resp.json() == {"authenticated": False, "passwordSet": True}

    async def test_login_sets_secure_cookie(self):
        resp, token = await self.login()
        assert resp.status == 200
        assert token
        cookie = SimpleCookie(resp.headers["Set-Cookie"])[SESSION_COOKIE]
        assert cookie["secure"]
        assert cookie["httponly"]
        assert cookie["samesite"] == "Strict"

        resp = await self.client.get("/api/session", headers=self.auth_headers(token))
        assert (await resp.json())["authenticated"] is True

    async def test_wrong_password_then_lockout(self):
        for _ in range(3):
            resp, token = await self.login("wrong password")
            assert resp.status == 401
            assert token is None
        resp, _ = await self.login()
        assert resp.status == 429
        assert (await resp.json())["retryAfter"] > 0

    async def test_bad_login_body(self):
        resp = await self.client.post("/api/login", data="nonsense")
        assert resp.status == 400
        resp = await self.client.post("/api/login", json={"password": 5})
        assert resp.status == 400

    async def test_login_without_password_set(self):
        (self.tmp / "pw.json").unlink()
        resp, _ = await self.login()
        assert resp.status == 503

    async def test_cross_site_login_refused(self):
        resp = await self.client.post(
            "/api/login", json={"password": PASSWORD}, headers={"Origin": "https://evil.example"}
        )
        assert resp.status == 403

    async def test_logout_ends_session(self):
        _, token = await self.login()
        resp = await self.client.post("/api/logout", headers=self.auth_headers(token))
        assert resp.status == 204
        resp = await self.client.get("/api/session", headers=self.auth_headers(token))
        assert (await resp.json())["authenticated"] is False

    async def test_websocket_refused_without_login(self):
        resp = await self.client.get("/ws")
        assert resp.status == 401

    async def test_websocket_refused_from_other_site(self):
        _, token = await self.login()
        headers = {**self.auth_headers(token), "Origin": "https://evil.example"}
        resp = await self.client.get("/ws", headers=headers)
        assert resp.status == 403

    async def test_websocket_proxies_to_rosbridge(self):
        _, token = await self.login()
        host = f"{self.server.host}:{self.server.port}"
        headers = {**self.auth_headers(token), "Origin": f"https://{host}"}
        ws = await self.client.ws_connect("/ws", headers=headers)
        await ws.send_str(json.dumps({"op": "subscribe"}))
        msg = await asyncio.wait_for(ws.receive(), 5)
        assert msg.data == 'echo:{"op": "subscribe"}'
        await ws.send_bytes(b"\x01\x02")
        msg = await asyncio.wait_for(ws.receive(), 5)
        assert msg.data == b"echo:\x01\x02"
        await ws.close()

    async def test_websocket_reports_rosbridge_down(self):
        _, token = await self.login()
        await self.rosbridge.close()
        resp = await self.client.get("/ws", headers=self.auth_headers(token))
        assert resp.status == 502

    async def test_static_files_and_app_fallback(self):
        resp = await self.client.get("/")
        assert resp.status == 200
        assert await resp.text() == "<html>app</html>"
        assert resp.headers["Cache-Control"] == "no-cache"
        assert resp.headers["X-Frame-Options"] == "DENY"

        resp = await self.client.get("/assets/app-123.js")
        assert resp.status == 200
        assert "immutable" in resp.headers["Cache-Control"]

        resp = await self.client.get("/some/unknown/page")
        assert await resp.text() == "<html>app</html>"

    async def test_static_cannot_escape_web_root(self):
        resp = await self.client.get("/../secret.txt")
        assert "secret" != await resp.text()
        resp = await self.client.get("/%2e%2e/secret.txt")
        assert "secret" != await resp.text()


class RedirectAppTest(AioHTTPTestCase):
    async def get_application(self):
        return make_redirect_app(8443)

    async def test_redirects_to_https(self):
        resp = await self.client.get("/robot?x=1", allow_redirects=False, headers={"Host": "r4:8080"})
        assert resp.status == 301
        assert resp.headers["Location"] == "https://r4:8443/robot?x=1"


class ChangePasswordTest(DashboardAppTest):
    async def change(self, token, current, new):
        return await self.client.post(
            "/api/password", json={"current": current, "new": new}, headers=self.auth_headers(token)
        )

    async def test_requires_login(self):
        resp = await self.client.post("/api/password", json={"current": PASSWORD, "new": "another one"})
        assert resp.status == 401

    async def test_wrong_current_password(self):
        _, token = await self.login()
        resp = await self.change(token, "not it", "another one")
        assert resp.status == 403
        assert self.passwords.verify(PASSWORD)

    async def test_too_short(self):
        _, token = await self.login()
        resp = await self.change(token, PASSWORD, "short")
        assert resp.status == 400
        assert self.passwords.verify(PASSWORD)

    async def test_change_keeps_this_device_and_logs_out_others(self):
        _, mine = await self.login()
        _, other = await self.login()
        resp = await self.change(mine, PASSWORD, "another one")
        assert resp.status == 200
        new_token = resp.cookies[SESSION_COOKIE].value
        assert self.passwords.verify("another one")
        assert not self.passwords.verify(PASSWORD)

        async def authed(token):
            r = await self.client.get("/api/session", headers=self.auth_headers(token))
            return (await r.json())["authenticated"]

        assert await authed(new_token)
        assert not await authed(mine)
        assert not await authed(other)


class MapConfigTest(DashboardAppTest):
    async def test_map_config_needs_login_and_tile_hosts_are_allowed(self):
        resp = await self.client.get("/api/map-config")
        assert resp.status == 401
        _, token = await self.login()
        resp = await self.client.get("/api/map-config", headers=self.auth_headers(token))
        assert (await resp.json())["layers"][0]["id"] == "osm"
        csp = resp.headers["Content-Security-Policy"]
        assert "img-src 'self' data: blob: https://tile.openstreetmap.org" in csp
        assert "worker-src 'self' blob:" in csp
