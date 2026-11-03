import ssl

from r5_dashboard_server.main import parse_args, strip_ros_args
from r5_dashboard_server.tls import ensure_certificate, make_ssl_context


def test_strip_ros_args():
    assert strip_ros_args(["--https-port", "1"]) == ["--https-port", "1"]
    assert strip_ros_args(["--https-port", "1", "--ros-args", "-r", "__node:=x"]) == [
        "--https-port",
        "1",
    ]
    assert strip_ros_args(["--ros-args", "-r", "a:=b", "--", "--http-port", "0"]) == [
        "--http-port",
        "0",
    ]


def test_parse_args_defaults():
    args = parse_args(["--ros-args", "-r", "__node:=server"])
    assert args.https_port == 8443
    assert args.http_port == 8080
    assert args.rosbridge_url == "ws://127.0.0.1:9090"


def test_certificate_created_once_and_loads(tmp_path):
    cert, key = ensure_certificate(tmp_path)
    first = cert.read_bytes()
    cert2, _ = ensure_certificate(tmp_path)
    assert cert2.read_bytes() == first
    assert isinstance(make_ssl_context(cert, key), ssl.SSLContext)
    assert oct(key.stat().st_mode & 0o777) == "0o600"
