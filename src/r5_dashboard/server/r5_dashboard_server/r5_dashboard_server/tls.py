"""Self-signed TLS certificate for the dashboard (spec NF-10a).

The certificate is made once, on first start, and reused afterwards. Each browser shows a warning
the first time and the user accepts it once.
"""

from __future__ import annotations

import datetime
import ipaddress
import socket
import ssl
from pathlib import Path

from cryptography import x509
from cryptography.hazmat.primitives import hashes, serialization
from cryptography.hazmat.primitives.asymmetric import ec
from cryptography.x509.oid import NameOID

from .auth import write_private_file

_VALID_DAYS = 3650


def _subject_names() -> list[x509.GeneralName]:
    hostname = socket.gethostname()
    names: list[x509.GeneralName] = [x509.DNSName("localhost"), x509.DNSName(hostname)]
    if "." not in hostname:
        names.append(x509.DNSName(f"{hostname}.local"))
    addresses = {"127.0.0.1"}
    try:
        for info in socket.getaddrinfo(hostname, None, family=socket.AF_INET):
            addresses.add(info[4][0])
    except OSError:
        pass
    names.extend(x509.IPAddress(ipaddress.ip_address(a)) for a in sorted(addresses))
    return names


def ensure_certificate(tls_dir: Path) -> tuple[Path, Path]:
    """Return (cert, key) paths, creating a self-signed certificate if there isn't one."""
    cert_path = tls_dir / "cert.pem"
    key_path = tls_dir / "key.pem"
    if cert_path.is_file() and key_path.is_file():
        return cert_path, key_path

    key = ec.generate_private_key(ec.SECP256R1())
    name = x509.Name([x509.NameAttribute(NameOID.COMMON_NAME, f"R5 robot ({socket.gethostname()})")])
    now = datetime.datetime.now(datetime.timezone.utc)
    cert = (
        x509.CertificateBuilder()
        .subject_name(name)
        .issuer_name(name)
        .public_key(key.public_key())
        .serial_number(x509.random_serial_number())
        .not_valid_before(now - datetime.timedelta(days=1))
        .not_valid_after(now + datetime.timedelta(days=_VALID_DAYS))
        .add_extension(x509.SubjectAlternativeName(_subject_names()), critical=False)
        .add_extension(x509.BasicConstraints(ca=False, path_length=None), critical=True)
        .sign(key, hashes.SHA256())
    )
    write_private_file(
        key_path,
        key.private_bytes(
            serialization.Encoding.PEM,
            serialization.PrivateFormat.PKCS8,
            serialization.NoEncryption(),
        ).decode("ascii"),
    )
    write_private_file(cert_path, cert.public_bytes(serialization.Encoding.PEM).decode("ascii"))
    return cert_path, key_path


def make_ssl_context(cert_path: Path, key_path: Path) -> ssl.SSLContext:
    context = ssl.create_default_context(ssl.Purpose.CLIENT_AUTH)
    context.minimum_version = ssl.TLSVersion.TLSv1_2
    context.load_cert_chain(cert_path, key_path)
    return context
