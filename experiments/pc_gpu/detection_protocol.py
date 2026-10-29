from __future__ import annotations

import json
import socket
import struct
from dataclasses import asdict, dataclass, field
from typing import Any, Dict

_HEADER = struct.Struct("!I")
_MAX_MESSAGE_BYTES = 8 * 1024 * 1024


@dataclass
class DetectionResult:
    found: bool
    center_x: int = 0
    center_y: int = 0
    width: int = 0
    height: int = 0
    confidence: float = 0.0
    class_id: int = -1
    class_name: str = ""
    frame_width: int = 0
    frame_height: int = 0
    inference_ms: float = 0.0
    server_fps: float = 0.0
    source: str = "pc_gpu"
    error: str = ""
    # V2: list of ALL PERSON boxes, not just confidence's best match.
    # V1 callers remain compatible but cannot authorize presenter-locked AUTO.
    protocol_version: int = 1
    frame_seq: int = 0
    people: list[dict[str, Any]] = field(default_factory=list)
    # V2 optional pose extension, aligned to people[] order.
    # Unsupported/old PC model supplies empty defaults (no gesture permission).
    gesture_indices: list[int] = field(default_factory=list)
    aim_points: list[dict[str, Any]] = field(default_factory=list)


    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)

    @classmethod
    def from_dict(cls, payload: Dict[str, Any]) -> "DetectionResult":
        allowed = cls.__dataclass_fields__.keys()
        filtered = {key: value for key, value in payload.items() if key in allowed}
        return cls(**filtered)


def recv_exact(sock: socket.socket, size: int) -> bytes:
    chunks = bytearray()
    while len(chunks) < size:
        chunk = sock.recv(size - len(chunks))
        if not chunk:
            raise ConnectionError("socket closed while receiving data")
        chunks.extend(chunk)
    return bytes(chunks)


def send_message(sock: socket.socket, payload: bytes) -> None:
    if len(payload) > _MAX_MESSAGE_BYTES:
        raise ValueError(f"message too large: {len(payload)} bytes")
    sock.sendall(_HEADER.pack(len(payload)))
    sock.sendall(payload)


def recv_message(sock: socket.socket) -> bytes:
    raw_size = recv_exact(sock, _HEADER.size)
    (size,) = _HEADER.unpack(raw_size)
    if size > _MAX_MESSAGE_BYTES:
        raise ValueError(f"incoming message too large: {size} bytes")
    return recv_exact(sock, size)


def send_json(sock: socket.socket, payload: Dict[str, Any]) -> None:
    send_message(
        sock,
        json.dumps(payload, ensure_ascii=False, separators=(",", ":")).encode("utf-8"),
    )


def recv_json(sock: socket.socket) -> Dict[str, Any]:
    data = recv_message(sock)
    parsed = json.loads(data.decode("utf-8"))
    if not isinstance(parsed, dict):
        raise ValueError("JSON payload must be an object")
    return parsed
