from __future__ import annotations

import json
import math
import socket
import threading
import time
from dataclasses import dataclass
from typing import Callable


QUANTUMX_SCHEMA_VERSION = 1
QUANTUMX_ALLOWED_STATUSES = {
    "ok",
    "stale",
    "overrange",
    "disconnected",
    "invalid_channel",
}


class ForceProtocolError(ValueError):
    """Raised when a force-source message violates the public schema."""


@dataclass(frozen=True)
class ForceSample:
    source: str
    sequence: int
    timestamp_utc_ns: int
    force_1_n: float | None
    force_2_n: float | None
    force_total_n: float | None
    status: str
    channel_1_status: str
    channel_2_status: str
    force_1_mean_20_n: float | None = None
    force_2_mean_20_n: float | None = None
    force_total_mean_20_n: float | None = None
    force_1_raw_n: float | None = None
    force_2_raw_n: float | None = None
    force_total_raw_n: float | None = None
    raw_force: float | None = None
    raw_1_mv_v: float | None = None
    raw_2_mv_v: float | None = None

    @property
    def sample_id(self) -> tuple[str, int, int]:
        return self.source, self.timestamp_utc_ns, self.sequence

    @property
    def valid(self) -> bool:
        return self.status == "ok" and self.force_total_n is not None


def _required_int(payload: dict, name: str, minimum: int = 0) -> int:
    value = payload.get(name)
    if isinstance(value, bool) or not isinstance(value, int) or value < minimum:
        raise ForceProtocolError(f"{name} must be an integer >= {minimum}")
    return value


def _status(payload: dict, name: str) -> str:
    value = payload.get(name)
    if value not in QUANTUMX_ALLOWED_STATUSES:
        allowed = ", ".join(sorted(QUANTUMX_ALLOWED_STATUSES))
        raise ForceProtocolError(f"{name} must be one of: {allowed}")
    return value


def _optional_finite_float(payload: dict, name: str) -> float | None:
    value = payload.get(name)
    if value is None:
        return None
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ForceProtocolError(f"{name} must be a finite number or null")
    value = float(value)
    if not math.isfinite(value):
        raise ForceProtocolError(f"{name} must be finite")
    return value


def parse_quantumx_message(line: str | bytes) -> ForceSample:
    if isinstance(line, bytes):
        try:
            line = line.decode("utf-8")
        except UnicodeDecodeError as exc:
            raise ForceProtocolError("message is not valid UTF-8") from exc

    try:
        payload = json.loads(line)
    except json.JSONDecodeError as exc:
        raise ForceProtocolError(f"invalid JSON: {exc.msg}") from exc
    if not isinstance(payload, dict):
        raise ForceProtocolError("message root must be a JSON object")

    schema_version = _required_int(payload, "schema_version", minimum=1)
    if schema_version != QUANTUMX_SCHEMA_VERSION:
        raise ForceProtocolError(
            f"unsupported schema_version {schema_version}; expected {QUANTUMX_SCHEMA_VERSION}"
        )

    sequence = _required_int(payload, "sequence")
    timestamp_utc_ns = _required_int(payload, "timestamp_utc_ns")
    status = _status(payload, "status")
    channel_1_status = _status(payload, "channel_1_status")
    channel_2_status = _status(payload, "channel_2_status")
    force_1_n = _optional_finite_float(payload, "force_1_n")
    force_2_n = _optional_finite_float(payload, "force_2_n")
    force_total_n = _optional_finite_float(payload, "force_total_n")

    if status == "ok":
        if channel_1_status != "ok" or channel_2_status != "ok":
            raise ForceProtocolError("status ok requires both channel statuses to be ok")
        if force_1_n is None or force_2_n is None or force_total_n is None:
            raise ForceProtocolError("status ok requires both channel values and the total")
        calculated_total = force_1_n + force_2_n
        if not math.isclose(force_total_n, calculated_total, rel_tol=1e-9, abs_tol=1e-9):
            raise ForceProtocolError(
                f"force_total_n {force_total_n} does not equal force_1_n + force_2_n {calculated_total}"
            )
    elif (force_1_n is None or force_2_n is None) and force_total_n is not None:
        raise ForceProtocolError("force_total_n must be null when either channel value is null")

    return ForceSample(
        source="quantumx",
        sequence=sequence,
        timestamp_utc_ns=timestamp_utc_ns,
        force_1_n=force_1_n,
        force_2_n=force_2_n,
        force_total_n=force_total_n,
        status=status,
        channel_1_status=channel_1_status,
        channel_2_status=channel_2_status,
        force_1_mean_20_n=_optional_finite_float(payload, "force_1_mean_20_n"),
        force_2_mean_20_n=_optional_finite_float(payload, "force_2_mean_20_n"),
        force_total_mean_20_n=_optional_finite_float(payload, "force_total_mean_20_n"),
        force_1_raw_n=_optional_finite_float(payload, "force_1_raw_n"),
        force_2_raw_n=_optional_finite_float(payload, "force_2_raw_n"),
        force_total_raw_n=_optional_finite_float(payload, "force_total_raw_n"),
        raw_force=_optional_finite_float(payload, "force_total_raw_n"),
        raw_1_mv_v=_optional_finite_float(payload, "raw_1_mv_v"),
        raw_2_mv_v=_optional_finite_float(payload, "raw_2_mv_v"),
    )


class UniqueForceAccumulator:
    """Threshold and average unique force samples for one impulse."""

    def __init__(self, threshold_n: float):
        if threshold_n < 0:
            raise ValueError("threshold_n must be >= 0")
        self.threshold_n = threshold_n
        self.started = False
        self.done = False
        self._seen_ids: set[tuple[str, int, int]] = set()
        self.total_sum = 0.0
        self.total_count = 0
        self.force_1_sum = 0.0
        self.force_1_count = 0
        self.force_2_sum = 0.0
        self.force_2_count = 0

    def add(self, sample: ForceSample | None) -> bool:
        if sample is None or sample.sample_id in self._seen_ids:
            return False
        self._seen_ids.add(sample.sample_id)
        if self.done or not sample.valid:
            return False

        force_total = sample.force_total_n
        if force_total is None:
            return False
        if abs(force_total) < self.threshold_n:
            if self.started:
                self.done = True
            return False

        self.started = True
        self.total_sum += force_total
        self.total_count += 1
        if sample.force_1_n is not None:
            self.force_1_sum += sample.force_1_n
            self.force_1_count += 1
        if sample.force_2_n is not None:
            self.force_2_sum += sample.force_2_n
            self.force_2_count += 1
        return True

    @staticmethod
    def _average(total: float, count: int) -> float | None:
        return total / count if count else None

    @property
    def average_total_n(self) -> float | None:
        return self._average(self.total_sum, self.total_count)

    @property
    def average_force_1_n(self) -> float | None:
        return self._average(self.force_1_sum, self.force_1_count)

    @property
    def average_force_2_n(self) -> float | None:
        return self._average(self.force_2_sum, self.force_2_count)


class QuantumXTcpClient:
    """Reconnectable NDJSON client for the local QuantumX bridge."""

    def __init__(
        self,
        host: str,
        port: int,
        on_sample: Callable[[ForceSample], None],
        on_status: Callable[[str], None],
        stale_after_seconds: float = 0.25,
        reconnect_after_seconds: float = 1.0,
    ):
        self.host = host
        self.port = port
        self.on_sample = on_sample
        self.on_status = on_status
        self.stale_after_seconds = stale_after_seconds
        self.reconnect_after_seconds = reconnect_after_seconds
        self.running = False
        self._thread: threading.Thread | None = None
        self._socket: socket.socket | None = None
        self._socket_lock = threading.Lock()

    def start(self) -> None:
        if self.running:
            return
        self.running = True
        self._thread = threading.Thread(target=self._run, name="quantumx-tcp", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self.running = False
        with self._socket_lock:
            sock = self._socket
            self._socket = None
        if sock is not None:
            try:
                sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            sock.close()
        if self._thread and self._thread is not threading.current_thread():
            self._thread.join(timeout=1.5)
        self._thread = None

    def _emit_status(self, status: str) -> None:
        try:
            self.on_status(status)
        except Exception:
            pass

    def _run(self) -> None:
        while self.running:
            try:
                self._receive_connection()
            except OSError as exc:
                if self.running:
                    self._emit_status(f"QuantumX bridge disconnected: {exc}")
            if self.running:
                deadline = time.monotonic() + self.reconnect_after_seconds
                while self.running and time.monotonic() < deadline:
                    time.sleep(0.05)
        self._emit_status("QuantumX bridge disconnected")

    def _receive_connection(self) -> None:
        self._emit_status(f"Connecting to QuantumX bridge at {self.host}:{self.port}")
        sock = socket.create_connection((self.host, self.port), timeout=2.0)
        sock.settimeout(0.1)
        with self._socket_lock:
            if not self.running:
                sock.close()
                return
            self._socket = sock

        self._emit_status(f"QuantumX bridge connected at {self.host}:{self.port}")
        buffer = bytearray()
        last_valid_time = time.monotonic()
        stale_reported = False
        last_sequence: int | None = None
        try:
            while self.running:
                try:
                    chunk = sock.recv(4096)
                except socket.timeout:
                    chunk = None
                if chunk == b"":
                    raise ConnectionError("peer closed the connection")
                if chunk:
                    buffer.extend(chunk)
                    if len(buffer) > 1_000_000:
                        raise ForceProtocolError("NDJSON receive buffer exceeded 1 MB")
                    while b"\n" in buffer:
                        raw_line, _, remainder = buffer.partition(b"\n")
                        buffer[:] = remainder
                        if not raw_line.strip():
                            continue
                        try:
                            sample = parse_quantumx_message(raw_line)
                        except ForceProtocolError as exc:
                            self._emit_status(f"QuantumX protocol error: {exc}")
                            continue
                        if last_sequence is not None and sample.sequence <= last_sequence:
                            self._emit_status(
                                f"QuantumX ignored duplicate/out-of-order sequence {sample.sequence}"
                            )
                            continue
                        last_sequence = sample.sequence
                        last_valid_time = time.monotonic()
                        stale_reported = False
                        self.on_sample(sample)
                if not stale_reported and time.monotonic() - last_valid_time > self.stale_after_seconds:
                    self._emit_status("QuantumX data stale")
                    stale_reported = True
        finally:
            with self._socket_lock:
                if self._socket is sock:
                    self._socket = None
            sock.close()
