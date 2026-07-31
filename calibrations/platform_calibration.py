from __future__ import annotations

import datetime as dt
import json
import math
import statistics
import uuid
from dataclasses import asdict, dataclass, field, replace
from pathlib import Path
from typing import Iterable


CALIBRATION_SCHEMA_VERSION = 1
STANDARD_GRAVITY_M_S2 = 9.80665
MAX_CALIBRATION_MASS_G = 400.0
MIN_VALID_SAMPLES = 10


class CalibrationError(ValueError):
    pass


@dataclass(frozen=True)
class CalibrationMeasurement:
    weight_g: float
    placement_index: int
    sample_count: int
    force_1_mean_n: float
    force_2_mean_n: float
    total_mean_n: float
    total_std_n: float

    @property
    def expected_force_n(self) -> float:
        return self.weight_g * STANDARD_GRAVITY_M_S2 / 1000.0

    @classmethod
    def from_dict(cls, payload: dict) -> "CalibrationMeasurement":
        try:
            measurement = cls(
                weight_g=float(payload["weight_g"]),
                placement_index=int(payload["placement_index"]),
                sample_count=int(payload["sample_count"]),
                force_1_mean_n=float(payload["force_1_mean_n"]),
                force_2_mean_n=float(payload["force_2_mean_n"]),
                total_mean_n=float(payload["total_mean_n"]),
                total_std_n=float(payload["total_std_n"]),
            )
        except (KeyError, TypeError, ValueError) as exc:
            raise CalibrationError(f"Invalid calibration measurement: {exc}") from exc
        values = (
            measurement.weight_g,
            measurement.force_1_mean_n,
            measurement.force_2_mean_n,
            measurement.total_mean_n,
            measurement.total_std_n,
        )
        if measurement.placement_index < 1 or measurement.sample_count < 1:
            raise CalibrationError("Placement index and sample count must be positive.")
        if not all(math.isfinite(value) for value in values):
            raise CalibrationError("Calibration measurements must be finite.")
        return measurement


@dataclass(frozen=True)
class PlatformCalibration:
    profile_id: str
    name: str
    created_utc: str
    device_ip: str
    channels: tuple[dict, ...]
    gravity_m_s2: float
    gain: float
    tare_1_n: float
    tare_2_n: float
    settle_seconds: float
    acquisition_seconds: float
    placement_repeats: int
    measurements: tuple[CalibrationMeasurement, ...] = field(default_factory=tuple)
    quality: dict = field(default_factory=dict)
    zeroed_utc: str | None = None

    def corrected_pair(
        self,
        force_1_n: float | None,
        force_2_n: float | None,
    ) -> tuple[float | None, float | None, float | None]:
        if force_1_n is None or force_2_n is None:
            return None, None, None
        corrected_1 = self.gain * (force_1_n - self.tare_1_n)
        corrected_2 = self.gain * (force_2_n - self.tare_2_n)
        return corrected_1, corrected_2, corrected_1 + corrected_2

    def apply(self, sample):
        mean_1, mean_2, mean_total = self.corrected_pair(
            sample.force_1_n,
            sample.force_2_n,
        )
        fast_1, fast_2, fast_total = self.corrected_pair(
            sample.force_1_mean_20_n,
            sample.force_2_mean_20_n,
        )
        raw_1, raw_2, raw_total = self.corrected_pair(
            sample.force_1_raw_n,
            sample.force_2_raw_n,
        )
        return replace(
            sample,
            force_1_n=mean_1,
            force_2_n=mean_2,
            force_total_n=mean_total,
            force_1_mean_20_n=fast_1,
            force_2_mean_20_n=fast_2,
            force_total_mean_20_n=fast_total,
            force_1_raw_n=raw_1,
            force_2_raw_n=raw_2,
            force_total_raw_n=raw_total,
            raw_force=raw_total,
            uncalibrated_force_1_n=sample.force_1_n,
            uncalibrated_force_2_n=sample.force_2_n,
            uncalibrated_force_total_n=sample.force_total_n,
            uncalibrated_force_1_mean_20_n=sample.force_1_mean_20_n,
            uncalibrated_force_2_mean_20_n=sample.force_2_mean_20_n,
            uncalibrated_force_total_mean_20_n=sample.force_total_mean_20_n,
            uncalibrated_force_1_raw_n=sample.force_1_raw_n,
            uncalibrated_force_2_raw_n=sample.force_2_raw_n,
            uncalibrated_force_total_raw_n=sample.force_total_raw_n,
            calibration_profile_id=self.profile_id,
            calibration_zeroed_utc=self.zeroed_utc,
        )

    def with_tare(
        self,
        tare_1_n: float,
        tare_2_n: float,
        zeroed_utc: str | None = None,
    ) -> "PlatformCalibration":
        return replace(
            self,
            tare_1_n=float(tare_1_n),
            tare_2_n=float(tare_2_n),
            zeroed_utc=zeroed_utc or utc_now_iso(),
        )

    def to_dict(self) -> dict:
        return {
            "schema_version": CALIBRATION_SCHEMA_VERSION,
            "profile_id": self.profile_id,
            "name": self.name,
            "created_utc": self.created_utc,
            "device_ip": self.device_ip,
            "channels": list(self.channels),
            "gravity_m_s2": self.gravity_m_s2,
            "gain": self.gain,
            "tare_1_n": self.tare_1_n,
            "tare_2_n": self.tare_2_n,
            "settle_seconds": self.settle_seconds,
            "acquisition_seconds": self.acquisition_seconds,
            "placement_repeats": self.placement_repeats,
            "measurements": [asdict(measurement) for measurement in self.measurements],
            "quality": self.quality,
            "zeroed_utc": self.zeroed_utc,
        }

    @classmethod
    def from_dict(cls, payload: dict) -> "PlatformCalibration":
        if not isinstance(payload, dict):
            raise CalibrationError("Calibration profile root must be an object.")
        if payload.get("schema_version") != CALIBRATION_SCHEMA_VERSION:
            raise CalibrationError(
                f"Unsupported calibration schema {payload.get('schema_version')!r}."
            )
        try:
            calibration = cls(
                profile_id=str(payload["profile_id"]),
                name=str(payload["name"]),
                created_utc=str(payload["created_utc"]),
                device_ip=str(payload["device_ip"]),
                channels=tuple(dict(channel) for channel in payload["channels"]),
                gravity_m_s2=float(payload["gravity_m_s2"]),
                gain=float(payload["gain"]),
                tare_1_n=float(payload["tare_1_n"]),
                tare_2_n=float(payload["tare_2_n"]),
                settle_seconds=float(payload["settle_seconds"]),
                acquisition_seconds=float(payload["acquisition_seconds"]),
                placement_repeats=int(payload["placement_repeats"]),
                measurements=tuple(
                    CalibrationMeasurement.from_dict(item)
                    for item in payload.get("measurements", [])
                ),
                quality=dict(payload.get("quality", {})),
                zeroed_utc=(
                    None
                    if payload.get("zeroed_utc") in (None, "")
                    else str(payload["zeroed_utc"])
                ),
            )
        except (KeyError, TypeError, ValueError) as exc:
            raise CalibrationError(f"Invalid calibration profile: {exc}") from exc
        calibration.validate()
        return calibration

    @classmethod
    def load(cls, path: str | Path) -> "PlatformCalibration":
        try:
            with open(path, encoding="utf-8") as profile_file:
                payload = json.load(profile_file)
        except (OSError, json.JSONDecodeError) as exc:
            raise CalibrationError(f"Could not load calibration profile: {exc}") from exc
        return cls.from_dict(payload)

    def save(self, path: str | Path) -> None:
        path = Path(path)
        temporary_path = path.with_suffix(path.suffix + ".tmp")
        try:
            path.parent.mkdir(parents=True, exist_ok=True)
            with open(temporary_path, "w", encoding="utf-8") as profile_file:
                json.dump(self.to_dict(), profile_file, indent=2, ensure_ascii=False)
                profile_file.write("\n")
            temporary_path.replace(path)
        except OSError as exc:
            raise CalibrationError(f"Could not save calibration profile: {exc}") from exc

    def validate(self) -> None:
        numeric_values = (
            self.gravity_m_s2,
            self.gain,
            self.tare_1_n,
            self.tare_2_n,
            self.settle_seconds,
            self.acquisition_seconds,
        )
        if not self.profile_id or not self.name:
            raise CalibrationError("Profile ID and name must not be empty.")
        if not all(math.isfinite(value) for value in numeric_values):
            raise CalibrationError("Calibration profile contains non-finite values.")
        if self.gain <= 0.0:
            raise CalibrationError("Calibration gain must be greater than zero.")
        if self.gravity_m_s2 <= 0.0:
            raise CalibrationError("Gravity must be greater than zero.")
        if self.settle_seconds < 0.0 or self.acquisition_seconds <= 0.0:
            raise CalibrationError("Calibration timing values are invalid.")
        if self.placement_repeats < 1:
            raise CalibrationError("Placement repeats must be positive.")
        if len(self.channels) != 2:
            raise CalibrationError("Exactly two force channels are required.")


def utc_now_iso() -> str:
    return dt.datetime.now(dt.timezone.utc).isoformat(timespec="milliseconds")


def parse_weight_grams(text: str) -> float:
    normalized = str(text).strip().replace(",", ".")
    try:
        value = float(normalized)
    except ValueError as exc:
        raise CalibrationError(f"Invalid weight: {text!r}") from exc
    if not math.isfinite(value):
        raise CalibrationError("Weights must be finite.")
    if value < 0.0 or value > MAX_CALIBRATION_MASS_G:
        raise CalibrationError(
            f"Weights must be between 0 and {MAX_CALIBRATION_MASS_G:g} g."
        )
    return value


def validate_weight_list(values: Iterable[str | float]) -> list[float]:
    weights = [parse_weight_grams(str(value)) for value in values]
    nonzero = sorted({round(weight, 9) for weight in weights if weight > 0.0})
    if len(nonzero) < 2:
        raise CalibrationError("Enter at least two different non-zero weights.")
    return [0.0, *nonzero]


def measurement_from_samples(
    weight_g: float,
    placement_index: int,
    samples: Iterable,
) -> CalibrationMeasurement:
    unique_samples = {}
    for sample in samples:
        if (
            sample is None
            or not sample.valid
            or sample.force_1_raw_n is None
            or sample.force_2_raw_n is None
        ):
            continue
        unique_samples[sample.sample_id] = sample
    if len(unique_samples) < MIN_VALID_SAMPLES:
        raise CalibrationError(
            f"Only {len(unique_samples)} valid unique samples; at least "
            f"{MIN_VALID_SAMPLES} are required."
        )
    force_1_values = [sample.force_1_raw_n for sample in unique_samples.values()]
    force_2_values = [sample.force_2_raw_n for sample in unique_samples.values()]
    totals = [
        force_1 + force_2
        for force_1, force_2 in zip(force_1_values, force_2_values)
    ]
    return CalibrationMeasurement(
        weight_g=float(weight_g),
        placement_index=int(placement_index),
        sample_count=len(totals),
        force_1_mean_n=statistics.mean(force_1_values),
        force_2_mean_n=statistics.mean(force_2_values),
        total_mean_n=statistics.mean(totals),
        total_std_n=statistics.stdev(totals) if len(totals) > 1 else 0.0,
    )


def fit_platform_calibration(
    measurements: Iterable[CalibrationMeasurement],
    *,
    name: str,
    device_ip: str,
    channels: Iterable[dict],
    settle_seconds: float = 1.0,
    acquisition_seconds: float = 1.0,
    placement_repeats: int = 3,
) -> PlatformCalibration:
    points = tuple(measurements)
    zero_points = [point for point in points if math.isclose(point.weight_g, 0.0)]
    loaded_points = [point for point in points if point.weight_g > 0.0]
    distinct_weights = {round(point.weight_g, 9) for point in loaded_points}
    if not zero_points:
        raise CalibrationError("A 0 g measurement is required.")
    if len(distinct_weights) < 2:
        raise CalibrationError("At least two different non-zero weights are required.")

    tare_1 = statistics.mean(point.force_1_mean_n for point in zero_points)
    tare_2 = statistics.mean(point.force_2_mean_n for point in zero_points)
    tare_total = tare_1 + tare_2
    regression_points = [
        (
            point.total_mean_n - tare_total,
            point.expected_force_n,
            point.weight_g,
        )
        for point in loaded_points
    ]
    denominator = sum(measured * measured for measured, _, _ in regression_points)
    if denominator <= 1e-18:
        raise CalibrationError("Measured force span is too small for calibration.")
    gain = sum(
        measured * expected for measured, expected, _ in regression_points
    ) / denominator
    if not math.isfinite(gain) or gain <= 0.0:
        raise CalibrationError("Calculated calibration gain is invalid.")

    residuals = [
        gain * measured - expected for measured, expected, _ in regression_points
    ]
    rmse = math.sqrt(sum(value * value for value in residuals) / len(residuals))
    max_abs_error = max(abs(value) for value in residuals)
    expected_values = [expected for _, expected, _ in regression_points]
    expected_mean = statistics.mean(expected_values)
    total_variation = sum((value - expected_mean) ** 2 for value in expected_values)
    residual_variation = sum(value * value for value in residuals)
    r_squared = (
        1.0
        if total_variation <= 1e-18 and residual_variation <= 1e-18
        else 1.0 - residual_variation / total_variation
        if total_variation > 1e-18
        else None
    )

    spreads = {}
    spread_warnings = []
    for weight in sorted(distinct_weights):
        calibrated_values = [
            gain * (point.total_mean_n - tare_total)
            for point in loaded_points
            if math.isclose(point.weight_g, weight, rel_tol=0.0, abs_tol=1e-9)
        ]
        spread = max(calibrated_values) - min(calibrated_values)
        spreads[f"{weight:g}"] = spread
        expected = weight * STANDARD_GRAVITY_M_S2 / 1000.0
        if spread > max(0.02, 0.01 * expected):
            spread_warnings.append(
                f"Position spread at {weight:g} g is {spread:.4f} N."
            )

    max_expected = max(expected_values)
    warnings = []
    if not 0.8 <= gain <= 1.2:
        warnings.append(f"Gain {gain:.6f} is outside 0.8 to 1.2.")
    if rmse > max(0.01, 0.01 * max_expected):
        warnings.append(f"RMSE {rmse:.4f} N exceeds the recommended limit.")
    residual_limit_exceeded = any(
        abs(residual) > max(0.02, 0.02 * expected)
        for residual, expected in zip(residuals, expected_values)
    )
    if residual_limit_exceeded:
        warnings.append(
            f"Maximum residual {max_abs_error:.4f} N exceeds the recommended limit."
        )
    warnings.extend(spread_warnings)

    quality = {
        "rmse_n": rmse,
        "max_abs_error_n": max_abs_error,
        "residuals_n": residuals,
        "r_squared": r_squared,
        "position_spread_n_by_weight_g": spreads,
        "max_position_spread_n": max(spreads.values()) if spreads else 0.0,
        "warnings": warnings,
    }
    calibration = PlatformCalibration(
        profile_id=str(uuid.uuid4()),
        name=name.strip() or "Platform calibration",
        created_utc=utc_now_iso(),
        device_ip=device_ip,
        channels=tuple(dict(channel) for channel in channels),
        gravity_m_s2=STANDARD_GRAVITY_M_S2,
        gain=gain,
        tare_1_n=tare_1,
        tare_2_n=tare_2,
        settle_seconds=float(settle_seconds),
        acquisition_seconds=float(acquisition_seconds),
        placement_repeats=int(placement_repeats),
        measurements=points,
        quality=quality,
        zeroed_utc=utc_now_iso(),
    )
    calibration.validate()
    return calibration


def preserve_uncalibrated_sample(sample):
    return replace(
        sample,
        uncalibrated_force_1_n=sample.force_1_n,
        uncalibrated_force_2_n=sample.force_2_n,
        uncalibrated_force_total_n=sample.force_total_n,
        uncalibrated_force_1_mean_20_n=sample.force_1_mean_20_n,
        uncalibrated_force_2_mean_20_n=sample.force_2_mean_20_n,
        uncalibrated_force_total_mean_20_n=sample.force_total_mean_20_n,
        uncalibrated_force_1_raw_n=sample.force_1_raw_n,
        uncalibrated_force_2_raw_n=sample.force_2_raw_n,
        uncalibrated_force_total_raw_n=sample.force_total_raw_n,
    )
