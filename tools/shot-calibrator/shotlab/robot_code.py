from __future__ import annotations

import ast
import hashlib
import operator
import re
from dataclasses import dataclass
from pathlib import Path


JAVA_CONSTANT_PATTERN = re.compile(
    r"public\s+static\s+final\s+(?:double|int)\s+([A-Z][A-Z0-9_]*)\s*=\s*(.*?);",
    re.DOTALL,
)
UNIT_CALL_PATTERN = re.compile(r"Units\.(inchesToMeters|feetToMeters)\(([^()]*)\)")
UNIT_FACTORS = {
    "inchesToMeters": 0.0254,
    "feetToMeters": 0.3048,
}

REQUIRED_CONSTANTS = {
    "ShotModelConstants": (
        "MIN_DISTANCE_METERS",
        "MAX_DISTANCE_METERS",
        "BALL_MASS_KG",
        "BALL_DIAMETER_METERS",
        "TOP_WHEEL_DIAMETER_METERS",
        "BOTTOM_WHEEL_DIAMETER_METERS",
        "BOTTOM_TO_TOP_RPM_RATIO",
        "RELEASE_HEIGHT_METERS",
        "HUB_OPENING_SPAN_METERS",
        "HUB_RIM_HEIGHT_METERS",
        "HUB_BALL_CENTER_HEIGHT_METERS",
        "HUB_RIM_MARGIN_METERS",
        "MIN_ENTRY_ANGLE_DEGREES",
        "AIR_DENSITY_KG_PER_CUBIC_METER",
        "GRAVITY_METERS_PER_SECOND_SQUARED",
        "DRAG_COEFFICIENT",
        "WIND_X_METERS_PER_SECOND",
        "VELOCITY_TRANSFER",
        "SPIN_TRANSFER",
        "HOOD_OFFSET_DEGREES",
        "DRAG_SCALE",
        "LIFT_SLOPE",
        "MAX_LIFT_COEFFICIENT",
        "SPIN_DECAY_PER_SECOND",
        "FLIGHT_TIME_WEIGHT",
        "ENTRY_ANGLE_WEIGHT",
        "MECHANISM_EFFORT_WEIGHT",
        "RPM_POLICY_DISTANCE_SQUARED",
        "RPM_POLICY_DISTANCE",
        "RPM_POLICY_CONSTANT",
        "HOOD_POLICY_DISTANCE_SQUARED",
        "HOOD_POLICY_DISTANCE",
        "HOOD_POLICY_CONSTANT",
    ),
    "ShooterConstants": ("FLYWHEEL_IDLE_RPM", "MAX_RPM"),
    "HoodConstants": ("HOOD_MIN_ANGLE", "HOOD_MAX_ANGLE"),
}

EMPIRICAL_JAVA_TO_MODEL = {
    "VELOCITY_TRANSFER": "velocity_transfer",
    "SPIN_TRANSFER": "spin_transfer",
    "HOOD_OFFSET_DEGREES": "hood_offset_deg",
    "DRAG_SCALE": "drag_scale",
    "LIFT_SLOPE": "lift_slope",
    "MAX_LIFT_COEFFICIENT": "max_lift_coefficient",
    "SPIN_DECAY_PER_SECOND": "spin_decay_per_s",
}

APP_STATE_CONSTANTS = {
    "model_ball_mass": "BALL_MASS_KG",
    "model_ball_diameter": "BALL_DIAMETER_METERS",
    "model_drag_coefficient": "DRAG_COEFFICIENT",
    "model_air_density": "AIR_DENSITY_KG_PER_CUBIC_METER",
    "model_gravity": "GRAVITY_METERS_PER_SECOND_SQUARED",
    "model_wind_x": "WIND_X_METERS_PER_SECOND",
    "model_min_entry_angle": "MIN_ENTRY_ANGLE_DEGREES",
    "model_rim_margin_in": "HUB_RIM_MARGIN_METERS",
    "model_release_height": "RELEASE_HEIGHT_METERS",
    "model_top_wheel_diameter": "TOP_WHEEL_DIAMETER_METERS",
    "model_bottom_wheel_diameter": "BOTTOM_WHEEL_DIAMETER_METERS",
    "model_min_rpm": "FLYWHEEL_IDLE_RPM",
    "model_max_rpm": "MAX_RPM",
    "model_min_hood": "HOOD_MIN_ANGLE",
    "model_max_hood": "HOOD_MAX_ANGLE",
    "model_rpm_ratio": "BOTTOM_TO_TOP_RPM_RATIO",
    "model_velocity_transfer": "VELOCITY_TRANSFER",
    "model_spin_transfer": "SPIN_TRANSFER",
    "model_hood_offset_deg": "HOOD_OFFSET_DEGREES",
    "model_drag_scale": "DRAG_SCALE",
    "model_lift_slope": "LIFT_SLOPE",
    "model_max_lift_coefficient": "MAX_LIFT_COEFFICIENT",
    "model_spin_decay_per_s": "SPIN_DECAY_PER_SECOND",
    "model_time_weight": "FLIGHT_TIME_WEIGHT",
    "model_entry_weight": "ENTRY_ANGLE_WEIGHT",
    "model_effort_weight": "MECHANISM_EFFORT_WEIGHT",
    "model_hub_opening_span": "HUB_OPENING_SPAN_METERS",
    "model_hub_rim_height": "HUB_RIM_HEIGHT_METERS",
    "model_shot_distance_min": "MIN_DISTANCE_METERS",
    "model_shot_distance_max": "MAX_DISTANCE_METERS",
}


class RobotCodeParseError(ValueError):
    pass


@dataclass(frozen=True)
class RobotShotConfig:
    source_path: Path
    source_hash: str
    values: dict[str, float]

    @property
    def empirical_parameters(self) -> dict[str, float]:
        return {
            model_name: self.values[java_name]
            for java_name, model_name in EMPIRICAL_JAVA_TO_MODEL.items()
        }

    @property
    def app_state_values(self) -> dict[str, float]:
        values = {
            state_key: self.values[java_name]
            for state_key, java_name in APP_STATE_CONSTANTS.items()
        }
        values["model_rim_margin_in"] /= 0.0254
        return values

    @property
    def runtime_policy(self) -> dict[str, tuple[float, float, float]]:
        return {
            "rpm": (
                self.values["RPM_POLICY_DISTANCE_SQUARED"],
                self.values["RPM_POLICY_DISTANCE"],
                self.values["RPM_POLICY_CONSTANT"],
            ),
            "hood": (
                self.values["HOOD_POLICY_DISTANCE_SQUARED"],
                self.values["HOOD_POLICY_DISTANCE"],
                self.values["HOOD_POLICY_CONSTANT"],
            ),
        }


def default_robot_constants_path() -> Path:
    return Path(__file__).resolve().parents[3] / "src/main/java/frc/robot/Constants.java"


def load_robot_shot_config(path: Path | str | None = None) -> RobotShotConfig:
    source_path = Path(path) if path is not None else default_robot_constants_path()
    try:
        source = source_path.read_text(encoding="utf-8")
    except OSError as error:
        raise RobotCodeParseError(f"Could not read robot constants at {source_path}: {error}") from error

    parsed: dict[str, float] = {}
    missing: list[str] = []
    for class_name, required_names in REQUIRED_CONSTANTS.items():
        class_values = _parse_constant_class(source, class_name)
        for name in required_names:
            if name not in class_values:
                missing.append(f"{class_name}.{name}")
            else:
                parsed[name] = class_values[name]

    if missing:
        raise RobotCodeParseError(
            "Robot shot configuration is incomplete; missing " + ", ".join(missing)
        )

    return RobotShotConfig(
        source_path=source_path.resolve(),
        source_hash=hashlib.sha256(source.encode("utf-8")).hexdigest(),
        values=parsed,
    )


def robot_config_rows(config: RobotShotConfig) -> list[dict[str, str | float]]:
    groups = (
        (
            "Physics fit",
            (
                "VELOCITY_TRANSFER",
                "SPIN_TRANSFER",
                "HOOD_OFFSET_DEGREES",
                "DRAG_SCALE",
                "LIFT_SLOPE",
                "MAX_LIFT_COEFFICIENT",
                "SPIN_DECAY_PER_SECOND",
            ),
        ),
        (
            "Mechanism",
            (
                "TOP_WHEEL_DIAMETER_METERS",
                "BOTTOM_WHEEL_DIAMETER_METERS",
                "RELEASE_HEIGHT_METERS",
                "FLYWHEEL_IDLE_RPM",
                "MAX_RPM",
                "HOOD_MIN_ANGLE",
                "HOOD_MAX_ANGLE",
            ),
        ),
        (
            "Runtime policy",
            (
                "RPM_POLICY_DISTANCE_SQUARED",
                "RPM_POLICY_DISTANCE",
                "RPM_POLICY_CONSTANT",
                "HOOD_POLICY_DISTANCE_SQUARED",
                "HOOD_POLICY_DISTANCE",
                "HOOD_POLICY_CONSTANT",
            ),
        ),
    )
    return [
        {"Group": group, "Java constant": name, "Loaded value": config.values[name]}
        for group, names in groups
        for name in names
    ]


def _parse_constant_class(source: str, class_name: str) -> dict[str, float]:
    body = _class_body(source, class_name)
    values: dict[str, float] = {}
    for match in JAVA_CONSTANT_PATTERN.finditer(body):
        name, expression = match.groups()
        try:
            values[name] = float(_evaluate_java_numeric_expression(expression, values))
        except (RobotCodeParseError, ZeroDivisionError):
            continue
    return values


def _class_body(source: str, class_name: str) -> str:
    declaration = re.search(rf"\bclass\s+{re.escape(class_name)}\s*\{{", source)
    if declaration is None:
        raise RobotCodeParseError(f"Could not find {class_name} in robot constants.")
    opening_brace = source.find("{", declaration.start())
    depth = 0
    for index in range(opening_brace, len(source)):
        character = source[index]
        if character == "{":
            depth += 1
        elif character == "}":
            depth -= 1
            if depth == 0:
                return source[opening_brace + 1 : index]
    raise RobotCodeParseError(f"{class_name} has an unclosed class body.")


def _evaluate_java_numeric_expression(expression: str, names: dict[str, float]) -> float:
    normalized = re.sub(r"//.*", "", expression).strip()
    while True:
        match = UNIT_CALL_PATTERN.search(normalized)
        if match is None:
            break
        method, argument = match.groups()
        replacement = f"(({argument}) * {UNIT_FACTORS[method]})"
        normalized = normalized[: match.start()] + replacement + normalized[match.end() :]
    try:
        tree = ast.parse(normalized, mode="eval")
    except SyntaxError as error:
        raise RobotCodeParseError(f"Unsupported Java numeric expression: {expression.strip()}") from error
    return float(_evaluate_ast(tree.body, names))


def _evaluate_ast(node: ast.AST, names: dict[str, float]) -> float:
    binary_operators = {
        ast.Add: operator.add,
        ast.Sub: operator.sub,
        ast.Mult: operator.mul,
        ast.Div: operator.truediv,
    }
    unary_operators = {ast.UAdd: operator.pos, ast.USub: operator.neg}

    if isinstance(node, ast.Constant) and isinstance(node.value, (int, float)):
        return float(node.value)
    if isinstance(node, ast.Name) and node.id in names:
        return names[node.id]
    if isinstance(node, ast.BinOp) and type(node.op) in binary_operators:
        return binary_operators[type(node.op)](
            _evaluate_ast(node.left, names),
            _evaluate_ast(node.right, names),
        )
    if isinstance(node, ast.UnaryOp) and type(node.op) in unary_operators:
        return unary_operators[type(node.op)](_evaluate_ast(node.operand, names))
    raise RobotCodeParseError("Java constant expression contains unsupported syntax.")
