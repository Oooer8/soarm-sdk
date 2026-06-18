from __future__ import annotations

import contextlib
import io
import json
import mimetypes
import re
import time
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from threading import Event, Lock, RLock, Thread, current_thread
from typing import Any
from urllib.parse import parse_qs, unquote, urlparse

from .arm import SOARM
from .calibration import AllJointsRangeRecorder, capture_zero_ticks
from .config import SOARMConfig
from .diagnostics import calibration_ready_from_report
from .errors import SOARMError
from .hardware import ServoBus
from .model import JointState
from .application.workflows import (
    apply_sweep_calibration,
    config_payload,
    fk_payload,
    ik_payload,
    is_calibrated,
    joint_state_payload,
    move_response_payload,
    recommended_move_duration,
    robot_model_state,
    workflow_payload,
)


ANSI_RE = re.compile(r"\x1b\[[0-9;]*m")
STATIC_ROOT = Path(__file__).with_name("webapp")
ASSET_ROOT = Path(__file__).with_name("assets")


class SOARMWebServer(ThreadingHTTPServer):
    allow_reuse_address = True


def _strip_ansi(text: str) -> str:
    return ANSI_RE.sub("", text)


class SOARMWebHandler(BaseHTTPRequestHandler):
    config_path = Path("configs/soarm.yaml")
    mock = False
    session_state: dict[str, Any] = {
        "status_passed": False,
        "calibration_ready": False,
        "calibrated": False,
        "torque_enabled": False,
        "last_status_lines": [],
        "_sweep_recorder": None,   # AllJointsRangeRecorder instance during sweep
        "_control_arm": None,
        "_control_mock": None,
        "_control_lock": RLock(),
        "_feedback_lock": Lock(),
    }

    def do_GET(self) -> None:
        parsed = urlparse(self.path)
        if parsed.path == "/api/session":
            self._handle_session()
        elif parsed.path == "/api/config":
            self._handle_config()
        elif parsed.path == "/api/ports":
            self._handle_ports()
        elif parsed.path == "/api/robot/model":
            self._handle_robot_model()
        elif parsed.path == "/api/state":
            self._handle_state(parsed.query)
        elif parsed.path.startswith("/assets/"):
            self._serve_asset(parsed.path)
        else:
            self._serve_static(parsed.path)

    def do_HEAD(self) -> None:
        parsed = urlparse(self.path)
        if parsed.path.startswith("/assets/"):
            self._serve_asset(parsed.path, send_body=False)
        else:
            self._serve_static(parsed.path, send_body=False)

    def do_POST(self) -> None:
        parsed = urlparse(self.path)
        if parsed.path == "/api/config/identity":
            self._handle_save_identity()
        elif parsed.path == "/api/config/frequencies":
            self._handle_save_frequencies()
        elif parsed.path == "/api/status":
            self._handle_status()
        elif parsed.path == "/api/calibration/disable":
            self._handle_calibration_disable()
        elif parsed.path == "/api/calibration/capture":
            self._handle_calibration_capture()
        elif parsed.path == "/api/calibration/sweep/start":
            self._handle_calibration_sweep_start()
        elif parsed.path == "/api/calibration/sweep/stop":
            self._handle_calibration_sweep_stop()
        elif parsed.path == "/api/calibration/save":
            self._handle_calibration_save()
        elif parsed.path == "/api/torque":
            self._handle_torque()
        elif parsed.path == "/api/move":
            self._handle_move()
        elif parsed.path == "/api/fk":
            self._handle_fk()
        elif parsed.path == "/api/ik":
            self._handle_ik()
        else:
            self._send_json(HTTPStatus.NOT_FOUND, {"error": "Unknown endpoint"})

    def log_message(self, format: str, *args: Any) -> None:
        print(f"web: {self.address_string()} - {format % args}")

    def log_request(self, code: int | str = "-", size: int | str = "-") -> None:
        parsed = urlparse(self.path)
        if (
            self.command == "GET"
            and parsed.path == "/api/state"
            and str(code).startswith(("2", "3"))
        ):
            return
        super().log_request(code, size)

    def _handle_config(self) -> None:
        try:
            config = SOARMConfig.from_file(self.config_path)
        except SOARMError as exc:
            self._send_json(HTTPStatus.BAD_REQUEST, {"error": str(exc)})
            return
        self._send_json(HTTPStatus.OK, config_payload(self.config_path, config))

    def _handle_session(self) -> None:
        try:
            config = SOARMConfig.from_file(self.config_path)
        except SOARMError as exc:
            self._send_json(HTTPStatus.BAD_REQUEST, {"error": str(exc)})
            return
        self._send_json(
            HTTPStatus.OK,
            {
                "config": config_payload(self.config_path, config),
                "workflow": self._workflow_payload(config),
                "torque_enabled": self._torque_enabled(config),
                "mock": self.mock,
            },
        )

    def _handle_ports(self) -> None:
        try:
            ports = ServoBus.list_ports()
        except Exception as exc:
            self._send_json(HTTPStatus.INTERNAL_SERVER_ERROR, {"error": str(exc)})
            return
        self._send_json(HTTPStatus.OK, {"ports": ports})

    def _handle_save_identity(self) -> None:
        try:
            data = self._read_json()
            name = str(data.get("name", ""))
            raw_port = data.get("port")
            port = str(raw_port).strip() if raw_port is not None else ""
            config = SOARMConfig.from_file(self.config_path)
            updated = config.replace_arm_identity(name=name, port=port or None)
            updated.save(self.config_path)
            self._reset_status_gate()
            self._close_control_arm(reset_torque=True)
        except (SOARMError, ValueError, TypeError) as exc:
            self._send_json(HTTPStatus.BAD_REQUEST, {"error": str(exc)})
            return
        self._send_json(HTTPStatus.OK, config_payload(self.config_path, updated))

    def _handle_save_frequencies(self) -> None:
        try:
            data = self._read_json()
            config = SOARMConfig.from_file(self.config_path)
            updated = config.replace_runtime_settings(
                control_hz=int(data.get("control_hz", config.arm.control_hz)),
                trajectory_input_hz=int(
                    data.get(
                        "trajectory_input_hz",
                        config.frequencies.trajectory_input_hz,
                    )
                ),
                feedback_hz=int(data.get("feedback_hz", config.frequencies.feedback_hz)),
                web_motion_hz=int(data.get("web_motion_hz", config.frequencies.web_motion_hz)),
                web_state_hz=int(data.get("web_state_hz", config.frequencies.web_state_hz)),
                web_render_hz=int(data.get("web_render_hz", config.frequencies.web_render_hz)),
            )
            updated.save(self.config_path)
            self._reset_status_gate()
            self._close_control_arm(reset_torque=True)
        except (SOARMError, ValueError, TypeError) as exc:
            self._send_json(HTTPStatus.BAD_REQUEST, {"error": str(exc)})
            return
        self._send_json(HTTPStatus.OK, config_payload(self.config_path, updated))

    def _handle_status(self) -> None:
        output = io.StringIO()
        try:
            data = self._read_json()
            mock = bool(data.get("mock", self.mock))
            config = SOARMConfig.from_file(self.config_path)
            status = self._collect_status(config, mock=mock, output=output)
        except Exception as exc:
            captured = [_strip_ansi(line) for line in output.getvalue().splitlines()]
            self._send_json(
                HTTPStatus.INTERNAL_SERVER_ERROR,
                {"error": str(exc), "lines": captured},
            )
            return

        self.session_state["status_passed"] = status["passed"]
        self.session_state["calibration_ready"] = status["calibration_ready"]
        self.session_state["last_status_lines"] = status["lines"]
        if not self._control_gate_open(config):
            self.session_state["torque_enabled"] = False
        self._send_json(
            HTTPStatus.OK,
            {
                "passed": status["passed"],
                "calibration_ready": status["calibration_ready"],
                "mock": mock,
                "lines": status["lines"],
                "workflow": self._workflow_payload(config),
            },
        )

    def _handle_calibration_disable(self) -> None:
        output = io.StringIO()
        try:
            data = self._read_json()
            mock = bool(data.get("mock", self.mock))
            config = SOARMConfig.from_file(self.config_path)
            if not self._status_gate_open(config):
                self._send_json(
                    HTTPStatus.CONFLICT,
                    {"error": "Run a calibration-ready status check before calibration."},
                )
                return
            self._close_control_arm(reset_torque=True)
            with contextlib.redirect_stdout(output), contextlib.redirect_stderr(output):
                arm = SOARM.from_config(self.config_path, mock=mock)
                with arm:
                    arm.disable()
        except Exception as exc:
            captured = [_strip_ansi(line) for line in output.getvalue().splitlines()]
            self._send_json(
                HTTPStatus.INTERNAL_SERVER_ERROR,
                {"error": str(exc), "lines": captured},
            )
            return

        clean_lines = [_strip_ansi(line) for line in output.getvalue().splitlines()]
        self._send_json(
            HTTPStatus.OK,
            {
                "mock": mock,
                "lines": clean_lines,
                "message": "Servo torque disabled for manual calibration.",
            },
        )

    def _handle_calibration_sweep_start(self) -> None:
        """Start an AllJointsRangeRecorder sweep in the background.

        Expects JSON body::

            {
                "mock": bool,
                "zero_ticks": {"joint_name": tick, ...},
                "margin_ticks": int,        # optional, default 0
                "motion_threshold": int,    # optional, default 100
                "filter_window": int        # optional, default 3
            }
        """
        arm: SOARM | None = None
        try:
            data = self._read_json()
            mock = bool(data.get("mock", self.mock))
            raw_zero_ticks = data.get("zero_ticks")
            if not isinstance(raw_zero_ticks, dict) or not raw_zero_ticks:
                raise ValueError("zero_ticks must be a non-empty object keyed by joint name")
            zero_ticks: dict[str, int] = {str(k): int(v) for k, v in raw_zero_ticks.items()}
            config = SOARMConfig.from_file(self.config_path)
            if not self._status_gate_open(config):
                self._send_json(
                    HTTPStatus.CONFLICT,
                    {"error": "Run a calibration-ready status check before calibration."},
                )
                return
            self._close_control_arm(reset_torque=True)

            # Stop any running recorder first
            existing = self.session_state.get("_sweep_recorder")
            if existing is not None:
                try:
                    existing.stop()
                except Exception:  # noqa: BLE001
                    pass

            arm = SOARM(config, mock=mock)
            arm.connect()

            recorder = AllJointsRangeRecorder(
                config,
                arm.bus,
                zero_ticks,
                margin_ticks=int(data.get("margin_ticks", 0)),
                motion_threshold=int(data.get("motion_threshold", 100)),
                filter_window=int(data.get("filter_window", 3)),
            )
            recorder.start()
            self.session_state["_sweep_recorder"] = recorder
            # Keep the arm bus alive; the recorder holds a reference to it
            self.session_state["_sweep_arm"] = arm
            arm = None
        except Exception as exc:
            if arm is not None:
                try:
                    arm.disconnect()
                except Exception:  # noqa: BLE001
                    pass
            self._send_json(HTTPStatus.BAD_REQUEST, {"error": str(exc)})
            return
        self._send_json(
            HTTPStatus.OK,
            {
                "message": "Sweep recording started.",
                "settings": {
                    "margin_ticks": recorder.margin_ticks,
                    "motion_threshold": recorder.motion_threshold,
                    "filter_window": recorder.filter_window,
                    "poll_interval": recorder.poll_interval,
                },
            },
        )

    def _handle_calibration_sweep_stop(self) -> None:
        """Stop the running sweep and return per-joint results."""
        recorder: AllJointsRangeRecorder | None = self.session_state.get("_sweep_recorder")
        if recorder is None:
            self._send_json(HTTPStatus.CONFLICT, {"error": "No sweep is currently running."})
            return
        try:
            results = recorder.stop()
        except Exception as exc:
            self._send_json(HTTPStatus.BAD_REQUEST, {"error": str(exc)})
            return
        finally:
            self.session_state["_sweep_recorder"] = None
            arm: SOARM | None = self.session_state.pop("_sweep_arm", None)
            if arm is not None:
                try:
                    arm.disconnect()
                except Exception:  # noqa: BLE001
                    pass

        payload_results = [
            {
                "name": r.name,
                "zero_tick": r.zero_tick,
                "raw_min_tick": r.raw_min_tick,
                "raw_max_tick": r.raw_max_tick,
                "safe_min_tick": r.safe_min_tick,
                "safe_max_tick": r.safe_max_tick,
                "inferred_direction": r.inferred_direction,
                "range_ticks": r.range_ticks,
                "well_excited": r.well_excited,
                "safe_min_rad": r.safe_min_rad,
                "safe_max_rad": r.safe_max_rad,
            }
            for r in results.values()
        ]
        self._send_json(HTTPStatus.OK, {"results": payload_results})

    def _handle_calibration_capture(self) -> None:
        output = io.StringIO()
        try:
            data = self._read_json()
            mock = bool(data.get("mock", self.mock))
            sample = str(data.get("sample", "zero"))
            config = SOARMConfig.from_file(self.config_path)
            if not self._status_gate_open(config):
                self._send_json(
                    HTTPStatus.CONFLICT,
                    {"error": "Run a calibration-ready status check before calibration."},
                )
                return
            if sample != "zero":
                raise ValueError("Only zero tick capture is supported by the web calibration flow")
            with contextlib.redirect_stdout(output), contextlib.redirect_stderr(output):
                arm = SOARM(config, mock=mock)
                with arm:
                    ticks = capture_zero_ticks(config, arm.bus)
            payload = {"mock": mock, "sample": sample, "ticks": ticks}
        except Exception as exc:
            captured = [_strip_ansi(line) for line in output.getvalue().splitlines()]
            self._send_json(
                HTTPStatus.BAD_REQUEST,
                {"error": str(exc), "lines": captured},
            )
            return

        payload["lines"] = [_strip_ansi(line) for line in output.getvalue().splitlines()]
        self._send_json(HTTPStatus.OK, payload)

    def _handle_calibration_save(self) -> None:
        """Save calibration from dynamic sweep results.

        Expects JSON body::

            {
                "mock": bool,
                "sweep_results": [
                    {
                        "name": str,
                        "zero_tick": int,
                        "safe_min_tick": int,
                        "safe_max_tick": int,
                        "inferred_direction": int   # optional, echoed from sweep results
                    },
                    ...
                ]
            }

        """
        output = io.StringIO()
        try:
            data = self._read_json()
            mock = bool(data.get("mock", self.mock))
            config = SOARMConfig.from_file(self.config_path)
            updated, summary = apply_sweep_calibration(config, data.get("sweep_results"))
            if not mock:
                updated.save(self.config_path)
            self.session_state["calibrated"] = True
        except (SOARMError, ValueError, TypeError, KeyError) as exc:
            self._send_json(HTTPStatus.BAD_REQUEST, {"error": str(exc)})
            return

        post_status: dict[str, Any] | None = None
        post_status_error: str | None = None
        try:
            post_status = self._collect_status(updated, mock=mock, output=output)
            self.session_state["status_passed"] = post_status["passed"]
            self.session_state["calibration_ready"] = post_status["calibration_ready"]
            self.session_state["last_status_lines"] = post_status["lines"]
        except Exception as exc:  # noqa: BLE001
            post_status_error = str(exc)
            post_status = {
                "passed": False,
                "calibration_ready": False,
                "lines": [_strip_ansi(line) for line in output.getvalue().splitlines()],
            }
            self.session_state["status_passed"] = False
            self.session_state["calibration_ready"] = False
            self.session_state["last_status_lines"] = post_status["lines"]
        finally:
            self._close_control_arm(reset_torque=True)

        self._send_json(
            HTTPStatus.OK,
            {
                "config": config_payload(self.config_path, updated),
                "summary": summary,
                "saved": not mock,
                "post_status": post_status,
                "post_status_error": post_status_error,
                "message": "Mock calibration computed; config was not written."
                if mock
                else "Calibration saved to config.",
                "workflow": self._workflow_payload(updated),
            },
        )

    def _handle_robot_model(self) -> None:
        try:
            config = SOARMConfig.from_file(self.config_path)
            if not self._control_gate_open(config):
                self._send_json(
                    HTTPStatus.CONFLICT,
                    {
                        "error": "Run a passing status check after calibration before loading the SOARM URDF.",
                        "workflow": self._workflow_payload(config),
                    },
                )
                return
            model_state = robot_model_state(config)
            self._send_json(
                HTTPStatus.OK,
                {
                    "name": model_state["name"],
                    "urdf_url": "/assets/soarm101/urdf/so_arm101.urdf",
                    "model_source": {
                        "repository": "MuammerBay/isaac_so_arm101",
                        "url": "https://github.com/MuammerBay/isaac_so_arm101/blob/main/src/isaac_so_arm101/robots/trs_so101/urdf/so_arm101.urdf",
                    },
                    "viewer": "soarm-web",
                    "joint_order": model_state["joint_order"],
                    "config": config_payload(self.config_path, config),
                    "model_joints": model_state["model_joints"],
                    "fk": model_state["fk"],
                    "workflow": self._workflow_payload(config),
                    "torque_enabled": self._torque_enabled(config),
                },
            )
        except SOARMError as exc:
            self._send_json(HTTPStatus.BAD_REQUEST, {"error": str(exc)})

    def _handle_state(self, query: str = "") -> None:
        output = io.StringIO()
        try:
            config = SOARMConfig.from_file(self.config_path)
            if not self._is_calibrated(config):
                self._send_json(
                    HTTPStatus.CONFLICT,
                    {"error": "Read joint state after calibration."},
                )
                return
            mock = self._mock_from_query(query)
            self._ensure_feedback_sampler(config, mock=mock)
            states, cached, sample_age_ms = self._cached_feedback_states(config, mock=mock)
            if states is not None:
                payload = joint_state_payload(config, states, mock=mock)
                payload["torque_enabled"] = self._torque_enabled(config)
                payload["feedback"] = {
                    "hz": config.frequencies.feedback_hz,
                    "cached": cached,
                    "age_ms": sample_age_ms,
                }
                self._send_json(HTTPStatus.OK, payload)
                return
            with contextlib.redirect_stdout(output), contextlib.redirect_stderr(output):
                with self._control_lock():
                    arm = self._control_arm(config, mock=mock)
                    states = self._sample_feedback_states(config, arm)
            self._store_feedback_sample(config, mock=mock, states=states)
        except Exception as exc:
            self._close_control_arm()
            self._send_json(
                HTTPStatus.INTERNAL_SERVER_ERROR,
                {
                    "error": str(exc),
                    "lines": [_strip_ansi(line) for line in output.getvalue().splitlines()],
                },
            )
            return

        payload = joint_state_payload(config, states, mock=mock)
        payload["torque_enabled"] = self._torque_enabled(config)
        payload["feedback"] = {
            "hz": config.frequencies.feedback_hz,
            "cached": False,
            "age_ms": 0,
        }
        self._send_json(HTTPStatus.OK, payload)

    def _handle_torque(self) -> None:
        output = io.StringIO()
        try:
            data = self._read_json()
            enabled = bool(data.get("enabled", False))
            mock = bool(data.get("mock", self.mock))
            config = SOARMConfig.from_file(self.config_path)
            if not self._control_gate_open(config):
                if enabled:
                    self._send_json(
                        HTTPStatus.CONFLICT,
                        {
                            "error": "Joint enable is available after calibration and a passing status check.",
                            "workflow": self._workflow_payload(config),
                            "torque_enabled": False,
                        },
                    )
                    return
                self._close_control_arm(reset_torque=True)
                self.session_state["torque_enabled"] = False
                self._send_json(
                    HTTPStatus.OK,
                    {
                        "mock": mock,
                        "torque_enabled": False,
                        "workflow": self._workflow_payload(config),
                    },
                )
                return

            with contextlib.redirect_stdout(output), contextlib.redirect_stderr(output):
                with self._control_lock():
                    arm = self._control_arm(config, mock=mock)
                    if enabled:
                        arm.enable()
                        result = arm.bus.enable_torque()
                    else:
                        result = arm.bus.disable_torque()
                        arm._enabled = False
                    self.session_state["torque_enabled"] = enabled
                    states = self._sample_feedback_states(config, arm)
            self._store_feedback_sample(config, mock=mock, states=states)
        except Exception as exc:
            self._close_control_arm(reset_torque=True)
            self._send_json(
                HTTPStatus.BAD_REQUEST,
                {
                    "error": str(exc),
                    "lines": [_strip_ansi(line) for line in output.getvalue().splitlines()],
                },
            )
            return

        payload = joint_state_payload(config, states, mock=mock)
        payload["torque_enabled"] = bool(enabled)
        payload["result"] = result
        payload["workflow"] = self._workflow_payload(config)
        payload["lines"] = [_strip_ansi(line) for line in output.getvalue().splitlines()]
        self._send_json(HTTPStatus.OK, payload)

    def _handle_move(self) -> None:
        output = io.StringIO()
        try:
            data = self._read_json()
            config = SOARMConfig.from_file(self.config_path)
            if not self._control_gate_open(config):
                self._send_json(
                    HTTPStatus.CONFLICT,
                    {
                        "error": "Joint control is available after calibration and a passing status check.",
                        "workflow": self._workflow_payload(config),
                    },
                )
                return
            targets = data.get("targets")
            if not isinstance(targets, dict):
                raise ValueError("targets must be an object keyed by joint name")
            clean_targets = {str(name): float(value) for name, value in targets.items()}
            sync = bool(data.get("sync", False))
            mock = bool(data.get("mock", self.mock))
            requested_duration = data.get("duration")
            duration = None if requested_duration is None else float(requested_duration)
            wait = bool(data.get("wait", False))
            if sync and not self._torque_enabled(config):
                self._send_json(
                    HTTPStatus.CONFLICT,
                    {
                        "error": "Turn on joint enable before sending motion commands.",
                        "workflow": self._workflow_payload(config),
                        "torque_enabled": False,
                    },
                )
                return

            states = {}
            if sync:
                with contextlib.redirect_stdout(output), contextlib.redirect_stderr(output):
                    with self._control_lock():
                        arm = self._control_arm(config, mock=mock)
                        if duration is None:
                            current = arm.motion.read_positions_rad()
                            duration = recommended_move_duration(
                                config,
                                current=current,
                                target=clean_targets,
                            )
                        arm.move_joints(clean_targets, duration=duration, wait=wait)
                        states = arm.get_joint_states()
        except Exception as exc:
            self._close_control_arm()
            self._send_json(
                HTTPStatus.BAD_REQUEST,
                {
                    "error": str(exc),
                    "lines": [_strip_ansi(line) for line in output.getvalue().splitlines()],
                },
            )
            return

        payload = move_response_payload(
            config,
            mock=mock,
            synced=sync,
            duration=duration,
            targets=clean_targets,
            states=states,
            lines=[_strip_ansi(line) for line in output.getvalue().splitlines()],
        )
        payload["torque_enabled"] = self._torque_enabled(config)
        self._send_json(
            HTTPStatus.OK,
            payload,
        )

    def _handle_fk(self) -> None:
        try:
            data = self._read_json()
            config = SOARMConfig.from_file(self.config_path)
            positions = data.get("positions")
            if positions is None:
                positions = {}
            if not isinstance(positions, dict):
                raise ValueError("positions must be an object keyed by joint name")
            self._send_json(HTTPStatus.OK, fk_payload(config, positions))
        except (SOARMError, ValueError, TypeError) as exc:
            self._send_json(HTTPStatus.BAD_REQUEST, {"error": str(exc)})

    def _handle_ik(self) -> None:
        try:
            data = self._read_json()
            config = SOARMConfig.from_file(self.config_path)
            target = data.get("target")
            if not isinstance(target, dict):
                raise ValueError("target must include x, y, and z")
            elbow = str(data.get("elbow", "down"))
            self._send_json(HTTPStatus.OK, ik_payload(config, target, elbow=elbow))
        except (SOARMError, ValueError, TypeError) as exc:
            self._send_json(HTTPStatus.BAD_REQUEST, {"error": str(exc)})

    def _collect_status(
        self,
        config: SOARMConfig,
        *,
        mock: bool,
        output: io.StringIO | None = None,
    ) -> dict[str, Any]:
        output = output or io.StringIO()
        with contextlib.redirect_stdout(output), contextlib.redirect_stderr(output):
            arm = SOARM(config, mock=mock)
            with arm:
                lines = arm.diagnostics()
        clean_lines = [_strip_ansi(line) for line in output.getvalue().splitlines() + lines]
        passed = not any("[FAIL]" in line for line in clean_lines)
        calibration_ready = calibration_ready_from_report(clean_lines)
        return {
            "passed": passed,
            "calibration_ready": calibration_ready,
            "lines": clean_lines,
        }

    def _is_calibrated(self, config: SOARMConfig) -> bool:
        return is_calibrated(
            config,
            session_calibrated=bool(self.session_state.get("calibrated")),
        )

    def _control_gate_open(self, config: SOARMConfig) -> bool:
        return bool(self._is_calibrated(config) and self.session_state.get("status_passed"))

    def _torque_enabled(self, config: SOARMConfig) -> bool:
        return bool(self._control_gate_open(config) and self.session_state.get("torque_enabled"))

    def _status_gate_open(self, _config: SOARMConfig) -> bool:
        return bool(
            self.session_state.get("status_passed")
            or self.session_state.get("calibration_ready")
        )

    def _reset_status_gate(self) -> None:
        self.session_state["status_passed"] = False
        self.session_state["calibration_ready"] = False
        self.session_state["torque_enabled"] = False
        self.session_state["last_status_lines"] = []

    def _workflow_payload(self, config: SOARMConfig) -> dict[str, Any]:
        return workflow_payload(
            config,
            status_passed=bool(self.session_state.get("status_passed")),
            calibration_ready=bool(self.session_state.get("calibration_ready")),
            session_calibrated=bool(self.session_state.get("calibrated")),
        )

    def _control_lock(self):
        lock = self.session_state.get("_control_lock")
        if lock is None:
            lock = RLock()
            self.session_state["_control_lock"] = lock
        return lock

    def _feedback_lock(self):
        lock = self.session_state.get("_feedback_lock")
        if lock is None:
            lock = Lock()
            self.session_state["_feedback_lock"] = lock
        return lock

    def _control_arm(self, config: SOARMConfig, *, mock: bool) -> SOARM:
        arm: SOARM | None = self.session_state.get("_control_arm")
        if arm is not None and self.session_state.get("_control_mock") == mock and arm.connected:
            return arm
        if arm is not None:
            try:
                arm.disconnect()
            except Exception:  # noqa: BLE001
                pass
        arm = SOARM(config, mock=mock)
        arm.connect()
        if self._torque_enabled(config):
            arm.enable()
            arm.bus.enable_torque()
        self.session_state["_control_arm"] = arm
        self.session_state["_control_mock"] = mock
        return arm

    def _feedback_signature(self, config: SOARMConfig, *, mock: bool) -> tuple[Any, ...]:
        joint_signature = tuple(
            (name, joint.id, joint.zero_tick, joint.direction)
            for name, joint in config.joints.items()
        )
        return (
            mock,
            config.arm.port,
            config.arm.baudrate,
            config.arm.control_hz,
            config.frequencies.feedback_hz,
            joint_signature,
        )

    def _ensure_feedback_sampler(self, config: SOARMConfig, *, mock: bool) -> None:
        signature = self._feedback_signature(config, mock=mock)
        with self._feedback_lock():
            thread: Thread | None = self.session_state.get("_feedback_thread")
            if (
                thread is not None
                and thread.is_alive()
                and self.session_state.get("_feedback_signature") == signature
            ):
                return

        self._stop_feedback_sampler()

        stop_event = Event()
        thread = Thread(
            target=self._feedback_loop,
            args=(signature, stop_event, mock),
            daemon=True,
            name="soarm-feedback",
        )
        with self._feedback_lock():
            self.session_state["_feedback_signature"] = signature
            self.session_state["_feedback_stop"] = stop_event
            self.session_state["_feedback_thread"] = thread
            self.session_state["_feedback_states"] = None
            self.session_state["_feedback_error"] = None
            self.session_state["_feedback_sample_time"] = None
        thread.start()

    def _feedback_loop(self, signature: tuple[Any, ...], stop_event: Event, mock: bool) -> None:
        while not stop_event.is_set():
            started = time.monotonic()
            try:
                config = SOARMConfig.from_file(self.config_path)
                if self._feedback_signature(config, mock=mock) != signature:
                    break
                with self._control_lock():
                    arm = self._control_arm(config, mock=mock)
                    states = self._sample_feedback_states(config, arm)
                self._store_feedback_sample(config, mock=mock, states=states)
                interval = 1.0 / max(1, int(config.frequencies.feedback_hz))
            except Exception as exc:  # noqa: BLE001
                self._store_feedback_error(signature, str(exc))
                interval = 0.25

            elapsed = time.monotonic() - started
            stop_event.wait(max(0.0, interval - elapsed))

    def _sample_feedback_states(self, config: SOARMConfig, arm: SOARM) -> dict[str, JointState]:
        ticks = arm.bus.read_positions(strict=False)
        now = time.monotonic()
        voltages = self.session_state.get("_feedback_voltages") or {}
        voltage_at = float(self.session_state.get("_feedback_voltage_at") or 0.0)
        if now - voltage_at >= 1.0:
            voltages = arm.bus.read_voltages(strict=False)
            self.session_state["_feedback_voltages"] = voltages
            self.session_state["_feedback_voltage_at"] = now

        states: dict[str, JointState] = {}
        for name, joint in config.joints.items():
            tick = ticks.get(joint.id)
            voltage = voltages.get(joint.id)
            states[name] = JointState(
                name=name,
                id=joint.id,
                position_tick=tick,
                position_rad=None if tick is None else joint.tick_to_rad(tick),
                voltage=voltage,
                online=tick is not None,
            )
        return states

    def _store_feedback_sample(
        self,
        config: SOARMConfig,
        *,
        mock: bool,
        states: dict[str, JointState],
    ) -> None:
        signature = self._feedback_signature(config, mock=mock)
        with self._feedback_lock():
            if self.session_state.get("_feedback_signature") != signature:
                return
            self.session_state["_feedback_states"] = states
            self.session_state["_feedback_sample_time"] = time.time()
            self.session_state["_feedback_error"] = None

    def _store_feedback_error(self, signature: tuple[Any, ...], message: str) -> None:
        with self._feedback_lock():
            if self.session_state.get("_feedback_signature") == signature:
                self.session_state["_feedback_error"] = message

    def _cached_feedback_states(
        self,
        config: SOARMConfig,
        *,
        mock: bool,
    ) -> tuple[dict[str, JointState] | None, bool, int | None]:
        signature = self._feedback_signature(config, mock=mock)
        with self._feedback_lock():
            if self.session_state.get("_feedback_signature") != signature:
                return None, False, None
            states = self.session_state.get("_feedback_states")
            sampled_at = self.session_state.get("_feedback_sample_time")
        if not states:
            return None, False, None
        age_ms = None if sampled_at is None else max(0, int((time.time() - float(sampled_at)) * 1000))
        return dict(states), True, age_ms

    def _stop_feedback_sampler(self) -> None:
        with self._feedback_lock():
            stop_event: Event | None = self.session_state.pop("_feedback_stop", None)
            thread: Thread | None = self.session_state.pop("_feedback_thread", None)
            self.session_state.pop("_feedback_signature", None)
            self.session_state.pop("_feedback_states", None)
            self.session_state.pop("_feedback_sample_time", None)
            self.session_state.pop("_feedback_error", None)
            self.session_state.pop("_feedback_voltages", None)
            self.session_state.pop("_feedback_voltage_at", None)

        if stop_event is not None:
            stop_event.set()
        if thread is not None and thread.is_alive() and thread is not current_thread():
            thread.join(timeout=1.0)

    def _close_control_arm(self, *, reset_torque: bool = False) -> None:
        self._stop_feedback_sampler()
        if reset_torque:
            self.session_state["torque_enabled"] = False
        with self._control_lock():
            arm: SOARM | None = self.session_state.pop("_control_arm", None)
            self.session_state.pop("_control_mock", None)
            if arm is not None:
                try:
                    arm.disconnect()
                except Exception:  # noqa: BLE001
                    pass

    def _mock_from_query(self, query: str) -> bool:
        values = parse_qs(query).get("mock")
        if not values:
            return self.mock
        return values[-1].lower() in {"1", "true", "yes", "on"}

    def _read_json(self) -> dict[str, Any]:
        length = int(self.headers.get("Content-Length", "0"))
        if length == 0:
            return {}
        raw = self.rfile.read(length)
        return json.loads(raw.decode("utf-8"))

    def _serve_static(self, request_path: str, *, send_body: bool = True) -> None:
        if request_path in ("", "/"):
            request_path = "/index.html"
        relative = Path(unquote(request_path.lstrip("/")))
        if relative.is_absolute() or ".." in relative.parts:
            self.send_error(HTTPStatus.FORBIDDEN)
            return

        path = STATIC_ROOT / relative
        if not path.is_file():
            self.send_error(HTTPStatus.NOT_FOUND)
            return

        content_type, _ = mimetypes.guess_type(path.name)
        content = path.read_bytes()
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", content_type or "application/octet-stream")
        self.send_header("Content-Length", str(len(content)))
        self.end_headers()
        if send_body:
            self.wfile.write(content)

    def _serve_asset(self, request_path: str, *, send_body: bool = True) -> None:
        relative = Path(unquote(request_path.removeprefix("/assets/")))
        if relative.is_absolute() or ".." in relative.parts:
            self.send_error(HTTPStatus.FORBIDDEN)
            return
        path = ASSET_ROOT / relative
        if not path.is_file():
            self.send_error(HTTPStatus.NOT_FOUND)
            return
        content_type, _ = mimetypes.guess_type(path.name)
        content = path.read_bytes()
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", content_type or "application/octet-stream")
        self.send_header("Content-Length", str(len(content)))
        self.end_headers()
        if send_body:
            self.wfile.write(content)

    def _send_json(self, status: HTTPStatus, payload: dict[str, Any]) -> None:
        content = json.dumps(payload).encode("utf-8")
        try:
            self.send_response(status)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(content)))
            self.end_headers()
            self.wfile.write(content)
        except (BrokenPipeError, ConnectionResetError):
            return


def run_web(
    *,
    config_path: str | Path = "configs/soarm.yaml",
    host: str = "127.0.0.1",
    port: int = 8765,
    mock: bool = False,
) -> int:
    path = Path(config_path)
    handler = type(
        "ConfiguredSOARMWebHandler",
        (SOARMWebHandler,),
        {
            "config_path": path,
            "mock": mock,
            "session_state": {
                "status_passed": False,
                "calibration_ready": False,
                "calibrated": False,
                "torque_enabled": False,
                "last_status_lines": [],
                "_control_arm": None,
                "_control_mock": None,
                "_control_lock": RLock(),
                "_feedback_lock": Lock(),
            },
        },
    )
    server = SOARMWebServer((host, int(port)), handler)
    url = f"http://{host}:{server.server_port}/"
    print(f"SOARM web setup: {url}")
    print(f"Config: {path}")
    print("Press Ctrl-C to stop.")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nStopping SOARM web setup.")
    finally:
        stop_event: Event | None = handler.session_state.get("_feedback_stop")
        thread: Thread | None = handler.session_state.get("_feedback_thread")
        if stop_event is not None:
            stop_event.set()
        if thread is not None and thread.is_alive():
            thread.join(timeout=1.0)
        for key in ("_control_arm", "_sweep_arm"):
            arm = handler.session_state.get(key)
            if arm is not None:
                try:
                    arm.disconnect()
                except Exception:  # noqa: BLE001
                    pass
        server.server_close()
    return 0
