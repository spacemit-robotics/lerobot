#!/usr/bin/env python

import sys
from collections import deque
from dataclasses import fields
from pathlib import Path
from types import SimpleNamespace

import torch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import lerobot_record_fp16
import spine_runtime
from benchmark_act_dummy_inference import benchmark_select_action
from lerobot_record_fp16 import _remove_legacy_dtype_argument

from lerobot.policies.act.configuration_act import ACTConfig
from lerobot.policies.act.modeling_act import ACT, ACTPolicy


def _make_policy_with_cached_action() -> ACTPolicy:
    policy = object.__new__(ACTPolicy)
    torch.nn.Module.__init__(policy)
    policy.config = SimpleNamespace(temporal_ensemble_coeff=None)
    policy._action_queue = deque([torch.tensor([[1.0, 2.0]], dtype=torch.float16)])
    return policy


def test_runtime_extension_does_not_add_a_persistent_config_field():
    spine_runtime.install_model_runtime()

    assert "dtype" not in {item.name for item in fields(ACTConfig)}
    assert ACT.forward.__module__ == "spine_runtime"


def test_fp16_processor_override_adds_float16_without_changing_device():
    overrides = {"device_processor": {"device": "cpu"}}

    merged = spine_runtime._with_fp16_preprocessor_overrides(overrides)

    assert merged["device_processor"] == {"device": "cpu", "float_dtype": "float16"}
    assert overrides == {"device_processor": {"device": "cpu"}}


def test_fp16_processor_override_preserves_explicit_dtype():
    overrides = {"device_processor": {"device": "cpu", "float_dtype": "float32"}}

    merged = spine_runtime._with_fp16_preprocessor_overrides(overrides)

    assert merged["device_processor"] == {"device": "cpu", "float_dtype": "float32"}


def test_cached_action_is_detected_only_for_act_policy():
    policy = _make_policy_with_cached_action()

    assert spine_runtime._policy_has_cached_action(policy)
    assert not spine_runtime._policy_has_cached_action(SimpleNamespace(_action_queue=policy._action_queue))


def test_temporal_ensemble_disables_cached_action_fast_path():
    policy = _make_policy_with_cached_action()
    policy.config.temporal_ensemble_coeff = 0.01

    assert not spine_runtime._policy_has_cached_action(policy)


def test_predict_action_skips_full_pipeline_for_cached_action(monkeypatch):
    policy = _make_policy_with_cached_action()

    def fail_full_pipeline(*args, **kwargs):
        raise AssertionError("full prediction pipeline should not run for a cached ACT action")

    monkeypatch.setattr(spine_runtime, "_ORIGINAL_PREDICT_ACTION", fail_full_pipeline)

    action = spine_runtime._predict_action_fp16(
        observation={},
        policy=policy,
        device=torch.device("cpu"),
        preprocessor=fail_full_pipeline,
        postprocessor=lambda value: value,
        use_amp=False,
    )

    assert torch.equal(action, torch.tensor([[1.0, 2.0]], dtype=torch.float16))


def test_predict_action_logs_full_pipeline_timing(monkeypatch, caplog):
    expected_action = torch.tensor([[1.0, 2.0]], dtype=torch.float16)
    monkeypatch.setenv("LEROBOT_PREDICT_TIMING", "1")
    monkeypatch.setattr(spine_runtime, "_ORIGINAL_PREDICT_ACTION", lambda **kwargs: expected_action)

    with caplog.at_level("INFO"):
        action = spine_runtime._predict_action_fp16(
            observation={},
            policy=SimpleNamespace(),
            device=torch.device("cpu"),
            preprocessor=None,
            postprocessor=None,
            use_amp=False,
        )

    assert action is expected_action
    assert "Predict action timing path=full total=" in caplog.text


def test_benchmark_resets_policy_for_every_warmup_and_timed_iteration():
    class FakePolicy:
        def __init__(self):
            self.parameter = torch.nn.Parameter(torch.zeros(1))
            self.reset_count = 0
            self.select_count = 0

        def parameters(self):
            yield self.parameter

        def reset(self):
            self.reset_count += 1

        def select_action(self, batch):
            self.select_count += 1
            return batch

    policy = FakePolicy()

    latencies = benchmark_select_action(policy, {}, warmup=3, iters=2)

    assert len(latencies) == 2
    assert policy.reset_count == 5
    assert policy.select_count == 5


def test_legacy_policy_dtype_argument_is_removed():
    arguments = [
        "--robot.type=so101_follower",
        "--policy.dtype=float16",
        "--policy.device=cpu",
    ]

    assert _remove_legacy_dtype_argument(arguments) == [
        "--robot.type=so101_follower",
        "--policy.device=cpu",
    ]


def test_split_legacy_policy_dtype_argument_is_removed():
    arguments = ["--policy.dtype", "float16", "--policy.device=cpu"]

    assert _remove_legacy_dtype_argument(arguments) == ["--policy.device=cpu"]


def test_cli_override_sanitizer_preserves_other_overrides():
    kwargs = {
        "cli_overrides": ["--device=cpu", "--dtype=float16", "--n_action_steps=50"],
        "local_files_only": True,
    }

    sanitized = spine_runtime._sanitize_cli_overrides(kwargs)

    assert sanitized == {
        "cli_overrides": ["--device=cpu", "--n_action_steps=50"],
        "local_files_only": True,
    }
    assert len(kwargs["cli_overrides"]) == 3


def test_record_entrypoint_initializes_logging_before_runtime(monkeypatch):
    from lerobot.scripts import lerobot_record
    from lerobot.utils import utils

    events: list[str] = []
    monkeypatch.setattr(sys, "argv", ["lerobot-record", "--policy.dtype=float16", "--policy.device=cpu"])
    monkeypatch.setattr(utils, "init_logging", lambda: events.append("logging"))
    monkeypatch.setattr(lerobot_record_fp16, "install_record_runtime", lambda: events.append("runtime"))
    monkeypatch.setattr(lerobot_record, "main", lambda: events.append("record"))

    lerobot_record_fp16.main()

    assert events == ["logging", "runtime", "record"]
    assert sys.argv == ["lerobot-record", "--policy.device=cpu"]
