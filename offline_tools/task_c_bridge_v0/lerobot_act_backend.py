"""Local LeRobot ACT backend for command-free Task-C validation.

The backend loads a checkpoint, owns its preprocessing state, and returns
postprocessed physical action chunks.  It imports no ROS or robot adapter.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np


EXPECTED_STATE_DIM = 13
EXPECTED_ACTION_DIM = 7
EXPECTED_N_OBS_STEPS = 1
EXPECTED_CHUNK_SIZE = 100


def _canonical_torch_device(device: Any, torch_module: Any) -> Any:
    """Resolve an implicit CUDA device to the process's current CUDA index."""

    resolved = torch_module.device(device)
    if resolved.type == "cuda" and resolved.index is None:
        return torch_module.device("cuda", torch_module.cuda.current_device())
    return resolved


@dataclass(frozen=True)
class RecordedObservation:
    policy_input: dict[str, Any]
    dataset_root: str
    episode: int
    frame: int
    global_index: int
    timestamp_s: float
    tcp_position_mm: np.ndarray
    gripper_closed: bool

    def __post_init__(self) -> None:
        position = np.asarray(self.tcp_position_mm, dtype=np.float64)
        if position.shape != (3,) or not np.all(np.isfinite(position)):
            raise ValueError("recorded observation TCP must be finite XYZ")
        object.__setattr__(self, "tcp_position_mm", position.copy())


class LeRobotACTBackend:
    """One resident ACT model with an isolated policy/preprocessor state."""

    def __init__(
        self,
        checkpoint: str | Path,
        *,
        device: str = "cuda",
        strict: bool = True,
    ) -> None:
        import torch
        from lerobot.configs.policies import PreTrainedConfig
        from lerobot.policies import make_pre_post_processors
        from lerobot.policies.act.configuration_act import ACTConfig
        from lerobot.policies.act.modeling_act import ACTPolicy

        checkpoint_path = Path(checkpoint).expanduser().resolve()
        if not (checkpoint_path / "model.safetensors").is_file():
            raise FileNotFoundError(
                f"checkpoint is missing model.safetensors: {checkpoint_path}"
            )
        if device.startswith("cuda") and not torch.cuda.is_available():
            raise RuntimeError(f"CUDA requested but unavailable: {device}")

        config = PreTrainedConfig.from_pretrained(
            checkpoint_path,
            local_files_only=True,
        )
        if not isinstance(config, ACTConfig):
            raise TypeError(
                f"expected ACT checkpoint, got {type(config).__name__}: "
                f"{checkpoint_path}"
            )
        self._validate_config(config)
        config.device = device

        policy = ACTPolicy.from_pretrained(
            checkpoint_path,
            config=config,
            local_files_only=True,
            strict=strict,
        )
        policy.to(torch.device(device))
        policy.eval()
        preprocessor, postprocessor = make_pre_post_processors(
            policy_cfg=config,
            pretrained_path=str(checkpoint_path),
            preprocessor_overrides={"device_processor": {"device": device}},
        )

        self.checkpoint = checkpoint_path
        self.device = torch.device(device)
        self.config = config
        self.policy = policy
        self.preprocessor = preprocessor
        self.postprocessor = postprocessor
        self.input_keys = tuple(config.input_features)
        self.action_dim = int(config.action_feature.shape[0])
        self.action_steps = int(config.n_action_steps)

    @staticmethod
    def _validate_config(config: Any) -> None:
        state_shape = tuple(config.robot_state_feature.shape)
        action_shape = tuple(config.action_feature.shape)
        expected_images = {
            "observation.images.front": (3, 480, 640),
            "observation.images.side": (3, 480, 640),
            "observation.images.zed_rgb": (3, 376, 672),
        }
        actual_images = {
            name: tuple(feature.shape)
            for name, feature in config.image_features.items()
        }
        errors: list[str] = []
        if state_shape != (EXPECTED_STATE_DIM,):
            errors.append(f"state_shape={state_shape}")
        if action_shape != (EXPECTED_ACTION_DIM,):
            errors.append(f"action_shape={action_shape}")
        if int(config.n_obs_steps) != EXPECTED_N_OBS_STEPS:
            errors.append(f"n_obs_steps={config.n_obs_steps}")
        if int(config.chunk_size) != EXPECTED_CHUNK_SIZE:
            errors.append(f"chunk_size={config.chunk_size}")
        if int(config.n_action_steps) != EXPECTED_CHUNK_SIZE:
            errors.append(f"n_action_steps={config.n_action_steps}")
        if actual_images != expected_images:
            errors.append(f"image_features={actual_images}")
        if config.temporal_ensemble_coeff is not None:
            errors.append(
                f"temporal_ensemble_coeff={config.temporal_ensemble_coeff}"
            )
        if errors:
            raise ValueError(
                "checkpoint contract differs from recorded A0509 ACT V0: "
                + ", ".join(errors)
            )

    def reset(self) -> None:
        self.policy.reset()
        for processor in (self.preprocessor, self.postprocessor):
            reset = getattr(processor, "reset", None)
            if callable(reset):
                reset()

    def _validate_raw_observation(
        self,
        observation: Any,
    ) -> dict[str, Any]:
        import torch

        if not isinstance(observation, dict):
            raise TypeError("ACT observation must be a dictionary")
        missing = [key for key in self.input_keys if key not in observation]
        if missing:
            raise KeyError(f"ACT observation is missing keys: {missing}")
        raw = {key: observation[key] for key in self.input_keys}
        for key, feature in self.config.input_features.items():
            value = raw[key]
            if not isinstance(value, torch.Tensor):
                value = torch.as_tensor(value, dtype=torch.float32)
                raw[key] = value
            expected = tuple(feature.shape)
            if tuple(value.shape) != expected:
                raise ValueError(
                    f"{key} shape {tuple(value.shape)} does not match {expected}"
                )
            if not torch.isfinite(value).all():
                raise ValueError(f"{key} contains non-finite values")
        return raw

    def infer(self, observation: Any) -> np.ndarray:
        """Return one unbatched, physical-unit ACT action chunk."""

        import torch

        raw = self._validate_raw_observation(observation)
        batch = self.preprocessor(raw)
        with torch.inference_mode():
            normalized = self.policy.predict_action_chunk(batch)
            physical = self.postprocessor(normalized)
        if self.device.type == "cuda":
            torch.cuda.synchronize(self.device)
        actions = physical.detach().to(dtype=torch.float32, device="cpu").numpy()
        if actions.ndim != 3 or actions.shape[0] != 1:
            raise RuntimeError(
                f"ACT returned unexpected postprocessed shape {actions.shape}"
            )
        actions = actions[0]
        expected_shape = (self.action_steps, self.action_dim)
        if actions.shape != expected_shape:
            raise RuntimeError(
                f"ACT returned {actions.shape}, expected {expected_shape}"
            )
        if not np.all(np.isfinite(actions)):
            raise RuntimeError("ACT returned non-finite physical actions")
        return actions.astype(np.float64, copy=False)


class ResidentLeRobotACTBackend(LeRobotACTBackend):
    """Adapt an ACT policy already loaded by ``build_rollout_context``.

    The rollout context remains the sole owner of ACT-A model loading. This
    adapter borrows that resident model and its isolated processor pair so the
    Task-C AsyncPolicySession can share only the explicit GPU arbiter with the
    separately loaded ACT-B session.
    """

    def __init__(
        self,
        policy: Any,
        preprocessor: Any,
        postprocessor: Any,
        *,
        device: str,
        checkpoint: str | Path,
    ) -> None:
        import torch
        from lerobot.policies.act.configuration_act import ACTConfig

        config = policy.config
        if not isinstance(config, ACTConfig):
            raise TypeError(
                f"expected resident ACT policy, got {type(config).__name__}"
            )
        self._validate_config(config)
        requested_device = _canonical_torch_device(device, torch)
        parameter = next(policy.parameters(), None)
        parameter_device = (
            None
            if parameter is None
            else _canonical_torch_device(parameter.device, torch)
        )
        if parameter_device is not None and parameter_device != requested_device:
            raise ValueError(
                "resident ACT-A device differs from shadow session device: "
                f"{parameter_device} != {requested_device}"
            )
        self.checkpoint = Path(checkpoint).expanduser().resolve()
        self.device = requested_device
        self.config = config
        self.policy = policy
        self.preprocessor = preprocessor
        self.postprocessor = postprocessor
        self.input_keys = tuple(config.input_features)
        self.action_dim = int(config.action_feature.shape[0])
        self.action_steps = int(config.n_action_steps)


def load_recorded_observation(
    dataset_root: str | Path,
    *,
    episode: int,
    frame: int,
    repo_id: str,
    video_backend: str = "pyav",
) -> RecordedObservation:
    """Load an exact dataset frame without relying on an assumed FPS/index."""

    from lerobot.datasets import LeRobotDataset

    root = Path(dataset_root).expanduser().resolve()
    dataset = LeRobotDataset(
        repo_id=repo_id,
        root=root,
        episodes=[int(episode)],
        video_backend=video_backend,
    )
    matching_metadata = [
        value
        for value in dataset.meta.episodes
        if int(value["episode_index"]) == int(episode)
    ]
    if not matching_metadata:
        raise KeyError(f"episode {episode} is absent from {root}")
    episode_metadata = matching_metadata[0]
    length = int(episode_metadata["length"])
    if frame < 0 or frame >= length:
        raise IndexError(
            f"frame {frame} is outside episode {episode} length {length}"
        )

    # When episodes= is supplied, the local dataset index starts at zero;
    # metadata retains the original dataset-global interval for provenance.
    item = dataset[int(frame)]
    actual_episode = int(item["episode_index"].item())
    actual_frame = int(item["frame_index"].item())
    if (actual_episode, actual_frame) != (int(episode), int(frame)):
        raise RuntimeError(
            "dataset episode/frame mapping mismatch: "
            f"requested {(episode, frame)}, got {(actual_episode, actual_frame)}"
        )

    state = item["observation.state"].detach().cpu().numpy()
    if state.shape != (EXPECTED_STATE_DIM,):
        raise ValueError(f"recorded state shape is {state.shape}, expected (13,)")
    policy_input = {
        key: value.detach().clone()
        for key, value in item.items()
        if key == "observation.state" or key.startswith("observation.images.")
    }
    global_index = int(episode_metadata["dataset_from_index"]) + int(frame)
    return RecordedObservation(
        policy_input=policy_input,
        dataset_root=str(root),
        episode=int(episode),
        frame=int(frame),
        global_index=global_index,
        timestamp_s=float(item["timestamp"].item()),
        tcp_position_mm=state[6:9],
        gripper_closed=bool(round(float(state[12]))),
    )
