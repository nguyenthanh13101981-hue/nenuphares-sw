"""Configuration and CdCT specification loader (no hardcoded local home paths)."""

import os
from pathlib import Path
from typing import Any, Dict

import yaml


def find_repo_root() -> Path:
    """Locate repository root containing config/raft_profiles without hardcoded home paths."""
    env_dir = os.environ.get('NENUPHAR_CONFIG_DIR')
    if env_dir:
        candidate = Path(env_dir).resolve()
        if candidate.is_dir():
            return candidate.parent

    search_roots = [Path(__file__).resolve(), Path.cwd().resolve()]
    for root in search_roots:
        for parent in (root, *root.parents):
            if (parent / 'config' / 'raft_profiles').is_dir():
                return parent
    raise FileNotFoundError('Could not locate repository root with config/raft_profiles')


def find_config_dir() -> Path:
    """Resolve the repository config directory without hardcoding hull geometry."""
    env_dir = os.environ.get('NENUPHAR_CONFIG_DIR')
    if env_dir:
        candidate = Path(env_dir)
        if candidate.is_dir():
            return candidate

    return find_repo_root() / 'config'


def _load_yaml(path: Path) -> Dict[str, Any]:
    with path.open('r', encoding='utf-8') as f:
        data = yaml.safe_load(f)
    if not isinstance(data, dict):
        raise ValueError(f'Expected mapping in YAML file: {path}')
    return data


def load_raft_profile(profile_name: str = 'v1_70x50') -> Dict[str, Any]:
    """Load raft geometry profile (e.g. v1_70x50 or v1_98x68) per TBD-06."""
    cfg_dir = find_config_dir()
    return _load_yaml(cfg_dir / 'raft_profiles' / f'{profile_name}.yaml')


def load_energy_config() -> Dict[str, Any]:
    """Load A.9 energy thresholds and shunt/BLE rules."""
    return _load_yaml(find_config_dir() / 'energy_A9.yaml')


def load_corridor_config() -> Dict[str, Any]:
    """Load TR-4.16 / D-16 corridor and S4/TR-4.1 bridge/obstacle thresholds."""
    return _load_yaml(find_config_dir() / 'corridor_D16.yaml')


def load_cdct_spec() -> Dict[str, Any]:
    """Load machine-readable CdCT Final 2.0 spec (docs/cdct_spec_a8_a10.yaml)."""
    return _load_yaml(find_repo_root() / 'docs' / 'cdct_spec_a8_a10.yaml')
