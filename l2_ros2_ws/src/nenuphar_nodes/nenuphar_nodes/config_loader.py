"""Configuration loader for raft geometry, energy, and corridor YAML files."""

import os
from pathlib import Path
from typing import Any, Dict

import yaml


def find_config_dir() -> Path:
    """Resolve the repository config directory without hardcoding hull geometry."""
    env_dir = os.environ.get('NENUPHAR_CONFIG_DIR')
    if env_dir:
        candidate = Path(env_dir)
        if candidate.is_dir():
            return candidate

    search_roots = [Path(__file__).resolve(), Path.cwd().resolve()]
    for root in search_roots:
        for parent in (root, *root.parents):
            candidate = parent / 'config'
            if (candidate / 'raft_profiles').is_dir():
                return candidate

    fallback = Path('/home/nenuphar/nenuphares-sw/config')
    if fallback.is_dir():
        return fallback
    raise FileNotFoundError('Could not locate nenuphares-sw/config directory')


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
    """Load D-16 corridor and S4/TR-4.1 bridge/obstacle thresholds."""
    return _load_yaml(find_config_dir() / 'corridor_D16.yaml')
