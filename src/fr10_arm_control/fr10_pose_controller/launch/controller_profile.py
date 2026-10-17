"""Resolve reusable parameter overlays before launch, never in the control tick."""
from pathlib import Path

import yaml


def load_controller_profile(path, _parents=()):
    path = Path(path).resolve()
    if path in _parents:
        raise ValueError('Cyclic controller profile inheritance')
    document = yaml.safe_load(path.read_text())
    if not isinstance(document, dict):
        raise ValueError('Controller profile must be a mapping')
    node = document.get('fr10_pose_controller', {})
    parameters = node.get('ros__parameters', {})
    if not isinstance(parameters, dict):
        raise ValueError('Controller ros__parameters must be a mapping')
    base = document.get('extends')
    resolved = load_controller_profile(path.parent / base, (*_parents, path)) if base else {}
    resolved.update(parameters)
    return resolved
