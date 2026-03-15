import argparse
import math
from pathlib import Path
from typing import Any, Iterable

import yaml


def _collect_yaml_files(workspace: Path) -> list[Path]:
    candidates: list[Path] = []
    roots = [workspace / 'src', workspace / 'routes', workspace / 'maps']

    for root in roots:
        if not root.exists():
            continue
        for path in root.rglob('*.yaml'):
            if path.is_file():
                candidates.append(path)
        for path in root.rglob('*.yml'):
            if path.is_file():
                candidates.append(path)

    return sorted(set(candidates))


def _is_finite_number(value: Any) -> bool:
    if isinstance(value, bool):
        return False
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def _validate_waypoint_schema(data: Any, path: Path) -> tuple[list[str], list[str]]:
    errors: list[str] = []
    warnings: list[str] = []

    waypoints: Any = None

    # Supported format A:
    # frame_id: map
    # waypoints: [{x, y, yaw...}, ...]
    if isinstance(data, dict) and 'waypoints' in data:
        waypoints = data.get('waypoints')

    # Supported format B:
    # - {x, y, yaw...}
    # - {x, y, yaw...}
    elif isinstance(data, list):
        waypoints = data

    if waypoints is None:
        return errors, warnings

    if not isinstance(waypoints, list):
        errors.append(f'{path}: key "waypoints" must be a list')
        return errors, warnings

    if not waypoints:
        warnings.append(f'{path}: waypoint list is empty (template or unfinished route)')
        return errors, warnings

    for idx, wp in enumerate(waypoints):
        if not isinstance(wp, dict):
            errors.append(f'{path}: waypoint[{idx}] must be a map/dict')
            continue

        if 'x' not in wp or 'y' not in wp:
            errors.append(f'{path}: waypoint[{idx}] must include x and y')
            continue

        if not _is_finite_number(wp.get('x')) or not _is_finite_number(wp.get('y')):
            errors.append(f'{path}: waypoint[{idx}] has non-finite x/y')

        has_yaw = 'yaw' in wp
        has_yaw_deg = 'yaw_deg' in wp
        if has_yaw and has_yaw_deg:
            errors.append(f'{path}: waypoint[{idx}] should use yaw or yaw_deg, not both')
        if has_yaw and not _is_finite_number(wp.get('yaw')):
            errors.append(f'{path}: waypoint[{idx}] has invalid yaw')
        if has_yaw_deg and not _is_finite_number(wp.get('yaw_deg')):
            errors.append(f'{path}: waypoint[{idx}] has invalid yaw_deg')

    return errors, warnings


def _validate_yaml_file(path: Path) -> tuple[list[str], list[str]]:
    errors: list[str] = []
    warnings: list[str] = []

    try:
        content = path.read_text(encoding='utf-8')
    except OSError as exc:
        errors.append(f'{path}: cannot read file: {exc}')
        return errors, warnings

    try:
        data = yaml.safe_load(content)
    except yaml.YAMLError as exc:
        errors.append(f'{path}: YAML parse error: {exc}')
        return errors, warnings

    if data is None:
        warnings.append(f'{path}: empty YAML document')
        return errors, warnings

    waypoint_errors, waypoint_warnings = _validate_waypoint_schema(data, path)
    errors.extend(waypoint_errors)
    warnings.extend(waypoint_warnings)

    if isinstance(data, dict) and 'frame_id' in data and not isinstance(data.get('frame_id'), str):
        errors.append(f'{path}: frame_id must be a string')

    return errors, warnings


def _print_messages(title: str, items: Iterable[str]) -> None:
    items = list(items)
    if not items:
        return
    print(title)
    for msg in items:
        print(f'  - {msg}')


def _warning_path(warning: str) -> Path | None:
    # Warning format is expected as: "<path>: <message>"
    if ': ' not in warning:
        return None
    raw_path = warning.split(': ', 1)[0]
    try:
        return Path(raw_path)
    except OSError:
        return None


def _apply_strict_profile(
    workspace: Path,
    warnings: list[str],
    profile: str,
    route_globs: list[str],
) -> tuple[list[str], list[str]]:
    if profile == 'none':
        return [], warnings

    promoted_errors: list[str] = []
    remaining_warnings: list[str] = []

    for warning in warnings:
        path = _warning_path(warning)
        promote = False

        if (
            profile == 'routes-nonempty'
            and 'waypoint list is empty' in warning
            and path is not None
        ):
            try:
                rel = path.resolve().relative_to(workspace)
                rel_text = rel.as_posix()
                if any(rel.match(pattern) for pattern in route_globs):
                    promote = True
            except ValueError:
                promote = False

        if promote:
            promoted_errors.append(f'{warning} [strict-profile={profile}]')
        else:
            remaining_warnings.append(warning)

    return promoted_errors, remaining_warnings


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description='Validate YAML config files for the AGV workspace.',
    )
    parser.add_argument(
        '--workspace',
        type=Path,
        default=Path.cwd(),
        help='Workspace root path (default: current directory).',
    )
    parser.add_argument(
        '--strict',
        action='store_true',
        help='Treat warnings as errors.',
    )
    parser.add_argument(
        '--strict-profile',
        choices=['none', 'routes-nonempty'],
        default='none',
        help='Apply predefined strict checks. routes-nonempty fails when selected route files have empty waypoints.',
    )
    parser.add_argument(
        '--strict-route-glob',
        action='append',
        default=['routes/*.yaml', 'routes/**/*.yaml'],
        help='Glob (relative to workspace) for route files affected by strict profile. Repeatable.',
    )
    args = parser.parse_args(argv)

    workspace = args.workspace.resolve()
    yaml_files = _collect_yaml_files(workspace)

    if not yaml_files:
        print(f'No YAML files found under {workspace}')
        return 1

    all_errors: list[str] = []
    all_warnings: list[str] = []

    for path in yaml_files:
        errors, warnings = _validate_yaml_file(path)
        all_errors.extend(errors)
        all_warnings.extend(warnings)

    promoted_errors, all_warnings = _apply_strict_profile(
        workspace,
        all_warnings,
        args.strict_profile,
        args.strict_route_glob,
    )
    all_errors.extend(promoted_errors)

    print(f'Workspace: {workspace}')
    print(f'YAML files checked: {len(yaml_files)}')

    _print_messages('Errors:', all_errors)
    _print_messages('Warnings:', all_warnings)

    if all_errors:
        print('Validation status: FAILED')
        return 2
    if args.strict and all_warnings:
        print('Validation status: FAILED (strict mode)')
        return 3

    print('Validation status: PASSED')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
