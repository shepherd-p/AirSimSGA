"""Release-only environment resolution; never starts or changes a simulator."""
import os
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]


def sdk_path():
    value = os.environ.get('AIRSIMSGA_SDK_PATH')
    if not value:
        raise ValueError('Set AIRSIMSGA_SDK_PATH to backend/client/python/projectairsim/src')
    path = Path(value).expanduser().resolve()
    if not (path / 'projectairsim').is_dir():
        raise FileNotFoundError(f'SDK source directory missing: {path}')
    return path


def zonegraph_path():
    value = os.environ.get('AIRSIMSGA_ZONEGRAPH')
    if value:
        return Path(value).expanduser().resolve()
    project = os.environ.get('AIRSIMSGA_CITYSAMPLE_PROJECT')
    if not project:
        raise ValueError('Set AIRSIMSGA_CITYSAMPLE_PROJECT or AIRSIMSGA_ZONEGRAPH; export using AG.ZoneGraph.Export')
    return Path(project).expanduser().resolve() / 'Saved/AGCitySampleZoneGraph.json'


def output_root():
    return Path(os.environ.get('AIRSIMSGA_OUTPUT_ROOT', str(ROOT/'outputs'))).expanduser().resolve()


def apply_runtime(config):
    c = dict(config)
    for key, env in [('ip','AIRSIMSGA_HOST'), ('port_topics','AIRSIMSGA_PORT_TOPICS'), ('port_services','AIRSIMSGA_PORT_SERVICES')]:
        if os.environ.get(env):
            c[key] = int(os.environ[env]) if key.startswith('port') else os.environ[env]
    return c


def resolve_config(config):
    def resolve(value):
        if isinstance(value, dict):
            return {key:resolve(item) for key,item in value.items()}
        if isinstance(value, list):
            return [resolve(item) for item in value]
        if isinstance(value, str) and value.startswith('${') and value.endswith('}'):
            key = value[2:-1]
            if not os.environ.get(key):
                raise ValueError(f'Missing required setting: {key}')
            return os.environ[key]
        return value
    c = apply_runtime(resolve(config))
    c['sdk_path'] = str(sdk_path())
    for key in ['sim_config_path','calibration_file']:
        if c.get(key):
            path = Path(c[key]).expanduser()
            c[key] = str(path.resolve() if path.is_absolute() else (ROOT/path).resolve())
    return c
