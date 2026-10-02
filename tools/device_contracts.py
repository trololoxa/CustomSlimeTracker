"""Host device evidence contracts. No firmware thresholds/configuration are changed."""
from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
import re
import struct

from release_manifest import SCHEMA, sha256_file

ENVIRONMENTS = {'BOARD_LOLIN_C3_MINI_' + name for name in
                ('PRODUCTION', 'PRODUCTION_DIAG', 'USB_DIAG', 'SLIM', 'DEBUG')}
IDENTITY = re.compile(r'[0-9a-f]{40}(?:\+[0-9a-f]{8}-dirty)?')
COUNTERS = ('runtime_samples', 'prepared_output_seq', 'fifo_runtime_raw_queue_overflow',
            'fifo_runtime_mag_queue_overflow', 'fifo_runtime_batch_capacity_failures',
            'fifo_runtime_drain_failures', 'sensor_recovery_request_count',
            'sensor_recovery_failure_count', 'sensor_progress_fault_count',
            'mag_runtime_samples', 'mag_trusted_samples')
STRICT_COUNTERS = COUNTERS[2:5]
TRANSIENT_COUNTERS = COUNTERS[5:9]


def uint(fields, key):
    value = fields.get(key, '')
    if not re.fullmatch(r'\d+', value) or not 0 <= int(value) <= 0xffffffff:
        raise ValueError(f'missing/invalid uint32: {key}')
    return int(value)


def delta(before, after):
    value = (after - before) & 0xffffffff
    if value >= 0x80000000:
        raise ValueError('counter decreased/reset; observation cannot span reboot')
    return value


def check_version(fields, identity, environment):
    expected = dict(serial_protocol='serial-cli-v1', build_git=identity,
                    build_pio_env=environment)
    for key, value in expected.items():
        if fields.get(key) != value:
            raise ValueError(f'device {key} mismatch: expected {value}, got {fields.get(key)}')


@dataclass(frozen=True)
class SmokeBudget:
    max_transients: int = 2  # per counter: a single event can increment several counters
    max_blackout_s: float = 4.0
    max_pose_age_us: int = 100000
    require_mag: bool = False


class SmokeEvidence:
    def __init__(self, budget=SmokeBudget()):
        self.budget = budget
        self.frames = []
        self.blackout_since = None
        self.degraded_states = set()

    def add(self, fields, at):
        if not math.isfinite(at) or (self.frames and at <= self.frames[-1]['at_s']):
            raise ValueError('nonmonotonic host observation time')
        counters = {key: uint(fields, key) for key in COUNTERS}
        uptime = uint(fields, 'uptime_ms')
        try:
            q = [float(x) for x in fields['quat'].split(',')]
        except (KeyError, ValueError) as exc:
            raise ValueError('missing/invalid quaternion') from exc
        if len(q) != 4 or not all(math.isfinite(x) for x in q) or not 0.99 <= sum(x*x for x in q) <= 1.01:
            raise ValueError('invalid/nonunit quaternion')
        state = fields.get('tracking_state')
        allowed = {'TRACKING_6DOF', 'TRACKING_6DOF_MAG_YAW', 'DEGRADED_MAG',
                   'DEGRADED_ACCEL', 'DEGRADED_TIMING', 'RECOVERING', 'STARTUP_CONVERGENCE'}
        if state not in allowed or fields.get('sensor_recovery_exhausted') != 'no':
            raise ValueError('fatal/unsupported tracking state or exhausted recovery')
        for key in ('prepared_output_valid', 'tracking_recovery_active'):
            if fields.get(key) not in ('yes', 'no'):
                raise ValueError(f'missing/invalid {key}')
        if 'sensor_recovery_state' not in fields or 'sensor_progress_current_fault' not in fields:
            raise ValueError('incomplete recovery evidence')
        fresh = fields['prepared_output_valid'] == 'yes'
        if fresh:
            fresh = uint(fields, 'prepared_output_publish_age_us') <= self.budget.max_pose_age_us
        healthy = (fresh and fields['tracking_recovery_active'] == 'no'
                   and fields['sensor_recovery_state'] == 'idle'
                   and fields['sensor_progress_current_fault'] == 'none'
                   and fields.get('sensor_progress_suppress_mask') == '0x0'
                   and state not in {'RECOVERING', 'STARTUP_CONVERGENCE', 'DEGRADED_TIMING'})
        if self.frames:
            previous, first = self.frames[-1], self.frames[0]
            if not delta(previous['uptime_ms'], uptime):
                raise ValueError('uptime did not progress')
            # All counters must remain monotonic even when their budget is permissive.
            changes = {key: delta(previous['counters'][key], counters[key]) for key in COUNTERS}
            healthy = healthy and changes['runtime_samples'] > 0 and changes['prepared_output_seq'] > 0
            for key in STRICT_COUNTERS + TRANSIENT_COUNTERS:
                limit = 0 if key in STRICT_COUNTERS else self.budget.max_transients
                if delta(first['counters'][key], counters[key]) > limit:
                    raise ValueError(f'{key} exceeds observation delta budget {limit}')
        if not healthy and self.blackout_since is None:
            self.blackout_since = self.frames[-1]['at_s'] if self.frames else at
        if self.blackout_since is not None and at - self.blackout_since > self.budget.max_blackout_s:
            raise ValueError('sensor/pose blackout exceeds budget')
        if healthy:
            self.blackout_since = None
        if state.startswith('DEGRADED'):
            self.degraded_states.add(state)
        self.frames.append(dict(at_s=at, uptime_ms=uptime, counters=counters,
                                healthy=healthy, tracking_state=state))

    def finish(self):
        if len(self.frames) < 3 or not all(f['healthy'] for f in self.frames[-2:]):
            raise ValueError('need at least three observations and two final healthy frames')
        first, last = self.frames[0], self.frames[-1]
        changes = {key: delta(first['counters'][key], last['counters'][key]) for key in COUNTERS}
        if self.budget.require_mag and not (changes['mag_runtime_samples'] and changes['mag_trusted_samples']):
            raise ValueError('requested magnetic stream/trust did not progress')
        return dict(scope='sampled serial sensor/pose smoke; not accuracy, WCET or Server proof',
                    observations=len(self.frames), deltas=changes,
                    degraded_states=sorted(self.degraded_states), budget=vars(self.budget))


def partition_layout(data):
    if len(data) not in (0xc00, 0x1000):
        raise ValueError('unexpected partition binary size')
    expected = [(1, 2, 0x9000, 0x5000, b'nvs', 0),
                (0, 0, 0x10000, 0x300000, b'app0', 0),
                (1, 0x82, 0x310000, 0xf0000, b'spiffs', 0)]
    for index, entry in enumerate(expected):
        magic, kind, subtype, offset, size, name, flags = struct.unpack_from('<HBBII16sI', data, index*32)
        if magic != 0x50aa or (kind, subtype, offset, size, name.rstrip(b'\0'), flags) != entry:
            raise ValueError('partition layout differs from reviewed no-OTA/NVS contract')
    tail = data[96:]
    if tail.startswith(b'\xeb\xeb' + b'\xff'*14):
        if tail[16:32] != hashlib.md5(data[:96]).digest():
            raise ValueError('partition MD5 mismatch')
        tail = tail[32:]
    if tail != b'\xff' * len(tail):
        raise ValueError('extra partition/invalid partition padding')


def load_bundle(root: Path, path: Path):
    if path.stat().st_size > 1024*1024:
        raise ValueError('oversized build manifest')
    manifest = json.loads(path.read_text(encoding='utf-8'))
    if not isinstance(manifest, dict) or manifest.get('schema') != SCHEMA:
        raise ValueError('expected release_manifest.py schema')
    if not isinstance(manifest.get('source'), dict) or not isinstance(manifest.get('build'), dict):
        raise ValueError('manifest source/build must be objects')
    identity = manifest['source'].get('identity', '')
    environment = manifest.get('build', {}).get('environment', '')
    if not isinstance(identity, str) or not isinstance(environment, str) or not IDENTITY.fullmatch(identity) or environment not in ENVIRONMENTS:
        raise ValueError('unknown source identity or non-uploadable environment')
    items = manifest.get('artifacts')
    if not isinstance(items, list) or not items:
        raise ValueError('manifest artifacts must be a nonempty list')
    files = {}
    for item in items:
        if (not isinstance(item, dict) or not isinstance(item.get('path'), str)
                or type(item.get('size_bytes')) is not int or not 0 < item['size_bytes'] <= 32*1024*1024
                or not isinstance(item.get('sha256'), str) or not re.fullmatch(r'[0-9a-f]{64}', item['sha256'])):
            raise ValueError('malformed artifact entry')
        file = (root / item['path']).resolve()
        if not file.is_relative_to(root.resolve()) or file.name in files:
            raise ValueError('artifact escapes project or duplicates a basename')
        if file.stat().st_size != item['size_bytes'] or file.stat().st_size > 32*1024*1024 or sha256_file(file) != item['sha256']:
            raise ValueError(f'artifact hash/size mismatch: {file.name}')
        files[file.name] = file
    if not {'firmware.bin', 'firmware.elf', 'partitions.bin'} <= files.keys():
        raise ValueError('manifest needs firmware.bin, firmware.elf and partitions.bin')
    app = files['firmware.bin'].read_bytes()
    if not 24 <= len(app) <= 0x300000 or app[0] != 0xe9 or app[12:14] != b'\x05\x00':
        raise ValueError('not an ESP32-C3 app image fitting factory slot')
    if identity.encode() + b'\0' not in app or environment.encode() + b'\0' not in app:
        raise ValueError('app embedded source/profile differs from manifest')
    elf = files['firmware.elf'].read_bytes()
    if (len(elf) < 20 or elf[:6] != b'\x7fELF\x01\x01' or elf[18:20] != b'\xf3\x00'
            or identity.encode() + b'\0' not in elf or environment.encode() + b'\0' not in elf):
        raise ValueError('ELF is not a matching-identity RISC-V artifact')
    partition_layout(files['partitions.bin'].read_bytes())
    return manifest, files
