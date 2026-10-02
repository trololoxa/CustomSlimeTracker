"""Explicit serial selection, host-wide cooperative lock, bounded CLI frames."""
from contextlib import contextmanager
import hashlib
import os
from pathlib import Path
import time

from process_lock import exclusive_lock

MAX_CAPTURE_BYTES = 2 * 1024 * 1024
MAX_FRAME_BYTES = 64 * 1024


def ports():
    try:
        from serial.tools.list_ports import comports
    except ImportError as exc:
        raise ValueError('pyserial is required in this Python environment') from exc
    return [dict(port=p.device, vid=p.vid, pid=p.pid, serial=p.serial_number,
                 description=p.description) for p in comports()]


def select_port(inventory, port, serial_number, vid, pid):
    matches = [p for p in inventory if os.path.normcase(p['port']) == os.path.normcase(port)]
    if len(matches) != 1:
        raise ValueError('explicit port is absent or ambiguous; run list again')
    device = matches[0]
    if not serial_number or (device['serial'], device['vid'], device['pid']) != (serial_number, vid, pid):
        raise ValueError('USB serial/VID/PID mismatch; no port opened')
    # Prevent aliases/duplicate USB serial numbers from providing false exclusivity.
    if sum((p['serial'], p['vid'], p['pid']) == (serial_number, vid, pid) for p in inventory) != 1:
        raise ValueError('USB identity is not unique; unplug the ambiguous device')
    return device


@contextmanager
def device_lock(device, directory=None):
    if directory is None:
        base = os.environ.get('LOCALAPPDATA') if os.name == 'nt' else os.environ.get('XDG_CACHE_HOME')
        directory = (Path(base) if base else Path.home() / '.cache') / 'tracker-device-locks'
    key = f"{device['vid']}:{device['pid']}:{device['serial']}".encode('utf-8')
    manager = exclusive_lock(Path(directory) / (hashlib.sha256(key).hexdigest() + '.lock'))
    try:
        manager.__enter__()
    except OSError as exc:
        raise ValueError('device lock unavailable; close the other device workflow') from exc
    try:
        yield
    finally:
        manager.__exit__(None, None, None)


@contextmanager
def open_serial(device):
    import serial
    # Configure before open; drivers/boards can still pulse control lines on open.
    stream = serial.Serial(port=None, baudrate=921600, timeout=0.1, write_timeout=1)
    if os.name != 'nt':
        stream.exclusive = True
    stream.dtr = False
    stream.rts = False
    stream.port = device['port']
    try:
        stream.open()
        yield stream
    finally:
        stream.close()


class Console:
    def __init__(self, stream, raw, clock=time.monotonic):
        self.stream, self.raw, self.clock = stream, raw, clock
        self.total = 0
        self.pending = b''
        self.faults = []
        self.smoke_active = False
        self.version_header_resyncs = 0

    def read(self):
        data = self.stream.read(1024)
        self.total += len(data)
        if self.total > MAX_CAPTURE_BYTES:
            raise ValueError('serial capture byte budget exceeded')
        self.raw.write(data)
        self.raw.flush()
        self.pending += data
        if len(self.pending) > MAX_FRAME_BYTES:
            raise ValueError('unterminated/oversized console line')
        lines = self.pending.split(b'\n')
        self.pending = lines.pop()
        decoded = [line.decode('utf-8', errors='replace').strip() for line in lines]
        for line in decoded:
            if ('Guru Meditation' in line or 'panic' in line.lower() or 'safe_mode=yes' in line
                    or (self.smoke_active and (line.startswith(('ESP-ROM:', 'rst:')) or '# boot_health ' in line))):
                self.faults.append(line[:300])
        return decoded

    def drain(self, duration=0.5):
        end = self.clock() + duration
        while self.clock() < end:
            self.read()
        self.pending = b''

    def request(self, command, header, end_key, timeout=3.0, *, initial_sync=False):
        # Callers only send read-only commands. Fixed framing rejects partial output.
        if command not in ('version', 'health'):
            raise ValueError('command is not read-only smoke vocabulary')
        if initial_sync and (command != 'version' or self.smoke_active):
            raise ValueError('header resync is only allowed for initial version handshake')
        if self.stream.write((command + '\n').encode('ascii')) != len(command) + 1:
            raise ValueError('short command write')
        end = self.clock() + timeout
        fields, seen, size = {}, False, 0
        while self.clock() < end:
            for line in self.read():
                size += len(line)
                if size > MAX_FRAME_BYTES:
                    raise ValueError('console response exceeds byte budget')
                # USB attachment may start inside an unterminated boot banner.
                # Only the first version handshake can establish a new boundary;
                # later frames remain strict, and raw bytes/faults are retained.
                if line == header or (initial_sync and line.endswith(header)):
                    if seen or line.count(header) != 1:
                        raise ValueError('duplicate response header')
                    if line != header:
                        self.version_header_resyncs += 1
                    seen = True
                elif seen and '=' in line and not line.startswith('#'):
                    key, value = line.split('=', 1)
                    if key in fields:
                        raise ValueError(f'duplicate response field: {key}')
                    fields[key] = value
                    if key == end_key:
                        return fields
        raise TimeoutError(f'incomplete {command} response within {timeout:g}s')
