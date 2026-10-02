#!/usr/bin/env python3
"""Explicit tracker device operations; list/flash-plan never open a device."""
from __future__ import annotations

import argparse
import importlib.util
from dataclasses import asdict
import json
import os
from pathlib import Path
import re
import shutil
import sys
import time

from device_contracts import (ENVIRONMENTS, IDENTITY, SmokeBudget, SmokeEvidence,
                              check_version, load_bundle)
from device_transport import Console, device_lock, open_serial, ports, select_port
from gate_reporting import GateReport
from release_manifest import sha256_file

ROOT = Path(__file__).resolve().parents[1]
ESPTOOL_VERSION = '4.9.0'
APP_READBACK_TIMEOUT_S = 600


def selected(args):
    return select_port(ports(), args.port, args.usb_serial, args.vid, args.pid)


def identity_args(parser):
    parser.add_argument('--port', required=True)
    parser.add_argument('--usb-serial', required=True)
    parser.add_argument('--vid', type=lambda x: int(x, 0), required=True)
    parser.add_argument('--pid', type=lambda x: int(x, 0), required=True)


def bounded_number(kind, low, high):
    def parse(value):
        number = kind(value)
        if not low <= number <= high:
            raise argparse.ArgumentTypeError(f'expected {low}..{high}')
        return number
    return parse


def parser():
    cli = argparse.ArgumentParser(description=__doc__)
    commands = cli.add_subparsers(dest='operation', required=True)
    commands.add_parser('list', help='USB enumeration only, no port open')
    for name in ('flash-plan', 'flash'):
        command = commands.add_parser(name, help='app-only update; preserves partition/NVS bytes')
        command.add_argument('--manifest', type=Path, required=True)
        if name == 'flash':
            identity_args(command)
            command.add_argument('--esptool', type=Path, help='optional script; default: this Python -m esptool (4.9.0)')
            command.add_argument('--execute', action='store_true', help='authorize reset/write of selected tracker')
    smoke = commands.add_parser('smoke', help='read-only CLI commands; serial open may reset some boards')
    identity_args(smoke)
    smoke.add_argument('--expect-build', required=True)
    smoke.add_argument('--environment', choices=sorted(ENVIRONMENTS), required=True)
    smoke.add_argument('--samples', type=bounded_number(int, 3, 30), default=6)
    smoke.add_argument('--require-mag', action='store_true', help='also require magnetic stream/trusted-sample progress')
    listen = commands.add_parser('listen', help='passive bounded boot/crash evidence; no commands or reboot')
    identity_args(listen)
    listen.add_argument('--seconds', type=bounded_number(float, 1, 60), default=15)
    symbol = commands.add_parser('symbolize', help='offline addresses using a manifest-matched ELF')
    symbol.add_argument('--manifest', type=Path, required=True)
    symbol.add_argument('--crash-log', type=Path, required=True)
    symbol.add_argument('--crash-build', required=True, help='exact identity recorded for the crashed firmware')
    symbol.add_argument('--addr2line', type=Path, required=True)
    return cli


def snapshot_bundle(manifest_path, report):
    manifest, sources = load_bundle(ROOT, manifest_path)
    destination = report.directory / 'artifacts'
    destination.mkdir()
    expected = {Path(item['path']).name: item['sha256'] for item in manifest['artifacts']}
    files = {}
    for name in ('firmware.bin', 'firmware.elf', 'partitions.bin'):
        target = destination / name
        shutil.copyfile(sources[name], target)
        if sha256_file(target) != expected[name]:
            raise ValueError('artifact changed during snapshot; no device opened')
        files[name] = target
    report.data['bundle'] = manifest
    report.save()
    return manifest, files


def smoke_run(args, report, console, clock=time.monotonic, sleep=time.sleep):
    if not IDENTITY.fullmatch(args.expect_build):
        raise ValueError('--expect-build must be a known exact build identity')
    console.drain()
    version = console.request('version', '# TRACKER FIRMWARE VERSION', 'command_session', initial_sync=True)
    if console.faults:
        raise ValueError('panic/safe-mode observed during initial handshake: ' + console.faults[-1])
    check_version(version, args.expect_build, args.environment)
    console.smoke_active = True
    budget = SmokeBudget(require_mag=args.require_mag)
    evidence = SmokeEvidence(budget)
    report.data.update(device_version=version, smoke_budget=asdict(budget),
                       initial_version_header_resyncs=console.version_header_resyncs)
    report.save()
    for index in range(args.samples):
        if index:
            sleep(1)
        fields = console.request('health', '# HEALTH', 'mean_dt_us')
        if console.faults:
            raise ValueError('boot/panic/safe-mode observed: ' + console.faults[-1])
        evidence.add(fields, clock())
        report.data['observations'] = evidence.frames
        report.save()
        if index == 0 or (index + 1) % 5 == 0 or index + 1 == args.samples:
            print(f"# SMOKE {index+1}/{args.samples}: {fields['tracking_state']}", flush=True)
    # Detect an intervening reflash as well as counter resets.
    check_version(console.request('version', '# TRACKER FIRMWARE VERSION', 'command_session'),
                  args.expect_build, args.environment)
    if console.faults:
        raise ValueError('boot/panic/safe-mode observed: ' + console.faults[-1])
    report.data['result'] = evidence.finish()
    report.save()


def checked_command(report, command, timeout_s=180):
    result = report.run(command, cwd=ROOT, timeout_s=timeout_s, env=os.environ.copy())
    if result.returncode:
        raise ValueError(f'device command failed (exit={result.returncode}); no automatic retry')


def flash_run(args, report, device, files):
    if args.esptool is None:
        spec = importlib.util.find_spec('esptool')
        if spec is None or not spec.origin:
            raise ValueError('esptool 4.9.0 is required in this Python; installation is a separate operation')
        tool = Path(spec.origin).resolve()
        executable = [sys.executable, '-m', 'esptool']
    else:
        tool = args.esptool.resolve()
        executable = [sys.executable, str(tool)]
    if not tool.is_file():
        raise ValueError('esptool entry file not found; installation is a separate operation')
    # Version validation is host-only and precedes reset/read/write.
    checked_command(report, [*executable, 'version'])
    version_log = report.directory / report.data['commands'][-1]['log']
    versions = set(re.findall(r'(?m)^(?:esptool(?:\.py)? v)?(\d+\.\d+(?:\.\d+)?)[ \t]*$',
                              version_log.read_text(encoding='utf-8', errors='replace')))
    if versions != {ESPTOOL_VERSION}:
        raise ValueError('controlled flash requires verified esptool 4.9.0; no device command issued')
    report.data['esptool_version'] = ESPTOOL_VERSION
    native_usb = (device['vid'], device['pid']) == (0x303a, 0x1001)
    before = 'usb_reset' if native_usb else 'default_reset'
    # esptool v4 skips CHANGE_BAUDRATE at its initial 115200 baud. The native
    # USB endpoint failed with the old tool; keep the verified 4.9.0 transport.
    baud = '115200' if native_usb else '460800'
    report.data['upload_baud'] = int(baud)
    report.data['reset_policy'] = dict(before=before, intermediate_after='no_reset', final_after='hard_reset')
    report.data['esptool_sha256'] = sha256_file(tool)
    report.save()

    def command(after, *parts, timeout_s=180):
        if selected(args) != device:
            raise ValueError('device enumeration changed; rerun list; no automatic rebind')
        checked_command(report, [*executable, '--chip', 'esp32c3',
            '--port', device['port'], '--baud', baud, '--before', before, '--after', after, *parts],
            timeout_s=timeout_s)

    partitions = report.directory / 'device-partitions.bin'
    command('no_reset', 'read_flash', '0x8000', '0x1000', str(partitions))
    expected = files['partitions.bin'].read_bytes().ljust(4096, b'\xff')
    if partitions.read_bytes() != expected:
        raise ValueError('device partition table mismatch; app NOT written; board may remain in ROM loader')
    app = files['firmware.bin']
    command('no_reset', 'write_flash', '0x10000', str(app))
    readback = report.directory / 'device-app-readback.bin'
    command('hard_reset', 'read_flash', '0x10000', str(app.stat().st_size), str(readback),
            timeout_s=APP_READBACK_TIMEOUT_S)
    if sha256_file(readback) != sha256_file(app):
        raise ValueError('app read-back mismatch; inspect evidence before any retry')
    report.data['result'] = dict(scope='app write/read-back only; boot and smoke NOT verified',
                                app_sha256=sha256_file(app), nvs_write_requested=False,
                                bootloader_write_requested=False)
    report.save()


def symbolize(args, report):
    manifest, files = snapshot_bundle(args.manifest, report)
    if args.crash_build != manifest['source']['identity']:
        raise ValueError('crash build identity differs from manifest; do not use current unrelated ELF')
    if args.crash_log.stat().st_size > 2*1024*1024:
        raise ValueError('crash log exceeds 2 MiB limit')
    raw = args.crash_log.read_bytes()
    (report.directory / 'crash.log').write_bytes(raw)
    addresses = list(dict.fromkeys(re.findall(rb'\b0x4[0-3][0-9a-fA-F]{6}\b', raw)))
    if not addresses or len(addresses) > 256:
        raise ValueError('need 1..256 unique ESP32-C3 code addresses')
    tool = args.addr2line.resolve()
    if not tool.is_file() or not tool.name.startswith('riscv32-esp-elf-addr2line'):
        raise ValueError('use the ESP32-C3 toolchain riscv32-esp-elf-addr2line executable')
    checked_command(report, [str(tool), '-a', '-f', '-C', '-i', '-e', str(files['firmware.elf']),
                             *(a.decode('ascii') for a in addresses)])
    report.data['result'] = dict(scope='offline symbolization; crash build identity supplied by caller, not attested by device',
                                crash_build=args.crash_build,
                                addresses=len(addresses), crash_sha256=sha256_file(report.directory / 'crash.log'))
    report.save()


def main(argv=None):
    args = parser().parse_args(argv)
    report = None
    try:
        if args.operation == 'list':
            print(json.dumps(ports(), ensure_ascii=False, indent=2))
            return 0
        if args.operation == 'flash-plan':
            manifest, files = load_bundle(ROOT, args.manifest)
            print(json.dumps(dict(environment=manifest['build']['environment'],
                identity=manifest['source']['identity'], app_sha256=sha256_file(files['firmware.bin']),
                steps=['match USB identity + lock', 'read/compare partition table',
                       'write app at 0x10000 only', 'read/compare app + reset'],
                opens_device=False, nvs_write_requested=False), indent=2))
            return 0
        if args.operation == 'smoke' and not IDENTITY.fullmatch(args.expect_build):
            raise ValueError('--expect-build must be a known exact build identity')
        if args.operation == 'flash' and not args.execute:
            raise ValueError('flash requires --execute; inspect flash-plan first')
        report = GateReport(ROOT, 'device')
        report.configure(args.operation, [], hardware=args.operation != 'symbolize')
        if args.operation == 'symbolize':
            symbolize(args, report)
        else:
            files = None
            if args.operation == 'flash':
                _, files = snapshot_bundle(args.manifest, report)
            device = selected(args)
            report.data['device'] = device
            report.save()
            with device_lock(device):
                if selected(args) != device:
                    raise ValueError('device identity changed after lock')
                if args.operation == 'flash':
                    flash_run(args, report, device, files)
                else:
                    with open_serial(device) as stream, (report.directory / 'serial.log').open('wb') as raw:
                        console = Console(stream, raw)
                        if args.operation == 'smoke':
                            smoke_run(args, report, console)
                        else:
                            console.drain(args.seconds)
                            report.data['result'] = dict(scope='passive capture only; no smoke/boot verdict', bytes=console.total, fault_lines=console.faults)
                        report.save()
        report.finish(0)
        print('OK device operation; see result.scope for verified coverage')
        return 0
    except (OSError, ValueError, TimeoutError, ImportError, KeyError, TypeError) as exc:
        print(f'FAIL device workflow: {exc}', file=sys.stderr)
        if report:
            report.note(str(exc))
            report.finish(1)
        return 1
    except KeyboardInterrupt:
        if report:
            report.finish(130, interrupted=True)
        return 130


if __name__ == '__main__':
    raise SystemExit(main())
