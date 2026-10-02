"""Device orchestration/fault contracts, fake USB/serial/flasher; no hardware used."""
import contextlib
import hashlib
import io
import json
from pathlib import Path
import struct
import sys
import types
import unittest
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'tools'))
import device_contracts as contracts
import device_transport as transport
import device_workflow as workflow
import validate_profile_matrix as profiles
from quality_gate_runtime import project_temp_directory
from release_manifest import sha256_file

ROOT = Path(__file__).resolve().parents[2]
IDENTITY = 'a' * 40
ENV = 'BOARD_LOLIN_C3_MINI_PRODUCTION_DIAG'
DEVICE = dict(port='COM13', serial='test-кириллица', vid=0x303a, pid=0x1001, description='test')


def health(sample=1, **changes):
    fields = {key: str(sample) for key in contracts.COUNTERS}
    for key in contracts.STRICT_COUNTERS + contracts.TRANSIENT_COUNTERS:
        fields[key] = '0'
    fields.update(uptime_ms=str(sample*1000), quat='1,0,0,0', tracking_state='DEGRADED_MAG',
                  sensor_recovery_exhausted='no', sensor_recovery_state='idle',
                  sensor_progress_current_fault='none', sensor_progress_suppress_mask='0x0',
                  prepared_output_valid='yes', prepared_output_publish_age_us='1000',
                  tracking_recovery_active='no')
    return fields | changes


def partitions():
    entries = [(1, 2, 0x9000, 0x5000, b'nvs'), (0, 0, 0x10000, 0x300000, b'app0'),
               (1, 0x82, 0x310000, 0xf0000, b'spiffs')]
    data = b''.join(struct.pack('<HBBII16sI', 0x50aa, *entry, 0) for entry in entries)
    return (data + b'\xeb\xeb' + b'\xff'*14 + hashlib.md5(data).digest()).ljust(0xc00, b'\xff')


class FakeSerial:
    def __init__(self, chunks):
        self.chunks = iter(chunks)
        self.writes = []
        self.t = 0

    def read(self, size):
        self.t += 0.1
        return next(self.chunks, b'')

    def write(self, data):
        self.writes.append(data)
        return len(data)


class EvidenceTests(unittest.TestCase):
    def test_progress_degraded_6d_and_single_transient_recovers(self):
        evidence = contracts.SmokeEvidence()
        for index in range(1, 6):
            changes = dict(fifo_runtime_drain_failures=str(int(index >= 2)))
            if index == 2:
                changes.update(prepared_output_valid='no', tracking_recovery_active='yes', tracking_state='RECOVERING')
            evidence.add(health(index, **changes), float(index))
        result = evidence.finish()
        self.assertEqual(result['deltas']['fifo_runtime_drain_failures'], 1)
        self.assertIn('DEGRADED_MAG', result['degraded_states'])

    def test_invalid_quat_and_fatal_or_incomplete_data_fail(self):
        cases = [dict(quat=q) for q in ('nan,0,0,0', 'inf,0,0,0', '0,0,0,0', '2,0,0,0', '1,0,0')]
        cases += [dict(tracking_state='SENSOR_FAULT'), dict(sensor_recovery_exhausted='yes'),
                  dict(prepared_output_valid='unknown'), dict(runtime_samples='-1')]
        for change in cases:
            with self.subTest(change=change), self.assertRaises(ValueError):
                contracts.SmokeEvidence().add(health(**change), 1)
        frame = health(); del frame['quat']
        with self.assertRaises(ValueError):
            contracts.SmokeEvidence().add(frame, 1)

    def test_budget_boundaries_reset_stale_and_terminal_recovery(self):
        for key in contracts.STRICT_COUNTERS + contracts.TRANSIENT_COUNTERS:
            evidence = contracts.SmokeEvidence()
            evidence.add(health(), 1)
            limit = 0 if key in contracts.STRICT_COUNTERS else 2
            with self.subTest(key=key), self.assertRaises(ValueError):
                evidence.add(health(2, **{key: str(limit+1)}), 2)
        evidence = contracts.SmokeEvidence()
        evidence.add(health(), 1)
        with self.assertRaises(ValueError):
            evidence.add(health(2, uptime_ms='1'), 2)
        for cause in ({'prepared_output_publish_age_us': '100001'}, {'runtime_samples': '1'},
                      {'prepared_output_seq': '1'}, {'sensor_progress_current_fault': 'ImuNoProgress'}):
            evidence = contracts.SmokeEvidence(); evidence.add(health(), 1)
            evidence.add(health(2, **cause), 2)
            evidence.add(health(3, **cause), 3)
            with self.subTest(cause=cause), self.assertRaises(ValueError):
                evidence.finish()
        evidence = contracts.SmokeEvidence(); evidence.add(health(), 1)
        evidence.add(health(2, prepared_output_valid='no'), 2)
        with self.assertRaises(ValueError):
            evidence.add(health(3), 6.01)  # recovery arriving after blackout budget is late
        self.assertEqual(contracts.delta(0xfffffffe, 2), 4)

    def test_required_mag_version_and_final_evidence(self):
        evidence = contracts.SmokeEvidence(contracts.SmokeBudget(require_mag=True))
        for i in range(1, 4):
            evidence.add(health(i, mag_trusted_samples='0'), i)
        with self.assertRaises(ValueError):
            evidence.finish()
        fields = dict(serial_protocol='serial-cli-v1', build_git=IDENTITY, build_pio_env=ENV)
        contracts.check_version(fields, IDENTITY, ENV)
        with self.assertRaises(ValueError):
            contracts.check_version(fields, 'b'*40, ENV)


class TransportTests(unittest.TestCase):
    def test_selection_never_guesses_and_duplicates_rejected(self):
        self.assertEqual(transport.select_port([DEVICE], 'COM13', DEVICE['serial'], 0x303a, 0x1001), DEVICE)
        for inventory, port, serial in (([], 'COM13', DEVICE['serial']), ([DEVICE], 'COM2', DEVICE['serial']),
                                       ([DEVICE], 'COM13', 'wrong'), ([DEVICE, DEVICE], 'COM13', DEVICE['serial'])):
            with self.assertRaises(ValueError):
                transport.select_port(inventory, port, serial, 0x303a, 0x1001)

    def test_lock_excludes_other_checkout_and_releases_on_exception(self):
        with project_temp_directory(ROOT, 'device-lock-') as directory:
            with self.assertRaisesRegex(RuntimeError, 'body'):
                with transport.device_lock(DEVICE, directory):
                    with self.assertRaises(ValueError):
                        with transport.device_lock(DEVICE, directory):
                            self.fail('overlap')
                    raise RuntimeError('body')
            with transport.device_lock(DEVICE, directory):
                pass

    def test_frames_chunking_timeout_duplicates_and_byte_bound(self):
        stream = FakeSerial([b'noise\n# HE', b'ALTH\nuptime_ms=1000\nmean_dt_us=1048\n'])
        raw = io.BytesIO(); console = transport.Console(stream, raw, lambda: stream.t)
        self.assertEqual(console.request('health', '# HEALTH', 'mean_dt_us')['uptime_ms'], '1000')
        self.assertEqual(stream.writes, [b'health\n'])
        self.assertIn(b'# HEALTH', raw.getvalue())
        for chunks in ([b'# HEALTH\nuptime_ms=1\n'], [b'# HEALTH\nx=1\nx=2\n'],
                       [b'banner# HEALTH\nmean_dt_us=1048\n'], [b'x'*65537]):
            stream = FakeSerial(chunks); console = transport.Console(stream, io.BytesIO(), lambda: stream.t)
            with self.assertRaises((ValueError, TimeoutError)):
                console.request('health', '# HEALTH', 'mean_dt_us', 0.5)
            self.assertLessEqual(stream.t, 0.6)
        with self.assertRaises(ValueError):
            console.request('reboot', '', '')
        stream.write = lambda _: 0
        with self.assertRaisesRegex(ValueError, 'short command write'):
            console.request('health', '# HEALTH', 'mean_dt_us')

    def test_initial_version_resync_preserves_strict_frames_and_faults(self):
        header = '# TRACKER FIRMWARE VERSION'
        for initial in (False, True):
            stream = FakeSerial([b'# boot_health reset=other safe_mode=no\n# build_profile=USB_',
                                 header.encode()+b'\nbuild_git=abc\ncommand_session=0\n'])
            raw = io.BytesIO(); console = transport.Console(stream, raw, lambda: stream.t)
            if initial:
                self.assertEqual(console.request('version', header, 'command_session', initial_sync=True)['build_git'], 'abc')
                self.assertEqual(console.version_header_resyncs, 1)
                self.assertIn(b'USB_# TRACKER', raw.getvalue())
            else:
                with self.assertRaises(TimeoutError):
                    console.request('version', header, 'command_session', timeout=0.5)
            console.smoke_active = True
            with self.assertRaises(ValueError):
                console.request('version', header, 'command_session', initial_sync=True)

    def test_passive_crash_capture_retains_following_lines(self):
        stream = FakeSerial([b'Guru Medi', b'tation Error\nMEPC: 0x42001234\n', b'end\n'])
        raw = io.BytesIO(); console = transport.Console(stream, raw, lambda: stream.t)
        console.drain(0.5)
        self.assertTrue(console.faults)
        self.assertIn(b'end\n', raw.getvalue())
        stream = FakeSerial([b'# boot_health reset=software safe_mode=no\n'])
        console = transport.Console(stream, io.BytesIO(), lambda: stream.t)
        console.smoke_active = True
        console.read()
        self.assertTrue(console.faults)


class BundleWorkflowTests(unittest.TestCase):
    def setUp(self):
        temp = project_temp_directory(ROOT, 'device-bundle-')
        self.addCleanup(temp.cleanup); self.root = Path(temp.name)
        self.files = {name: self.root/name for name in ('firmware.bin', 'firmware.elf', 'partitions.bin')}
        app = bytearray(24); app[0] = 0xe9; app[12] = 5
        self.files['firmware.bin'].write_bytes(app + IDENTITY.encode() + b'\0' + ENV.encode() + b'\0')
        elf = bytearray(20); elf[:6] = b'\x7fELF\x01\x01'; elf[18] = 243
        self.files['firmware.elf'].write_bytes(elf + IDENTITY.encode() + b'\0' + ENV.encode() + b'\0')
        self.files['partitions.bin'].write_bytes(partitions())
        self.manifest = self.root/'manifest.json'
        self.data = dict(schema=contracts.SCHEMA, source=dict(identity=IDENTITY), build=dict(environment=ENV),
            artifacts=[dict(path=p.name, size_bytes=p.stat().st_size, sha256=sha256_file(p)) for p in self.files.values()])
        self.write_manifest()

    def write_manifest(self):
        self.manifest.write_text(json.dumps(self.data), encoding='utf-8')

    def test_bundle_hash_source_bounds_and_partition(self):
        contracts.load_bundle(self.root, self.manifest)
        self.files['firmware.bin'].write_bytes(b'changed')
        with self.assertRaises(ValueError):
            contracts.load_bundle(self.root, self.manifest)
        for corruption in (partitions()[:96] + b'bad', partitions().replace(b'nvs', b'bad'),
                           partitions()[:112] + b'\0'*16 + partitions()[128:]):
            with self.assertRaises(ValueError):
                contracts.partition_layout(corruption)
        self.data['artifacts'][0]['path'] = '../escape.bin'; self.write_manifest()
        with self.assertRaises(ValueError):
            contracts.load_bundle(self.root, self.manifest)

    def report(self):
        return types.SimpleNamespace(directory=self.root, data={'commands': []}, save=lambda: None)

    def test_flash_steps_abort_and_preserve_partition_nvs(self):
        args = types.SimpleNamespace(esptool=self.root/'esptool.py')
        args.esptool.write_text('fake', encoding='utf-8')
        for failing in (None, 'version', 'read_flash', 'write_flash', 'readback', 'partition-mismatch', 'readback-mismatch'):
            report = self.report(); commands = []; deadlines = []
            def run(report, command, timeout_s=180):
                deadlines.append(timeout_s)
                commands.append(command)
                if command[-1] == 'version':
                    if failing == 'version':
                        raise ValueError('injected version failure')
                    (self.root/'version.log').write_text('esptool.py v4.9.0\n4.9.0\n', encoding='utf-8')
                    report.data['commands'].append({'log': 'version.log'})
                elif 'read_flash' in command:
                    app = command[-3] == '0x10000'
                    if failing == ('readback' if app else 'read_flash'):
                        raise ValueError('injected read failure')
                    data = self.files['firmware.bin'].read_bytes() if app else partitions().ljust(4096, b'\xff')
                    Path(command[-1]).write_bytes(b'bad' if failing == 'partition-mismatch' or (app and failing == 'readback-mismatch') else data)
                elif failing == 'write_flash':
                    raise ValueError('injected write failure')
            with mock.patch.object(workflow, 'checked_command', side_effect=run), mock.patch.object(workflow, 'selected', return_value=DEVICE):
                if failing:
                    with self.assertRaises(ValueError):
                        workflow.flash_run(args, report, DEVICE, self.files)
                else:
                    workflow.flash_run(args, report, DEVICE, self.files)
                    self.assertIn('NOT verified', report.data['result']['scope'])
                    self.assertEqual(deadlines, [180, 180, 180, 600])
                    self.assertEqual(report.data['esptool_version'], '4.9.0')
            writes = [c for c in commands if 'write_flash' in c]
            self.assertEqual(len(writes), 0 if failing in ('version', 'read_flash', 'partition-mismatch') else 1)
            if writes:
                self.assertEqual(writes[0][-2:], ['0x10000', str(self.files['firmware.bin'])])
            self.assertLessEqual(len(commands), 4)
            for command in commands[1:]:
                self.assertEqual(command[command.index('--before')+1], 'usb_reset')
                self.assertEqual(command[command.index('--baud')+1], '115200')
                self.assertEqual(report.data['upload_baud'], 115200)
                expected = 'hard_reset' if command[-3] == '0x10000' and 'read_flash' in command else 'no_reset'
                self.assertEqual(command[command.index('--after')+1], expected)

    def test_module_and_version_guard_precede_device_commands(self):
        args = workflow.parser().parse_args(['flash', '--manifest', str(self.manifest),
            '--port', DEVICE['port'], '--usb-serial', DEVICE['serial'], '--vid', '0x303a', '--pid', '0x1001'])
        self.assertIsNone(args.esptool)
        entry = self.root/'__init__.py'
        entry.write_text('fake module entry', encoding='utf-8')
        spec = types.SimpleNamespace(origin=str(entry))
        for version in ('4.9.0', '4.5.1', '4.12.0', '5.4.0', '', '4.5.1\n4.9.0'):
            with self.subTest(version=version):
                report = self.report()
                def run(report, command, timeout_s=180):
                    if command[-1] != 'version':
                        raise ValueError('stop before hardware')
                    self.assertEqual(command, [sys.executable, '-m', 'esptool', 'version'])
                    (self.root/'version.log').write_text(version+'\n', encoding='utf-8')
                    report.data['commands'].append({'log': 'version.log'})
                with mock.patch.object(workflow.importlib.util, 'find_spec', return_value=spec), \
                     mock.patch.object(workflow, 'checked_command', side_effect=run) as runner, \
                     mock.patch.object(workflow, 'selected', return_value=DEVICE) as select:
                    message = 'stop before hardware' if version == '4.9.0' else 'requires verified'
                    with self.assertRaisesRegex(ValueError, message):
                        workflow.flash_run(args, report, DEVICE, self.files)
                    self.assertEqual(runner.call_count, 2 if version == '4.9.0' else 1)
                    if version != '4.9.0':
                        select.assert_not_called()
        with mock.patch.object(workflow.importlib.util, 'find_spec', return_value=None), \
             mock.patch.object(workflow, 'checked_command') as runner:
            with self.assertRaisesRegex(ValueError, 'required in this Python'):
                workflow.flash_run(args, self.report(), DEVICE, self.files)
            runner.assert_not_called()

    def test_readback_deadline_does_not_retry_or_hide_failure(self):
        report = self.report()
        report.run = mock.Mock(return_value=types.SimpleNamespace(returncode=124))
        with self.assertRaisesRegex(ValueError, 'exit=124.*no automatic retry'):
            workflow.checked_command(report, ['fake-reader'], timeout_s=600)
        report.run.assert_called_once()
        self.assertEqual(report.run.call_args.kwargs['timeout_s'], 600)

    def test_uart_reset_and_usb_profile_contract(self):
        report = self.report()
        args = types.SimpleNamespace(esptool=self.root/'esptool.py')
        args.esptool.write_text('fake', encoding='utf-8')
        device = DEVICE | dict(vid=0x10c4, pid=0xea60)
        commands = []
        def run(report, command, timeout_s=180):
            commands.append(command)
            if command[-1] == 'version':
                (self.root/'version.log').write_text('esptool.py v4.9.0\n', encoding='utf-8')
                report.data['commands'].append({'log': 'version.log'})
            else:
                raise ValueError('stop before hardware')
        with mock.patch.object(workflow, 'checked_command', side_effect=run), mock.patch.object(workflow, 'selected', return_value=device):
            with self.assertRaisesRegex(ValueError, 'stop before hardware'):
                workflow.flash_run(args, report, device, self.files)
        self.assertEqual(commands[1][commands[1].index('--before')+1], 'default_reset')
        self.assertEqual(commands[1][commands[1].index('--baud')+1], '460800')
        self.assertEqual(report.data['upload_baud'], 460800)
        text = (ROOT/'platformio.ini').read_text(encoding='utf-8')
        self.assertEqual(profiles.validate_usb_diagnostic_contract(text), [])
        self.assertIn(profiles.USB_DIAG_ENV, contracts.ENVIRONMENTS)
        for old, new in (('QUEUE_BYTES=8192', 'QUEUE_BYTES=16384'),
                         ('MOTION_LIGHT_SLEEP=0', 'MOTION_LIGHT_SLEEP=1'),
                         ('upload_speed = 115200', 'upload_speed = 460800'),
                         ('before_reset = usb_reset', 'before_reset = default_reset'),
                         ('after_reset = hard_reset', 'after_reset = no_reset')):
            with self.subTest(setting=old):
                self.assertTrue(profiles.validate_usb_diagnostic_contract(text.replace(old, new)))

    def test_invalid_manifest_shapes_and_stale_elf_fail(self):
        original = json.loads(json.dumps(self.data))
        for change in ({'source': []}, {'build': 'bad'}, {'artifacts': {}},
                       {'artifacts': [dict(path=3)]}, {'source': {'identity': 'unknown'}}):
            self.data = original | change; self.write_manifest()
            with self.subTest(change=change), self.assertRaises(ValueError):
                contracts.load_bundle(self.root, self.manifest)
        self.data = original; self.write_manifest()
        self.files['firmware.elf'].write_bytes(b'wrong ELF')
        with self.assertRaises(ValueError):
            contracts.load_bundle(self.root, self.manifest)

    def test_snapshot_is_independent_and_symbolization_rejects_wrong_build(self):
        report = self.report()
        with mock.patch.object(workflow, 'ROOT', self.root):
            manifest, copied = workflow.snapshot_bundle(self.manifest, report)
        self.files['firmware.bin'].write_bytes(b'overwritten during unrelated build')
        self.assertNotEqual(copied['firmware.bin'].read_bytes(), self.files['firmware.bin'].read_bytes())
        args = types.SimpleNamespace(manifest=self.manifest, crash_build='b'*40)
        with mock.patch.object(workflow, 'snapshot_bundle', return_value=(manifest, copied)), mock.patch.object(workflow, 'checked_command') as command:
            with self.assertRaisesRegex(ValueError, 'crash build identity'):
                workflow.symbolize(args, report)
            command.assert_not_called()

    def test_smoke_orchestration_requires_final_progress_and_records_scope(self):
        args = types.SimpleNamespace(expect_build=IDENTITY, environment=ENV, samples=4, require_mag=False)
        version = dict(serial_protocol='serial-cli-v1', build_git=IDENTITY, build_pio_env=ENV)
        for fault in (None, 'timeout', 'wrong-build', 'stale', 'panic', 'initial-panic'):
            report = self.report()
            console = mock.Mock(faults=[], smoke_active=False, version_header_resyncs=0)
            count = 0
            def request(command, *rest, **kwargs):
                nonlocal count
                if command == 'version':
                    self.assertEqual(kwargs.get('initial_sync', False), not console.smoke_active)
                    if fault == 'initial-panic':
                        console.faults.append('Guru Meditation')
                    return version | ({'build_git': 'b'*40} if fault == 'wrong-build' else {})
                count += 1
                if fault == 'timeout':
                    raise TimeoutError('incomplete health response')
                if fault == 'panic':
                    console.faults.append('Guru Meditation')
                return health(count, **({'prepared_output_valid': 'no'} if fault == 'stale' else {}))
            console.request.side_effect = request
            with self.subTest(fault=fault):
                if fault:
                    with self.assertRaises((ValueError, TimeoutError)):
                        workflow.smoke_run(args, report, console, clock=lambda: float(count), sleep=lambda _: None)
                    self.assertNotIn('result', report.data)
                else:
                    workflow.smoke_run(args, report, console, clock=lambda: float(count), sleep=lambda _: None)
                    self.assertEqual(report.data['result']['observations'], 4)
                    self.assertEqual(console.request.call_count, 6)

    def test_dry_plan_and_flash_permission_do_not_enumerate_or_open(self):
        with mock.patch.object(workflow, 'ROOT', self.root), mock.patch.object(workflow, 'ports') as ports, contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
            self.assertEqual(workflow.main(['flash-plan', '--manifest', str(self.manifest)]), 0)
            self.assertEqual(workflow.main(['flash', '--manifest', str(self.manifest), '--esptool', 'fake',
                '--port', 'COM13', '--usb-serial', 'fake', '--vid', '1', '--pid', '1']), 1)
            self.assertEqual(workflow.main(['smoke', '--expect-build', 'unknown', '--environment', ENV,
                '--port', 'COM13', '--usb-serial', 'fake', '--vid', '1', '--pid', '1']), 1)
            ports.assert_not_called()


if __name__ == '__main__':
    unittest.main()
