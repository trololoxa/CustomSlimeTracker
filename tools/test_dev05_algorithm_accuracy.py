#!/usr/bin/env python3
"""Executable evidence validation and quantitative comparison contract tests."""
import copy
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
from unittest import mock

SPEC=importlib.util.spec_from_file_location('algorithm_accuracy',Path(__file__).parent/'replay/algorithm_accuracy.py')
a=importlib.util.module_from_spec(SPEC); SPEC.loader.exec_module(a)

def row(name,role):
    count=1 if name=='invalid_accel_gyro_continuity' else 1920 if name=='missing_accel_statistics' else 15360 if name=='yaw_accel_evidence_phases' else 28800 if name=='tilt_90_relock' else 57600 if name=='gyro_60s' else 1800 if name.startswith('mag_') else 11520
    return dict(schema=1,suite='dev05-v1',scenario=name,role=role,compiler='test-compiler',pointer_bits=64,platform='linux',
                input_fnv1a64='0'*16,config={k:0 for k in (a.MAG_CONFIG if name.startswith('mag_') else a.AHRS_CONFIG)},samples=count,
                requirement_met=role=='acceptance',recovery_ms=None,rejects=0,
                metrics={k:0 for k in a.ABSOLUTE},max_output_step_deg=0)

def evidence():
    return dict(schema=a.SCHEMA,label='fixture',source={'commit':'a'*40,'dirty':False},
                build={'flags':['-O2'],'sanitizer':'none','algorithm_harness_sha256':'b'*64,'algorithm_source_sha256':'c'*64},
                scenarios=[row(k,v) for k,v in a.SCENARIOS.items()])

class AccuracyTests(unittest.TestCase):
    def test_same_input_has_no_measurable_regression_but_keeps_open_contracts(self):
        data=evidence(); out=a.compare(data,copy.deepcopy(data))
        self.assertEqual(out['status'],'NO_MEASURABLE_REGRESSION')
        self.assertEqual(len(out['open_contracts']),5)
        self.assertEqual(a.compare(data,data,require_roadmap=True)['status'],'FAIL')

    def test_mixed_improvement_and_regression_cannot_cancel(self):
        old=evidence(); new=copy.deepcopy(old)
        old['scenarios'][0]['metrics']['orientation_rms_deg']=1
        new['scenarios'][1]['metrics']['orientation_max_deg']=1
        result=a.compare(old,new)
        self.assertEqual(result['status'],'FAIL')
        self.assertEqual({r['kind'] for r in result['changes']},{'improved','regressed'})

    def test_resolution_floor_and_recovery_loss(self):
        old=evidence(); new=copy.deepcopy(old)
        new['scenarios'][0]['metrics']['orientation_rms_deg']=1e-8
        self.assertEqual(a.compare(old,new)['changes'],[])
        old['scenarios'][0]['recovery_ms']=0
        self.assertEqual(a.compare(old,new)['status'],'FAIL')

    def test_regression_even_on_known_roadmap_case(self):
        old=evidence(); new=copy.deepcopy(old)
        new['scenarios'][-1]['metrics']['orientation_rms_deg']=1
        self.assertEqual(a.compare(old,new)['status'],'FAIL')

    def test_incompatible_inputs_config_harness_and_compiler_rejected(self):
        for key,value in [('input_fnv1a64','1'*16),('compiler','other'),('pointer_bits',32),('platform','windows'),('config',{k:1 for k in a.AHRS_CONFIG})]:
            old=evidence(); new=copy.deepcopy(old); new['scenarios'][0][key]=value
            with self.subTest(key=key),self.assertRaises(ValueError): a.compare(old,new)
        for key,value in [('flags',['-O0']),('sanitizer','leak'),('algorithm_harness_sha256','d'*64)]:
            old=evidence(); new=copy.deepcopy(old); new['build'][key]=value
            with self.subTest(key=key),self.assertRaises(ValueError): a.compare(old,new)
        old=evidence(); new=copy.deepcopy(old); new['build']['algorithm_source_sha256']='e'*64
        self.assertEqual(a.compare(old,new)['status'],'NO_MEASURABLE_REGRESSION')

    def test_missing_nonfinite_duplicate_shortened_failed_evidence_rejected(self):
        edits=[lambda d:d['scenarios'].pop(),lambda d:d['scenarios'].append(d['scenarios'][0]),
               lambda d:d['scenarios'][0]['metrics'].pop('orientation_rms_deg'),
               lambda d:d['scenarios'][0]['metrics'].update(orientation_rms_deg=float('nan')),
               lambda d:d['scenarios'][0]['metrics'].update(orientation_rms_deg=float('inf')),
               lambda d:d['scenarios'][0].update(samples=1),
               lambda d:d['scenarios'][0].update(requirement_met=False),
               lambda d:d['scenarios'][0].update(role='roadmap-0029'),
               lambda d:d['source'].update(commit=None),lambda d:d['build'].pop('algorithm_source_sha256')]
        for edit in edits:
            with self.subTest(edit=edit),self.assertRaises(ValueError):
                d=evidence(); edit(d); a.validate(d)
        for text in ('{"x":NaN}','{"x":1,"x":2}'):
            with self.assertRaises(ValueError): a.loads(text)
        d=evidence();d['scenarios'][0]['metrics']['orientation_rms_deg']=a.loads('1e999')
        with self.assertRaises(ValueError):a.validate(d)

    def test_export_from_windows_report_without_executing_commands(self):
        with tempfile.TemporaryDirectory() as td:
            root=Path(td); data=evidence()
            log='\n'.join('DEV05_RESULT '+json.dumps(r) for r in data['scenarios'])+'\nPASS test_dev05_algorithm_scenarios\n'
            (root/'0001.log').write_text(log)
            report=dict(schema='tracker-gate-run-v1',runner='native',state='completed',returncode=0,
                        source=data['source'],metadata=data['build'],commands=[dict(command=['H:\\repo\\test_dev05_algorithm_scenarios.exe'],log='0001.log',state='completed',returncode=0)])
            path=root/'summary.json';path.write_text(json.dumps(report))
            with mock.patch('subprocess.run',side_effect=AssertionError('must not execute argv')):
                result=a.export(path,'verified-test')
            self.assertEqual(len(result['scenarios']),24)
            self.assertEqual(len(result['raw_log_sha256']),64)
            for key,value in [('state','running'),('returncode',1)]:
                failed=copy.deepcopy(report);failed[key]=value;path.write_text(json.dumps(failed))
                with self.assertRaises(ValueError):a.export(path,'test')
            for name in ('../0001.log','C:\\private.log','/private.log'):
                report['commands'][0]['log']=name;path.write_text(json.dumps(report))
                with self.assertRaises(ValueError):a.export(path,'test')

    def test_atomic_export_preserves_previous_on_write_failure(self):
        with tempfile.TemporaryDirectory() as td:
            p=Path(td)/'metrics.json';p.write_text('old')
            with mock.patch.object(a,'replace_report',side_effect=PermissionError('locked')):
                with self.assertRaises(PermissionError):a.write(p,evidence())
            self.assertEqual(p.read_text(),'old')
            self.assertEqual(list(Path(td).glob('*.tmp')),[])

    def test_negative_or_nonfinite_budget_rejected(self):
        for budget in (-1,float('nan'),float('inf'),2):
            with self.assertRaises(ValueError):a.compare(evidence(),evidence(),budget)

    def test_export_cannot_overwrite_raw_log(self):
        import contextlib
        import io
        with tempfile.TemporaryDirectory() as td:
            root=Path(td);log=root/'0001.log';log.write_text('keep raw evidence')
            with contextlib.redirect_stderr(io.StringIO()):
                code=a.main(['export','--report',str(root/'summary.json'),'--label','test','--output',str(log)])
            self.assertEqual(code,2)
            self.assertEqual(log.read_text(),'keep raw evidence')

    def test_harness_and_firmware_fingerprints_are_separate(self):
        import run_standalone_tests as native
        with tempfile.TemporaryDirectory() as td:
            root=Path(td);tests=root/'tests/native';(tests/'dev05').mkdir(parents=True);(root/'src').mkdir()
            (tests/'test_dev05_algorithm_scenarios.cpp').write_text('case\n')
            (tests/'test_common.hpp').write_text('check\n'); (tests/'dev05/reference_rotation.hpp').write_text('oracle\n')
            (root/'src/a.cpp').write_text('before\n')
            with mock.patch.object(native,'ROOT',root),mock.patch.object(native,'TEST_DIR',tests):
                first=native.algorithm_fingerprints();(root/'src/a.cpp').write_text('after\n');second=native.algorithm_fingerprints()
                self.assertEqual(first['algorithm_harness_sha256'],second['algorithm_harness_sha256'])
                self.assertNotEqual(first['algorithm_source_sha256'],second['algorithm_source_sha256'])
                (tests/'dev05/reference_rotation.hpp').write_text('changed\n');third=native.algorithm_fingerprints()
                self.assertNotEqual(second['algorithm_harness_sha256'],third['algorithm_harness_sha256'])

if __name__=='__main__':
    unittest.main()
