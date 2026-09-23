#!/usr/bin/env python3
"""Export/compare DEV-05 production algorithm scenarios; never execute report argv.

An export is evidence packaging, NOT a claim that every roadmap contract passed.
"""
from __future__ import annotations
import argparse
import hashlib
import json
import math
from pathlib import Path
import re
import sys
import tempfile

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from gate_reporting import replace_report

SCHEMA = 'tracker-algorithm-accuracy-v1'
SCENARIOS = {
    **{name: 'acceptance' for name in ('static', 'gyro_axis', 'gyro_noncommuting',
        'clean_multi_axis', 'yaw_sine', 'missing_accel', 'dynamic_accel', 'tilt_25_relock',
        'seeded_noise_bias', 'continuous_dynamic_accel', 'saturated_accel', 'timestamp_jitter', 'gyro_60s', 'mag_continuous_interference', 'mag_90_relock', 'mag_clean_tilted', 'mag_disturbance_return',
        'mag_stale_return', 'mag_untrusted_tilt')},
    'tilt_90_relock': 'roadmap-0029', 'invalid_accel_gyro_continuity': 'roadmap-0029',
    'mag_180_relock': 'roadmap-0030',
    'missing_accel_statistics': 'roadmap-0029', 'yaw_accel_evidence_phases': 'roadmap-0029',
}
# Comparison resolution floors, NOT firmware trust gates or universal accuracy specs.
ABSOLUTE = {name: 0.0001 for name in ('orientation_rms_deg', 'orientation_p95_deg',
    'orientation_max_deg', 'tilt_rms_deg', 'yaw_rms_deg', 'final_orientation_deg')}
ABSOLUTE.update(norm_max_error=0.000001, lag_abs_ms=0.02, evidence_loss=0.000001)
AHRS_CONFIG = {'accelNormVarianceGoodG2', 'accelCorrectionEnabled', 'gyroDeadbandRadS', 'maxDtS', 'accelCorrectionDivisor', 'normalizeEvery', 'gyroNormAccelTrustGoodRadS', 'gyroNormAccelTrustBadRadS', 'accelInnovationBadRad', 'accelKp', 'accelNormVarianceBadG2', 'accelInnovationGoodRad', 'worldUp.x', 'accelNormBadErrorG', 'worldUp.y', 'minDtS', 'clampLargeDt', 'adaptiveAccelCorrection', 'accelNormVarianceAlpha', 'maxAccelCorrectionRadPerUpdate', 'worldUp.z', 'accelNormGoodErrorG'}
MAG_CONFIG = {'gyroNormBadDps', 'maxMagAgeMs', 'maxCorrectionRateDegS', 'enabled', 'reacquisitionEnabled', 'fallbackDtS', 'magDisturbanceCooldownMs', 'applyEnabled', 'reacquireMinFieldStableMs', 'accelBadCooldownMs', 'requireAccelTrusted', 'gyroMovingCooldownMs', 'gyroNormGoodDps', 'reacquireMaxHeadingRateDegS', 'horizontalNormBad', 'reacquireMaxCorrectionStepDeg', 'horizontalNormGood', 'reacquireMaxCorrectionRateDegS', 'accelTrustBad', 'reacquireInnovationMaxDeg', 'heading.requireTrustedMag', 'maxInnovationDeg', 'maxCorrectionStepDeg', 'reacquireTimeConstantS', 'heading.minHorizontalNorm', 'accelTrustGood', 'timeConstantS'}
MAX_BYTES = 8 * 1024 * 1024


def reject_constant(value):
    raise ValueError(f'nonfinite JSON constant: {value}')


def unique_pairs(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f'duplicate JSON field: {key}')
        result[key] = value
    return result


def loads(text):
    return json.loads(text, parse_constant=reject_constant, object_pairs_hook=unique_pairs)


def read(path):
    if path.stat().st_size > MAX_BYTES:
        raise ValueError('evidence file too large')
    return loads(path.read_text(encoding='utf-8'))


def number(value, *, nonnegative=True):
    if type(value) not in (int, float) or not math.isfinite(value) or (nonnegative and value < 0):
        raise ValueError(f'invalid metric/config number: {value!r}')
    return value


def validate_row(row):
    if not isinstance(row, dict) or row.get('schema') != 1 or row.get('suite') != 'dev05-v1':
        raise ValueError('unknown scenario schema/suite')
    name = row.get('scenario')
    if name not in SCENARIOS or row.get('role') != SCENARIOS[name]:
        raise ValueError('unknown scenario or altered acceptance role')
    if type(row.get('samples')) is not int or row['samples'] <= 0:
        raise ValueError('missing sample coverage')
    expected = 1 if name == 'invalid_accel_gyro_continuity' else 1920 if name == 'missing_accel_statistics' else 15360 if name == 'yaw_accel_evidence_phases' else 28800 if name == 'tilt_90_relock' else 57600 if name == 'gyro_60s' else 1800 if name.startswith('mag_') else 11520
    if row['samples'] != expected:
        raise ValueError(f'{name}: shortened scenario')
    if type(row.get('requirement_met')) is not bool:
        raise ValueError('missing contract verdict')
    if row['role'] == 'acceptance' and not row['requirement_met']:
        raise ValueError(f'{name}: failed acceptance is not successful evidence')
    if not isinstance(row.get('compiler'), str) or not row['compiler'] or row.get('pointer_bits') not in (32,64) or row.get('platform') not in ('windows','linux','other'):
        raise ValueError('missing compiler identity')
    if not isinstance(row.get('input_fnv1a64'), str) or not re.fullmatch('[0-9a-f]{16}',row['input_fnv1a64']):
        raise ValueError('missing input identity')
    cfg = row.get('config')
    if not isinstance(cfg,dict) or set(cfg) != (MAG_CONFIG if name.startswith('mag_') else AHRS_CONFIG):
        raise ValueError('incomplete configuration')
    for v in cfg.values():
        number(v,nonnegative=False)
    metrics = row.get('metrics')
    if not isinstance(metrics,dict) or set(metrics) != set(ABSOLUTE):
        raise ValueError('missing/unknown accuracy metrics')
    for v in metrics.values():
        number(v)
    if metrics['norm_max_error'] >= 0.0001:
        raise ValueError('invalid quaternion norm budget')
    if row.get('recovery_ms') is not None:
        number(row['recovery_ms'])
    if type(row.get('rejects')) is not int or row['rejects'] < 0:
        raise ValueError('invalid reject count')
    number(row.get('max_output_step_deg'))
    return row


def validate(data):
    if not isinstance(data,dict) or data.get('schema') != SCHEMA:
        raise ValueError('unknown accuracy report')
    rows=data.get('scenarios')
    if not isinstance(rows,list):
        raise ValueError('missing scenarios')
    by_name={}
    for row in rows:
        validate_row(row)
        if row['scenario'] in by_name:
            raise ValueError('duplicate scenario')
        by_name[row['scenario']]=row
    if set(by_name)!=set(SCENARIOS):
        raise ValueError('incomplete scenario suite')
    if not isinstance(data.get('build'),dict) or not isinstance(data['build'].get('flags'),list) or not all(isinstance(v,str) for v in data['build']['flags']):
        raise ValueError('missing build flags')
    if data['build'].get('sanitizer') not in ('none','address-undefined','leak'):
        raise ValueError('unknown sanitizer mode')
    for key in ('algorithm_harness_sha256','algorithm_source_sha256'):
        if not isinstance(data['build'].get(key),str) or not re.fullmatch('[0-9a-f]{64}',data['build'][key]):
            raise ValueError('missing algorithm build fingerprint')
    source=data.get('source')
    if not isinstance(source,dict) or not isinstance(source.get('commit'),str) or not re.fullmatch('[0-9a-fA-F]{40}|[0-9a-fA-F]{64}', source['commit']) or type(source.get('dirty')) is not bool:
        raise ValueError('missing source identity')
    if not isinstance(data.get('label'),str) or not data['label'].strip():
        raise ValueError('missing evidence label')
    return by_name


def export(native_report: Path, label: str):
    report=read(native_report)
    if not isinstance(report,dict) or report.get('schema')!='tracker-gate-run-v1' or report.get('runner')!='native' or report.get('state')!='completed' or type(report.get('returncode')) is not int or report['returncode']!=0:
        raise ValueError('requires completed successful native run')
    commands=report.get('commands')
    if not isinstance(commands,list):
        raise ValueError('missing commands')
    records=[]
    for record in commands:
        if not isinstance(record,dict) or record.get('state')!='completed' or type(record.get('returncode')) is not int or record['returncode']!=0:
            raise ValueError('incomplete or failed command')
        command=record.get('command')
        if not isinstance(command,list) or not command or not all(isinstance(x,str) for x in command):
            raise ValueError('invalid command record')
        if len(command)==1 and command[0].replace('\\','/').split('/')[-1] in ('test_dev05_algorithm_scenarios','test_dev05_algorithm_scenarios.exe'):
            records.append(record)
    if len(records)!=1:
        raise ValueError('requires exactly one scenario execution (build-only is insufficient)')
    filename=records[0].get('log')
    if not isinstance(filename,str) or not re.fullmatch(r'[0-9]+\.log',filename):
        raise ValueError('unsafe log path')
    log=native_report.parent/filename
    if log.is_symlink() or log.stat().st_size>MAX_BYTES:
        raise ValueError('unsafe/oversized log')
    text=log.read_text(encoding='utf-8')
    rows=[loads(line[len('DEV05_RESULT '):]) for line in text.splitlines() if line.startswith('DEV05_RESULT ')]
    if 'PASS test_dev05_algorithm_scenarios' not in text.splitlines():
        raise ValueError('missing executable completion')
    result={'schema':SCHEMA,'label':label,'source':report.get('source'),
            'build':report.get('metadata'),'native_report':str(native_report.resolve()),
            'native_report_sha256':hashlib.sha256(native_report.read_bytes()).hexdigest(),
            'raw_log_sha256':hashlib.sha256(log.read_bytes()).hexdigest(),
            'limitations':['synthetic module-level replay; not board/app/FIFO replay',
                'source metadata is startup HEAD/dirty, not an immutable worktree snapshot',
                'FNV input identity detects accidental differences, not adversarial tampering',
                'no absolute real-device accuracy or target timing claim'], 'scenarios':rows}
    validate(result)
    return result


def compare(before, after, relative=0.05, require_roadmap=False):
    number(relative)
    if relative > 1:
        raise ValueError('relative budget must be between zero and one')
    a,b=validate(before),validate(after)
    for key in ('flags','sanitizer','algorithm_harness_sha256'):
        if before['build'][key]!=after['build'][key]:
            raise ValueError(f'incomparable build {key}')
    changes=[]; gaps=[]
    for name in sorted(SCENARIOS):
        x,y=a[name],b[name]
        for key in ('suite','role','compiler','pointer_bits','platform','input_fnv1a64','config','samples'):
            if x[key]!=y[key]:
                raise ValueError(f'{name}: incomparable {key}')
        if not y['requirement_met']:
            gaps.append(name)
        if x['requirement_met'] != y['requirement_met']:
            changes.append({'scenario':name,'metric':'requirement_met','kind':'improved' if y['requirement_met'] else 'regressed'})
        for metric,floor in ABSOLUTE.items():
            old,new=x['metrics'][metric],y['metrics'][metric]
            budget=max(floor,abs(old)*relative)
            delta=new-old
            if abs(delta)>budget:
                changes.append({'scenario':name,'metric':metric,'before':old,'after':new,'delta':delta,'budget':budget,'kind':'regressed' if delta>0 else 'improved'})
        old,new=x['recovery_ms'],y['recovery_ms']
        if old is None and new is not None:
            changes.append({'scenario':name,'metric':'recovery_ms','kind':'improved','before':old,'after':new})
        elif old is not None and new is None:
            changes.append({'scenario':name,'metric':'recovery_ms','kind':'regressed','before':old,'after':new})
        elif old is not None and new is not None and abs(new-old)>max(20,old*relative):
            changes.append({'scenario':name,'metric':'recovery_ms','kind':'regressed' if new>old else 'improved','before':old,'after':new})
    failed=any(c['kind']=='regressed' for c in changes) or (require_roadmap and bool(gaps))
    return {'schema':'tracker-algorithm-comparison-v1','before':before['label'],'after':after['label'],
            'status':'FAIL' if failed else 'NO_MEASURABLE_REGRESSION',
            'relative_budget':relative,'absolute_floors':ABSOLUTE,'require_roadmap':require_roadmap,
            'open_contracts':gaps,'changes':changes,
            'interpretation':'Measured changes on identical synthetic inputs only; no aggregate accuracy score.'}


def write(path: Path,data):
    path.parent.mkdir(parents=True,exist_ok=True)
    tmp=None
    try:
        with tempfile.NamedTemporaryFile('w',dir=path.parent,prefix=path.name+'.',suffix='.tmp',encoding='utf-8',delete=False) as f:
            tmp=Path(f.name); json.dump(data,f,indent=2,allow_nan=False); f.write('\n')
        replace_report(tmp,path)
    finally:
        if tmp is not None:
            tmp.unlink(missing_ok=True)


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    sub=p.add_subparsers(dest='action',required=True)
    e=sub.add_parser('export'); e.add_argument('--report',type=Path,required=True); e.add_argument('--label',required=True); e.add_argument('--output',type=Path,required=True)
    c=sub.add_parser('compare'); c.add_argument('before',type=Path); c.add_argument('after',type=Path); c.add_argument('--output',type=Path,required=True); c.add_argument('--relative-budget',type=float,default=0.05); c.add_argument('--require-roadmap',action='store_true')
    args=p.parse_args(argv)
    try:
        inputs=[args.report] if args.action=='export' else [args.before,args.after]
        if args.output.resolve() in [x.resolve() for x in inputs]:
            raise ValueError('output must not overwrite input evidence')
        if args.action=='export':
            if args.output.resolve().is_relative_to(args.report.parent.resolve()):
                raise ValueError('export outside the native evidence directory to preserve raw logs')
            data=export(args.report,args.label); write(args.output,data)
            gaps=[r['scenario'] for r in data['scenarios'] if not r['requirement_met']]
            print(f'EXPORT complete: {len(data["scenarios"])} scenarios; open_contracts={len(gaps)}; report={args.output}')
            for name in gaps: print(f'OPEN {name}')
            return 0
        before,after=read(args.before),read(args.after)
        data=compare(before,after,args.relative_budget,args.require_roadmap)
        data['before_sha256']=hashlib.sha256(args.before.read_bytes()).hexdigest()
        data['after_sha256']=hashlib.sha256(args.after.read_bytes()).hexdigest()
        write(args.output,data)
        print(f'{data["status"]}: changes={len(data["changes"])}; open_contracts={len(data["open_contracts"])}; report={args.output}')
        for change in data['changes'][:8]: print(f'{change["kind"]}: {change["scenario"]} {change["metric"]}')
        return int(data['status']=='FAIL')
    except (OSError,ValueError,TypeError,KeyError) as exc:
        print(f'ERROR algorithm evidence: {exc}',file=sys.stderr); return 2
if __name__=='__main__':
    raise SystemExit(main())
