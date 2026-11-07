from pathlib import Path
import json

R=Path(__file__).resolve().parents[1]
H=R/'experiments/revision_v7'
C=H/'catalog'
C.mkdir(exist_ok=True)
groups={'calibration':[],'matched_comparisons':[],'repeated_refinement':[]}
for p in sorted((H/'results').glob('*.json')):
    r=json.loads(p.read_text(encoding='utf-8'))
    stage='repeated_refinement' if r['method']=='refine' else ('calibration' if r['method']=='obca' else 'matched_comparisons')
    stem=p.stem
    log=H/'logs'/f'{stem}.log'
    assert log.is_file(),log
    groups[stage].append(dict(case=r['case_id'],method=r['method'],beta=r['beta'],
        points=r['nodes'],solver_status=r['solver_status'],solver_success=r['solver_success'],
        collision_frames=r['interpolation_check']['collision_samples'],
        input=f"experiments/inputs/case{r['case_id']:02d}.json",
        result=p.relative_to(R).as_posix(),log=log.relative_to(R).as_posix(),
        dense_collision_data=p.with_suffix('.npz').relative_to(R).as_posix()))
for stage,rows in groups.items():
    (C/f'{stage}.json').write_text(json.dumps(rows,indent=2),encoding='utf-8')
assert [len(groups[k]) for k in groups]==[72,84,61]
index=dict(current_manuscript='v9; numerical results unchanged from the complete v7 MA27 rerun',
    current_experiment_root='experiments/revision_v7',current_solver='IPOPT 3.14.19 with HSL MA27',
    current_attempts=217,benchmark_commit='476ef8573d9e22578be5c3d42514a9a29f27d07a',
    groups={k:dict(attempts=len(v),index=(C/f'{k}.json').relative_to(R).as_posix()) for k,v in groups.items()},
    directories={
        'experiments/inputs':'All twelve benchmark inputs and timed Hybrid A* references, JSON and MATLAB MAT formats.',
        'experiments/source/CommonParking':'Benchmark and initialization source snapshot with upstream license notices.',
        'experiments/revision_v7':'Current source, parameters, result summaries, numerical checks and figure generators.',
        'experiments/revision_v7/results':'Every successful or failed solver return, initial guess, settings and dense interpolation arrays.',
        'experiments/revision_v7/logs':'Complete IPOPT logs for all 217 current attempts.',
        'experiments/revision_v7/refinement':'Per-case sequences and cumulative costs of all refinement attempts.',
        'experiments/revision_v7/catalog':'Stage-specific links from each attempt to its input, output, log and dense collision data.',
        'experiments/results':'Historical fixed-mesh study; superseded and not used for current paper tables.',
        'experiments/logs':'Historical fixed-mesh solver logs.',
        'experiments/revision_v5':'Historical MUMPS study, including diagnostic model variants; superseded by the MA27 rerun.',
        'experiments/revision_v6':'Historical geometry/proof checks and figure source; no new NLP runs.',
        'manuscript/figures':'Generated experiment and geometric figures.',
        'manuscript/tables':'Generated current result tables and historical tables needed by retained checks.'},
    commands={
        'verify_published_files':'python experiments/verify_release.py',
        'verify_and_regenerate_current_outputs':'python experiments/revision_v7/reproduce.py',
        'recompute_all_dense_collision_checks':'python experiments/revision_v7/verify_interpolation.py',
        'independent_geometry_check':'python experiments/revision_v7/geometry_checks.py',
        'rerun_all_current_solves':'python experiments/revision_v7/reproduce.py --fresh',
        'regenerate_references':'Run experiments/prepare_initials.m in MATLAB.'},
    dependencies='experiments/requirements.txt',
    linear_solver_library='Supply an HSL library through BME_HSL_LIBRARY. HSL binaries are external dependencies, not redistributed.',
    references={'HSL':'https://www.hsl.rl.ac.uk/','CasADi':'https://doi.org/10.1007/s12532-018-0139-4',
        'IPOPT':'https://doi.org/10.1007/s10107-004-0559-y'},
    timing='One retained timed run per configuration. Rerun wall times can differ. Current main comparison beta=0.6 remains prespecified.',
    placement='BME uniform initialization uses the same automatically estimated count and free interval durations as adaptive initialization.',
    data_policy='Successful and unsuccessful returns retained; any included historical variants are explicitly separated from current manuscript results.')
index['directories']={p:description for p,description in index['directories'].items() if (R/p).exists()}
index['external_benchmark']={'repository':'https://github.com/libai1943/CommonParking',
    'commit':'476ef8573d9e22578be5c3d42514a9a29f27d07a','fetch_command':'python experiments/fetch_benchmark.py'}
(R/'experiments/experiment-index.json').write_text(json.dumps(index,indent=2),encoding='utf-8')
print(json.dumps({k:len(v) for k,v in groups.items()}))
