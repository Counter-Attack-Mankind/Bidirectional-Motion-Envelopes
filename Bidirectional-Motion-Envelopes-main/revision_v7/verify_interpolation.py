from pathlib import Path
import json,shutil,uuid
from check_interpolation import check
import numpy as np

H=Path(__file__).resolve().parent
count=0
T=(H.parents[1]/'work'/f'collision-audit-{uuid.uuid4().hex}').resolve()
assert T.is_relative_to((H.parents[1]/'work').resolve())
T.mkdir(parents=True)
try:
    for source in sorted((H/'results').glob('*.json')):
        original=json.loads(source.read_text(encoding='utf-8'))['interpolation_check']
        target=T/source.name
        shutil.copy2(source,target)
        actual=check(target)
        for key in ['samples','collision_samples','no_detected_overlap','solver_success']:
            assert actual[key]==original[key],(source.name,key)
        for key in ['maximum_step_s','max_overlap_m2','minimum_distance_m']:
            assert abs(actual[key]-original[key])<=1e-10,(source.name,key)
        with np.load(source.with_suffix('.npz')) as saved,np.load(target.with_suffix('.npz')) as recomputed:
            assert set(saved.files)==set(recomputed.files)
            for key in saved.files:
                assert np.allclose(saved[key],recomputed[key],rtol=0,atol=1e-10),(source.name,key)
        count+=1
        target.unlink()
        target.with_suffix('.npz').unlink()
finally:
    if not any(T.iterdir()):T.rmdir()
assert count==217
print(json.dumps(dict(recomputed_trajectories=count,all_dense_arrays_and_collision_metrics_match=True,
    source_results_modified=False,interpolation='linear position and unwrapped heading')))
