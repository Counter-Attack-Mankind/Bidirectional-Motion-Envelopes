from pathlib import Path
import argparse,datetime,json,subprocess,sys
H=Path(__file__).resolve().parent
p=argparse.ArgumentParser()
p.add_argument('--solve',action='store_true',help='Run missing NLP experiments sequentially with MA27.')
p.add_argument('--fresh',action='store_true',help='Archive saved runs and repeat all solves.')
a=p.parse_args()
if not (H.parents[1]/'experiments/inputs/case01.json').is_file():
    raise SystemExit('Run experiments/fetch_benchmark.py, then run experiments/prepare_initials.m in MATLAB to generate the twelve timed references.')
def run(name,*args):
    subprocess.run([sys.executable,str(H/name),*args],cwd=H.parents[1],check=True)
if a.fresh:
    archive=H/'archive'/datetime.datetime.now().strftime('%Y%m%d-%H%M%S')
    archive.mkdir(parents=True)
    for name in ['results','logs','refinement','failures']:
        if (H/name).exists():(H/name).rename(archive/name)
run('verify_mesh.py')
run('verify_proofs.py')
if a.solve or a.fresh:
    run('run_study.py','calibration')
    beta=json.loads((H/'selection.json').read_text())['selected_beta']
    for stage in ['comparison','motionbuffer','refinement']:
        run('run_study.py',stage,'--beta',str(beta))
run('summarize.py')
run('verify_results.py')
for result in sorted((H/'results').glob('*.json')):
    if not result.with_suffix('.npz').exists():run('check_interpolation.py',str(result))
run('make_figures.py')
