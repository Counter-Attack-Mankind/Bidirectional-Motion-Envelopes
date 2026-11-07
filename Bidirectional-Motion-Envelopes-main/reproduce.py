"""Run all missing optimizations, audit saved results and regenerate figures.

Existing results are preserved. To run every optimization again, use a new
copy of the package with an empty experiments/results/ directory.
"""
from pathlib import Path
import subprocess,sys
root=Path(__file__).resolve().parents[1]
exp=root/'experiments'
(exp/'results').mkdir(exist_ok=True)
(exp/'logs').mkdir(exist_ok=True)
subprocess.run([sys.executable,str(exp/'run_suite.py')],cwd=root,check=True)
for case in range(1,13):
 stem=f'triangle-n100-p0.00-case{case:02d}-eps1e-04'
 if (exp/'results'/f'{stem}.json').exists():continue
 print('RUN',stem,flush=True)
 with (exp/'logs'/f'{stem}.log').open('w',encoding='utf-8') as log:
  subprocess.run([sys.executable,str(exp/'run_benchmark.py'),'--case',str(case),
   '--method','triangle','--area-epsilon','0.0001'],cwd=root,
   stdout=log,stderr=subprocess.STDOUT,check=True,timeout=260)
for script in ['evaluate_results.py','verify_results.py','make_figures.py']:
 subprocess.run([sys.executable,str(exp/script)],cwd=root,check=True)
print('Complete: 120 runs accounted for, audits updated, figures generated.')
