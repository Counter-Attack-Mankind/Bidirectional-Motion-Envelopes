"""Regenerate every manuscript result table from the retained final-model runs."""
from pathlib import Path
import json
import statistics as st
import csv

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
OUT = ROOT/'manuscript/tables'
OUT.mkdir(parents=True,exist_ok=True)

def read(stem):
    return json.loads((HERE/'results'/f'{stem}.json').read_text())

def eligible(r):
    return r['solver_success'] and r['feasible']

def accepted(r):
    return eligible(r) and r['interpolation_check']['no_detected_overlap']

def rows(method, beta=.6):
    return [read(f'{method}-b{beta:.1f}-case{i:02d}') for i in range(1,13)]

def summarize(rr, times=None):
    ee = [r for r in rr if eligible(r)]
    collisions = sum(r['interpolation_check']['collision_samples'] for r in ee)
    samples = sum(r['interpolation_check']['samples'] for r in ee)
    return dict(solver_success=sum(r['solver_success'] for r in rr),
                feasible_solves=len(ee), accepted=sum(accepted(r) for r in rr),
                collision_frames=collisions, evaluated_frames=samples,
                collision_percent=100*collisions/samples if samples else None,
                median_s=st.median(times or [r['solve_wall_s'] for r in rr]),
                total_s=sum(times or [r['solve_wall_s'] for r in rr]),
                mean_nodes=st.mean(r['nodes'] for r in rr),
                median_mesh_s=st.median(r['mesh_selection_s'] for r in rr),
                median_select_solve_s=st.median(r['mesh_selection_s']+r['solve_wall_s'] for r in rr),
                min_nodes=min(r['nodes'] for r in rr), max_nodes=max(r['nodes'] for r in rr))

cal=[]
for beta in [.5,.6,.7,.8,.9,1.0]:
    ss=summarize(rows('obca',beta));ss['beta']=beta;cal.append(ss)
timing_rank_choice=min(cal,key=lambda r:(-r['accepted'],r['median_s'],r['mean_nodes']))['beta']
chosen=json.loads((HERE/'selection.json').read_text())['selected_beta']
methods=['obca','triangle','body','uniform','buffer','discs','motionbuffer','placement']
report={'selected_beta':chosen,'timing_rank_choice':timing_rank_choice,'calibration':cal,'methods':{m:summarize(rows(m,chosen)) for m in methods}}
ref=[json.loads((HERE/'refinement'/f'case{i:02d}.json').read_text()) for i in range(1,13)]
rr=[read(r['attempts'][-1]['result']) for r in ref]
report['methods']['refine']=summarize(rr,[r['cumulative_solve_wall_s'] for r in ref])
report['methods']['refine']['median_mesh_s']=st.median(read(r['attempts'][0]['result'])['mesh_selection_s'] for r in ref)
report['methods']['refine']['median_select_solve_s']=st.median(read(r['attempts'][0]['result'])['mesh_selection_s']+r['cumulative_solve_wall_s'] for r in ref)
report['refinement']=ref
report['n_solver_calls']=72+7*12+sum(len(r['attempts']) for r in ref)
report['timing_definition']='NLP solver wall time, excluding construction, initialization and geometry checking. Refinement sums all NLP attempts.'
report['frame_definition']='Linear position/unwrapped heading interpolation at <=1 ms plus all nodes. Collision area >1e-10 m^2. Aggregate frame statistics use successful and feasible solves only.'
report['proposed']=[dict(case_id=r['case_id'],nodes=r['nodes'],solve_s=r['solve_wall_s'],duration_s=r['duration_s'],
    h_min_s=min(r['dt']),h_max_s=max(r['dt']),init_min_s=min(r['initial_dt']),init_max_s=max(r['initial_dt']),
    initialization_s=r['initialization_s'],mesh_selection_s=r['mesh_selection_s'],build_s=r['solver_build_s'],
    constraint_violation=r['max_constraint_violation'],collision_frames=r['interpolation_check']['collision_samples']) for r in rows('obca')]
(HERE/'summary.json').write_text(json.dumps(report,indent=2))

with (HERE/'case-results.csv').open('w',newline='',encoding='utf-8-sig') as f:
    fields=['method','case_id','nodes','solver_status','solver_success','feasible','accepted','solve_s','collision_frames','samples','collision_percent']
    w=csv.DictWriter(f,fieldnames=fields);w.writeheader()
    for m in methods+['refine']:
        for r in rr if m=='refine' else rows(m):
            ck=r['interpolation_check'];t=r['solve_wall_s'] if m!='refine' else ref[r['case_id']-1]['cumulative_solve_wall_s']
            w.writerow(dict(method=m,case_id=r['case_id'],nodes=r['nodes'],solver_status=r['solver_status'],solver_success=r['solver_success'],
                            feasible=r['feasible'],accepted=accepted(r),solve_s=t,collision_frames=ck['collision_samples'],samples=ck['samples'],collision_percent=ck['collision_sample_percent']))

def save(name, header, lines, footer):
    footer=footer.replace(r'\smallskip\parbox',r'\par\smallskip\parbox')
    (OUT/f'{name}.tex').write_text(header+'\n'+'\n'.join(lines)+'\n'+footer,encoding='utf-8')


save('beta-v7',r"""\begin{table}[t]
\caption{Step-Factor Study on All Twelve Cases}\label{tab:beta}
\centering\footnotesize
\begin{tabular}{@{}crrrr@{}}\toprule
$\beta$&Mean $M$&Range of $M$&Collision-free&Median (s)\\\midrule""",
 [f"{r['beta']:.1f}&{r['mean_nodes']:.1f}&{r['min_nodes']}--{r['max_nodes']}&{r['accepted']}/12&{r['median_s']:.3f}\\\\" for r in cal],
 r"""\bottomrule\end{tabular}
\par\smallskip\parbox{\columnwidth}{\footnotesize $M=N+1$. Collision-free solutions satisfy the solver, NLP-feasibility, and dense body--obstacle checks defined in the text. Medians include all twelve attempts.}
\end{table}""")
save('cases-v7',r"""\begin{table}[t]
\caption{BME--OBCA with Automatically Selected Points}\label{tab:cases}
\centering\footnotesize
\begin{tabular}{@{}rrrrrl@{}}\toprule
Case&$M$&Select (ms)&Solve (s)&$T$ (s)&$h_k$ range (s)\\\midrule""",
 [f"{r['case_id']:02d}&{r['nodes']}&{1000*r['mesh_selection_s']:.1f}&{r['solve_s']:.3f}&{r['duration_s']:.2f}&{r['h_min_s']:.3f}--{r['h_max_s']:.3f}\\\\" for r in report['proposed']],
 r"""\bottomrule\end{tabular}
\par\smallskip\parbox{\columnwidth}{\footnotesize $\beta=0.6$. All twelve runs terminate successfully, satisfy the NLP constraints, and have zero collision frames. Select measures the obstacle-aware mesh construction.}
\end{table}""")
labels={'obca':r'BME--OBCA','triangle':'Triangle Area','body':'Body--OBCA','uniform':'Uniform--OBCA','buffer':'Uniform + 2 cm','discs':'Two covering discs','motionbuffer':'Motion-radius buffer','placement':'BME--uniform placement','refine':'Check--refine'}
meshes={'obca':'Adaptive $M$','triangle':'Adaptive $M$','body':'Adaptive $M$','uniform':'Uniform $2M$','buffer':'Uniform $2M$','discs':'Uniform $2M$','motionbuffer':'Adaptive $M$','placement':'Matched $M$, free $h_k$','refine':r'Uniform $M$ to $10M$'}
method_order=['obca','placement','triangle','body','uniform','buffer','discs','motionbuffer','refine']
save('comparison-v7',r"""\begin{table*}[t]
\caption{Matched Collision-Model and Node-Placement Comparisons}\label{tab:comparison}
\centering\footnotesize
\begin{tabular}{@{}llccrrr@{}}\toprule
Method&Initial mesh&Solved&Collision-free&Collision frames (\%)&Select (ms)&Solve (s)\\\midrule""",
 [f"{labels[m]}&{meshes[m]}&{r['solver_success']}/12&{r['accepted']}/12&{r['collision_frames']:,} ({r['collision_percent']:.3f})&{1000*r['median_mesh_s']:.1f}&{r['median_s']:.3f}\\\\" for m in method_order for r in [report['methods'][m]]],
 r"""\bottomrule\end{tabular}
\par\smallskip\parbox{\textwidth}{\footnotesize Solved denotes successful or acceptable IPOPT termination. Collision-free additionally requires NLP feasibility and zero detected physical-body overlap with the original obstacles. Collision frames pool only successfully solved, NLP-feasible trajectories. Select and Solve are medians over all twelve cases. Every variant uses the selected $M$ as its count reference. Check--refine reports cumulative solve time per case and its first count-selection time.}
\end{table*}""")
save('refinement-v7',r"""\begin{table}[t]
\caption{Repeated Uniform Refinement with 20\% Node Increments}\label{tab:refinement}
\centering\footnotesize
\begin{tabular}{@{}rrrrrrc@{}}\toprule
Case&Initial $M$&Final $M$&Runs&Time (s)&Frames&Free\\\midrule""",
 [f"{r['case_id']:02d}&{r['base_nodes']}&{r['final_nodes']}&{len(r['attempts'])}&{r['cumulative_solve_wall_s']:.2f}&{r['attempts'][-1]['collision_samples']}&{'Yes' if r['accepted'] else 'No'}\\\\" for r in ref],
 r"""\bottomrule\end{tabular}
\par\smallskip\parbox{\columnwidth}{\footnotesize Time sums all attempted NLP solves. Frames counts body--obstacle overlaps in the last return, including failed solves. Free indicates a collision-free solution under the common criteria.}
\end{table}""")
print(json.dumps({'selected_beta':chosen,'methods':report['methods'],'n_solver_calls':report['n_solver_calls']},indent=2))

