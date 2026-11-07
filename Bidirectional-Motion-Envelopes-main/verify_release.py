from pathlib import Path
import hashlib,json

R=Path(__file__).resolve().parents[1]
manifest=json.loads((R/'experiments/release-manifest.json').read_text(encoding='utf-8'))
for name,record in manifest['files'].items():
    p=R/name
    assert p.is_file(),name
    assert p.stat().st_size==record['bytes'],name
    assert hashlib.sha256(p.read_bytes()).hexdigest()==record['sha256'],name
index=json.loads((R/'experiments/experiment-index.json').read_text(encoding='utf-8'))
total=0
for stage,group in index['groups'].items():
    rows=json.loads((R/group['index']).read_text(encoding='utf-8'))
    assert len(rows)==group['attempts']
    for row in rows:
        for key in ['input','result','log','dense_collision_data']:
            assert (R/row[key]).is_file(),row[key]
        log=(R/row['log']).read_text(encoding='utf-8',errors='replace')
        assert 'running with linear solver ma27' in log,row['log']
    total+=len(rows)
assert total==217
print(json.dumps(dict(verified_files=len(manifest['files']),current_attempts=total,
    all_current_logs_ma27=True,all_stage_links_present=True)))
