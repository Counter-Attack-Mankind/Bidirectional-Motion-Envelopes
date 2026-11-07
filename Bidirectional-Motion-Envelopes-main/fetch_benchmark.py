from pathlib import Path
import io,shutil,urllib.request,zipfile

HERE=Path(__file__).resolve().parent
COMMIT='476ef8573d9e22578be5c3d42514a9a29f27d07a'
DESTINATION=HERE/'source/CommonParking'
if DESTINATION.exists():
    raise SystemExit('Benchmark directory already exists; it was not overwritten.')
url=f'https://codeload.github.com/libai1943/CommonParking/zip/{COMMIT}'
request=urllib.request.Request(url,headers={'User-Agent':'Bidirectional-Motion-Envelopes'})
with urllib.request.urlopen(request,timeout=120) as response:
    archive=zipfile.ZipFile(io.BytesIO(response.read()))
prefix=f'CommonParking-{COMMIT}/'
DESTINATION.mkdir(parents=True)
for item in archive.infolist():
    if item.is_dir() or not item.filename.startswith(prefix):continue
    relative=Path(item.filename[len(prefix):])
    target=(DESTINATION/relative).resolve()
    if not target.is_relative_to(DESTINATION.resolve()):raise ValueError('Invalid archive path.')
    target.parent.mkdir(parents=True,exist_ok=True)
    with archive.open(item) as src,target.open('wb') as dst:shutil.copyfileobj(src,dst)
print(DESTINATION)
