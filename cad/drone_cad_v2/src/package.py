"""Package deterministic project deliverables, excluding environments and caches."""
from pathlib import Path
import zipfile
ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'output'
with zipfile.ZipFile(OUT/'Manufacturing.zip','w',zipfile.ZIP_DEFLATED,compresslevel=6) as z:
    for folder in ['3mf','stl','step','reports']:
        for f in sorted((OUT/folder).glob('*')):
            if f.is_file() and f.suffix not in ['.log','.txt'] and 'VISUAL_ONLY' not in f.name:
                z.write(f,f.relative_to(OUT))
    z.write(ROOT/'README.md','README.md')
with zipfile.ZipFile(OUT/'Source_and_reports.zip','w',zipfile.ZIP_DEFLATED,compresslevel=6) as z:
    for folder in ['src','config','tests']:
        for f in sorted((ROOT/folder).glob('*')):
            if f.is_file(): z.write(f,f.relative_to(ROOT))
    for f in [ROOT/'README.md',ROOT/'build.ps1',ROOT/'requirements.txt',ROOT/'requirements-lock.txt']:
        z.write(f,f.relative_to(ROOT))
    for f in sorted((OUT/'reports').glob('*')):
        if f.is_file() and f.suffix in ['.md','.json','.xml']: z.write(f,f.relative_to(ROOT))
print('Created Manufacturing.zip and Source_and_reports.zip')
