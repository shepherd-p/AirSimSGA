"""Collect a dated, lossless figure snapshot; leave source experiments untouched."""
from pathlib import Path
import csv
import hashlib
import html
import json
import re
import shutil
from collections import Counter
from datetime import datetime
from urllib.parse import quote
from PIL import Image, ImageOps, ImageDraw

ROOT = Path(__file__).resolve().parent
if ROOT.name == '来源说明':
    ROOT = ROOT.parent.parent
PAPER = ROOT / 'paper_RAL_updated_20261001'
DEST = ROOT / 'ALL_EXPERIMENT_FIGURES_20261001'
EXT = {'.png', '.jpg', '.jpeg', '.pdf', '.svg'}
GROUPS = {
    '00_论文正式用图': '最新正文实际引用的 Fig1–Fig8；PNG 用于预览，PDF 用于排版，架构另附 SVG。',
    '01_总览底板与独立子图': '最新底板及单独子图。底板为用户提供的展示素材，不代表新增三域同步实验。',
    '02_空地协同建图': 'CityPark 封存结果：原场景、高清点云、彩色表面、轨迹。',
    '03_空中辅助道路导航': '当前正文的 City Sample 三路口实验；原始截图与标注图分别保存。UGV 画面是车辆外部观察视角。',
    '04_多机跟踪与出口围堵': '成功运行截图与诊断视角分开。诊断运行后段发生碰撞，不能作为成功运行的连续帧证据。',
    '05_移动无人艇降落': '已封存移动甲板降落：场景、识别序列、三维轨迹及误差曲线。',
    '06_静态甲板降落对照': '已封存静态甲板对照：场景、识别序列、轨迹及误差曲线。',
    '07_多平台并行运行': 'Bellevue 中六次 UAV/UGV 规模实验；不含 USV，不属于 City Sample 运行。',
    '08_路障绕行补充': '保留短路段及街区路障实验供后续选用，不替代当前正文的三路口导航。',
    '09_历史城市实验补充': '旧版城市实验与汇总图，仅作历史补充，不作为当前正文首选图。',
}
records = []
seen = set()

def digest(p):
    sha = hashlib.sha256()
    with p.open('rb') as f:
        for block in iter(lambda: f.read(1024 * 1024), b''):
            sha.update(block)
    return sha.hexdigest()

def add(source, group, sub='', name=None, note=''):
    source = Path(source)
    if not source.is_file():
        raise FileNotFoundError(source)
    relative = Path(group) / sub / (name or source.name)
    if relative.as_posix() in seen:
        raise ValueError(f'Duplicate destination: {relative}')
    seen.add(relative.as_posix())
    target = DEST / relative
    target.parent.mkdir(parents=True, exist_ok=True)
    sha = digest(source)
    if target.exists() and digest(target) != sha:
        raise RuntimeError(f'Refusing to overwrite changed snapshot: {target}')
    if not target.exists():
        shutil.copy2(source, target)
    assert digest(target) == sha
    dimensions = ''
    if source.suffix.lower() in {'.png', '.jpg', '.jpeg'}:
        with Image.open(target) as im:
            dimensions = f'{im.width} × {im.height}'
            im.verify()
    records.append(dict(file=relative.as_posix(), group=group, source=str(source),
                        sha256=sha, bytes=target.stat().st_size,
                        dimensions=dimensions, note=note))

def folder(source, group, sub='', note=''):
    source = Path(source)
    if not source.is_dir():
        raise FileNotFoundError(source)
    for p in sorted(source.iterdir()):
        if p.is_file() and p.suffix.lower() in EXT:
            add(p, group, sub, note=note)

def build():
    DEST.mkdir(exist_ok=True)
    g = list(GROUPS)
    stems = re.findall(r'includegraphics\[[^]]*\]\{(Fig[^}]+)\.pdf\}',
                       (PAPER / 'airsimsga_ral.tex').read_text(encoding='utf-8'))
    assert len(stems) == 8 and len(set(stems)) == 8
    for stem in stems:
        for ext in ('.png', '.pdf'):
            add(PAPER / 'Figure_v2' / (stem + ext), g[0], note='最新正文正式引用')
    add(PAPER / 'Figure_v2/Fig2_architecture.svg', g[0], note='可编辑矢量架构图')
    add(PAPER / 'Assets/overview_sea_ground_air.png', g[1], '总览底板', note='用户提供的平台展示底板')
    for p in sorted((PAPER / 'Panels').iterdir()):
        # These small legacy plots have been superseded by direct vector axes in current figures.
        if p.suffix.lower() in EXT and p.name not in {'landing_trajectory.png', 'map_trajectory.png'}:
            add(p, g[1], '独立子图', note='论文拼图所用素材；更高分辨率原图见对应实验目录')
    mapping = ROOT / 'archives/plaza_evidence_20260928_v1/data'
    folder(mapping / 'mapping/20260928_000727_997657/plaza_results', g[2], '重建与轨迹')
    folder(mapping / 'scene_overview/20260928_014618', g[2], '原场景与分割')
    nav = ROOT / 'outputs/citysample_aerial_route_navigation/20261001_023924_720614_air_ground'
    folder(nav, g[3], '原始截图')
    folder(nav / 'derived', g[3], '标注图')
    folder(ROOT / 'outputs/citysample_pursuit/20260930_041328_715126_seed0', g[4], '成功运行_原始截图')
    folder(ROOT / 'outputs/citysample_pursuit/20260930_055549_928810_seed0', g[4],
           '诊断运行_目标视角', note='独立诊断运行，后段碰撞；不可与成功运行混作连续序列')
    for name in ['track_overview.jpg', 'track_uav_target.jpg', 'track_ugv_target.jpg']:
        add(PAPER / 'Panels' / name, g[4], '当前标注图',
            note='overview 对应成功运行；UAV/UGV 目标视角来自独立诊断运行')
    add(ROOT / 'paper_RAL/figures/track_target_views.png', g[4], '诊断运行_目标视角',
        note='诊断运行视角序列拼图，不是成功运行的连续记录')
    landing = ROOT / 'archives/moving_deck_verified_20260928_v2/suite'
    folder(landing / 'trial_001_v0.50_predictive/derived', g[5])
    folder(landing / 'trial_002_v0.00_predictive/derived', g[6])
    for run in sorted((ROOT / 'outputs/urban_scale').iterdir()):
        if run.is_dir() and run.name.startswith('20260929_'):
            folder(run, g[7], run.name)
    add(ROOT / 'delivery/urban/platform_scale_results.png', g[7])
    for run in ['20260930_042724_885397_air', '20260930_183535_434159_air']:
        folder(ROOT / 'outputs/citysample_navigation' / run, g[8], '短路段/' + run)
    district = ROOT / 'outputs/citysample_district_navigation/20260930_084659_347353_air_ground'
    folder(district, g[8], '街区路障/原始截图', note='历史街区路障方案，非当前正文导航主实验')
    folder(district / 'derived', g[8], '街区路障/标注图', note='历史街区路障方案')
    old = json.loads((ROOT / 'ALL_EXPERIMENT_FIGURES/figure_sources.json').read_text(encoding='utf-8-sig'))
    for item in old:
        if item['group'] in {'navigation_air', 'navigation_ground', 'pursuit_south', 'pursuit_north', 'urban_summary'}:
            if Path(item['source']).name == 'platform_scale_results.png':
                continue
            src = Path(item['source'])
            add(src, g[9], item['group'], name=item['file'], note='历史实验补充，不作为当前正文首选图')
    for p in sorted((ROOT / 'delivery/urban').glob('*.png')):
        if p.name != 'platform_scale_results.png':
            add(p, g[9], '历史汇总图', note='旧版城市实验结果，不替代当前 City Sample 实验')
    add(ROOT / 'outputs/urban_pursuit/20260929_011707_027572_seed1/scene_start.png',
        g[9], 'Bellevue原场景', note='当前 Fig3 的 Bellevue 场景来源')
    # Copy provenance documents separately, never present them as experiment figures.
    meta = DEST / '来源说明'
    meta.mkdir(exist_ok=True)
    for p in sorted((PAPER / 'Figure_v2').glob('*.json')):
        shutil.copy2(p, meta / p.name)
    for name in ['EVIDENCE.md', 'containment_overlay_manifest.json', 'verification_report.json']:
        shutil.copy2(PAPER / name, meta / name)
    shutil.copy2(nav / 'derived/annotation_manifest.json', meta / 'navigation_annotation_manifest.json')
    shutil.copy2(__file__, meta / Path(__file__).name)
    write_indexes()

def write_indexes():
    manifest = DEST / 'figure_sources.json'
    manifest.write_text(json.dumps(records, ensure_ascii=False, indent=2), encoding='utf-8')
    with (DEST / '图片来源索引.csv').open('w', encoding='utf-8-sig', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=list(records[0]))
        writer.writeheader()
        writer.writerows(records)
    thumbs = DEST / '_预览缩略图'
    thumbs.mkdir(exist_ok=True)
    cards, poster_images = [], []
    for i, rec in enumerate(records):
        path = DEST / rec['file']
        if path.suffix.lower() not in {'.png', '.jpg', '.jpeg'}:
            continue
        with Image.open(path) as im:
            im = im.convert('RGB')
            im.thumbnail((560, 350), Image.Resampling.LANCZOS)
            thumb = thumbs / f'{i:03d}.jpg'
            im.save(thumb, quality=86)
            if rec['group'].startswith('00_'):
                poster_images.append((path.stem, im.copy()))
        url = quote(rec['file'])
        others = []
        for ext in ['.pdf', '.svg']:
            counterpart = path.with_suffix(ext)
            if counterpart.exists():
                others.append(f'<a href="{quote(counterpart.relative_to(DEST).as_posix())}">{ext[1:].upper()}</a>')
        label = html.escape(path.name)
        cards.append((rec['group'], f'<article data-search="{html.escape(rec["file"], quote=True)}">'
                      f'<a href="{url}"><img loading="lazy" src="{quote(thumb.relative_to(DEST).as_posix())}" alt="{label}"></a>'
                      f'<b>{label}</b><small>{html.escape(str(Path(rec["file"]).parent))}</small>'
                      f'<small>{rec["dimensions"]} · 原尺寸原格式复制</small><p>{html.escape(rec["note"])}</p>'
                      f'<a href="{url}">打开高清图</a> {" · ".join(others)}</article>'))
    sections = []
    for group, desc in GROUPS.items():
        sections.append(f'<section id="{group}"><h2>{group}</h2><p>{desc}</p><div class="grid">'
                        + ''.join(card for gr, card in cards if gr == group) + '</div></section>')
    nav = ' · '.join(f'<a href="#{g}">{g[3:]}</a>' for g in GROUPS)
    page = '''<!doctype html><html lang="zh-CN"><meta charset="utf-8"><title>AirSimSGA 最新统一图片目录</title>
<style>body{font:15px/1.6 "Segoe UI","Microsoft YaHei",sans-serif;background:#f4f6fa;color:#17263a;margin:30px auto;max-width:1600px;padding:0 22px}header{background:white;padding:24px;border-radius:12px}a{color:#145eb2}nav{line-height:2.3}input{padding:12px;width:min(600px,90%);font:inherit;margin-top:12px}section{margin-top:35px}.grid{display:grid;grid-template-columns:repeat(auto-fill,minmax(330px,1fr));gap:16px}article{background:white;border:1px solid #d7dfeb;border-radius:8px;padding:14px;overflow-wrap:anywhere}article img{width:100%;height:230px;object-fit:contain;background:#edf1f6}b,small{display:block}small{color:#586b83}article p{font-size:13px;min-height:20px}.hidden{display:none}</style>
<header><h1>AirSimSGA · 最新统一图片目录</h1><p>2026-10-01｜正文正式图、高清原图、标注图与历史补充。点击缩略图打开完整原图；缩略图仅用于浏览。</p>
<p><a href="00_论文正式用图/">论文正式用图 Fig1–Fig8</a> · <a href="图片来源索引.csv">图片来源索引 CSV</a> · <a href="figure_sources.json">路径与 SHA-256</a></p><nav>''' + nav + '''</nav><input id="q" placeholder="搜索文件名、实验分类、视角……"></header>''' + ''.join(sections) + '''
<script>document.querySelector('#q').addEventListener('input',e=>{let q=e.target.value.toLowerCase();document.querySelectorAll('article').forEach(a=>a.classList.toggle('hidden',!a.dataset.search.toLowerCase().includes(q)));});</script></html>'''
    (DEST / '打开图片总览.html').write_text(page, encoding='utf-8')
    poster = Image.new('RGB', (1200, 4 * 400), '#edf1f6')
    draw = ImageDraw.Draw(poster)
    for i, (label, im) in enumerate(poster_images):
        x, y = (i % 2) * 600, (i // 2) * 400
        tile = ImageOps.contain(im, (570, 350), Image.Resampling.LANCZOS)
        poster.paste(tile, (x + (600-tile.width)//2, y + 20 + (350-tile.height)//2))
        draw.text((x+20, y+375), label, fill='#17263a')
    poster.save(DEST / '论文八张图预览.jpg', quality=92)
    counts = Counter(r['group'] for r in records)
    size = sum(r['bytes'] for r in records)
    lines = ['# AirSimSGA 最新统一图片目录', '', '整理日期：2026-10-01。以 paper_RAL_updated_20261001 的当前正文为准。', '',
             '**先打开 `打开图片总览.html`；论文排版只取 `00_论文正式用图`。**', '',
             f'共 {len(records)} 个图片/矢量文件（同图 PNG/PDF/SVG 分别计数），原尺寸复制共 {size/1024**2:.1f} MiB。', '',
             '| 目录 | 文件数 | 内容 |', '|---|---:|---|']
    lines += [f'| {g} | {counts[g]} | {desc} |' for g, desc in GROUPS.items()]
    lines += ['', '## 使用与溯源', '',
              '- 正式用图共 8 张：每张 PNG + PDF，Fig2 另有 SVG。旧论文草稿、旧总览和旧架构未列入正式用图。',
              '- 高清文件未缩放、未重编码；每个文件均核对源文件与副本 SHA-256。预览缩略图单独存放。',
              '- `figure_sources.json` 与 `图片来源索引.csv` 逐图记录原始绝对路径、大小、分辨率及 SHA-256。',
              '- `来源说明` 保存原有作图来源清单和实验核验说明；这不是一次新的实验验证。',
              '- 最新 Fig1 使用用户提供的湖泊底板；当前 Fig4/5/7/8 曲线位于整图 PDF 内，优先使用矢量版本。',
              '- 独立子图有论文裁剪版；未裁剪的更高清文件在各实验目录内。',
              '- 跟踪诊断视角与成功运行已分开，勿将不同运行拼成同一连续实验。',
              '- 旧统一目录及封存实验原件保持原样。此目录只汇总图片；原始点云、传感器数组、轨迹 CSV、事件日志仍在来源目录。',
              '- 本目录为 2026-10-01 快照，不会随原文件自动更新。', '']
    (DEST / 'README.md').write_text('\n'.join(lines), encoding='utf-8-sig')
    report = {'created_at': datetime.now().astimezone().isoformat(), 'files': len(records),
              'raster_images': len(cards), 'source_copy_bytes': size, 'groups': dict(counts),
              'source_destination_hashes_verified': True, 'paper_figure_count': 8}
    (DEST / '整理核验报告.json').write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps(report, ensure_ascii=False, indent=2))

if __name__ == '__main__':
    build()
