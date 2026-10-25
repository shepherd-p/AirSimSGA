"""Verify and index four completed experiment recordings."""
from pathlib import Path
import argparse
import html
import json
import shutil
import cv2
import numpy as np
from record_experiment import sha, write_json

ITEMS = [('mapping', '01_空地协同三维建图', 2),
         ('navigation', '02_空中辅助道路导航', 4),
         ('tracking', '03_多机跟踪与出口围堵', 1),
         ('landing', '04_移动无人艇降落', 1)]


def inspect(path):
    cap = cv2.VideoCapture(str(path))
    frames = round(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    fps = cap.get(cv2.CAP_PROP_FPS)
    size = [round(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), round(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))]
    samples = []
    images = []
    for ratio in [.1, .35, .65, .9]:
        cap.set(cv2.CAP_PROP_POS_FRAMES, min(frames-1, round(frames*ratio)))
        ok, img = cap.read()
        if not ok:
            raise RuntimeError(f'Video decode failed: {path}')
        roi = img[70:660, :1200]
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        if np.std(gray) < 8:
            raise RuntimeError(f'Blank scene video: {path}')
        samples.append({'ratio': ratio, 'scene_mean': float(gray.mean()), 'scene_std': float(gray.std())})
        images.append(cv2.resize(gray, (320, 180)))
    cap.release()
    motion = [float(np.abs(a.astype(float)-b.astype(float)).mean()) for a,b in zip(images[:-1], images[1:])]
    if max(motion) < .5:
        raise RuntimeError(f'No moving scene content: {path}')
    return {'frames': frames, 'fps': fps, 'dimensions': size, 'duration_sec': frames/fps,
            'samples': samples, 'sample_scene_changes': motion, 'sha256': sha(path)}


def main():
    p = argparse.ArgumentParser()
    p.add_argument('root', type=Path)
    p.add_argument('--mapping', type=Path, required=True)
    p.add_argument('--navigation', type=Path, required=True)
    p.add_argument('--tracking', type=Path, required=True)
    p.add_argument('--landing', type=Path, required=True)
    a = p.parse_args()
    results, cards = [], []
    for task, name, speed in ITEMS:
        folder = getattr(a, task)
        link = folder/'experiment_link.json'
        experiment = Path(json.loads(link.read_text())['path']) if link.exists() else folder/'experiment'
        result_file = experiment/('manifest.json' if task in {'mapping','landing'} else 'summary.json')
        result = json.loads(result_file.read_text())
        if result['status'] != 'completed' or (task == 'landing' and not result.get('success')):
            raise RuntimeError(f'Experiment was not completed: {task}, {result.get("status")}')
        if result.get('error') or result.get('collision_events') or result.get('collision_count', 0):
            raise RuntimeError(f'Experiment has recorded errors or collisions: {task}')
        if task == 'tracking' and not result.get('target_blocked'):
            raise RuntimeError('Tracking did not establish observed containment')
        originals = json.loads((folder/'camera_originals'/'recording_manifest.json').read_text())
        if originals['queue_drops'] or originals['errors']:
            raise RuntimeError(f'Recording reports lost frames or encoding errors: {task}')
        presentation = folder/('presentation.mp4' if speed == 1 else f'presentation_{speed:g}x.mp4')
        report = inspect(presentation)
        presentation_manifest = presentation.with_name(presentation.stem+'_manifest.json')
        metadata = json.loads(presentation_manifest.read_text())
        if metadata['sha256'] != report['sha256'] or metadata['frames'] != report['frames']:
            raise RuntimeError(f'Presentation differs from its recorded manifest: {task}')
        for key, camera in originals['cameras'].items():
            if sha(folder/'camera_originals'/camera['file']) != camera['sha256']:
                raise RuntimeError(f'Original camera video hash differs: {task}/{key}')
        target = a.root/(name+'.mp4')
        if target.exists() and sha(target) != sha(presentation):
            raise RuntimeError('Refusing to overwrite different delivery video: '+str(target))
        if not target.exists():
            shutil.copy2(presentation, target)
        preview = a.root/(name+'_预览.jpg')
        shutil.copy2(folder/'preview_3.jpg', preview)
        report.update(task=task, file=target.name, recording_folder=str(folder),
                      experiment_source=str(experiment), playback_speed=speed, experiment_status=result['status'],
                      experiment_result=str(result_file), experiment_result_sha256=sha(result_file),
                      presentation_manifest=str(presentation_manifest),
                      original_camera_count=len(originals['cameras']), original_camera_hashes_verified=True)
        results.append(report)
        duration = f'{int(report["duration_sec"]//60)}:{int(report["duration_sec"]%60):02d}'
        cards.append(f'<section><h2>{name[3:]}</h2><video controls preload="metadata" poster="{preview.name}" src="{target.name}"></video>'
                     f'<p>{duration} · 1600 × 900 · H.264 · {speed}倍速</p>'
                     f'<p><a href="{target.name}">打开视频</a> · <a href="{folder.relative_to(a.root).as_posix()}/camera_originals/">各视角完整原始录像与时间戳</a></p></section>')
    write_json(a.root/'视频核验与来源.json', results)
    page = '<!doctype html><html lang="zh-CN"><meta charset="utf-8"><title>AirSimSGA 四类实验视频</title><style>body{font:16px/1.7 "Segoe UI","Microsoft YaHei",sans-serif;background:#f3f5f8;color:#183047;margin:30px auto;max-width:1300px;padding:0 20px}section{margin:35px 0;border-top:1px solid #bdc9d6;padding-top:15px}video{display:block;width:100%;aspect-ratio:16/9;background:#111}a{color:#1465ae}p{margin:8px 0}</style><h1>AirSimSGA 四类实验视频</h1><p>2026-10-02 真实仿真运行录制。场景、多视角与轨迹来自各视频对应的同一次运行；倍速明确标注。</p>'+''.join(cards)+'</html>'
    (a.root/'打开视频总览.html').write_text(page, encoding='utf-8')
    lines = ['# AirSimSGA 四类仿真实验视频', '', '录制日期：2026-10-02。先打开 `打开视频总览.html`。', '',
             '| 展示视频 | 时长 | 倍速 |', '|---|---:|---:|']
    lines += [f'| {r["file"]} | {r["duration_sec"]:.1f} s | {r["playback_speed"]}x |' for r in results]
    lines += ['', '每类实验目录中的 camera_originals 保存各相机完整 H.264 录像、曝光元数据、视频帧与相机时间戳索引、定期无损 PNG 原帧。',
              '', '正式展示版为本目录根部四个中文命名 MP4。各 take 目录保留本次录制过程与诊断，中止录像不作为成功实验交付。',
              '', '展示视频仅作排版与已标明的时间压缩；未生成或插值载具运动。全局标签是记录位姿投影，不是视觉检测输出。跟踪机载框来自分割，降落甲板框来自 AprilTag 图像检测。',
              '', '视频记录是新的重复运行；原论文封存数据保持独立。各视频对应的当前运行路径及状态见 视频核验与来源.json。',
              '', '跟踪录像使用独立的四分之一墙钟速度配置，5 ms 物理步长与原控制算法保留；展示按仿真时间播放。车辆分割编号独立预留，环境交通在地图加载前关闭。配置和启动参数保存在对应录制目录。',
              '', '模型范围：导航和本次跟踪录像均采用现有 FastPhysics 简单驱动及中心接触隔离链接，完整轮地接触反馈未验证。本次跟踪的接触配置与九月原封存实验不同，实际加载配置和辅助源码已保存。移动艇为独立构建中的运动与甲板接触模型，本次录像不构成完整水动力模型验证。',
              '', '文件完整性、画面解码、非空场景及画面变化均已核验。哈希只能验证文件一致性，不替代第三方认证。', '']
    (a.root/'README.md').write_text('\n'.join(lines), encoding='utf-8-sig')
    print(json.dumps(results, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
