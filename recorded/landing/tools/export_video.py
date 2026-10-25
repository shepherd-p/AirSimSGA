import json,shutil,subprocess,sys
from pathlib import Path
from record_water_landing import imageio_ffmpeg
root=Path(__file__).resolve().parents[1]
take=root/sys.argv[1]
manifest=json.loads((take/'presentation_manifest.json').read_text(encoding='utf-8-sig'))
raw=json.loads((take/'camera_originals/recording_manifest.json').read_text(encoding='utf-8-sig'))
start,end=manifest['simulation_interval_sec']
offset=start-raw['cameras']['overview']['start_time_ns']/1e9
shutil.copy2(take/'presentation.mp4',root/'moving_usv_landing_30fps.mp4')
subprocess.run([imageio_ffmpeg.get_ffmpeg_exe(),'-hide_banner','-loglevel','warning','-y',
 '-ss',str(offset),'-i',str(take/'camera_originals/overview/original_rgb.mp4'),'-t',str(end-start),
 '-an','-c:v','libx264','-preset','veryfast','-crf','18','-pix_fmt','yuv420p','-movflags','+faststart',
 str(root/'fixed_camera_overview_30fps.mp4')],check=True)
for name in ['runtime_receipt.json','launch_receipt.json','protocol.json']:
 shutil.copy2(root/name,take/name)
print('Exported synchronized annotated presentation and clean fixed-camera view.')
