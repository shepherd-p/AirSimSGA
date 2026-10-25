"""Append an explicit static result card using the verified final log, not new motion."""
import json,subprocess,cv2,shutil,sys
from pathlib import Path
from record_water_landing import Encoder,sha,imageio_ffmpeg
root=Path(__file__).resolve().parents[1];take=root/(sys.argv[1] if len(sys.argv)>1 else 'results/landing_take_01')
q=json.loads((root/'quality_report.json').read_text(encoding='utf-8-sig'));assert q['result']['success'] and q['landing']['final_supported_dwell_sec']>=3
assert Path(q['take']).resolve()==take.resolve(),'Run the motion-only video audit for this take first'
original=take/'presentation.mp4';motion=take/'presentation_motion_only.mp4'
assert not motion.exists();shutil.move(original,motion)
cap=cv2.VideoCapture(str(motion));cap.set(cv2.CAP_PROP_POS_FRAMES,int(cap.get(cv2.CAP_PROP_FRAME_COUNT))-1);ok,canvas=cap.read();cap.release();assert ok
cv2.rectangle(canvas,(0,0),(1600,68),(21,24,29),-1)
cv2.putText(canvas,'Landing verified | final recorded state',(25,44),cv2.FONT_HERSHEY_SIMPLEX,1.05,(236,240,245),2,cv2.LINE_AA)
cv2.rectangle(canvas,(0,675),(1600,900),(21,24,29),-1)
lines=[f"Continuous valid support: {q['landing']['final_supported_dwell_sec']:.1f} s | Rotor speed: 0 rad/s | Rotor thrust: 0 N",f"Final horizontal error: {q['landing']['final_horizontal_error_m']:.3f} m | USV speed: {q['boat']['final_speed_mps']:.2f} m/s",'Success criterion: verified deck contact and motor-off dwell','Static result summary from the final log; no additional simulated motion is shown.']
for i,line in enumerate(lines):cv2.putText(canvas,line,(25,720+i*45),cv2.FONT_HERSHEY_SIMPLEX,.72 if i<3 else .57,(228,235,242),2 if i<3 else 1,cv2.LINE_AA)
card=take/'verified_result_card.mp4';e=Encoder(card,1600,900,30)
for _ in range(60):e.write(canvas)
e.close();cv2.imwrite(str(take/'preview_contact.jpg'),canvas,[cv2.IMWRITE_JPEG_QUALITY,96])
concat=take/'concat_result_card.txt';concat.write_text("file '"+motion.as_posix()+"'\nfile '"+card.as_posix()+"'\n",encoding='utf-8')
subprocess.run([imageio_ffmpeg.get_ffmpeg_exe(),'-hide_banner','-loglevel','warning','-y','-f','concat','-safe','0','-i',str(concat),'-c','copy','-movflags','+faststart',str(original)],check=True)
p=take/'presentation_manifest.json';m=json.loads(p.read_text(encoding='utf-8-sig'));m['motion_only_sha256']=sha(motion);m['sha256']=sha(original);m['frames']+=60;m['same_run_result_end_card']={'frames':60,'duration_sec':2,'source':str(take/'experiment/trajectory.csv'),'source_sha256':sha(take/'experiment/trajectory.csv'),'description':'Explicit static summary of final verified log state; last recorded RGB background held, not additional simulation or new camera exposures'};p.write_text(json.dumps(m,indent=2),encoding='utf-8')
shutil.copy2(original,root/'moving_usv_landing_30fps.mp4')
print('Appended verified static result card')
