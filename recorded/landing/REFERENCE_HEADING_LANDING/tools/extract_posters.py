from pathlib import Path
import cv2
root=Path(__file__).resolve().parents[1];take=root/'results/landing_take_01'
for video,pairs in [(take/'presentation.mp4',[(.999,take/'preview_contact.jpg'),(.999,root/'preview_landing.jpg')]),(root/'fixed_camera_overview_30fps.mp4',[(.45,root/'preview_water_and_wake.jpg'),(.999,root/'preview_clean_landing.jpg')])]:
 cap=cv2.VideoCapture(str(video));count=int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
 for fraction,path in pairs:
  cap.set(cv2.CAP_PROP_POS_FRAMES,min(count-1,int(count*fraction)))
  ok,frame=cap.read();assert ok,str(video)
  assert cv2.imwrite(str(path),frame,[cv2.IMWRITE_JPEG_QUALITY,96])
 cap.release()
print('Saved previews from actual exported video frames')
