"""Publish English review page only from a completed, audited live run."""
import json,sys,hashlib,shutil
from pathlib import Path
root=Path(__file__).resolve().parents[1]
take=root/sys.argv[1]
q=json.loads((root/'quality_report.json').read_text(encoding='utf-8-sig'))
assert q['result']['success']
a=q['cameras']['overview']['independent_exposure_rate_trial_sim_hz'];b=q['cameras']['down']['independent_exposure_rate_trial_sim_hz']
held=q['cameras']['overview']['held_output_frames_in_trial'];frames=q['cameras']['overview']['output_frames_in_trial']
rel=take.relative_to(root).as_posix()
checks=[('Vessel travel',f"{q['boat']['travel_distance_m']:.2f} m"),('Final vessel speed',f"{q['boat']['final_speed_mps']:.3f} m/s"),('Reverse samples exceeding 1 mm',str(q['boat']['reverse_samples_over_1mm'])),('Final horizontal landing error',f"{q['landing']['final_horizontal_error_m']:.3f} m"),('Supported dwell with stopped rotors',f"{q['landing']['final_supported_dwell_sec']:.1f} s"),('Observer camera capture',f'{a:.2f} unique exposures per simulation second'),('Onboard camera capture',f'{b:.2f} unique exposures per simulation second'),('Held observer output frames',f'{held}/{frames} ({100*held/frames:.2f}%)'),('Output video',f"30 fps, {q['presentation']['duration_sec']:.2f} s, 1× simulation time"),('Observer position and orientation','Fixed, verified against exposure metadata'),('Recording queue','No dropped queued exposures or encoding errors')]
rows=''.join(f'<tr><td>{k}</td><td>{v}</td></tr>' for k,v in checks)
page=f'''<!doctype html><html lang="en"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>AirSimSGA | Moving-USV landing</title><style>body{{margin:0;background:#101924;color:#e7eef7;font:16px/1.65 Arial,sans-serif}}main{{max-width:1120px;margin:auto;padding:28px}}h1{{font-size:29px;line-height:1.3}}h2{{margin-top:30px;font-size:22px}}video{{display:block;width:100%;background:#000;border-radius:8px}}a{{color:#69cafa}}.note{{padding:16px 20px;border-left:4px solid #69cafa;background:#1b2b3b;margin:20px 0}}table{{width:100%;border-collapse:collapse}}td{{padding:10px 12px;border-bottom:1px solid #304255}}td:first-child{{width:50%;color:#bfcfe0}}.links{{display:flex;gap:18px;flex-wrap:wrap}}small{{color:#bacadd}}img{{width:100%;border-radius:6px}}</style><main><h1>AirSimSGA: UAV Landing on a Moving USV</h1><p>A live simulation rerun with continuous thrust-driven vessel motion, a fixed observer camera, small water ripples and a visible stern wake. The UAV detects the deck marker, approaches the moving deck, establishes support and stops its rotors.</p><video controls preload="metadata" poster="preview_landing.jpg"><source src="moving_usv_landing_30fps.mp4" type="video/mp4"></video><p class="links"><a href="moving_usv_landing_30fps.mp4">Annotated synchronized video</a><a href="fixed_camera_overview_30fps.mp4">Clean observer video</a></p><div class="note"><b>Motion and rendering scope.</b> Vessel motion uses planar surge–sway–yaw dynamics with mass, added inertia, Coriolis terms and drag. The hull uses a calibrated, near-level vertical anchor. Ripples and foam are visual effects driven by simulation time and vessel speed; they do not apply physical wave forces. This demonstration does not validate six-DOF wave–buoyancy coupling or fluid-flow simulation.</div><h2>Clean fixed-camera view</h2><video controls preload="metadata"><source src="fixed_camera_overview_30fps.mp4" type="video/mp4"></video><h2>Measured capture and landing results</h2><table>{rows}</table><p><small>Camera rates count unique source exposure timestamps per simulation second. They are separate from the encoded 30 fps and are not measurements of Unreal viewport fps. A slower wall-clock schedule allows denser camera sampling. Playback uses the original simulation timeline. The annotated video ends with a two-second static result card citing the final verified log; the clean observer view contains only the recorded motion. Missing exposures can be held and are explicitly recorded in the frame index. No optical-flow interpolation or synthesized vehicle motion is used.</small></p><h2>Evidence and reproducibility</h2><p class="links"><a href="quality_report.json">Video and motion audit</a><a href="{rel}/experiment_result.json">Landing result</a><a href="{rel}/experiment/trajectory.csv">Trajectories and contact states</a><a href="{rel}/experiment/commands.csv">Control commands</a><a href="protocol.json">Protocol</a><a href="MODEL_AND_RENDERING_SCOPE.md">Model and rendering scope</a><a href="build_receipt.json">Build provenance</a><a href="preservation_report.json">Preservation check</a><a href="provenance_clarification.json">Build-source field clarification</a></p><p>Original encoded RGB streams, lossless keyframes, recognition frames, frame-to-exposure indices, timestamp metadata, topic payloads, source snapshots and the resolved configuration remain in <code>{rel}</code>. Landing completion requires verified contact, valid support and stopped rotors throughout the required dwell.</p></main></html>'''
(root/'video_overview.html').write_text(page,encoding='utf-8')
for src,dst in [('preview_contact.jpg','preview_landing.jpg'),('preview_2.jpg','preview_approach.jpg')]:
 p=take/src
 if p.exists():shutil.copy2(p,root/dst)
(root/'README.txt').write_text(f'''AirSimSGA moving-USV landing — independent water and motion revision

Open video_overview.html.
Annotated video: moving_usv_landing_30fps.mp4
Clean observer: fixed_camera_overview_30fps.mp4
Completed run: {rel}
Audit: quality_report.json
Scope: MODEL_AND_RENDERING_SCOPE.md

Observer sampling: {a:.4f} unique exposures/simulation second.
Onboard sampling: {b:.4f} unique exposures/simulation second.
Output: 30 fps, 1x simulation time; no motion interpolation.
Planar 3DOF vessel dynamics and near-level deck support; water waves/foam are visual only.
The original recordings, source, original map and compiled baseline remain unchanged.

Run tools/start_simulator.ps1 with UE 5.2, then tools/verify_runtime.py.
Use the existing projectairsim310 Python environment.
Run tools/record_water_landing.py landing --output <NEW_ABSOLUTE_TAKE_DIRECTORY>.
Run tools/compose_water_landing.py landing <TAKE_DIRECTORY> --speed 1.
Run tools/export_video.py <RELATIVE_TAKE_DIRECTORY>.
Run tools/check_video.py <RELATIVE_TAKE_DIRECTORY>.
For the explicit final-state result card, run tools/append_result_card.py <RELATIVE_TAKE_DIRECTORY> once after the initial video audit, then rerun tools/check_video.py. The motion-only version is retained separately.
Run tools/audit_delivery.py <RELATIVE_TAKE_DIRECTORY>.
Run tools/publish_water_page.py <RELATIVE_TAKE_DIRECTORY>.
Do not overwrite an existing take. The CityPark resource junctions are read-only inputs; save authored resources only under Content/SGAWaterLab.
''',encoding='utf-8')
print('Published English review page')
