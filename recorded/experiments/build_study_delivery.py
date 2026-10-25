"""Build a portable result gallery from explicitly selected, audited study runs."""
import csv
import html
import json
from pathlib import Path
import shutil
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from ag.storage import ROOT,write_json

FIGURES={
    'navigation':[('scene_overview_labeled','同次运行的空地平台场景'),('navigation_panel','空中规划、地面视角与实测轨迹'),
        ('navigation_map','传感器观测地图、规划与执行路径'),('air_ground_trajectories_3d','无人机与无人车三维轨迹'),('navigation_metrics','导航误差')],
    'tracking':[('scene_overview_labeled','同次运行的无人机、跟踪车与目标车'),('occlusion_sequence','同步遮挡前、遮挡中、重新捕获'),
        ('tracking_metrics_panel','轨迹、跟踪间距误差和距离'),('target_estimation_error','观测与融合估计误差'),
        ('ugv_heading_error','无人车视线方向误差'),('tracking_visibility','空地互补可见性'),('tracking_map','实景底图与实测轨迹投影')],
    'formation':[('scene_overview_labeled','同一帧中的四架无人机和三辆无人车'),('trajectories_3d','七平台实测三维轨迹'),
        ('formation_top','参考路线与实际轨迹'),('formation_separation','实测最小平台间距')]}
TITLES={'navigation':'实验二：空中辅助地面导航','tracking':'实验三：空地协同目标跟踪','formation':'实验四：四机三车并行运行'}


def build():
    selection=json.loads((ROOT/'verification/cooperative_selected_runs.json').read_text(encoding='utf-8'))
    out=ROOT/'delivery/cooperative_studies';out.mkdir(parents=True,exist_ok=True)
    performance=[];summary={};sections=[]
    for task,relative in selection['successful_runs'].items():
        run=ROOT/relative;results=run/'study_results';metrics=json.loads((results/'metrics.json').read_text(encoding='utf-8'))
        audit=json.loads((results/'audit.json').read_text(encoding='utf-8'))
        if audit['status']!='passed':raise ValueError('Audit is not passed: '+task)
        manifest=json.loads((run/'manifest.json').read_text(encoding='utf-8'))
        summary[task]={'run':relative,'manifest':manifest,'metrics':metrics,'audit':audit}
        target=out/task;target.mkdir(exist_ok=True)
        for filename in ('metrics.json','audit.json','figure_sources.json','performance_table.md','sensor_rates.csv','scene_overview.png'):
            shutil.copy2(results/filename,target/filename)
        cards=[]
        for name,caption in FIGURES[task]+[('performance','本次实际负载下的性能')]:
            shutil.copy2(results/(name+'.png'),target/(name+'.png'))
            if (results/(name+'.pdf')).exists():shutil.copy2(results/(name+'.pdf'),target/(name+'.pdf'))
            cards.append(f'<figure><a href="{task}/{name}.png"><img loading="lazy" src="{task}/{name}.png" alt="{html.escape(caption)}"></a><figcaption>{caption}</figcaption></figure>')
        sections.append(f'<section id="{task}"><h2>{TITLES[task]}</h2><p>运行记录：<code>{relative}</code> · 数据核查通过。点击图片查看原尺寸。</p>'+''.join(cards)+'</section>')
        stats=metrics['performance']
        performance.append({'task':task,'controlled_platforms':len(manifest['path_lengths_m']),
            **{key:stats[key]['mean'] for key in ('fps','memory_mb','cpu_total_percent','real_time_factor')},
            'camera_bundles':audit['checks']['cameras']['original_payload_bundles'],
            'lidar_packets':audit['checks']['lidar']['unique_packets'],
            'pose_collision_records':audit['checks']['pose_collision_topics']['records']})
    with (out/'performance_comparison.csv').open('w',newline='',encoding='utf-8') as stream:
        writer=csv.DictWriter(stream,fieldnames=list(performance[0]));writer.writeheader();writer.writerows(performance)
    fig,ax=plt.subplots(figsize=(13.4,3.5));ax.axis('off')
    data=[[r['task'].title(),str(r['controlled_platforms']),f'{r["fps"]:.2f}',f'{r["memory_mb"]:.1f}',f'{r["cpu_total_percent"]:.1f}',f'{r["real_time_factor"]:.3f}'] for r in performance]
    table=ax.table(cellText=data,colLabels=['Study','Platforms','Mean FPS','UE memory (MiB)','UE CPU total (%)','Sim/wall ratio'],
                   cellLoc='center',loc='center',colWidths=[.19,.1,.12,.22,.22,.15])
    table.auto_set_font_size(False);table.set_fontsize(11);table.scale(1,2)
    for (row,col),cell in table.get_celld().items():
        cell.set_edgecolor('#cfd7df')
        if row==0:cell.set_facecolor('#e8eef5');cell.set_text_props(weight='bold')
    ax.set_title('Measured performance with active sensors and evidence recording',fontsize=15,pad=10)
    fig.text(.5,.05,'Same existing offscreen UE process; different sensor/recorder workloads. One run per task. 100% CPU = one core.',ha='center',fontsize=10)
    fig.savefig(out/'performance_comparison.png',dpi=300,bbox_inches='tight');fig.savefig(out/'performance_comparison.pdf',bbox_inches='tight');plt.close(fig)
    nav=summary['navigation']['metrics'];tracking=summary['tracking']['metrics'];formation=summary['formation']['metrics']
    overview=f'''<ul><li>导航：实际行驶 {nav['ugv_path_length_m']:.2f} m；终点误差 {nav['goal_error_m']:.3f} m；路径横向 RMSE {nav['cross_track_error_m']['rmse']:.3f} m。</li>
<li>跟踪：目标绕行 {tracking['target_path_length_m']:.2f} m；记录 {tracking['ground_only_under_canopy_samples']} 个顶棚下地面单独可见控制样本；无人机重新捕获成功。</li>
<li>七平台：四机各两圈方形、三车各一圈圆形；含初始样本的最小距离 {formation['minimum_pairwise_distance_m']:.3f} m。</li></ul>'''
    page='''<!doctype html><html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1">
<title>ProjectAirSim 空地协同实验结果</title><style>
body{font:17px/1.7 "Segoe UI","Microsoft YaHei",sans-serif;color:#182b40;background:#eef2f6;margin:0}main{max-width:1400px;margin:auto;padding:32px}
h1{font-size:32px}h2{font-size:25px}section{background:white;border-radius:8px;padding:24px;margin:28px 0}img{max-width:100%;height:auto}figure{margin:26px 0;border:1px solid #e2e7ed;padding:10px}figcaption{font-size:16px;text-align:center}a{color:#215cc2}code{overflow-wrap:anywhere}nav a{margin-right:24px}
</style><main><h1>ProjectAirSim 空地协同实验结果</h1><p>基于 CityPark 实际仿真运行。原始场景图来自同次运行的相机传感器；彩色标框和轨迹投影为明示的后处理标注。三维轨迹与误差使用保存的物理位姿时间戳。</p>
<nav><a href="#navigation">空中辅助导航</a><a href="#tracking">协同跟踪</a><a href="#formation">四机三车</a></nav>'''+overview+''.join(sections)+'''
<section><h2>性能与证据范围</h2><img src="performance_comparison.png" alt="实测性能表"><p>这是不同传感器和记录负载下的单次测量，不是固定负载的扩展性对照。相机保留按间隔选取的同步原始帧；LiDAR 保留过滤后的测量回波；位姿及碰撞主题保留采集窗口内的原始消息。感知采用仿真分割标签与深度基线，定位使用仿真位姿。导航为平坦广场路线；遮挡物是运行时真实几何顶棚；七平台为集中控制的并行参考路线。</p>
<p>详细指标与图像来源见各目录的 metrics.json、audit.json、figure_sources.json。完整证据、处理源码和失败试跑另存于独立校验归档。</p></section></main></html>'''
    (out/'index.html').write_text(page,encoding='utf-8');write_json(out/'summary.json',summary)
    print(json.dumps({'gallery':str(out/'index.html'),'performance':performance},indent=2),flush=True)


if __name__=='__main__':build()
