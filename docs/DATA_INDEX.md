# 当前实验数据索引

下列路径相对于 `F:/EpicGamesEnv`。

| 用途 | 保留目录 |
|---|---|
| 论文三维建图，完整封存数据及源码快照 | `ProjectAirSim_AG_Experiments/archives/plaza_evidence_20260928_v1` |
| 建图工作数据及派生结果 | `ProjectAirSim_AG_Experiments/outputs/mapping/20260928_000727_997657` |
| 视频建图原始相机、传感器与轨迹 | `ProjectAirSim_AG_Experiments/EXPERIMENT_VIDEOS_20261002/01_plaza_mapping_take2` |
| 三路口导航，478.47 m | `ProjectAirSim_AG_Experiments/outputs/citysample_aerial_route_navigation/20261002_222236_038213_air_ground` |
| 导航完整相机录像与时间戳 | `ProjectAirSim_AG_Experiments/EXPERIMENT_VIDEOS_20261002/02_citysample_navigation` |
| 2 UAV + 2 拦截车 + Target | `ProjectAirSim_AG_Experiments/outputs/citysample_pursuit/20261002_234725_440573_seed0` |
| 围堵原始相机录像与时间戳 | `ProjectAirSim_AG_Experiments/EXPERIMENT_VIDEOS_20261002/03_citysample_tracking_take7` |
| 最终移动无人艇降落及完整相机/识别/控制输入 | `LandingWaterLab/REFERENCE_HEADING_LANDING/results/landing_take_01` |
| 降落方向、水面、录像及传感器输入验证 | `LandingWaterLab/REFERENCE_HEADING_LANDING` 根目录报告 |
| 当前降落标定及采集依据 | `ProjectAirSim_AG_Experiments/outputs/landing/20260928_062726_457859` |
| Bellevue 1+1、2+2、3+3 各两次编队运行 | `ProjectAirSim_AG_Experiments/outputs/urban_scale` |
| 论文场景图的 Bellevue 来源会话 | `ProjectAirSim_AG_Experiments/outputs/urban_pursuit/20260929_011707_027572_seed1` |
| 广场同框场景图 | `ProjectAirSim_AG_Experiments/outputs/scene_overview/20260928_014618`，封存副本亦包含 |

封存建图与工作目录是独立副本，未把封存文件硬链接到可写工作目录。校验依据：`maintenance/preserved_file_hashes.json`；清理后复核：`maintenance/preserved_after_cleanup.json`。
