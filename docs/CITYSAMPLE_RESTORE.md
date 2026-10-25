# City Sample 后续恢复

City Sample 工程已由作者删除，本轮未再删除地图或引擎。当前论文和视频不依赖重新启动场景即可阅读。

需要重跑城市导航或跟踪时：

1. 重新获取匹配本次实验的 Epic City Sample。当前启动脚本使用 **UE 5.2** 和 `Big_City_LVL`；不同地图版本的道路 lane ID 可能变化。
2. 按原路径恢复到 `F:/EpicGamesEnv/CitySample/CitySample.uproject`，或修改 `citysample_scene/start_citysample_simulator.ps1` 及相关本地设置。
3. 恢复匹配的 ProjectAirSim 插件。主运行工程仍在 `F:/EpicGamesEnv/ProjectAirSim`，源码快照位于 `native/profiles/main/`；降落使用独立插件。
4. `source/native/citysample/AGZoneGraphExport.cpp` 保存了自定义道路导出代码。按 `source/docs/assets.md` 将它加入 CitySample 模块并配置 ZoneGraph、Json、JsonUtilities 依赖，再编译工程。
5. 启动地图后执行 `AG.ZoneGraph.Export`，恢复 `Saved/AGCitySampleZoneGraph.json`。完整旧 City Sample 工程及其 Saved 目录不在本整理包中。路线、lane ID、导出哈希和场景配置保存在 `recorded/experiments/citysample_scene/generated` 与原始实验记录内。
6. 核对道路导出 SHA-256。若与配置中的 `a4896e6051a09dcb4e16c241ae67b7523a7ff242d6503cad24cec1cc87b60069` 不同，应重新检查路段、车道方向、出生点和交叉口，并重新生成任务配置。
7. 使用保留的 `start_citysample_simulator.ps1 -ControlledTraffic` 启动受控交通场景，再执行 `run_aerial_route_navigation.py` 或 `run_pursuit.py`，输出到新目录。

本文件是恢复清单，不表示删除场景后已经重新验证运行。重新下载地图不会自动恢复自定义插件和道路导出代码。
