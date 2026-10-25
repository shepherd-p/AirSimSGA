# AirSimSGA

海—地—空异构无人系统协同仿真平台。仓库包含 Python 实验接口、C++ 原生扩展、
四类协同实验的控制与配置程序，以及论文对应的数值记录和分析程序。

| 实验 | 内容 |
| --- | --- |
| 空地协同三维建图 | CityPark 广场绕行、LiDAR/RGB-D采集、点云/彩色表面重建 |
| 空中辅助道路导航 | City Sample 多路口观测、路线规划与无人车执行 |
| 多机跟踪与出口围堵 | 两架无人机、两辆拦截无人车和一辆目标车 |
| 移动无人艇视觉降落 | AprilTag视觉引导、平面艇动力学、甲板接触与停桨联锁 |

另提供多平台并行运行配置与资源记录。已录制的环境为 Windows、UE 5.2 和 Python 3.10。

## 使用

```powershell
git clone https://github.com/shepherd-p/AirSimSGA.git
cd AirSimSGA
python -m pip install -e ".[landing,reconstruction]"
python -m airsimsga list
Copy-Item settings/local.example.json settings/local.json
```

先按[构建说明](docs/build.md)准备匹配的模拟器与SDK，按[场景说明](docs/assets.md)
获取地图和模型，再配置 `settings/local.json`。具体命令见[实验说明](docs/experiments.md)。
最终降落实验的独立构建、源码快照和端口见[录制版本说明](docs/recorded_release.md)。

`data/paper/` 提供论文数值数据与不依赖模拟器的分析脚本。
`recorded/` 保留最终实验的控制/配置/录制程序，其中的历史本机路径需按新环境调整。
引擎、地图、模型和完整原始传感器归档需单独获取。

AirSimSGA新增代码采用MIT许可证，保留上游与第三方代码的原有版权和许可。
引用信息见 `CITATION.cff`，依赖来源见 `THIRD_PARTY_NOTICES.md`。
