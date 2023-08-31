# SRM-Vision-2024

## 功能

| 模块名    | 功能                                        | 依赖              |
| :-------- | :------------------------------------------ | ----------------- |
| autoaim   | 自瞄                                        | common, nn, video |
| autopilot | 导航                                        | video             |
| common    | 通用模块，方便代码编写（考虑后续移除ekf.h） | **NULL**          |
| coord     | 坐标系变换                                  | common            |
| core      | 核心，负责调用各模块                        | **ALL**           |
| nn        | 神经网络                                    | common            |
| robot     | 机器人控制                                  | common, autopilot |
| video     | 视频源                                      | common            |
| viewer    | 信息查看（图像、参数等）                    | common            |




## 项目迁移进度

- [ ]  autoaim
  - [ ]  detector
  - [ ]  predictor
- [ ]  autopilot
- [x]  common
- [x]  coord
- [ ]  core
- [ ]  nn
  - [x]  coreml （已提pr）
  - [ ]  tensorrt
- [ ]  robot
- [x]  video
- [ ]  viewer