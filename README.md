# SRM-Vision-2024

## 功能

| 模块名    | 功能                                        |
| :-------- | :------------------------------------------ |
| autoaim   | 自瞄                                        |
| autopilot | 导航                                        |
| common    | 通用模块，方便代码编写（考虑后续移除ekf.h） |
| coord     | 坐标系变换                                  |
| core      | 核心，负责调用各模块                        |
| nn        | 神经网络                                    |
| robot     | 机器人控制                                  |
| video     | 视频源                                      |
| viewer    | 信息查看（图像、参数等）                    |




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