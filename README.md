# Vision-2024

## 环境配置

### 基础软件包

``ubuntu``
```shell
sudo apt install libopencv-dev libceres-dev libyaml-cpp-dev
```

``macOS``
```shell
ibrew install opencv ceres-solver
```
### 海康相机库
[自行下载sdk](https://www.hikrobotics.com/cn/machinevision/service/download?module=0)

### 神经网络

``ubuntu``

参照飞书文档自行安装cuda、cudnn、tensorrt

``macOS``

无需配置


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
  - [ ]  monocular
  - [ ]  stereo
  - [x]  config
- [ ]  nn
  - [x]  coreml
  - [ ]  tensorrt (testing)
- [ ]  robot
- [x]  video
- [ ]  viewer