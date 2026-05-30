# QGimbal-vision C++

这是 Python 版 `QGimbal-vision` 的 C++/CMake 重写版本，目标运行环境为 Linux。

矩形识别逻辑来自 `Rectangle-recognition` 项目：转灰度图、5x5 高斯模糊、Canny 50/150 边缘检测、查找外部轮廓、`approxPolyDP` 多边形近似、筛选四点矩形、按 `0.05..0.80` 图像面积比例过滤、按 26:17 宽高比和 30% 容差过滤，最后选取面积最大的矩形。

控制和串口行为从 Python 项目迁移而来：

- PID 将矩形中心追踪到图像中心。
- 检测到目标时打开激光，目标丢失时关闭激光。
- 串口包格式为 `cmd + float yaw + float pitch + crc8`，小端序，共 10 字节。
- 速度控制命令为 `0x04`。

## Linux 依赖

Ubuntu/Debian 系统可以直接安装：

```bash
sudo apt update
sudo apt install -y build-essential cmake pkg-config libopencv-dev v4l-utils
```

调试摄像头和串口时，建议额外安装：

```bash
sudo apt install -y minicom setserial
```

如果你的 Linux 镜像没有合适的 OpenCV 开发包，也可以从源码编译安装 OpenCV：

```bash
sudo apt update
sudo apt install -y build-essential cmake git pkg-config \
  libgtk-3-dev libavcodec-dev libavformat-dev libswscale-dev \
  libv4l-dev libjpeg-dev libpng-dev libtiff-dev

git clone --depth 1 https://github.com/opencv/opencv.git
cmake -S opencv -B opencv/build -D CMAKE_BUILD_TYPE=Release -D CMAKE_INSTALL_PREFIX=/usr/local
cmake --build opencv/build -j"$(nproc)"
sudo cmake --install opencv/build
sudo ldconfig
```

## 设备权限

把当前用户加入摄像头和串口常用权限组：

```bash
sudo usermod -aG video,dialout "$USER"
```

执行后需要注销并重新登录，权限才会生效。

检查摄像头设备：

```bash
ls -l /dev/video*
v4l2-ctl --list-devices
v4l2-ctl -d /dev/video0 --list-formats-ext
```

检查串口设备：

```bash
ls -l /dev/ttyS* /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
```

## 编译

进入项目目录后执行：

```bash
cmake -S . -B build -D CMAKE_BUILD_TYPE=Release
cmake --build build -j"$(nproc)"
```

如果手动安装 OpenCV 后 CMake 仍然找不到 OpenCV，可以显式指定 OpenCV 的 CMake 配置目录：

```bash
cmake -S . -B build -D CMAKE_BUILD_TYPE=Release -D OpenCV_DIR=/usr/local/lib/cmake/opencv4
cmake --build build -j"$(nproc)"
```

## 运行

显示窗口模式：

```bash
./build/qgimbal_vision --camera 0 --display 1
```

无窗口模式，只在终端输出 FPS、矩形中心和控制量：

```bash
./build/qgimbal_vision --camera 0 --display 0 --print-interval 0.5
```

启用云台串口控制：

```bash
./build/qgimbal_vision --camera 0 --display 1 --serial-port /dev/ttyS1 --serial-baud 115200 --max-rpm 120 --deadband-px 6
```

只测试视觉识别，不发送串口数据：

```bash
./build/qgimbal_vision --serial-port none
```

如果摄像头支持 MJPG 高帧率模式，可以显式指定：

```bash
./build/qgimbal_vision --camera 0 --display 1 --fourcc MJPG
```

显示窗口模式下按 `q` 或 `ESC` 退出。无窗口模式下按 `Ctrl+C` 退出。

## Linux 常见问题

如果通过 SSH 运行时打不开图形窗口，可以改用无窗口模式：

```bash
./build/qgimbal_vision --display 0
```

也可以开启 X forwarding，或者直接在设备本机桌面环境中运行。

如果 `/dev/ttyS1` 不是云台串口，请替换为实际设备，例如 `/dev/ttyUSB0` 或 `/dev/ttyACM0`。

如果摄像头能打开但帧率偏低，先查看摄像头支持的格式：

```bash
v4l2-ctl -d /dev/video0 --list-formats-ext
```

然后尝试更换摄像头编号，或使用 `--fourcc MJPG`。

## 常用参数

```text
--camera N                摄像头编号
--display 0|1             是否显示 OpenCV 窗口
--print-interval SEC      无窗口模式下的终端打印间隔
--fourcc MJPG             摄像头像素格式
--control 0|1             是否启用 PID 控制输出
--max-rpm RPM             yaw/pitch 最大转速
--deadband-px PX          图像像素死区
--lost-timeout SEC        目标丢失后复位控制器的超时时间
--serial-port PORT        串口设备，测试时可设为 none
--serial-baud BAUD        串口波特率
--min-area-ratio R        矩形最小面积比例
--max-area-ratio R        矩形最大面积比例
--aspect-ratio R          目标宽高比
--aspect-tolerance R      宽高比容差
--canny-low V             Canny 低阈值
--canny-high V            Canny 高阈值
```
