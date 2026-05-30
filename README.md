# QGimbal-vision C++

C++/CMake rewrite of the Python `QGimbal-vision` project for Linux deployment.

The rectangle detection logic is ported from `Rectangle-recognition`: grayscale,
5x5 Gaussian blur, Canny 50/150, external contours, `approxPolyDP`, 4-point
rectangle filtering, `0.05..0.80` area ratio, 26:17 aspect ratio with 30%
tolerance, then selecting the largest rectangle.

The control and serial behavior are ported from the Python project:

- PID tracks rectangle center to image center.
- Target present turns laser on; target lost turns laser off.
- Serial packets are `cmd + float yaw + float pitch + crc8`, little-endian, 10 bytes.
- Speed command uses command `0x04`.

## Linux Dependencies

Ubuntu/Debian:

```bash
sudo apt update
sudo apt install -y build-essential cmake pkg-config libopencv-dev v4l-utils
```

Optional tools that are useful while debugging cameras and serial devices:

```bash
sudo apt install -y minicom setserial
```

If your Linux image does not provide a recent enough OpenCV package, build
OpenCV from source instead:

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

## Device Permissions

Add the current user to the camera and serial groups:

```bash
sudo usermod -aG video,dialout "$USER"
```

Log out and log back in after running `usermod`.

Check camera devices:

```bash
ls -l /dev/video*
v4l2-ctl --list-devices
v4l2-ctl -d /dev/video0 --list-formats-ext
```

Check serial devices:

```bash
ls -l /dev/ttyS* /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
```

## Build

From this project directory:

```bash
cmake -S . -B build -D CMAKE_BUILD_TYPE=Release
cmake --build build -j"$(nproc)"
```

If CMake cannot find OpenCV after a manual install, point it at the OpenCV
CMake config directory:

```bash
cmake -S . -B build -D CMAKE_BUILD_TYPE=Release -D OpenCV_DIR=/usr/local/lib/cmake/opencv4
cmake --build build -j"$(nproc)"
```

## Run

GUI mode:

```bash
./build/qgimbal_vision --camera 0 --display 1
```

Headless mode:

```bash
./build/qgimbal_vision --camera 0 --display 0 --print-interval 0.5
```

Control output:

```bash
./build/qgimbal_vision --camera 0 --display 1 --serial-port /dev/ttyS1 --serial-baud 115200 --max-rpm 120 --deadband-px 6
```

Disable serial while testing:

```bash
./build/qgimbal_vision --serial-port none
```

Use MJPG explicitly if the camera supports high FPS in MJPG mode:

```bash
./build/qgimbal_vision --camera 0 --display 1 --fourcc MJPG
```

Exit GUI mode with `q` or `ESC`. Exit headless mode with `Ctrl+C`.

## Common Linux Notes

If the GUI window does not open over SSH, either run headless:

```bash
./build/qgimbal_vision --display 0
```

or enable X forwarding / run from the machine's local desktop session.

If `/dev/ttyS1` is not your gimbal port, replace it with the correct device,
for example `/dev/ttyUSB0` or `/dev/ttyACM0`.

If the camera opens but FPS is low, inspect supported modes with:

```bash
v4l2-ctl -d /dev/video0 --list-formats-ext
```

then try another camera index or use `--fourcc MJPG`.

## Useful Options

```text
--camera N
--display 0|1
--print-interval SEC
--fourcc MJPG
--control 0|1
--max-rpm RPM
--deadband-px PX
--lost-timeout SEC
--serial-port PORT
--serial-baud BAUD
--min-area-ratio R
--max-area-ratio R
--aspect-ratio R
--aspect-tolerance R
--canny-low V
--canny-high V
```
