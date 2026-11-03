# as2_platform_dji_osdk

[![build](https://github.com/aerostack2/as2_platform_dji_osdk/actions/workflows/build.yaml/badge.svg)](https://github.com/aerostack2/as2_platform_dji_osdk/actions/workflows/build.yaml)
[![ROS 2 Humble](https://img.shields.io/badge/ROS_2-Humble-22314E?logo=ros)](https://docs.ros.org/en/humble/)
[![ROS 2 Jazzy](https://img.shields.io/badge/ROS_2-Jazzy_unsupported-lightgrey?logo=ros)](#ros-2-jazzy-is-not-supported)
[![License](https://img.shields.io/badge/License-BSD_3--Clause-blue.svg)](LICENSE)
[![arXiv](https://img.shields.io/badge/arXiv-2303.18237-b31b1b.svg)](https://arxiv.org/abs/2303.18237)

[Aerostack2](https://github.com/aerostack2/aerostack2) platform for DJI drones that use the
[DJI Onboard SDK](https://github.com/dji-sdk/Onboard-SDK) (OSDK), such as the Matrice 210 V2 and
Matrice 300 RTK.

> [!WARNING]
> **This package supports ROS 2 Humble only. It won't be migrated to Jazzy.**
>
> - **OSDK is deprecated.** DJI no longer develops it, and newer drones (Matrice 30, Matrice 350
>   RTK, Matrice 4) only support the [Payload SDK](https://github.com/dji-sdk/Payload-SDK). For
>   those, use [`as2_platform_dji_psdk`](https://github.com/aerostack2/as2_platform_dji_psdk).
> - **OSDK needs FFmpeg 4.** Its camera stream decoder
>   (`osdk-core/advanced-sensing/camera_stream/src/dji_camera_stream_decoder.cpp`) uses FFmpeg APIs
>   that were removed in FFmpeg 5, such as `avcodec_register_all`, `avcodec_decode_video2` and
>   `AVPicture`.
> - **Jazzy ships a newer FFmpeg.** Ubuntu 22.04 (Humble) ships FFmpeg 4.4, so the build works.
>   Ubuntu 24.04 (Jazzy) ships FFmpeg 6.1, so the build fails. The same applies to pixi and
>   RoboStack, where conda-forge ships FFmpeg 8 and later.

## Requirements

- Ubuntu 22.04 with [ROS 2 Humble](https://docs.ros.org/en/humble/Installation.html)
- [Aerostack2](https://aerostack2.github.io/_00_getting_started/index.html)
- A DJI developer app (`app_id` and `app_key`) from the [DJI Developer Center](https://developer.dji.com/)

The OSDK itself is fetched at build time from
[`aerostack2/Onboard-SDK`](https://github.com/aerostack2/Onboard-SDK). Its system dependencies
(`libusb-1.0-dev`, `ffmpeg`, `libopencv-dev`) are declared in `package.xml`.

## Build

From the root of your workspace:

```bash
rosdep install -y -r --ignore-src --from-paths src --packages-up-to as2_platform_dji_osdk
colcon build --packages-up-to as2_platform_dji_osdk
```

## Usage

Set your app credentials and serial ports in `config/user_config_file.txt`:

```text
app_id : 123456
app_key : 0123456789abcdefghijklmnopqrstuvwxyz
device : /dev/ttyUSBx
baudrate : 921600
acm_port : /dev/ttyACMx
```

Then launch the platform:

```bash
ros2 launch as2_platform_dji_osdk as2_platform_dji_osdk_launch.py namespace:=drone0
```

| Argument | Default | Description |
| --- | --- | --- |
| `namespace` | `$AEROSTACK2_SIMULATION_DRONE_ID` | Drone namespace |
| `user_config_file` | `config/user_config_file.txt` | DJI app credentials and serial ports |
| `control_modes_file` | `config/control_modes.yaml` | Platform control modes |

Platform parameters (command and info rates, advanced sensing, camera, MOP channel) are in
`config/platform_config_file.yaml`, and MOP channel parameters are in
`config/mop_handler_config_file.yaml`.

## Test

```bash
colcon test --packages-select as2_platform_dji_osdk
colcon test-result --verbose
```

## Citation

If you use Aerostack2 in your research, please cite:

> M. Fernandez-Cortizas, M. Molina, P. Arias-Perez, R. Perez-Segui, D. Perez-Saura and P. Campoy,
> 2023, ["Aerostack2: A software framework for developing multi-robot aerial systems"](https://arxiv.org/abs/2303.18237),
> arXiv 2303.18237.

## License

[BSD 3-Clause](LICENSE)
