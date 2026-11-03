# dji_matrice_platform

## ROS 2 support

This package supports **ROS 2 Humble** only. It won't be migrated to Jazzy or later.

DJI no longer develops the [Onboard SDK](https://github.com/dji-sdk/Onboard-SDK) (OSDK), and
newer DJI drones only support the [Payload SDK](https://github.com/dji-sdk/Payload-SDK). The
OSDK camera stream decoder uses FFmpeg APIs that were removed in FFmpeg 5, so it doesn't build on
Ubuntu 24.04 (FFmpeg 6), which Jazzy targets. For new DJI platforms, use
[as2_platform_dji_psdk](https://github.com/aerostack2/as2_platform_dji_psdk).
