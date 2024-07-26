// Copyright 2024 Universidad Politécnica de Madrid
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Universidad Politécnica de Madrid nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

/**
 * @file dji_matrice_platform.cpp
 *
 * DJI Platform class header file.
 *
 * @authors Miguel Fernández Cortizas
 *          Rafael Perez-Segui
 *          Pedro Arias Pérez
 */

#ifndef AS2_PLATFORM_DJI_OSDK__DJI_MATRICE_PLATFORM_HPP_
#define AS2_PLATFORM_DJI_OSDK__DJI_MATRICE_PLATFORM_HPP_

#include <chrono>
#include <cmath>
#include <memory>
#include <thread>
#include <tuple>
#include <string>
#include <vector>

// ros includes
#include <rclcpp/logging.hpp>
#include <rclcpp/timer.hpp>
#include "as2_core/aerial_platform.hpp"
#include "as2_core/names/topics.hpp"
#include "as2_core/sensor.hpp"
#include "as2_core/utils/frame_utils.hpp"
#include "as2_core/utils/tf_utils.hpp"
#include "as2_msgs/msg/thrust.hpp"
#include "dji_telemetry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/string.hpp"

// dji includes
#include "dji_linux_helpers.hpp"
#include "dji_vehicle.hpp"
#include "osdk_platform.h"  // NOLINT
#include "osdkhal_linux.h"  // NOLINT

// opencv includes
#include <opencv2/opencv.hpp>

#include "dji_camera_handler.hpp"
#include "dji_mop_handler.hpp"
#include "dji_subscriber.hpp"
#include "opencv2/highgui/highgui.hpp"

#define RELIABLE_RECV_ONCE_BUFFER_SIZE (1024)
#define RELIABLE_SEND_ONCE_BUFFER_SIZE (1024)

bool getBroadcastData(DJI::OSDK::Vehicle * vehicle, int responseTimeout = 1);

class DJIMatricePlatform : public as2::AerialPlatform
{
  bool enable_mop_channel_ = false;
  bool enable_advanced_sensing_ = false;
  bool has_mode_settled_ = false;
  bool command_changes_ = false;
  uint8_t control_flag_ = 0x00;
  DJI::OSDK::FlightController::JoystickMode dji_joystick_mode_;
  std::shared_ptr<LinuxSetup> linux_env_ptr_;
  Vehicle * vehicle_ = nullptr;

  bool publish_camera_ = false;

public:
  DJIMatricePlatform(
    int argc, char ** argv,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~DJIMatricePlatform()
  {
    for (auto & sub : dji_subscriptions_) {
      sub->stop();
    }
    delete vehicle_;
  }

  std::shared_ptr<DJICameraHandler> camera_handler_;
  std::shared_ptr<DJIMopHandler> mop_handler_;
  std::shared_ptr<DJIGimbalHandler> gimbal_handler_;
  std::shared_ptr<DJICameraTrigger> camera_trigger_;

  std::vector<DJISubscription::SharedPtr> dji_subscriptions_;

  void configureSensors() override;
  // void publishSensorData()override {};

  bool ownSetArmingState(bool state) override;
  bool ownSetOffboardControl(bool offboard) override;
  bool ownSetPlatformControlMode(
    const as2_msgs::msg::ControlMode & msg) override;
  bool ownSendCommand() override;

  bool ownTakeoff() override;
  bool ownLand() override;

  void ownStopPlatform() override
  {
    vehicle_->flightController->emergencyBrakeAction();
  }
  void ownKillSwitch() override
  {
    RCLCPP_ERROR(
      get_logger(),
      "Kill switch activated for DJI Matrice. \n A DJI won't kill "
      "switch use the Remote Controller to land the drone.");
  }

private:
  void printDJIError(ErrorCode::ErrorCodeType error);
  int djiInitVehicle();
  void djiReadTelemetry() {}
  void djiReadBattery() {}
  void djiConfigureSensors()
  {
    vehicle_->djiBattery->subscribeBatteryWholeInfo(true);
  }

public:
  void start()
  {
    if (djiInitVehicle() < 0) {
      // RCLCPP_ERROR(get_logger(), "DJI Matrice Platform: Failed to initialize
      // vehicle.");
      throw std::runtime_error(
              "DJI Matrice Platform: Failed to initialize vehicle.");
      return;
    }

    configureSensors();

    for (auto & sub : dji_subscriptions_) {
      sub->start();
    }

    if (enable_mop_channel_) {
      mop_handler_ = std::make_shared<DJIMopHandler>(vehicle_, this);
    }

    // ownSetArmingState(true);
  }

  void run_test()
  {
    if (djiInitVehicle() < 0) {
      return;
    }
    std::cout << "Vehicle initialized, starting.\n";

    // bool enableSubscribeBatteryWholeInfo = true;
    // BatteryWholeInfo batteryWholeInfo;
    // SmartBatteryDynamicInfo firstBatteryDynamicInfo;
    // SmartBatteryDynamicInfo secondBatteryDynamicInfo;
    // const int waitTimeMs = 500;
    // while (rclcpp::ok()) {
    //   vehicle_->djiBattery->getBatteryWholeInfo(batteryWholeInfo);
    //   DSTATUS("(It's valid only for M210V2)batteryCapacityPercentage is
    //   %ld%\n",
    //           batteryWholeInfo.batteryCapacityPercentage);
    //   vehicle_->djiBattery->getSingleBatteryDynamicInfo(
    //       DJIBattery::RequestSmartBatteryIndex::FIRST_SMART_BATTERY,
    //       firstBatteryDynamicInfo);
    //   DSTATUS("battery index %d batteryCapacityPercent is %ld%\n",
    //           firstBatteryDynamicInfo.batteryIndex,
    //           firstBatteryDynamicInfo.batteryCapacityPercent);
    //   DSTATUS("battery index %d currentVoltage is %ldV\n",
    //   firstBatteryDynamicInfo.batteryIndex,
    //           firstBatteryDynamicInfo.currentVoltage / 1000);
    //   DSTATUS("battery index %d batteryTemperature is %ld\n",
    //   firstBatteryDynamicInfo.batteryIndex,
    //           firstBatteryDynamicInfo.batteryTemperature / 10);
    //   vehicle_->djiBattery->getSingleBatteryDynamicInfo(
    //       DJIBattery::RequestSmartBatteryIndex::SECOND_SMART_BATTERY,
    //       secondBatteryDynamicInfo);
    //   DSTATUS("battery index %d batteryCapacityPercent is %ld%\n",
    //           secondBatteryDynamicInfo.batteryIndex,
    //           secondBatteryDynamicInfo.batteryCapacityPercent);
    //   DSTATUS("battery index %d currentVoltage is %ldV\n",
    //   secondBatteryDynamicInfo.batteryIndex,
    //           secondBatteryDynamicInfo.currentVoltage / 1000);
    //   DSTATUS("battery index %d batteryTemperature is %ld\n",
    //   secondBatteryDynamicInfo.batteryIndex,
    //           secondBatteryDynamicInfo.batteryTemperature / 10);
    //   OsdkOsal_TaskSleepMs(waitTimeMs);
    // }
    // getBroadcastData(vehicle_);
    //
  }
};

#endif  // AS2_PLATFORM_DJI_OSDK__DJI_MATRICE_PLATFORM_HPP_
