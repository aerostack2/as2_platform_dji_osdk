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

/**
 * @brief Request one broadcast telemetry packet from the vehicle.
 *
 * @param vehicle OSDK vehicle handle.
 * @param responseTimeout Timeout, in seconds.
 * @return true if the packet was received.
 */
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
  /**
   * @brief Construct the DJI Matrice platform and initialize the OSDK vehicle.
   *
   * @param argc Argument count forwarded to the OSDK linux environment.
   * @param argv Argument values forwarded to the OSDK linux environment.
   * @param options Node options.
   */
  DJIMatricePlatform(
    int argc, char ** argv,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  /**
   * @brief Destroy the DJI Matrice Platform object, stopping its subscriptions.
   */
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

  /**
   * @brief Create the sensor interfaces the platform publishes.
   */
  void configureSensors() override;
  // void publishSensorData()override {};

  /**
   * @brief Arm or disarm the vehicle.
   *
   * @param state True to arm, false to disarm.
   * @return true if the vehicle accepted the request.
   */
  bool ownSetArmingState(bool state) override;
  /**
   * @brief Enter or leave offboard control.
   *
   * @param offboard True to take control, false to release it.
   * @return true if the vehicle accepted the request.
   */
  bool ownSetOffboardControl(bool offboard) override;
  /**
   * @brief Accept a control mode requested through the platform interface.
   *
   * @param msg Requested control mode.
   * @return true if the platform accepts the mode.
   */
  bool ownSetPlatformControlMode(
    const as2_msgs::msg::ControlMode & msg) override;
  /**
   * @brief Send the current actuator commands to the vehicle.
   *
   * @return true if the command was sent.
   */
  bool ownSendCommand() override;

  /**
   * @brief Take off with the platform own takeoff routine.
   *
   * @return true if the takeoff finished successfully.
   */
  bool ownTakeoff() override;
  /**
   * @brief Land with the platform own landing routine.
   *
   * @return true if the landing finished successfully.
   */
  bool ownLand() override;

  /**
   * @brief Hold the vehicle in place with the OSDK emergency brake.
   */
  void ownStopPlatform() override
  {
    vehicle_->flightController->emergencyBrakeAction();
  }
  /**
   * @brief Kill switch. A DJI cannot be killed from the SDK: use the remote.
   */
  void ownKillSwitch() override
  {
    RCLCPP_ERROR(
      get_logger(),
      "Kill switch activated for DJI Matrice. \n A DJI won't kill "
      "switch use the Remote Controller to land the drone.");
  }

private:
  /**
   * @brief Log an OSDK error code with its human readable message.
   *
   * @param error OSDK error code.
   */
  void printDJIError(ErrorCode::ErrorCodeType error);
  /**
   * @brief Initialize the OSDK vehicle and take control authority.
   *
   * @return 0 on success.
   */
  int djiInitVehicle();
  /**
   * @brief Read the vehicle telemetry. Not implemented: the subscriptions push it.
   */
  void djiReadTelemetry() {}
  /**
   * @brief Read the battery state. Not implemented: the subscriptions push it.
   */
  void djiReadBattery() {}
  /**
   * @brief Start the OSDK telemetry subscriptions the platform publishes.
   */
  void djiConfigureSensors()
  {
    vehicle_->djiBattery->subscribeBatteryWholeInfo(true);
  }

public:
  /**
   * @brief Initialize the vehicle, configure the sensors and start the
   * telemetry subscriptions.
   */
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

  /**
   * @brief Run the OSDK sample flight sequence, for bring-up testing.
   */
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
