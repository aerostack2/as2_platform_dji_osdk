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
 * @file dji_mop_handler.cpp
 *
 * DJI MOP class header file.
 *
 * @authors Miguel Fernández Cortizas
 *          Rafael Perez-Segui
 *          Pedro Arias Pérez
 */

#ifndef AS2_PLATFORM_DJI_OSDK__DJI_MOP_HANDLER_HPP_
#define AS2_PLATFORM_DJI_OSDK__DJI_MOP_HANDLER_HPP_

#include <vector>
#include <tuple>
#include <string>
#include <mutex>  // std::mutex
#include <queue>  // std::queue
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include "as2_core/names/topics.hpp"
#include "as2_core/node.hpp"
#include "std_msgs/msg/string.hpp"

#include "dji_linux_helpers.hpp"
#include "dji_vehicle.hpp"
#include "osdk_platform.h"  // NOLINT
#include "osdkhal_linux.h"  // NOLINT

#define RELIABLE_RECV_ONCE_BUFFER_SIZE (1024)
#define RELIABLE_SEND_ONCE_BUFFER_SIZE (1024)
#define MSG_DELIMITER '\r'

class DJIMopHandler
{
  DJI::OSDK::Vehicle * vehicle_ptr_;
  as2::Node * node_ptr_;
  DJI::OSDK::MopPipeline * pipeline_ = NULL;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr uplink_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr downlink_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr keep_alive_sub_;

public:
  DJIMopHandler(DJI::OSDK::Vehicle * vehicle, as2::Node * node)
  : vehicle_ptr_(vehicle), node_ptr_(node)
  {
    node_ptr_->declare_parameter("channel_id", 49152);
    mop_channel_id_ = node_ptr_->get_parameter("channel_id").as_int();

    node_ptr_->declare_parameter("sending_retries", 3);
    mop_sending_retries_ = node_ptr_->get_parameter("sending_retries").as_int();

    node_ptr_->declare_parameter("read_rate", 10);
    mop_read_rate_ = node_ptr_->get_parameter("read_rate").as_int();

    node_ptr_->declare_parameter("write_rate", 500);
    mop_write_rate_ = node_ptr_->get_parameter("write_rate").as_int();

    node_ptr_->declare_parameter("reconnection_rate", 5000);
    mop_reconnection_rate_ =
      node_ptr_->get_parameter("reconnection_rate").as_int();

    uplink_pub_ = node_ptr_->create_publisher<std_msgs::msg::String>(
      "/uplink", as2_names::topics::global::qos);

    // Create a custom QoS profile with the desired settings
    rclcpp::QoS custom_qos_profile =
      rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    downlink_sub_ = node_ptr_->create_subscription<std_msgs::msg::String>(
      "/downlink", custom_qos_profile,
      std::bind(&DJIMopHandler::downlinkCB, this, std::placeholders::_1));

    keep_alive_sub_ = node_ptr_->create_subscription<std_msgs::msg::String>(
      "/keep_alive", custom_qos_profile,
      std::bind(&DJIMopHandler::keepAliveCB, this, std::placeholders::_1));

    static auto timer_ = node_ptr_->create_timer(
      std::chrono::milliseconds(mop_reconnection_rate_), [this]() {
        // Check if thread is already running to launch a new mopServer
        if (mop_communication_th_.get_id() == std::thread::id()) {
          RCLCPP_INFO(node_ptr_->get_logger(), "CREATING NEW MOP CHANNEL");
          mop_communication_th_ = std::thread(
            &DJIMopHandler::mopCommunicationFnc, this, mop_channel_id_);
        }
        if (mop_send_th_.get_id() == std::thread::id()) {
          // RCLCPP_INFO(node_ptr_->get_logger(), "NEW SEND THREAD");

          mop_send_th_ =
          std::thread(&DJIMopHandler::mopSendFnc, this, mop_channel_id_);
        }

        // If connection closed, wait to join thread before launching a new
        // one
        if (closed_) {
          mop_communication_th_.join();
          mop_send_th_.join();
          closed_ = false;
        }
      });
  }

  ~DJIMopHandler()
  {
    mop_communication_th_.join();
    mop_send_th_.join();
    OsdkOsal_Free(recvBuf_);
    OsdkOsal_Free(sendBuf_);
    pipeline_->~MopPipeline();
    vehicle_ptr_->mopServer->~MopServer();
  }

  void downlinkCB(const std_msgs::msg::String::SharedPtr msg);
  void keepAliveCB(const std_msgs::msg::String::SharedPtr msg);
  void mopCommunicationFnc(int id);
  void mopSendFnc(int id);

private:
  bool getReady();
  bool send();
  void parseData(MopPipeline::DataPackType data);
  std::string bytesToString(const uint8_t * data, size_t len);
  std::tuple<std::vector<std::string>, std::string> checkString(
    const std::string & input, char delimiter);
  void publishUplink(const MopPipeline::DataPackType * dataPack);
  void close();

private:
  // MOP configuration parameters
  int mop_channel_id_;
  int mop_sending_retries_;
  int mop_read_rate_;
  int mop_write_rate_;
  int mop_reconnection_rate_;

private:
  std::queue<std::string> msg_queue_;
  std::mutex queue_mtx_;
  std::atomic<bool> connected_ = false;  // when read msg from downlink
  std::atomic<bool> closed_ = false;     // when open new MOP Server
  std::string status_ = "{}\r";
  std::string missed_msg_ = "";
  std::thread mop_communication_th_;
  std::thread mop_send_th_;
  uint8_t * recvBuf_;
  uint8_t * sendBuf_;
  MopPipeline::DataPackType readPack_;
  MopPipeline::DataPackType writePack_;
};
#endif  // AS2_PLATFORM_DJI_OSDK__DJI_MOP_HANDLER_HPP_
