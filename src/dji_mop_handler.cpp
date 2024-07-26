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
 * DJI MOP class implementation file.
 *
 * @authors Miguel Fernández Cortizas
 *          Rafael Perez-Segui
 *          Pedro Arias Pérez
 */

#include "dji_mop_handler.hpp"

void DJIMopHandler::keepAliveCB(const std_msgs::msg::String::SharedPtr msg)
{
  status_ = msg->data + MSG_DELIMITER;
}

void DJIMopHandler::downlinkCB(const std_msgs::msg::String::SharedPtr msg)
{
  if (!connected_) {
    return;
  }
  queue_mtx_.lock();
  std::string data = msg->data + MSG_DELIMITER;
  msg_queue_.push(data);
  queue_mtx_.unlock();
}

bool DJIMopHandler::send()
{
  DJI::OSDK::MOP::MopErrCode ret;

  // Always appending status to send
  std::string data = status_;

  queue_mtx_.lock();
  if (msg_queue_.size() > 0) {
    // Appending msg from queue
    data.append(msg_queue_.front());
    msg_queue_.pop();
  }
  queue_mtx_.unlock();

  writePack_.length = strlen(data.c_str());
  memcpy(sendBuf_, data.c_str(), writePack_.length);

  for (int retry = 0; retry < mop_sending_retries_; retry++) {
    ret = pipeline_->sendData(writePack_, &writePack_.length);
    switch (ret) {
      case DJI::OSDK::MOP::MopErrCode::MOP_PASSED:
        // RCLCPP_INFO(node_ptr_->get_logger(), "[Keep Alive] Send %d bytes:
        // %s",
        //             writePack_.length, data.c_str());
        return true;
      case DJI::OSDK::MOP::MopErrCode::MOP_CONNECTIONCLOSE:
        connected_ = false;
        return false;
      default:
        // RCLCPP_INFO(node_ptr_->get_logger(),
        //             "[SEND FAILED] MOP Code %d. Tried to send %d bytes", ret,
        //             writePack_.length);
        break;
    }
  }
  return false;
}

bool DJIMopHandler::getReady()
{
  recvBuf_ = reinterpret_cast<uint8_t *>(OsdkOsal_Malloc(RELIABLE_RECV_ONCE_BUFFER_SIZE));
  if (recvBuf_ == NULL) {
    RCLCPP_ERROR(node_ptr_->get_logger(), "Osdk_malloc recvbuffer error");
    return false;
  }
  readPack_ = {reinterpret_cast<uint8_t *>(recvBuf_), RELIABLE_RECV_ONCE_BUFFER_SIZE};

  sendBuf_ = reinterpret_cast<uint8_t *>(OsdkOsal_Malloc(RELIABLE_SEND_ONCE_BUFFER_SIZE));
  if (sendBuf_ == NULL) {
    RCLCPP_ERROR(node_ptr_->get_logger(), "Osdk_malloc sendbuffer error");
    return false;
  }
  writePack_ = {reinterpret_cast<uint8_t *>(sendBuf_), RELIABLE_SEND_ONCE_BUFFER_SIZE};

  return true;
}

void DJIMopHandler::mopCommunicationFnc(int id)
{
  vehicle_ptr_->initMopServer();

  DJI::OSDK::MOP::PipelineID _id(id);
  DJI::OSDK::MOP::PipelineType type = DJI::OSDK::MOP::PipelineType::RELIABLE;
  DJI::OSDK::MOP::MopErrCode ret = DJI::OSDK::MOP::MopErrCode::MOP_NOTREADY;

  ret = vehicle_ptr_->mopServer->accept(_id, type, pipeline_);
  if (ret != DJI::OSDK::MOP::MOP_PASSED) {
    RCLCPP_WARN(node_ptr_->get_logger(), "Accept MOP err code: %d", ret);
    connected_ = false;
  } else {
    RCLCPP_INFO(node_ptr_->get_logger(), "New client accepted");
    connected_ = getReady();
  }

  while (connected_) {
    // Read
    memset(recvBuf_, 0, RELIABLE_RECV_ONCE_BUFFER_SIZE);
    readPack_.length = RELIABLE_RECV_ONCE_BUFFER_SIZE;
    ret = pipeline_->recvData(readPack_, &readPack_.length);

    switch (ret) {
      case DJI::OSDK::MOP::MOP_PASSED:
        // RCLCPP_INFO(node_ptr_->get_logger(), "[Read] Received %d bytes",
        //             readPack_.length);
        parseData(readPack_);
        break;
      case DJI::OSDK::MOP::MOP_TIMEOUT:
        // RCLCPP_WARN(node_ptr_->get_logger(), "READ TIMEOUT");
        break;
      case DJI::OSDK::MOP::MOP_CONNECTIONCLOSE:
        connected_ = false;
        break;
      default:
        break;
    }

    // do sleep
    OsdkOsal_TaskSleepMs(mop_read_rate_);
  }
  close();
  RCLCPP_ERROR(node_ptr_->get_logger(), "Connection closed. Exiting..");
}

void DJIMopHandler::mopSendFnc(int id)
{
  while (!connected_) {
    OsdkOsal_TaskSleepMs(mop_write_rate_);
  }

  // RCLCPP_INFO(node_ptr_->get_logger(), "WRITE READY");

  while (connected_) {
    send();
    OsdkOsal_TaskSleepMs(mop_write_rate_);
  }
}

void DJIMopHandler::close()
{
  connected_ = false;
  closed_ = true;

  // clear queue
  std::queue<std::string> empty;
  std::swap(msg_queue_, empty);
  missed_msg_ = "";

  vehicle_ptr_->mopServer->close(pipeline_->getId());
  pipeline_->~MopPipeline();
  pipeline_ = NULL;
  vehicle_ptr_->mopServer->~MopServer();
}

void DJIMopHandler::parseData(MopPipeline::DataPackType data)
{
  std::string data_str = bytesToString(readPack_.data, readPack_.length);
  if (missed_msg_ != "") {
    data_str.insert(0, missed_msg_);
    missed_msg_ = "";
  }
  auto [msg_list, missed_msg] = checkString(data_str, MSG_DELIMITER);
  missed_msg_ = missed_msg;

  std_msgs::msg::String msg = std_msgs::msg::String();
  for (const std::string & _msg : msg_list) {
    msg.data = _msg;
    // RCLCPP_INFO(node_ptr_->get_logger(), "UPLINK MSG: %s\n", _msg.c_str());
    uplink_pub_->publish(msg);
  }
}

std::tuple<std::vector<std::string>, std::string> DJIMopHandler::checkString(
  const std::string & input, char delimiter)
{
  std::vector<std::string> parts;
  std::stringstream ss(input);
  std::string part;
  std::string last_part = "";

  while (std::getline(ss, part, delimiter)) {
    parts.push_back(part);
  }
  if (parts.size() > 0 && input.back() != delimiter) {
    last_part = parts.back();
    // RCLCPP_INFO(node_ptr_->get_logger(), "MISSED MSG: %s",
    // last_part.c_str());
    parts.pop_back();
  }
  return {parts, last_part};
}

std::string DJIMopHandler::bytesToString(const uint8_t * data, size_t len)
{
  std::string result(reinterpret_cast<const char *>(data), len);
  return result;
}
