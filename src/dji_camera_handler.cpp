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
 * @file dji_camera_handler.cpp
 *
 * DJI Camera Handler class implementation file.
 *
 * @authors Miguel Fernández Cortizas
 *          Rafael Perez-Segui
 *          Pedro Arias Pérez
 */

#include "as2_platform_dji_osdk/dji_camera_handler.hpp"
#include <opencv2/highgui.hpp>

void camera_cb(CameraRGBImage pImg, void * userData)
{
  std::cout << "camera_cb" << std::endl;
  auto img =
    cv::Mat(pImg.height, pImg.width, CV_8UC3, reinterpret_cast<void *>(pImg.rawData.data()));
  // create fullscreen window
  cv::imshow("camera", img);
  cv::waitKey(1);
}

void main_camera_cb(CameraRGBImage pImg, void * userData)
{
  static bool first = true;
  // std::cout << "camera_cb" << std::endl;
  auto img =
    cv::Mat(pImg.height, pImg.width, CV_8UC3, reinterpret_cast<void *>(pImg.rawData.data()));
  // convert to RGB
  cv::cvtColor(img, img, cv::COLOR_BGR2RGB);

  if (userData != nullptr) {
    as2::sensors::Camera * camera_ptr = (as2::sensors::Camera *)userData;
    camera_ptr->updateData(img);
  } else {
    // print original image size
    if (first) {
      std::cout << "original image size: " << img.size() << std::endl;
    }

    // create fullscreen window
    if (first) {
      cv::namedWindow("main_camera", cv::WINDOW_KEEPRATIO);
      // move to second screen
      cv::resizeWindow("main_camera", img.size().width, img.size().height);
      cv::moveWindow("main_camera", 1920, 0);
      first = false;
    }
    // cv::namedWindow("main_camera", cv::WINDOW_FULLSCREEN);
    cv::imshow("main_camera", img);
    cv::waitKey(1);
  }
}
