// Copyright 2023 Universidad Politécnica de Madrid
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
//    * Neither the name of the Universidad Politécnica de Madrid nor the names
//    of its
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
 * @file as2_platform_dji_osdk_gtest.cpp
 *
 * DJI OSDK platform node gtest
 *
 * @author Rafael Perez-Segui <r.psegui@upm.es>
 */

#include <gtest/gtest.h>
#include <iostream>
#include <memory>
#include <string>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "dji_matrice_platform.hpp"

std::shared_ptr<DJIMatricePlatform> get_node(
  const std::string & name_space = "as2_platform_dji_osdk")
{
  const std::string package_path =
    ament_index_cpp::get_package_share_directory("as2_platform_dji_osdk");
  const std::string control_modes_config_file = package_path + "/config/control_modes.yaml";
  const std::string platform_config_file = package_path + "/config/platform_config_file.yaml";
  const std::string mop_handler_config_file = package_path + "/config/mop_handler_config_file.yaml";
  const std::string user_config_file = package_path + "/config/user_config_file.txt";

  std::vector<std::string> node_args = {
    "--ros-args",
    "-r",
    "__ns:=/" + name_space,
    "-p",
    "namespace:=" + name_space,
    "-p",
    "control_modes_file:=" + control_modes_config_file,
    "--params-file",
    platform_config_file,
    "--params-file",
    mop_handler_config_file,
  };

  rclcpp::NodeOptions node_options;
  node_options.arguments(node_args);


  // Add user_config_file as an argument
  std::vector<char *> argv;
  for (const auto & arg : node_args) {
    argv.push_back(const_cast<char *>(arg.c_str()));
  }
  argv.push_back(const_cast<char *>(user_config_file.c_str()));

  int argc = argv.size();

  return std::make_shared<DJIMatricePlatform>(argc, argv.data(), node_options);
}

TEST(PlatformGazeboGTest, Constructor) {
  EXPECT_NO_THROW(get_node());
  auto node = get_node();

  // Spin the node
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin_some();
}

int main(int argc, char * argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  auto result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
