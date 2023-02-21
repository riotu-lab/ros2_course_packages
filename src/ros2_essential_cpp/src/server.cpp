/*
 * Copyright (c) 2023 Anis Koubaa.
 *
 * Licensed under the Creative Commons Attribution-NonCommercial-ShareAlike
 * 4.0 International License.
 * 
 * You may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *     https://creativecommons.org/licenses/by-nc-sa/4.0/legalcode
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "rclcpp/rclcpp.hpp"
#include "ros2_interfaces_cpp/srv/add_two_ints.hpp"

#include <memory>

void add(const std::shared_ptr<ros2_interfaces_cpp::srv::AddTwoInts::Request> request,
          std::shared_ptr<ros2_interfaces_cpp::srv::AddTwoInts::Response>      response)
{
  response->sum = request->a + request->b;
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\na: %ld" " b: %ld",
                request->a, request->b);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%ld]", (long int)response->sum);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("add_two_ints_server");

  rclcpp::Service<ros2_interfaces_cpp::srv::AddTwoInts>::SharedPtr service =
    node->create_service<ros2_interfaces_cpp::srv::AddTwoInts>("add_two_ints", &add);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to add two ints.");

  rclcpp::spin(node);
  rclcpp::shutdown();
}