// Copyright 2023 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ROS2_CONTROL_DEMO_LRMATE_200ID__LRMATE_200ID_HARDWARE_HPP_
#define ROS2_CONTROL_DEMO_LRMATE_200ID__LRMATE_200ID_HARDWARE_HPP_

#include "string"
#include "unordered_map"
#include "vector"
#include "netinet/in.h"
#include "fcntl.h"
#include "unistd.h"
#include "cmath"
#include "rclcpp/rclcpp.hpp"

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "command_encode.hpp"

using hardware_interface::return_type;

namespace ros2_control_demo_lrmate_200id
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using namespace softdesign::CommandEncode;

class HARDWARE_INTERFACE_PUBLIC LRMATE_200ID : public hardware_interface::SystemInterface
{
public:
    CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
    CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    CallbackReturn on_activate(const rclcpp_lifecycle::State & /*previous_state*/) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) override;

    return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
    return_type write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override;

protected:
    /// The size of this vector is (standard_interfaces_.size() x nr_joints)
    std::vector<double> joint_position_command_;
    std::vector<double> pre_joint_position_command_;
    std::vector<double> joint_position_;

    std::unordered_map<std::string, std::vector<std::string>> joint_interfaces = {
        {"position", {}}
    };

private:
    static constexpr int FANUC_PORT = 12000;
    int serverSocket_, clientSocket_;
    struct sockaddr_in serverAddress_, clientAddress_;
    struct CommandHeader {
        int robot_num;
        int cmd_id;
    } __attribute__((packed));

    int startServer(int port);
    int acceptClient();
    bool is_command_changed();

    rclcpp::Logger get_logger() const {
        return rclcpp::get_logger("LRMATE_200ID");
    }
};

}  // namespace ros2_control_demo_lrmate_200id

#endif  // ROS2_CONTROL_DEMO_LRMATE_200ID__LRMATE_200ID_HARDWARE_HPP_
