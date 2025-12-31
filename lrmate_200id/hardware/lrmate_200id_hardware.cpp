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

#include "ros2_control_demo_lrmate_200id/lrmate_200id_hardware.hpp"
#include <string>
#include <vector>

namespace ros2_control_demo_lrmate_200id
{
CallbackReturn LRMATE_200ID::on_init(const hardware_interface::HardwareInfo & info)
{
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
    {
        return CallbackReturn::ERROR;
    }

    // robot has 6 joints and 2 interfaces
    joint_position_.assign(6, 0);
    joint_position_command_.assign(6, 0);
    pre_joint_position_command_.assign(6, 0);

    for (const auto & joint : info_.joints)
    {
        for (const auto & interface : joint.state_interfaces)
        {
            joint_interfaces[interface.name].push_back(joint.name);
        }
    }

    return CallbackReturn::SUCCESS;
}

CallbackReturn LRMATE_200ID::on_configure(const rclcpp_lifecycle::State & previous_state)
{
    serverSocket_ = startServer(FANUC_PORT);
    clientSocket_ = acceptClient();
    return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> LRMATE_200ID::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;

    int ind = 0;
    for (const auto & joint_name : joint_interfaces["position"])
    {
        state_interfaces.emplace_back(joint_name, "position", &joint_position_[ind++]);
    }

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> LRMATE_200ID::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    int ind = 0;
    for (const auto & joint_name : joint_interfaces["position"])
    {
        command_interfaces.emplace_back(joint_name, "position", &joint_position_command_[ind++]);
    }

    return command_interfaces;
}

return_type LRMATE_200ID::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
    CommandHeader header = {0, CMD_GET_JOINT_INFO};
    send(clientSocket_, &header, sizeof(header), 0);

    float values[6];
    ssize_t n = recv(clientSocket_, values, sizeof(values), 0);

    if (n == sizeof(values)) {
        for (size_t i = 0; i < joint_position_.size(); i++) {
            joint_position_[i] = values[i] * (M_PI / 180.0); // deg2rad
        }
    } else {
        RCLCPP_ERROR(get_logger(), "Read joints failed!");
        return return_type::ERROR;
    }
    return return_type::OK;
}

return_type LRMATE_200ID::write(const rclcpp::Time &, const rclcpp::Duration &)
{   
    if (is_command_changed()) {
        CommandHeader header = {0, CMD_MOVE};
        send(clientSocket_, &header, sizeof(header), 0);

        float values[6];
        for (size_t i = 0; i < joint_position_command_.size(); ++i) {
            values[i] = static_cast<float>(joint_position_command_[i] * (180.0 / M_PI)); // rad2deg
        }

        ssize_t n = send(clientSocket_, values, sizeof(values), 0);
        
        if (n != sizeof(values)) {
            RCLCPP_ERROR(get_logger(), "Write joints failed!");
            return return_type::ERROR;
        }
        pre_joint_position_command_ = joint_position_command_;
    }
    return return_type::OK;
}

int LRMATE_200ID::startServer(int port)
{
    int sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock == -1) {
        RCLCPP_ERROR(get_logger(), "Failed to create socket");
        return -1;
    }

    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(port);

    if (bind(sock, (struct sockaddr*)&address, sizeof(address)) == -1) {
        RCLCPP_ERROR(get_logger(), "Bind failed");
        close(sock);
        return -1;
    }

    if (listen(sock, 5) == -1) {
        RCLCPP_ERROR(get_logger(), "Listen failed");
        close(sock);
        return -1;
    }

    RCLCPP_INFO(get_logger(), "Server listening on port %d", port);
    return sock;
}

int LRMATE_200ID::acceptClient() {
    socklen_t client_len = sizeof(clientAddress_);
    int client = accept(serverSocket_, (struct sockaddr*)&clientAddress_, &client_len);

    if (client == -1) {
        return -1;  // 尚未有 client 嘗試連線
    }

    // 設定為非阻塞模式
    int flags = fcntl(client, F_GETFL, 0);
    fcntl(client, F_SETFL, flags | O_NONBLOCK);

    RCLCPP_INFO(get_logger(), "Client connected.");
    return client;
}

bool LRMATE_200ID::is_command_changed()
{
    // This function can check if the command has changed
    // and perform necessary actions if needed.
    if (joint_position_command_ != pre_joint_position_command_) {
        return true;
    }
    return false;
}

}  // namespace ros2_control_demo_lrmate_200id

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    ros2_control_demo_lrmate_200id::LRMATE_200ID, hardware_interface::SystemInterface)
