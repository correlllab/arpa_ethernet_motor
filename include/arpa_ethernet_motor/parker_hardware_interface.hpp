#ifndef ARPA_ETHERNET_MOTOR__PARKER_HARDWARE_INTERFACE_HPP_
#define ARPA_ETHERNET_MOTOR__PARKER_HARDWARE_INTERFACE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_component_interface_params.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "arpa_ethernet_motor/parker_core.hpp"

namespace arpa_ethernet_motor
{

class ParkerHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(ParkerHardwareInterface)

  // Lifecycle callbacks
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  // Interface exports
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  // Read/write called every control cycle
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Parker motor driver
  std::unique_ptr<ParkerCore> parker_;

  // Hardware parameters (from URDF)
  std::string host_;
  int port_;

  // State and command values
  double hw_position_state_;
  double hw_position_command_;

  // Joint name
  std::string joint_name_;
};

}  // namespace arpa_ethernet_motor

#endif  // ARPA_ETHERNET_MOTOR__PARKER_HARDWARE_INTERFACE_HPP_
