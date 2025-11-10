#include <eigen3/Eigen/Dense>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/switch.hpp>

namespace rmcs_core::test {
class OutputTest
    : public rmcs_executor::Component
    , public rclcpp::Node {

public:
    OutputTest()
        : Node{get_component_name(), rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , logger_(get_logger()) {
        register_output(get_parameter("motor_right").as_string(),motor_control_velocity_right);
        register_output(get_parameter("motor_left").as_string(),motor_control_velocity_left);

        register_output("/remote/joystick/left", remote_left_joystic_);
        register_output("/remote/joystick/right", remote_right_joystic_);
        register_output("/remote/switch/left", remote_left_switch_);
        register_output("/remote/switch/right", remote_right_switch_);

       
    }

    void update() override {
        using namespace rmcs_msgs;
        if ((*remote_left_switch_ == Switch::DOWN || *remote_left_switch_ == Switch::UNKNOWN)
            && (*remote_right_switch_ == Switch::DOWN || *remote_right_switch_ == Switch::UNKNOWN)) {
            *motor_control_velocity_right = nan_;
            *motor_control_velocity_left = nan_;

        } else {
            *motor_control_velocity_right= 20 * remote_right_joystic_->x();
            *motor_control_velocity_left= 20 * remote_left_joystic_->x();

          
        }
    }

private:
    rclcpp::Logger logger_;
    static constexpr double nan_ = std::numeric_limits<double>::quiet_NaN();

    OutputInterface<rmcs_msgs::Switch> remote_left_switch_;
    OutputInterface<rmcs_msgs::Switch> remote_right_switch_;

    OutputInterface<Eigen::Vector2d> remote_left_joystic_;
    OutputInterface<Eigen::Vector2d> remote_right_joystic_;

    OutputInterface<double> motor_control_velocity_right;
    OutputInterface<double> motor_control_velocity_left;

};

} // namespace rmcs_core::example

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::test::OutputTest, rmcs_executor::Component)