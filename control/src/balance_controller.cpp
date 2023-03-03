#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

#define RAD_TO_DEG (180.0f / M_PI)
#define DEG_TO_RAD (M_PI / 180.0f)

class BALANCE_CONTROLLER : public rclcpp::Node
{
public:
    explicit BALANCE_CONTROLLER(const std::string & node_name)
     : Node(node_name), int_error(0.0), d_error(0.0), previous_error(0.0)
     {
        declare_parameters_and_get_those();

        pub_tire_effort_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/effort_controllers/commands", 1);
        sub_imu_data_ = this->create_subscription<sensor_msgs::msg::Imu>("/wip_control_imu/data", 1, std::bind(&BALANCE_CONTROLLER::callback_sub_imu_data, this, std::placeholders::_1));
        
        RCLCPP_INFO(this->get_logger(), "started balance controller");

     }

private:
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_data_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_tire_effort_;

    void declare_parameters_and_get_those()
    {
        this->declare_parameter("p_gain", 10.0);
        this->declare_parameter("d_gain", 1.0);
        this->declare_parameter("i_gain", 0.0);
        this->declare_parameter("imu_target_angle", 0.0);

        p_gain = this->get_parameter("p_gain").as_double();
        d_gain = this->get_parameter("i_gain").as_double();
        i_gain = this->get_parameter("d_gain").as_double();

        imu_target_angle_rad = this->get_parameter("imu_target_angle").as_double() * DEG_TO_RAD;

    }

    // controller gain
    double p_gain, d_gain, i_gain;

    // imu target angle
    double imu_target_angle_rad;

    // for i_gain
    double int_error; 
    // for d_gain
    double previous_error, d_error;

    void callback_sub_imu_data(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        // transform quaterion to euler angle
        tf2::Quaternion q(
            msg->orientation.x,
            msg->orientation.y,
            msg->orientation.z,
            msg->orientation.w
        );
        
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        // get imu pitch anguler velocity
        double pitch_anguler_velocity = msg->angular_velocity.y;

        // pid control
        double pitch_error = imu_target_angle_rad - pitch;
        d_error = pitch_error - previous_error;
        int_error += pitch_error;

        double control_input = p_gain * pitch_error + i_gain * int_error + d_gain * d_error;

        // publish control input to tire effort controller
        std_msgs::msg::Float64MultiArray msg_to_publish;
        msg_to_publish.data.resize(2);

        msg_to_publish.data[0] = - control_input;
        msg_to_publish.data[1] = - control_input;

        pub_tire_effort_->publish(msg_to_publish);   


    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BALANCE_CONTROLLER>("wip_balance_control_node");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

