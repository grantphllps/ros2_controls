#include <chrono>
#include <functional>
#include <memory>
#include <array>
#include <tf2/LinearMath/Quaternion.h>

#include <iostream>
#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/float32.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

#define NODE_NAME "X_FORM_NODE"
#define PERIOD 100 //ms

using std::placeholders:: _1;

class RYPTXform : public rclcpp::Node {

    public:

        std::chrono::milliseconds control_update_period{PERIOD};

        RYPTXform(): Node(NODE_NAME) {

            x_subscriber = this->create_subscription<std_msgs::msg::Float32>(
                "x_effort", 2, std::bind(&RYPTXform::x_update_callback, this, _1)
            );
            y_subscriber = this->create_subscription<std_msgs::msg::Float32>(
                "y_effort", 2, std::bind(&RYPTXform::y_update_callback, this, _1)
            );
            z_subscriber = this->create_subscription<std_msgs::msg::Float32>(
                "z_effort", 2, std::bind(&RYPTXform::z_update_callback, this, _1)
            );

            //Timer
            timer = this->create_wall_timer(control_update_period, std::bind(&RYPTXform::timer_callback, this));
            
        }
    private:

        //Member variables
        

        //Most recent efforts - leaving as separate variables since they are updated asynchronously
        float x_effort = 0.0;
        float y_effort = 0.0;
        float z_effort = 0.0;

        //Declare publishers
        //rclcpp::Subscription<std_msgs::msg::Float32>

        //Declare subscribers
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr x_subscriber;
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr y_subscriber;
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr z_subscriber;

        //Declare timer interrupts
        rclcpp::TimerBase::SharedPtr timer;

        //Member functions

        //Subscriber callbacks
        void x_update_callback(const std_msgs::msg::Float32::SharedPtr msg) {
            x_effort = msg->data;
        }

        void y_update_callback(const std_msgs::msg::Float32::SharedPtr msg) {
            y_effort = msg->data;
        }

        void z_update_callback(const std_msgs::msg::Float32::SharedPtr msg) {
            z_effort = msg->data;
        }

        //Timer callback
        void timer_callback() {
            
            //TODO: condition commands x, y, z to prevent large rolls/pitches

            //get the rpyt command
            auto rypt = compute_rypt(x_effort, y_effort, z_effort);

            //convert rpy to quaternion

            //publish attitude command

            //publish thrust command
        }

        //Helper functions

        //Convert x_effort, y_effort, z_effort to a rypt command
        std::array<float, 4> compute_rypt(float Fx, float Fy, float Fz) {
            std::array<float, 4> RYPT;

            float roll = atan2(Fy, Fz);
            float pitch = atan2(-Fx, sqrt(Fy * Fy + Fz * Fz));
            float yaw = 0.0;

            float thrust = sqrt(Fx*Fx + Fy*Fy + Fz*Fz);

            RYPT[0] = roll;
            RYPT[1] = pitch;
            RYPT[2] = yaw;
            RYPT[3] = thrust;

            return RYPT;
        }

        //Convert euler rpy to robot-friendly quaternion
        geometry_msgs::msg::Quaternion euler_to_quat( std::array<float, 4> rypt ) {
            
            tf2::Quaternion tf_quaternion;
            tf_quaternion.setRPY(rypt[0], rypt[1], rypt[2]);

            auto command_quat = geometry_msgs::msg::Quaternion();

            command_quat.x = tf_quaternion.x();
            command_quat.y = tf_quaternion.y();
            command_quat.z = tf_quaternion.z();
            command_quat.w = tf_quaternion.w();

            return command_quat;

        }

};

int main() {
    std::cout << "get fukt" << std::endl;
}