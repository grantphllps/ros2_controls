#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <iostream>
#include <fstream>
#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/float32.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

#define NODE_NAME "PID_NODE"
#define PERIOD 100 //ms
#define KP 0
#define KI 0
#define KD 0

using std::placeholders::_1;
class PIDController : public rclcpp::Node {
    
    public:

        std::chrono::milliseconds control_update_period{PERIOD};

        PIDController(): Node(NODE_NAME) {
            //Publsihers
            output_publisher = this->create_publisher<std_msgs::msg::Float32>("pid_out", 10);

            //Subscribers
            input_subscriber = this->create_subscription<std_msgs::msg::Float32>(
                "/input", 10, std::bind(&PIDController::value_update_callback, this, _1)
            );

            gain_update = this->create_subscription<geometry_msgs::msg::PointStamped>(
                "/gain_update", 1, std::bind(&PIDController::gain_update_callback, this, _1)
            );

            reference_sub = this->create_subscription<std_msgs::msg::Float32>(
                "/ref", 1, std::bind(&PIDController::reference_update_callback, this, _1)
            );
            
            //Timer
            timer = this->create_wall_timer(control_update_period, std::bind(&PIDController::timer_callback, this));
        }

    private:

        //Member variables
        
        //Control gains
        float kp = KP;
        float ki = KI;
        float kd = KD;

        //Place to store the sensor/estimator reading
        float current_value = 0.0;

        float error = 0.0;
        float error_integral = 0.0;
        float error_derivative = 0.0;
        float error_previous = 0.0;

        //define the reference
        float reference_value = 0.0;

        float period_s = PERIOD / 1000.0;

        //Declare publishers
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr output_publisher;

        //Declare subscribers
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr input_subscriber;
        rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr gain_update;
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr reference_sub;

        //Declare timer interrupts
        rclcpp::TimerBase::SharedPtr timer;

        //Member functions

        //Subscriber callbacks
        void value_update_callback(const std_msgs::msg::Float32::SharedPtr msg) {
            //value callback
            current_value = msg->data;
        }

        void gain_update_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
            //Update the controller's gains
            kp = msg->point.x;
            ki = msg->point.y;
            kd = msg->point.z;

            RCLCPP_INFO(this->get_logger(), "UPDATING GAINS: KP: %f KI: %f KD: %f", kp, ki, kd);
        }

        void reference_update_callback(const std_msgs::msg::Float32::SharedPtr msg) {
            //Update the reference
            reference_value = msg->data;
        }

        //Timer callbacks
        void timer_callback() {
            //Timer callback
            auto output_message = std_msgs::msg::Float32();

            //compute error, error integral, error derivative
            update_errors();

            //compute output
            output_message.data = compute_output();

            //Write output to message
            output_publisher->publish(output_message);
        }

        //Helper functions

        void update_errors() {
            //TODO: Integral Limits
            //TODO: Integral rolloff
            
            //Save the old error
            error_previous = error;

            //Compute the "new" error
            error = reference_value - current_value;

            //Compute the error derivate
            error_derivative = (error_previous - error) / period_s;

            //Compute the error integral, using trapezoid rule
            error_integral = error_integral + ((error + error_previous) * (period_s / 2.0));
        }

        float compute_output() {
            //TODO: output bounds
            return kp*error + ki*error_integral + kd*error_derivative;
        }

};

int main(int argc, char* argv[]) {
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<PIDController>());
    rclcpp::shutdown();
    return 0;
}
