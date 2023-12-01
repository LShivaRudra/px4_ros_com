#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/actuator_motors.hpp>
// #include <px4_msgs/msg/actuator_outputs.hpp>
// #include <px4_msgs/msg/vehicle_thrust_setpoint.hpp>
// #include <px4_msgs/msg/vehicle_torque_setpoint.hpp>

using namespace std::chrono_literals;

class MotorControl : public rclcpp::Node
{
public:
	MotorControl() : Node("control_vect_advertiser")
	{

		publisher_ = this->create_publisher<px4_msgs::msg::ActuatorMotors>("/fmu/in/actuator_motors", 10);

		auto timer_callback = [this]()->void {
			auto control_vect = px4_msgs::msg::ActuatorMotors();
			control_vect.timestamp = std::chrono::time_point_cast<std::chrono::microseconds>(std::chrono::steady_clock::now()).time_since_epoch().count();
			// std::string name = "test";
			// std::copy(name.begin(), name.end(), debug_vect.name.begin());
			// debug_vect.x = 1.0;
			// debug_vect.y = 2.0;
			// debug_vect.z = 3.0;
            // control_vect.NUM_CONTROLS = 4;
            control_vect.control = {1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
			RCLCPP_INFO(this->get_logger(), "Publishing control_vect: time: %lu motor1: %f motor2: %f motor3: %f motor4: %f",
				    control_vect.timestamp, control_vect.control[0],control_vect.control[1],control_vect.control[2],control_vect.control[3]);

			this->publisher_->publish(control_vect);
		};

		timer_ = this->create_wall_timer(500ms, timer_callback);
	}

private:
	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::Publisher<px4_msgs::msg::ActuatorMotors>::SharedPtr publisher_;
};


int main(int argc, char *argv[])
{
	std::cout << "Starting motor_control advertiser node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<MotorControl>());

	rclcpp::shutdown();
	return 0;
}