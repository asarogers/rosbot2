#include "rclcpp/rclcpp.hpp"

class HelloWorldNode : public rclcpp::Node
{
	public:
		HelloWorldNode() : Node("Hello_world")
	{
		RCLCPP_INFO(this->get_logger(), "Hello, World!");
	}
};

int main(int argc, char ***argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<HelooWorldNode>());
	rclcpp::shutdown();
	return 0;
}

