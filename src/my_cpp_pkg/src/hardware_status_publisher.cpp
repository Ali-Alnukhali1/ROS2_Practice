#include "rclcpp/rclcpp.hpp"//for nodes
#include "my_robot_interfaces/msg/hardware_status.hpp"


class HardwareStatusPublisherNode : public rclcpp::Node{ //modify name
    public:
        HardwareStatusPublisherNode() : Node("hardware_status_publisher") //modify node name
        { 
           
        }
    private:
};

int main(int argc,char **argv){
    rclcpp::init(argc,argv);
    auto node = std::make_shared<HardwareStatusPublisherNode>(); //modify name
    rclcpp::spin(node); 
    rclcpp::shutdown();
    return 0;
}