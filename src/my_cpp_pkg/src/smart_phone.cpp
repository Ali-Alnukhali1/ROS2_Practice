#include "rclcpp/rclcpp.hpp"//for nodes
#include "example_interfaces/msg/string.hpp" //for publishers


class SmartphoneNode : public rclcpp::Node
{
public:
    SmartphoneNode() : Node("smart_phone")
    { 
         subscriber_ = this->create_subscription<example_interfaces::msg::String> (
            "robot_news", 10,
            std::bind(&SmartphoneNode::callbackRobotNews,this,std::placeholders::_1) // one argument
         ); 
          RCLCPP_INFO(this->get_logger(),"subscriber made");
    }
private:
    rclcpp::Subscription<example_interfaces::msg::String>::SharedPtr subscriber_ ;
    void callbackRobotNews(const example_interfaces::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(),"%s",msg->data.c_str());
    }
};

int main(int argc,char **argv){
    rclcpp::init(argc,argv);
    auto node = std::make_shared<SmartphoneNode>();
    rclcpp::spin(node); 
    rclcpp::shutdown();
    return 0;
}