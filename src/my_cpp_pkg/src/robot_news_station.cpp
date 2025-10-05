#include "rclcpp/rclcpp.hpp"//for nodes
#include "example_interfaces/msg/string.hpp" //for publishers

using namespace std::chrono_literals;


class RobobtNewsStationNode : public rclcpp::Node{
    public:
    
        RobobtNewsStationNode() : Node("robot_news_station")// node name
        { 
            this->declare_parameter("robot_name","ali_robot");
            robot_name_ = this->get_parameter("robot_name").as_string();
            publisher_ = this->create_publisher<example_interfaces::msg::String>("robot_news",10);
            timer_ = this->create_wall_timer(0.5s,std::bind(&RobobtNewsStationNode::publishNews,this));
            RCLCPP_INFO(this->get_logger(),"Robot news station has been started");
        }
    private:
    std::string robot_name_;
    rclcpp::Publisher<example_interfaces::msg::String>::SharedPtr publisher_;
    void publishNews()
    {
        auto msg = example_interfaces::msg::String();
        msg.data = std::string("Hi, this is ") + robot_name_ + std::string(" from the news station");
        publisher_->publish(msg);
    }
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc,char **argv){
    rclcpp::init(argc,argv);
    auto node = std::make_shared<RobobtNewsStationNode>();
    rclcpp::spin(node); 
    rclcpp::shutdown();
    return 0;
}