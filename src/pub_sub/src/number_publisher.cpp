#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int64.hpp"

using namespace std::chrono_literals;

// NumberPublisher node publishes an Int64 message with value `1`
// every 500 milliseconds on the "number" topic.
class NumberPublisher : public rclcpp::Node{
    public:
        // Constructor: initializes publisher and timer.
        NumberPublisher() : Node("number_publisher")
        { 
           // Create publisher for Int64 messages on "number" topic with queue size 10.
           publisher_= this->create_publisher<example_interfaces::msg::Int64>("number",10);
           
           // Create timer that calls publishNumber() every 500 ms.
           timer_ = this->create_wall_timer(0.5s,
                                            std::bind(&NumberPublisher::publishNumber,this));
           RCLCPP_INFO(this->get_logger(), "NumberPublisher node created");
        }
    private:
    // Publisher for Int64 messages.
    rclcpp::Publisher<example_interfaces::msg::Int64>::SharedPtr publisher_;

    // Timer to schedule message publishing.
    rclcpp::TimerBase::SharedPtr timer_;
    void publishNumber()
    {
        auto msg = example_interfaces::msg::Int64();
        msg.data = 1;
        publisher_->publish(msg);
    }    
};

// Main function: initializes ROS2, creates the node, spins it, then shuts down.
int main(int argc,char **argv){
    rclcpp::init(argc,argv);
    auto node = std::make_shared<NumberPublisher>();
    rclcpp::spin(node); 
    rclcpp::shutdown();
    return 0;
}