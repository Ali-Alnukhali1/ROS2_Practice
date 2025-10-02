#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int64.hpp"

using namespace std::chrono_literals;

// NumberCounter node subscribes to "number" topic, logs received values,
// and publishes an incrementing counter value every 500 milliseconds
// on the "number_count" topic.
class NumberCounter : public rclcpp::Node{
    public:
        // Constructor: initializes subscriber, publisher, and timer.
        NumberCounter() : Node("number_counter"), counter_(0)
        { 
            // Create subscriber to listen for Int64 messages on "number".
            subscriber_ = this->create_subscription<example_interfaces::msg::Int64> (
            "number", 10,
            std::bind(&NumberCounter::callbackNumber,this,std::placeholders::_1) // one argument
         ); 
           // Create publisher to publish counter values on "number_count".
           publisher_= this->create_publisher<example_interfaces::msg::Int64>("number_count",10);
           
           // Timer triggers publishNumberCount() every 500 ms.
           timer_ = this->create_wall_timer(0.5s,
                                            std::bind(&NumberCounter::publishNumberCount,this));
           RCLCPP_INFO(this->get_logger(), "NumberCounter node created");
        }
    private:
    // Subscription to "number" topic.
    rclcpp::Subscription<example_interfaces::msg::Int64>::SharedPtr subscriber_ ;
    
    // Counter value to be published.
    int counter_;

    // Publisher for counter messages.
    rclcpp::Publisher<example_interfaces::msg::Int64>::SharedPtr publisher_;
    
    // Timer for publishing counter values.
    rclcpp::TimerBase::SharedPtr timer_;

    // Callback for received messages on "number".
    void callbackNumber(const example_interfaces::msg::Int64::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(),"%ld",msg->data);
    }
     
    // Publishes incrementing counter value on "number_count".
    void publishNumberCount()
    {
        auto msg = example_interfaces::msg::Int64();
        msg.data = counter_++;
        publisher_->publish(msg);
    }  
    
};

// Main function: initializes ROS2, runs the node, then shuts down.
int main(int argc,char **argv){
    rclcpp::init(argc,argv);
    auto node = std::make_shared<NumberCounter>();
    rclcpp::spin(node); 
    rclcpp::shutdown();
    return 0;
}