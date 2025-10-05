#include "rclcpp/rclcpp.hpp"//for nodes

class NodeNameClass : public rclcpp::Node{ //modify name
    public:
        NodeName() : Node("Node_name") //modify node name
        { 
           
        }
    private:
};

int main(int argc,char **argv){
    rclcpp::init(argc,argv);
    auto node = std::make_shared<NodeName>(); //modify name
    rclcpp::spin(node); 
    rclcpp::shutdown();
    return 0;
}