#include "rclcpp/rclcpp.hpp"//for nodes
#include "example_interfaces/srv/add_two_ints.hpp" //for publishers and subscriber


using namespace std::placeholders;

class AddTwoIntsServerNode : public rclcpp::Node{
    public:
        AddTwoIntsServerNode() : Node("add_two_ints_server")
        { 
            //craete the service and connect to the "add_two_ints"    
            server_ = this->create_service<example_interfaces::srv::AddTwoInts>(
                    "add_two_ints",
                    std::bind(&AddTwoIntsServerNode::callbackAddTwoInts,this,_1,_2));
        RCLCPP_INFO(this->get_logger(),"add two ints service server node created ");

        }
    private:
    //make a server
    rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr server_;

    //make a call back that will process the request and give a response
    void callbackAddTwoInts( const example_interfaces::srv::AddTwoInts::Request::SharedPtr request,
                             const example_interfaces::srv::AddTwoInts::Response::SharedPtr response)
        {
            
            response->sum = request->a + request->b;
            // RCLCPP_INFO(this->get_logger(),"the sum is: %d + %d = %d",
            //             int(request->a),int(request->b),int(response->sum)); 
        }
};
int main(int argc,char **argv){
    rclcpp::init(argc,argv);
    auto node = std::make_shared<AddTwoIntsServerNode>();
    rclcpp::spin(node); 
    rclcpp::shutdown();
    return 0;
}