#include "rclcpp/rclcpp.hpp"//for nodes
#include "example_interfaces/srv/add_two_ints.hpp" //for services

using namespace std::chrono_literals;
using namespace std::placeholders;

class AddTwoIntsClientNode : public rclcpp::Node{ 
    public:
        AddTwoIntsClientNode() : Node("add_two_ints_client")
        { 
        //make a client and connect on the "add_two_ints"
         client_ = this->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");
        }

        //call this metod from main
        void addTwoInts(int a, int b)
        {   
            //if server is not up, wait 1 sec
            while(!client_->wait_for_service(1s))
            {
                RCLCPP_INFO(this->get_logger(),"waiting for the server...");
            } 
            
            //when the server is up assign the request 
            auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
            request->a = a;
            request->b = b;

            //send the request to the server and process the callback
            client_->async_send_request(request,std::bind(&AddTwoIntsClientNode::callbackCallAddTwoInts,this,_1));
        }
    private:

    void callbackCallAddTwoInts(const rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedFuture future){

        auto response =future.get();

        RCLCPP_INFO(this->get_logger(),"the sum is: %d", int(response->sum));
    }

    rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr client_;

};

int main(int argc,char **argv){
    rclcpp::init(argc,argv);
    auto node = std::make_shared<AddTwoIntsClientNode>();

    //add the two numbers
    node->addTwoInts(10,2);


    rclcpp::spin(node); 
    rclcpp::shutdown();
    return 0;
}