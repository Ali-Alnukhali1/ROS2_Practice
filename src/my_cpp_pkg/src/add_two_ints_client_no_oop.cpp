#include "rclcpp/rclcpp.hpp" //for nodes
#include "example_interfaces/srv/add_two_ints.hpp" //for services

using namespace std::chrono_literals;

int main(int argc,char **argv){
    rclcpp::init(argc,argv);
    //node name
    auto node = std::make_shared<rclcpp::Node>("add_two_ints_no_oop");

    //make a client and connect on the "add_two_ints"
    auto client = node->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");
  
    // if sever is not up yet wait 1 sec
   while(!client->wait_for_service(1s))
   {
        RCLCPP_INFO(node->get_logger(),"waiting for the server...");
   }
   
   //make the request a shared pointer so the server can access it
   auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
   request->a = 1;
   request->b = 2;

   //send the reque//modify namest to the server
   auto future = client->async_send_request(request);
   rclcpp::spin_until_future_complete(node,future);

   //obtain the response from the future
   auto response = std::make_shared<example_interfaces::srv::AddTwoInts::Response>();
   response = future.get(); 

   RCLCPP_INFO(node->get_logger(),"the sum is: %d + %d = %d",
                        int(request->a),int(request->b),int(response->sum));

   rclcpp::shutdown();
   return 0;
}