#include <rclcpp/rclcpp.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/float32.hpp>

#include <iostream>
#include <memory>

using namespace std::placeholders;

class MessageFilterTest : public rclcpp::Node
{
public:
  MessageFilterTest() : Node("test") 
  {
    rclcpp::Node::SharedPtr node = std::shared_ptr<rclcpp::Node>(this);
    cam_sub_ = std::make_unique<camSub>(node, "/camera/color/image_raw");
    obj_sub_ = std::make_unique<objSub>(node, "/movidius_ncs_stream/detected_objects");
    sync_sub_ = std::make_unique<sync>(*cam_sub_, *obj_sub_, 10);
    sync_sub_->registerCallback(std::bind(&MessageFilterTest::callback, this));
  };
  ~MessageFilterTest() = default;

private:
  using camSub = message_filters::Subscriber<sensor_msgs::msg::Image>;
  using objSub = message_filters::Subscriber<std_msgs::msg::Float32>;
  using sync =
    message_filters::TimeSynchronizer<sensor_msgs::msg::Image, std_msgs::msg::Float32>;

  std::unique_ptr<camSub> cam_sub_;
  std::unique_ptr<objSub> obj_sub_;
  std::unique_ptr<sync> sync_sub_;

  void callback() {};
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  std::cout << "hello world" << std::endl;

  return 0;
}
