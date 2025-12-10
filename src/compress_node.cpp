#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <image_transport/image_transport.hpp>
#include <image_transport/transport_hints.hpp>

class ImageCompressor : public rclcpp::Node {
public:
  ImageCompressor() : Node("image_compressor")
  {
    // BEST-EFFORT QoS profile for camera images
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;

    // Transport hint for raw input
    image_transport::TransportHints hints(this, "raw");

    sub_ = image_transport::create_subscription(
        this,
        "/triton_left/images",
        std::bind(&ImageCompressor::callback, this, std::placeholders::_1),
        hints.getTransport(),
        qos_profile);

    // Publisher with compressed transport
    pub_ = image_transport::create_publisher(
        this,
        "/triton_left_compressed");

    RCLCPP_INFO(this->get_logger(), "Image compressor started.");
    
  }

private:
  void log_debug(std::string msg) { RCLCPP_DEBUG(this->get_logger(), msg.c_str()); };
  void log_info(std::string msg) { RCLCPP_INFO(this->get_logger(), msg.c_str()); };
  void log_warn(std::string msg) { RCLCPP_WARN(this->get_logger(), msg.c_str()); };
  void log_err(std::string msg) { RCLCPP_ERROR(this->get_logger(), msg.c_str()); };


  void callback(const sensor_msgs::msg::Image::ConstSharedPtr &msg)
{
    // If no one is listening to the compressed topic, don't waste CPU copying/compressing.
    if (pub_.getNumSubscribers() == 0) {
      return;
    }
    // Make a copy of the incoming message
    // sensor_msgs::msg::Image out_msg = *msg;
    // std::string og_frame_id = out_msg.header.frame_id;

    // Force a stable frame ID so RViz stops dropping messages
    // out_msg.header.frame_id = "triton_left_camera";

    // pub_.publish(out_msg);
    pub_.publish(msg);
    // log_info(std::string("image ") + og_frame_id +
    //            " published to " + "/triton_left_compressed");
}


  image_transport::Subscriber sub_;
  image_transport::Publisher pub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImageCompressor>());
  rclcpp::shutdown();
  return 0;
}
