#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <geometry_msgs/msg/point.hpp>

class BallDetector : public rclcpp::Node {
public:
  BallDetector() : Node("ball_detector") {
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/panda/end_effector_camera/image_raw", 10, std::bind(&BallDetector::imageCallback, this, std::placeholders::_1));
    centroid_pub_ = this->create_publisher<geometry_msgs::msg::Point>("/ball_centroid", 10);
  }

private:
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    cv::Mat hsv;
    cv::cvtColor(cv_ptr->image, hsv, cv::COLOR_BGR2HSV);

    // Threshold for red
    cv::Mat mask1, mask2;
    cv::inRange(hsv, cv::Scalar(0, 70, 50), cv::Scalar(10, 255, 255), mask1);  // Lower red
    cv::inRange(hsv, cv::Scalar(170, 70, 50), cv::Scalar(180, 255, 255), mask2);  // Upper red
    cv::Mat mask = mask1 | mask2;

    // Find contours
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    if (!contours.empty()) {
      // Find largest contour
      auto max_contour = std::max_element(contours.begin(), contours.end(),
        [](const auto& a, const auto& b) { return cv::contourArea(a) < cv::contourArea(b); });
      cv::Moments m = cv::moments(*max_contour);
      if (m.m00 > 0) {
        geometry_msgs::msg::Point centroid;
        centroid.x = m.m10 / m.m00;  // u (pixel coords)
        centroid.y = m.m01 / m.m00;  // v
        centroid.z = 0.0;
        centroid_pub_->publish(centroid);
        RCLCPP_INFO(this->get_logger(), "Ball centroid: (%f, %f)", centroid.x, centroid.y);
      }
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr centroid_pub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BallDetector>());
  rclcpp::shutdown();
  return 0;
}
