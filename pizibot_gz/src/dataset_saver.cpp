#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>

#include <filesystem>
#include <iomanip>
#include <sstream>
#include <memory>

/**
 * @brief ROS2 node to save a dataset of RGB images and binary masks.
 *
 * This node synchronizes RGB and semantic images, then generates binary masks
 * based on a specified semantic class. Images are saved only when the robot
 * has moved sufficiently (distance > 0.05m or rotation > 30°).
 *
 * @param label Semantic class label (pixels with this label -> black, others -> white)
 * @param topic_rgb RGB image topic
 * @param topic_sem Semantic image topic
 * @param dataset_path Output folder path (will contain images/ and masks/ subfolders)
 */

namespace fs = std::filesystem;

class DatasetSaver : public rclcpp::Node
{
public:
  DatasetSaver()
      : Node("dataset_saver"), counter_(0)
  {
    this->declare_parameter<int>("label", 1);
    this->declare_parameter<std::string>("topic_sem", "/camera/semantic/labels_map");
    this->declare_parameter<std::string>("topic_rgb", "/camera/image_raw");
    this->declare_parameter<std::string>("dataset_path", "dataset");

    label_ = this->get_parameter("label").as_int();
    topic_sem_ = this->get_parameter("topic_sem").as_string();
    topic_rgb_ = this->get_parameter("topic_rgb").as_string();
    dataset_path_ = this->get_parameter("dataset_path").as_string();

    // Output paths
    images_path_ = fs::path(dataset_path_) / "images";
    masks_path_ = fs::path(dataset_path_) / "masks";
    fs::create_directories(images_path_);
    fs::create_directories(masks_path_);

    std::ostringstream oss;
    oss << std::setw(6) << std::setfill('0') << counter_;
    const std::string fname = oss.str();
    fs::path last_mask_create_path = masks_path_ / (fname + ".png");

    RCLCPP_INFO(this->get_logger(), "counter initialization");
    while (fs::exists(last_mask_create_path))
    {
      counter_++;
      std::ostringstream oss;
      oss << std::setw(6) << std::setfill('0') << counter_;
      const std::string fname = oss.str();
      last_mask_create_path = masks_path_ / (fname + ".png");
    }

    RCLCPP_INFO(this->get_logger(), "counter initialized : %ld", counter_);

    rgb_sub_.subscribe(this, topic_rgb_);
    sem_sub_.subscribe(this, topic_sem_);

    tf_sub_ = this->create_subscription<tf2_msgs::msg::TFMessage>("/tf", 10, std::bind(&DatasetSaver::tf_callback, this, std::placeholders::_1));

    // --- Policy and Synchronizer ---
    // Typedef for policy (ApproximateTime to tolerate small timestamp offsets)
    using Img = sensor_msgs::msg::Image;
    using ApproxSyncPolicy = message_filters::sync_policies::ApproximateTime<Img, Img>;
    // Queue size = 10 (typical value, adjust according to frequency)
    sync_ = std::make_shared<message_filters::Synchronizer<ApproxSyncPolicy>>(ApproxSyncPolicy(10), rgb_sub_, sem_sub_);

    // Bind the synchronized callback
    sync_->registerCallback(std::bind(&DatasetSaver::syncCallback, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "Dataset saver ready: listening to %s and %s", topic_rgb_.c_str(), topic_sem_.c_str());
  }

private:
  // message_filters subscribers (types)
  message_filters::Subscriber<sensor_msgs::msg::Image> rgb_sub_;
  message_filters::Subscriber<sensor_msgs::msg::Image> sem_sub_;

  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub_;

  // Shared synchronizer (must be a member to stay alive)
  std::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<
      sensor_msgs::msg::Image, sensor_msgs::msg::Image>>>
      sync_;

  fs::path images_path_, masks_path_;
  size_t counter_;

  double current_odom_baselink_distance_ = 0.0;
  double last_odom_baselink_distance_ = 0.0;
  double current_odom_baselink_yaw_ = 0.0;
  double last_odom_baselink_yaw_ = 0.0;

  void tf_callback(const tf2_msgs::msg::TFMessage::ConstSharedPtr tf_msg)
  {
    for (const auto &t : tf_msg->transforms)
    {
      if (t.header.frame_id == "odom" and t.child_frame_id == "base_link")
      {
        double x = t.transform.translation.x;
        double y = t.transform.translation.y;
        double z = t.transform.translation.z;
        current_odom_baselink_distance_ = sqrt(x * x + y * y + z * z);

        current_odom_baselink_yaw_ = tf2::getYaw(t.transform.rotation) * 180.0 / M_PI;
      }
    }
  }

  // Synchronized callback: receives two messages with close timestamps
  void syncCallback(
      const sensor_msgs::msg::Image::ConstSharedPtr &rgb_msg,
      const sensor_msgs::msg::Image::ConstSharedPtr &sem_msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received synchronized images: rgb timestamp = %.3f, sem timestamp = %.3f",
                rclcpp::Time(rgb_msg->header.stamp).seconds(),
                rclcpp::Time(sem_msg->header.stamp).seconds());

    RCLCPP_INFO(this->get_logger(), "Current distance odom -> base_link = %.3f, yaw = %.1f deg",
                current_odom_baselink_distance_, current_odom_baselink_yaw_);

    // Save if displacement > 0.05m OR rotation > 30°
    if (abs(current_odom_baselink_distance_ - last_odom_baselink_distance_) > 0.05 || abs(current_odom_baselink_yaw_ - last_odom_baselink_yaw_) > 30)
    {
      RCLCPP_INFO(this->get_logger(), "Movement threshold exceeded, saving images...");
      last_odom_baselink_distance_ = current_odom_baselink_distance_;
      last_odom_baselink_yaw_ = current_odom_baselink_yaw_;

      // ROS to OpenCV conversion
      cv_bridge::CvImageConstPtr cv_rgb_ptr;
      cv_bridge::CvImageConstPtr cv_sem_ptr;
      try
      {
        cv_rgb_ptr = cv_bridge::toCvShare(rgb_msg, "bgr8");
        // 'passthrough' retrieves the native type (often CV_32SC1 for labels)
        cv_sem_ptr = cv_bridge::toCvShare(sem_msg);
        RCLCPP_INFO(this->get_logger(), "Converted images to OpenCV format: rgb type = %d, sem type = %d",
                    cv_rgb_ptr->image.type(), cv_sem_ptr->image.type());
      }
      catch (const cv_bridge::Exception &e)
      {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
      }

      const cv::Mat &rgb = cv_rgb_ptr->image;
      const cv::Mat &sem = cv_sem_ptr->image;

      // Build binary mask: pixels with label_ -> 0 (black), others -> 255 (white)
      cv::Mat mask = (sem != label_);
      if (mask.type() != CV_8U)
      {
        mask.convertTo(mask, CV_8U);
      }
      // If (sem != label_) gives 0/1, scale it to 0/255:
      mask *= 255;

      RCLCPP_INFO(this->get_logger(), "Generated binary mask from semantic image");

      // Save files (sequential naming)
      std::ostringstream oss;
      oss << std::setw(6) << std::setfill('0') << counter_;
      const std::string fname = oss.str();

      cv::imwrite(images_path_ / (fname + ".png"), rgb);
      cv::imwrite(masks_path_ / (fname + ".png"), mask);

      RCLCPP_INFO(this->get_logger(), "%zu images saved", counter_);

      counter_++;
    }
  }

  // Configuration parameters
  int label_; // The class label that will be black in the binary image, all others will be white
  std::string topic_sem_;
  std::string topic_rgb_;
  std::string dataset_path_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DatasetSaver>());
  rclcpp::shutdown();
  return 0;
}