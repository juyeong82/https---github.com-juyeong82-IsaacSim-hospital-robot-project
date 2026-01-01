#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "apriltag_msgs/msg/april_tag_detection_array.hpp" // 표준 메시지 헤더
#include "geometry_msgs/msg/pose_stamped.hpp"

using std::placeholders::_1;

class DockPosePublisher : public rclcpp::Node
{
  public:
    DockPosePublisher()
    : Node("dock_pose_publisher")
    {
      // [수정] 토픽 이름을 표준 패키지 출력(/detections)에 맞춤
      subscription_ = this->create_subscription<apriltag_msgs::msg::AprilTagDetectionArray>(
      "/detections", 10, std::bind(&DockPosePublisher::detectionCallback, this, _1));
      
      publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("detected_dock_pose", 10);
      
      use_first_detection_ = this->declare_parameter("use_first_detection", false);
      dock_tag_id_ = this->declare_parameter("dock_tag_id", 4);
    }

  private:
    // [수정] 메시지 타입 변경 (isaac_ros -> apriltag_msgs)
    void detectionCallback(const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg)
    {
      geometry_msgs::msg::PoseStamped p;
      for (const auto& detection : msg->detections) {
        if (!use_first_detection_) {
            // 표준 메시지에서 id는 배열이지만, 단일 태그의 경우 첫번째 요소(id[0])를 확인
            if (!detection.id.empty() && detection.id[0] == dock_tag_id_) {
                p.header = msg->header;
                p.pose = detection.pose.pose.pose;
                publisher_->publish(p);
                return;
            }
        } else {
          p.header = msg->header;
          p.pose = detection.pose.pose.pose;
          publisher_->publish(p);
          return;
        }
      }
    }

    int dock_tag_id_;
    bool use_first_detection_;
    rclcpp::Subscription<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DockPosePublisher>());
  rclcpp::shutdown();
  return 0;
}