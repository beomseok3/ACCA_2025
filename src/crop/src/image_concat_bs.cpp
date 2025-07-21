#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class SingleCameraNode : public rclcpp::Node {
public:
    SingleCameraNode() : Node("single_camera_node") {
        // 구독자 설정
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "camera1/image_raw", 10,
            std::bind(&SingleCameraNode::imageCallback, this, std::placeholders::_1));

        // 퍼블리셔 설정
        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("concated_cam", 10);
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            // 메시지 메타데이터 출력
            RCLCPP_INFO(this->get_logger(), "Image header: frame_id=%s, width=%d, height=%d, encoding=%s",
                        msg->header.frame_id.c_str(), msg->width, msg->height, msg->encoding.c_str());

            // 이미지 데이터 확인
            if (msg->data.empty()) {
                RCLCPP_ERROR(this->get_logger(), "Received empty image data.");
                return;
            }

            // 인코딩 확인 및 변환
            if (msg->encoding != sensor_msgs::image_encodings::BGR8) {
                RCLCPP_WARN(this->get_logger(), "Unexpected encoding: %s. Converting to BGR8.", msg->encoding.c_str());
            }

            // cv_bridge를 통해 이미지를 변환
            cv_image1_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

            // 변환된 이미지 데이터 확인
            if (cv_image1_->image.empty()) {
                RCLCPP_ERROR(this->get_logger(), "OpenCV image is empty after conversion!");
                return;
            }

            RCLCPP_INFO(this->get_logger(), "Image received: width=%d, height=%d",
                        cv_image1_->image.cols, cv_image1_->image.rows);

            // 이미지 퍼블리시 시도
            tryPublish();
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        } catch (const cv::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "OpenCV exception: %s", e.what());
        }
    }

    void tryPublish() {
        if (cv_image1_) {
        // OpenCV 이미지 디버깅용 로그 추가
        RCLCPP_INFO(this->get_logger(), "Publishing image: cols=%d, rows=%d",
                    cv_image1_->image.cols, cv_image1_->image.rows);

        // OpenCV 이미지를 파일로 저장 (디버깅)
        try {
            cv::imwrite("/tmp/debug_image.jpg", cv_image1_->image);
            RCLCPP_INFO(this->get_logger(), "Image saved to /tmp/debug_image.jpg for verification.");
        } catch (const cv::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to save image: %s", e.what());
        }

        // 메시지로 변환 후 퍼블리시
        auto concat_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", cv_image1_->image).toImageMsg();
        image_pub_->publish(*concat_msg);
        RCLCPP_INFO(this->get_logger(), "Image published to /concated_cam.");
        } else {
            RCLCPP_WARN(this->get_logger(), "Image not ready for publishing.");
        }
    }

    // 멤버 변수 선언
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    cv_bridge::CvImagePtr cv_image1_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SingleCameraNode>());
    rclcpp::shutdown();
    return 0;
}
