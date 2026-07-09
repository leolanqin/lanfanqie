#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <cv_bridge/cv_bridge.h>

#include "yolo_msgs/msg/detection.hpp"
#include "yolo_msgs/msg/bounding_box.hpp"
#include "std_msgs/msg/header.hpp"

#include "yolo_detector/bpu_detect.hpp"  // 你的推理类

#include <cmath>

class BackYoloDetectorNode : public rclcpp::Node {
public:
    BackYoloDetectorNode()
        : Node("back_yolo_detector_node") {
        // 初始化 BPU 推理器
        detector_ = std::make_shared<BPU_Detect>();
        if (!detector_->Init()) {
            RCLCPP_FATAL(this->get_logger(), "检测器初始化失败");
            rclcpp::shutdown();
            return;
        }

        target_class_=this->declare_parameter<std::string>("target_class", "4");  // 默认值为 "4"

        // 订阅图像
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/back_usbcam/image_raw", 10,
            std::bind(&BackYoloDetectorNode::image_callback, this, std::placeholders::_1));

        call_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/call_back_yolo", 10,
            std::bind(&BackYoloDetectorNode::call_back_yolo_callback, this, std::placeholders::_1));

        // 发布识别结果
        detect_result_pub_ = this->create_publisher<yolo_msgs::msg::Detection>("/back_yolo_detector/detection", 10);
        move_result_pub_ = this->create_publisher<std_msgs::msg::String>("/go_backward_result", 10);

        // 发布速度控制命令
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // 发布地图切换命令 
        mapswitch_pub_ =  this->create_publisher<std_msgs::msg::String>("/mapswitch", 10);
        
        RCLCPP_INFO(this->get_logger(), "BACK_YOLO识别节点启动成功，目标识别数字设置为: %s", target_class_.c_str());
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        if (moving_) return;  // 移动中不再识别，避免重入冲突

        try {
            cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
            cv::Mat output_img;
            bool success = detector_->Detect(frame, output_img);

            if (success) {
                std::vector<std::string> detected_classes;
                for (size_t cls_id = 0; cls_id < detector_->GetClassNames().size(); ++cls_id) {
                    for (int i : detector_->GetIndices()[cls_id]) {
                        detected_classes.push_back(detector_->GetClassNames()[cls_id]);
                    }
                }

                if (!detected_classes.empty()) {
                    std::string current_class = detected_classes[0];

                    if (current_class == last_detected_class_) {
                        stable_count_++;
                    } else {
                        last_detected_class_ = current_class;
                        stable_count_ = 1;
                    }

                    if (stable_count_ >= STABLE_THRESHOLD) {
                        // 发布检测消息
                        yolo_msgs::msg::Detection detection_msg;
                        detection_msg.header.stamp = msg->header.stamp;
                        detection_msg.header.frame_id = msg->header.frame_id;

                        for (size_t cls_id = 0; cls_id < detector_->GetClassNames().size(); ++cls_id) {
                            for (int i : detector_->GetIndices()[cls_id]) {
                                const auto& box = detector_->GetBBoxes()[cls_id][i];
                                float score = detector_->GetScores()[cls_id][i];

                                yolo_msgs::msg::BoundingBox bbox;
                                bbox.class_name = detector_->GetClassNames()[cls_id];
                                bbox.score = score;
                                bbox.xmin = box.x;
                                bbox.ymin = box.y;
                                bbox.xmax = box.x + box.width;
                                bbox.ymax = box.y + box.height;

                                detection_msg.boxes.push_back(bbox);
                            }
                        }

                        detect_result_pub_->publish(detection_msg);
                        RCLCPP_INFO(this->get_logger(), "已发布 %lu 个目标（类别: %s，稳定 %d 次）",
                                    detection_msg.boxes.size(), current_class.c_str(), stable_count_);

                        target_class_ = this->get_parameter("target_class").as_string();

                        // 检测到目标房间号，启动后退
                        if (current_class == target_class_) {
                             // 只有 call_back_yolo_ 被赋值了（不为空），才允许移动
                            if (!call_back_yolo_.empty()) {
                                RCLCPP_WARN(this->get_logger(), "指令已就绪且检测到目标，开始移动！");
                                start_moving();
                            } else {
                                // 如果没收到指令，只打印提示，不执行 start_moving
                                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                                                    "识别到目标，但正在等待 /call_back_yolo_ 的楼层指令...");
                            }
                        }

                        stable_count_ = 0;
                        last_detected_class_.clear();
                    } else {
                        RCLCPP_INFO(this->get_logger(), "检测到 %s，但未达到稳定阈值(%d/%d)",
                                    current_class.c_str(), stable_count_, STABLE_THRESHOLD);
                    }
                } else {
                    RCLCPP_WARN(this->get_logger(), "检测成功但无有效目标");
                    stable_count_ = 0;
                    last_detected_class_.clear();
                }

            } else {
                RCLCPP_WARN(this->get_logger(), "检测失败");
                stable_count_ = 0;
                last_detected_class_.clear();
            }

        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "图像转换错误: %s", e.what());
        }
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {

        if (!has_recorded_start_pos_) {
            move_start_x_ = msg->pose.pose.position.x;
            move_start_y_ = msg->pose.pose.position.y;
            has_recorded_start_pos_ = true;
    
            RCLCPP_INFO(this->get_logger(), "记录起始位置: x=%.2f, y=%.2f", move_start_x_, move_start_y_);
            return;  // 第一帧只记录位置，不控制
        }

        // 计算移动距离
        double dx = msg->pose.pose.position.x - move_start_x_;
        double dy = msg->pose.pose.position.y - move_start_y_;
        double dist = std::sqrt(dx * dx + dy * dy);

        if (dist >= MOVE_DISTANCE) {
            geometry_msgs::msg::Twist stop_msg;
            cmd_pub_->publish(stop_msg);
            moving_ = false;

            // 发布移动成功的反馈消息
            auto move_res_msg = std_msgs::msg::String();
            move_res_msg.data = "GoBackwardSuccess";
            move_result_pub_->publish(move_res_msg);
            RCLCPP_INFO(this->get_logger(), "后退完成（%.2f 米），停止", dist);

            // 关闭 Odom 订阅
            odom_sub_.reset();
            has_recorded_start_pos_ = false;
            call_back_yolo_.clear(); // 清理信息，防止下次误触发
            RCLCPP_INFO(this->get_logger(), "任务完成，指令已重置。");

            // 发布地图切换消息（变量名改为 map_msg）
            auto map_msg = std_msgs::msg::String();
            map_msg.data = "F" + target_class_;   // 拼接得到如 "F4"
            mapswitch_pub_->publish(map_msg);
            RCLCPP_INFO(this->get_logger(), "发布地图切换命令，开始加载 %s 楼地图", target_class_.c_str());

             // 恢复图像订阅
            image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
                "/back_usbcam/image_raw", 10,
                std::bind(&BackYoloDetectorNode::image_callback, this, std::placeholders::_1));

            RCLCPP_INFO(this->get_logger(), "恢复图像识别");
        } else {
            geometry_msgs::msg::Twist cmd;
            cmd.linear.x = BACKWARD_SPEED;
            cmd_pub_->publish(cmd);
        }
    }

    void call_back_yolo_callback(const std_msgs::msg::String::SharedPtr msg){
          if ( !msg->data.empty() && msg->data=="1") {
            call_back_yolo_= msg->data;
            RCLCPP_INFO(this->get_logger(), "收到 former_yolo 指令: %s", call_back_yolo_.c_str());
        } 
    }

    void start_moving() {
        if (moving_) return;
        // 开始订阅 Odom
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom_combined", 10,
            std::bind(&BackYoloDetectorNode::odom_callback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "开始订阅 Odom，准备移动");

        // 停止图像订阅（释放资源）
        image_sub_.reset();
        RCLCPP_INFO(this->get_logger(), "开始后退 1.5 米，暂停图像识别");
        moving_ = true;
    }

private:
    // ROS 接口
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr call_sub_;
    rclcpp::Publisher<yolo_msgs::msg::Detection>::SharedPtr detect_result_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr  mapswitch_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr move_result_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    // 推理器
    std::shared_ptr<BPU_Detect> detector_;
    std::string last_detected_class_;
    int stable_count_ = 0;
    const int STABLE_THRESHOLD = 5;

    // 控制逻辑
    bool moving_ = false;
    bool has_recorded_start_pos_ = false;
    double move_start_x_ = 0.0;
    double move_start_y_ = 0.0;
    nav_msgs::msg::Odometry last_odom_;

    std::string call_back_yolo_ = "";
    std::string target_class_ = "";
    const double MOVE_DISTANCE = 1.5;      // 米
    const double BACKWARD_SPEED = -0.8;      // m/s
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BackYoloDetectorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
