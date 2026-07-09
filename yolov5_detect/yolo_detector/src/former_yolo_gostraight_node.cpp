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

class FormerYoloDetectorNode : public rclcpp::Node {
public:
    FormerYoloDetectorNode()
        : Node("former_yolo_detector_node") {
        // 初始化 BPU 推理器
        detector_ = std::make_shared<BPU_Detect>();
        if (!detector_->Init()) {
            RCLCPP_FATAL(this->get_logger(), "检测器初始化失败");
            rclcpp::shutdown();
            return;
        }

        // 订阅图像
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/former_usbcam/image_raw", 10,
            std::bind(&FormerYoloDetectorNode::image_callback, this, std::placeholders::_1));

        // 发布识别结果
        detect_result_pub_ = this->create_publisher<yolo_msgs::msg::Detection>("/former_yolo_detector/detection", 10);
        // 发布识别结果
        move_result_pub_ = this->create_publisher<std_msgs::msg::String>("/go_forward_result", 10);

        // 发布速度控制命令
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // 订阅楼层号
        next_floor_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/next_floor", 10,
            std::bind(&FormerYoloDetectorNode::next_floor_callback, this, std::placeholders::_1));

        // 发布启动舵机命令 
        stm32_serial_pub_ =  this->create_publisher<std_msgs::msg::String>("/serial/send", 10);

        call_back_yolo_pub_ =  this->create_publisher<std_msgs::msg::String>("/call_back_yolo", 10);

        param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this, "back_yolo_gostraight_node");

        RCLCPP_INFO(this->get_logger(), "FORMER_YOLO识别节点启动成功");
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

                        // 检测到 open，启动前进
                        if (current_class == "open") {
                            // 只有 next_floor_ 被赋值了（不为空），才允许移动
                            if (!next_floor_.empty()) {
                                RCLCPP_WARN(this->get_logger(), "next_floor 已收到，指令已就绪且检测到目标，开始移动！");
                                start_moving();
                            } else {
                                // 如果没收到指令，只打印提示，不执行 start_moving
                                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                                                    "识别到目标，但正在等待 next_floor 消息...");
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

            auto res_msg = std_msgs::msg::String();
            res_msg.data = "GoForwardSuccess";
            move_result_pub_->publish(res_msg);
            RCLCPP_INFO(this->get_logger(), "前进完成（%.2f 米），停止", dist);

            // 发送串口指令给 STM32
            auto serial_msg = std_msgs::msg::String();
            serial_msg.data = "1" + next_floor_; 
            stm32_serial_pub_->publish(serial_msg);
            RCLCPP_INFO(this->get_logger(), "前进完成,发送按键指令: %s", serial_msg.data.c_str());

            // 通知 back_yolo 节点开始识别
            auto call_back_yolo_msg = std_msgs::msg::String();
            call_back_yolo_msg.data = "1"; 
            call_back_yolo_pub_->publish(call_back_yolo_msg);
            RCLCPP_INFO(this->get_logger(), "向 back_yolo 节点发送任务开始指令: %s", call_back_yolo_msg.data.c_str());

            // 异步修改远程节点参数
            auto parameters = {rclcpp::Parameter("target_class", next_floor_)};
            param_client_->set_parameters(parameters, 
                [this](std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>> future) {
                    try {
                        auto results = future.get();
                        if (results[0].successful) {
                            RCLCPP_INFO(this->get_logger(), "远程节点参数修改成功！");
                        } else {
                            RCLCPP_ERROR(this->get_logger(), "远程节点拒绝修改参数: %s", results[0].reason.c_str());
                        }
                    } catch (const std::exception & e) {
                        RCLCPP_ERROR(this->get_logger(), "参数修改异常: %s", e.what());
                    }
                });
    

            // 关闭 Odom 订阅
            odom_sub_.reset();
            has_recorded_start_pos_ = false;
            next_floor_.clear(); // 清理楼层信息，防止下次误触发
            RCLCPP_INFO(this->get_logger(), "任务完成，指令已重置，等待下一个换层任务。");

             // 恢复图像订阅
            image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
                "/former_usbcam/image_raw", 10,
                std::bind(&FormerYoloDetectorNode::image_callback, this, std::placeholders::_1));

            RCLCPP_INFO(this->get_logger(), "恢复图像识别");
        } else {
            geometry_msgs::msg::Twist cmd;
            cmd.linear.x = FORWARD_SPEED;
            cmd_pub_->publish(cmd);
        }
    }

    void start_moving() {
        if (moving_) return;
        // 开始订阅 Odom
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom_combined", 10,
            std::bind(&FormerYoloDetectorNode::odom_callback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "开始订阅 Odom，准备移动");

        // 停止图像订阅（释放资源）
        image_sub_.reset();
        RCLCPP_INFO(this->get_logger(), "开始前进 1.5 米，暂停图像识别");
        moving_ = true;
    }

    void next_floor_callback(const std_msgs::msg::String::SharedPtr msg){
        next_floor_ = msg->data;
        RCLCPP_INFO(this->get_logger(), "收到目标楼层设定: %s", next_floor_.c_str());
    }

private:
    // ROS 接口
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<yolo_msgs::msg::Detection>::SharedPtr detect_result_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr move_result_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr stm32_serial_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr call_back_yolo_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr next_floor_sub_;
    rclcpp::AsyncParametersClient::SharedPtr param_client_;

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

    std::string next_floor_ = "";
    const double MOVE_DISTANCE = 1.5;      // 米
    const double FORWARD_SPEED = 0.8;      // m/s
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FormerYoloDetectorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
