#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav2_msgs/srv/load_map.hpp"
#include "nav2_msgs/srv/clear_entire_costmap.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"  
#include <map>
#include <string>
#include <vector>
#include <memory>
#include <rclcpp/timer.hpp>

class MapSwitcherNode : public rclcpp::Node
{
public:
  MapSwitcherNode(): Node("map_switcher_node")
  {
    // 初始化地图路径与初始位姿（可改成读取 YAML）
    map_paths_ = {
      {4, "/home/wheeltec/wheeltec_ros2/src/lanfanqie/stm32_map/maps/laboratory_corridor/floor4.yaml"},
      {6, "/home/wheeltec/wheeltec_ros2/src/lanfanqie/stm32_map/maps/laboratory_corridor/floor6.yaml"},
    };

    initial_poses_ = {
      {2, {0.0, 0.0, 0.0}}, 
      {3, {0.0, 0.0, 0.0}}, 
      {4, {0.0, 0.0, 0.0}},        // x, y, yaw for floor 4
      {6, {0.0, 0.0, 0.0}},       // floor 6    
    };

    map_client_ = this->create_client<nav2_msgs::srv::LoadMap>("/map_server/load_map");
    status_pub_ = this->create_publisher<std_msgs::msg::String>("/map_switch_status", 10);
    pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10);
    local_clear_client_ = this->create_client<nav2_msgs::srv::ClearEntireCostmap>("/local_costmap/clear_entirely_local_costmap");
    global_clear_client_ = this->create_client<nav2_msgs::srv::ClearEntireCostmap>("/global_costmap/clear_entirely_global_costmap");
    sub_ = this->create_subscription<std_msgs::msg::String>(
      "/mapswitch", 10,
      std::bind(&MapSwitcherNode::map_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "地图切换节点已启动。");
  }

private:
  void map_callback(const std_msgs::msg::String::SharedPtr msg)
  {

    char floor_char = msg->data[1];
    int floor = floor_char - '0';
    if (map_paths_.find(floor) == map_paths_.end()) {
      RCLCPP_WARN(this->get_logger(), "未知楼层编号：%d", floor);
      return;
    }

    std::string map_path = map_paths_[floor];
    RCLCPP_INFO(this->get_logger(), "切换至楼层 %d，加载地图：%s", floor, map_path.c_str());

    // 调用 LoadMap 服务
    auto request = std::make_shared<nav2_msgs::srv::LoadMap::Request>();
    request->map_url = map_path;

    while (rclcpp::ok() && !map_client_->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_WARN(this->get_logger(), "等待 map_server 服务中...");
    }

    // 异步调用 LoadMap 服务，并注册回调函数处理响应，this 捕获允许你访问类的所有成员变量和成员函数，包括 initial_poses_
    // floor 是局部变量，捕获是为了在 Lambda 里用这个具体的楼层编号
    map_client_->async_send_request(request,
      [this, floor](rclcpp::Client<nav2_msgs::srv::LoadMap>::SharedFuture future)
      {
        try {
          auto response = future.get();  // 尝试获取响应
    
          // 服务响应到了，但返回的是失败状态码
          // 从 future 中取出真正的响应结果（阻塞很短，但这在回调中是安全的）
          if (response->result != nav2_msgs::srv::LoadMap::Response::RESULT_SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "地图加载失败，错误码：%d", response->result);
            auto fail_msg = std_msgs::msg::String();
            fail_msg.data = "MAP_SWITCH_FAILED: " + std::to_string(floor);
            status_pub_->publish(fail_msg);
            return;
          }
    
          // 地图加载成功，发布初始位姿
          RCLCPP_INFO(this->get_logger(), "地图加载成功，准备发布初始位姿。");
          publish_initial_pose(initial_poses_[floor]);
          RCLCPP_INFO(this->get_logger(), "地图已加载，位姿已发布。等待定位系统稳定...");
          // 新增清除 costmap
          if (process_timer_) process_timer_->cancel(); // 如果有正在跑的定时器，重置它
          process_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000), // 建议 1000ms 比较保险
            [this, floor]() {on_timer_complete(floor);}); 
        } catch (const std::exception &e) {
          // future.get() 过程中出错（比如服务断开）
          RCLCPP_ERROR(this->get_logger(), "调用 map_server/load_map 服务失败，异常：%s", e.what());
          auto fail_msg = std_msgs::msg::String();
          fail_msg.data = "MAP_SWITCH_FAILED: " + std::to_string(floor);
          status_pub_->publish(fail_msg);
        }
      });    
  }

   void on_timer_complete(int floor) {
    // 定时器触发，说明 AMCL 应该已经处理完 InitialPose 了
    process_timer_->cancel(); // 停止定时器（它只需要跑一次）

    // 第四步：清理 Costmap
    clear_costmaps();
    RCLCPP_INFO(this->get_logger(), "代价地图已清理。");

    // 第五步：通知 OpenClaw 任务彻底完成
    auto status_msg = std_msgs::msg::String();
    status_msg.data = "MAP_SWITCH_COMPLETED: " + std::to_string(floor);
    status_pub_->publish(status_msg);

    RCLCPP_INFO(this->get_logger(), "---%s 楼层切换全流程完成，可以开始新导航 ---",status_msg.data.c_str());
  }

  void publish_initial_pose(const std::vector<float>& pose_data)
  {
    geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
    pose_msg.header.frame_id = "map";
    pose_msg.header.stamp = this->get_clock()->now();

    pose_msg.pose.pose.position.x = pose_data[0];
    pose_msg.pose.pose.position.y = pose_data[1];

    tf2::Quaternion q;
    q.setRPY(0, 0, pose_data[2]);
    pose_msg.pose.pose.orientation.x = q.x();
    pose_msg.pose.pose.orientation.y = q.y();
    pose_msg.pose.pose.orientation.z = q.z();
    pose_msg.pose.pose.orientation.w = q.w();

    // 设置协方差矩阵
    pose_msg.pose.covariance[0] = 0.25;
    pose_msg.pose.covariance[7] = 0.25;
    pose_msg.pose.covariance[35] = 0.0685389;

    pose_publisher_->publish(pose_msg);
    RCLCPP_INFO(this->get_logger(), "初始位姿已发布。");
  }

  void clear_costmaps()
  {
    auto request = std::make_shared<nav2_msgs::srv::ClearEntireCostmap::Request>();

    // 等待服务可用
    while (!local_clear_client_->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_WARN(this->get_logger(), "等待 /local_costmap/clear_entirely_local_costmap 服务...");
    }
    while (!global_clear_client_->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_WARN(this->get_logger(), "等待 /global_costmap/clear_entirely_global_costmap 服务...");
    }

    // 异步请求清除 local costmap
    local_clear_client_->async_send_request(request,
      [this](rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedFuture future)
      {
        try {
          future.get();  // 空服务没返回字段，但出错会抛异常
          RCLCPP_INFO(this->get_logger(), "Local Costmap 清除成功！");
        } catch (const std::exception &e) {
          RCLCPP_ERROR(this->get_logger(), "Local Costmap 清除失败：%s", e.what());
        }
      });    

    // 异步请求清除 global costmap
    global_clear_client_->async_send_request(request,
      [this](rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedFuture future)
      {
        try {
          future.get();  // 空服务没返回字段，但出错会抛异常
          RCLCPP_INFO(this->get_logger(), "Global Costmap 清除成功！");
        } catch (const std::exception &e) {
          RCLCPP_ERROR(this->get_logger(), "Global Costmap 清除失败：%s", e.what());
        }
      });
    
  }

  std::map<int, std::string> map_paths_;
  std::map<int, std::vector<float>> initial_poses_;
  rclcpp::Client<nav2_msgs::srv::LoadMap>::SharedPtr map_client_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr local_clear_client_;
  rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr global_clear_client_;
  rclcpp::TimerBase::SharedPtr process_timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MapSwitcherNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
