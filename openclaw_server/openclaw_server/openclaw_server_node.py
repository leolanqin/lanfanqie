import math
import threading
import time
import uuid
from datetime import datetime
from flask import Flask, request, jsonify

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import Quaternion

import requests
import os
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2


class OpenClawNavBridge(Node):
    def __init__(self):
        super().__init__("openclaw_nav_bridge")

        # 1. 导航 Action 客户端
        self._action_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self._nav_done_event = threading.Event()
        self._nav_result = False

        # 2. 状态锁
        self.is_busy = False

        self.bridge = CvBridge()
        self._latest_image = None
        self._goal_counter = 0
        self.create_subscription(Image, '/former_usbcam/image_raw', self.image_cb, 10)

        # 向 STM32 发送消息的发布者
        self.serial_pub = self.create_publisher(String, '/serial/send', 10)

        # 向 former_yolo 发送消息的发布者
        self.next_floor_pub = self.create_publisher(String, '/next_floor', 10)

        # 3. 订阅 MAP_SWITCH_COMPLETED 消息
        self._floor_switch_done_event = threading.Event()
        self._floor_switch_success = False
        self.create_subscription(
            String,
            '/map_switch_status',
            self.floor_switch_callback,
            10
        )

        # 4. 服务器地址（inspection_server，用来回调 floor_changed）
        self.server_url = "http://100.96.243.113:8001"

        # 5. 任务 ID：由外部请求头传入
        self._current_task_id = None

        # 6. 启动 Flask 服务
        self.app = Flask(__name__)
        self.setup_routes()
        self.flask_thread = threading.Thread(target=self.run_flask)
        self.flask_thread.daemon = True
        self.flask_thread.start()

        self.get_logger().info("OpenClaw Server Node initialized.")

    def image_cb(self, msg):
        self._latest_image = msg

    def take_photo_and_upload(self, goal_index=None):
        if self._latest_image is None:
            self.get_logger().warn("No image received yet, skipping photo")
            return False

        if self._current_task_id is None:
            self.get_logger().error("task_id not set! Upload aborted.")
            return False

        # 有 goal_index 则用（来自 plan 索引），否则用内部计数器兜底
        idx = goal_index if goal_index is not None else self._goal_counter
        self._goal_counter = max(self._goal_counter, idx + 1)  # 保证计数器不落后

        try:
            cv_img = self.bridge.imgmsg_to_cv2(self._latest_image, 'bgr8')
            # 使用 UUID 避免并发冲突
            tmp_uuid = uuid.uuid4().hex[:8]
            tmp_path = f'/tmp/insp_{idx}_{tmp_uuid}.jpg'
            cv2.imwrite(tmp_path, cv_img)

            url = f"{self.server_url}/upload"
            task_id = self._current_task_id

            with open(tmp_path, 'rb') as f:
                try:
                    r = requests.post(
                        url,
                        files={"image": f},
                        data={
                            "goal_index": str(idx),
                            "task_id": task_id
                        },
                        timeout=10
                    )
                    success = r.ok
                    if not success:
                        self.get_logger().error(f"Upload failed: status={r.status_code}, text={r.text}")
                except Exception as e:
                    self.get_logger().error(f"Upload exception: {e}")
                    success = False
        finally:
            # 确保清理临时文件
            if 'tmp_path' in locals():
                try:
                    os.remove(tmp_path)
                except OSError as e:
                    self.get_logger().warn(f"Failed to remove temp file {tmp_path}: {e}")

        return success

    def yaw_to_quaternion(self, yaw):
        q = Quaternion()
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q

    def navigate_to_point(self, x, y, yaw):
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            return False, "Nav2 server not found"

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.orientation = self.yaw_to_quaternion(yaw)

        self._nav_done_event.clear()
        send_goal_future = self._action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)
        self._nav_done_event.wait()
        return self._nav_result, ("Success" if self._nav_result else "Navigation failed")

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self._nav_result = False
            self._nav_done_event.set()
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        status = future.result().status
        self._nav_result = (status == 4)  # SUCCEEDED = 4
        self._nav_done_event.set()

    def send_to_stm32(self, message):
        """导航到电梯口后，通知 STM32"""
        msg = String()
        msg.data = message
        self.serial_pub.publish(msg)
        self.get_logger().info(f"发送至 STM32: {message}，开启电梯门")

    def send_next_floor(self, message):
        """导航到电梯口后，通知 former_yolo"""
        msg = String()
        msg.data = message
        self.next_floor_pub.publish(msg)
        self.get_logger().info(f"发送目标楼层(next_floor):{message} 至 former_yolo")

    def floor_switch_callback(self, msg):
        """
        收到 MAP_SWITCH_COMPLETED 时触发
        自动调用服务器 /floor_changed 接口
        消息格式：
          "MAP_SWITCH_COMPLETED:<floor>"
          "MAP_SWITCH_FAILED:<floor>"
        """
        if not (msg.data.startswith("MAP_SWITCH_COMPLETED") or msg.data.startswith("MAP_SWITCH_FAILED")): 
            return

        is_success = msg.data.startswith("MAP_SWITCH_COMPLETED") 

        # 必须包含冒号
        if ':' not in msg.data:
            self.get_logger().warn(f"消息格式错误，缺少楼层号: {msg.data}")
            return

        try:
            floor = int(msg.data.split(':')[1])
        except (ValueError, IndexError):
            self.get_logger().warn(f"无法解析楼层号: {msg.data}")
            return

        if is_success:
            self.get_logger().info(f"收到 MAP_SWITCH_COMPLETED，目标楼层: {floor}，通知服务器...")
            try:
                r = requests.post(
                    f"{self.server_url}/floor_changed",
                    json={"floor": floor, "success": True},
                    timeout=10
                )
                self.get_logger().info(f"服务器响应: {r.status_code} {r.text[:200]}")
            except Exception as e:
                self.get_logger().error(f"通知服务器失败: {e}")
            self._floor_switch_success = True
        else:                                                                                               # [MODIFIED] 新增失败处理分支
            self.get_logger().error(f"收到 MAP_SWITCH_FAILED，目标楼层: {floor}，换层失败！")
            self._floor_switch_success = False 

        #  set() 从 if/else 内部移到外面，无论成功失败都释放等待，不再硬等超时
        self._floor_switch_done_event.set()

    def wait_for_floor_switch(self, timeout_sec= 60):
        """
        等待 MAP_SWITCH_COMPLETED 消息
        返回 True/False
        """
        self._floor_switch_done_event.clear()
        self._floor_switch_success = False

        self.get_logger().info("等待换层完成...")
        ok = self._floor_switch_done_event.wait(timeout=timeout_sec)

        if ok:
            self.get_logger().info("换层完成")
        else:
            self.get_logger().error("换层超时")

        return ok and self._floor_switch_success

    # --- Flask 路由 ---

    def setup_routes(self):
        @self.app.route('/nav/goal', methods=['POST'])
        def receive_single_goal():
            if self.is_busy:
                return jsonify({"status": "error", "message": "Robot is busy"}), 429

            # 从请求头获取 task_id（推荐：由 run-inspection.sh 传入）
            task_id = request.headers.get('X-Task-ID')

            # 注入实例变量，供 upload 使用
            if task_id and task_id.startswith('insp_'):
                if task_id != self._current_task_id:
                    self._goal_counter = 0  # 新任务重置计数器
                self._current_task_id = task_id
            else:
                # 没有合法 task_id 清空，后续拍照上传会跳过
                self._current_task_id = None

            data = request.get_json()
            x = data.get('x')
            y = data.get('y')
            yaw = data.get('yaw', 0.0)
            is_elevator = data.get('is_elevator', False)
            next_floor = data.get('next_floor', None)
            goal_index = data.get('goal_index', None)

            if x is None or y is None:
                return jsonify({"status": "error", "message": "Invalid coordinates"}), 400

            self.is_busy = True
            try:
                success, msg = self.navigate_to_point(x, y, yaw)

                if success:
                    if is_elevator and next_floor is not None:
                        self.send_to_stm32("0" + str(next_floor))
                        self.get_logger().info(f"发送楼层切换指令至 STM32: 前往 {next_floor} 层")
                        self.send_next_floor(str(next_floor))
                        return jsonify({
                            "status": "success",
                            "message": "At elevator, switching floor"
                        }), 200
                    elif is_elevator:
                        self.send_to_stm32("NAV_ELEVATOR")
                        return jsonify({
                            "status": "success",
                            "message": "At elevator"
                        }), 200               
                    upload_success = self.take_photo_and_upload(goal_index=goal_index)    
                    if upload_success:
                        return jsonify({
                            "status": "success",
                            "message": "Reached and uploaded",
                            "task_id": task_id
                        }), 200
                    else:
                        return jsonify({
                            "status": "partial_success",
                            "message": "Reached but upload failed",
                            "task_id": task_id
                        }), 500
                else:
                    return jsonify({"status": "failed", "message": msg}), 500
            finally:
                self.is_busy = False

        @self.app.route('/floor_change', methods=['POST'])
        def handle_floor_change():
            if self.is_busy:
                return jsonify({"status": "error", "message": "Robot is busy"}), 429

            data = request.get_json()
            next_floor = data.get('next_floor')
            if next_floor is None:
                return jsonify({"status": "error", "message": "Missing next_floor"}), 400

            self.is_busy = True
            try:
                success = self.wait_for_floor_switch(timeout_sec=60)

                if success:
                    return jsonify({
                        "status": "success",
                        "message": f"Switched to floor {next_floor}",
                        "task_id": self._current_task_id
                    }), 200
                else:
                    return jsonify({
                        "status": "error",
                        "message": "Floor switch timeout or failed",
                        "task_id": self._current_task_id
                    }), 500
            finally:
                self.is_busy = False

        @self.app.route('/health', methods=['GET'])
        def health():
            return jsonify({
                "status": "alive",
                "busy": self.is_busy,
                "task_id": self._current_task_id,
                "node": "openclaw_nav_bridge"
            }), 200

    def run_flask(self):
        self.app.run(host='0.0.0.0', port=8000, debug=False)


def main(args=None):
    rclpy.init(args=args)
    node = OpenClawNavBridge()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()