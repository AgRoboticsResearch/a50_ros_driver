#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import socket
import json
import time
from threading import Thread

# 导入您的工作空间中定义的消息类型
from waterlinked_a50_ros_driver.msg import DVL, DVLBeam, DVLDR, ConfigCommand

class DVLSocketPublisher:
    """
    通过TCP Socket直接连接到DVL A50传感器，
    解析JSON数据，并将其发布为ROS1消息。
    """
    def __init__(self):
        rospy.init_node('dvl_socket_publisher', anonymous=True)

        # 从参数服务器获取配置
        self.ip_address = rospy.get_param('~ip_address', '192.168.2.95')
        self.port = rospy.get_param('~port', 16171)
        self.velocity_frame_id = rospy.get_param('~velocity_frame_id', 'dvl_velocity_link')
        self.position_frame_id = rospy.get_param('~position_frame_id', 'dvl_position_link')

        # 创建ROS Publishers
        self.pub_dvl_data = rospy.Publisher('dvl/data', DVL, queue_size=10)
        self.pub_dvl_pos = rospy.Publisher('dvl/position', DVLDR, queue_size=10)
        
        # 创建一个Subscriber来接收配置命令
        self.sub_config_command = rospy.Subscriber(
            'dvl/config/command', 
            ConfigCommand, 
            self.command_callback,
            queue_size=10
        )

        self.sock = None
        self.is_connected = False
        self.stop_thread = False
        
        # 启动一个独立的线程来处理网络连接和数据接收
        self.receive_thread = Thread(target=self.receive_loop)
        self.receive_thread.daemon = True
        self.receive_thread.start()

    def connect(self):
        """尝试连接到DVL传感器"""
        if self.is_connected:
            return True
        
        rospy.loginfo(f"正在尝试连接到 DVL at {self.ip_address}:{self.port}...")
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            # 设置超时以防连接阻塞
            self.sock.settimeout(5.0)
            self.sock.connect((self.ip_address, self.port))
            # 设置为非阻塞模式进行接收
            self.sock.settimeout(None) 
            self.is_connected = True
            rospy.loginfo("DVL 连接成功!")
            
            # 连接成功后，立即设置默认参数
            self.set_default_parameters()
            
            return True
        except socket.error as e:
            rospy.logwarn(f"DVL 连接失败: {e}")
            self.sock.close()
            self.sock = None
            self.is_connected = False
            return False

    def set_default_parameters(self):
        """连接成功后，设置默认参数。"""
        rospy.loginfo("正在设置默认参数: 'acoustic_enabled' to 'false'")
        default_command = {
            "command": "set_config",
            "parameters": {
                "acoustic_enabled": False
            }
        }
        self.send_to_dvl(default_command)

    def receive_loop(self):
        """在循环中处理连接和数据接收"""
        buffer = ''
        while not self.stop_thread:
            if not self.is_connected:
                if not self.connect():
                    # 如果连接失败，等待一段时间再重试
                    time.sleep(5)
                    continue
            
            try:
                # 从socket接收数据
                data = self.sock.recv(4096)
                if not data:
                    # 连接已断开
                    rospy.logwarn("DVL 连接已断开，正在尝试重新连接...")
                    self.is_connected = False
                    self.sock.close()
                    continue
                
                # 将字节数据解码为字符串并添加到缓冲区
                buffer += data.decode('utf-8')
                
                # JSON消息以换行符`\n`分隔
                while '\n' in buffer:
                    message_str, buffer = buffer.split('\n', 1)
                    if message_str:
                        try:
                            json_data = json.loads(message_str)
                            self.dispatch_data(json_data)
                        except json.JSONDecodeError:
                            rospy.logwarn(f"JSON 解析失败: {message_str}")

            except socket.error as e:
                rospy.logerr(f"Socket 错误: {e}")
                self.is_connected = False
                self.sock.close()

    def dispatch_data(self, json_data):
        """根据JSON内容判断并调用相应的发布函数"""
        if "altitude" in json_data:
            self.publish_velocity_report(json_data)
        elif "pitch" in json_data:
            self.publish_position_report(json_data)
        # 可以根据需要添加对其他响应类型的处理
        # elif "response_to" in json_data:
        #     rospy.loginfo(f"收到响应: {json_data}")

    def command_callback(self, msg):
        """处理接收到的配置命令并发送到DVL"""
        if not self.is_connected:
            rospy.logwarn("DVL 未连接，无法发送命令。")
            return

        command_to_send = {}
        if msg.command == "set_config":
            # 对特定参数进行类型转换
            if msg.parameter_name == "acoustic_enabled":
                value = msg.parameter_value.lower() == 'true'
            elif msg.parameter_name == "speed_of_sound":
                value = int(msg.parameter_value)
            # 其他参数可以根据需要添加
            else:
                value = msg.parameter_value
            
            command_to_send = {
                "command": "set_config",
                "parameters": {
                    msg.parameter_name: value
                }
            }
        elif msg.command in ["get_config", "calibrate_gyro", "reset_dead_reckoning"]:
            command_to_send = {"command": msg.command}
        else:
            rospy.logerr(f"不支持的命令: {msg.command}")
            return
        
        self.send_to_dvl(command_to_send)

    def send_to_dvl(self, command_dict):
        """将Python字典转换为JSON并发送到DVL"""
        try:
            json_str = json.dumps(command_dict) + '\n'
            self.sock.sendall(json_str.encode('utf-8'))
            rospy.loginfo(f"已发送命令到 DVL: {json_str.strip()}")
        except socket.error as e:
            rospy.logerr(f"发送命令失败: {e}")
            self.is_connected = False
            self.sock.close()

    def publish_velocity_report(self, data):
        """发布速度和声呐波束报告 (dvl/data)"""
        msg = DVL()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.velocity_frame_id

        msg.time = data.get("time", 0.0)
        msg.velocity.x = data.get("vx", 0.0)
        msg.velocity.y = data.get("vy", 0.0)
        msg.velocity.z = data.get("vz", 0.0)
        msg.fom = data.get("fom", 0.0)
        msg.altitude = data.get("altitude", 0.0)
        msg.velocity_valid = data.get("velocity_valid", False)
        msg.status = data.get("status", 0)
        msg.form = data.get("format", "")

        if "transducers" in data and len(data["transducers"]) == 4:
            beams_data = data["transducers"]
            msg.beams = [DVLBeam(
                id=b.get("id", 0),
                velocity=b.get("velocity", 0.0),
                distance=b.get("distance", 0.0),
                rssi=b.get("rssi", 0.0),
                nsd=b.get("nsd", 0.0),
                valid=b.get("beam_valid", False)
            ) for b in beams_data]

        self.pub_dvl_data.publish(msg)

    def publish_position_report(self, data):
        """发布航位推算报告 (dvl/position)"""
        msg = DVLDR()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.position_frame_id

        msg.time = data.get("ts", 0.0)
        msg.position.x = data.get("x", 0.0)
        msg.position.y = data.get("y", 0.0)
        msg.position.z = data.get("z", 0.0)
        msg.pos_std = data.get("std", 0.0)
        msg.roll = data.get("roll", 0.0)
        msg.pitch = data.get("pitch", 0.0)
        msg.yaw = data.get("yaw", 0.0)
        msg.type = data.get("type", "")
        msg.status = data.get("status", 0)
        msg.format = data.get("format", "")

        self.pub_dvl_pos.publish(msg)

    def shutdown(self):
        """关闭节点时清理资源"""
        rospy.loginfo("正在关闭 DVL Socket Publisher...")
        self.stop_thread = True
        if self.sock:
            self.sock.close()
        if self.receive_thread.is_alive():
            self.receive_thread.join()
        rospy.loginfo("已关闭。")

if __name__ == '__main__':
    try:
        publisher = DVLSocketPublisher()
        rospy.on_shutdown(publisher.shutdown)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
