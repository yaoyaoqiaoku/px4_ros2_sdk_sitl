#!/usr/bin/env python3
"""
航点任务上传器:接收航点任务并通过MAVLink协议上传到PX4
订阅: /mission/waypoints (std_msgs/Float32MultiArray)
功能: 通过MAVLink Mission协议将航点上传到PX4的dataman存储
UDP版本：适配PX4模拟器（SITL）
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, UInt16
from pymavlink import mavutil
import threading
import time
import math

class MavlinkMission(Node):
    def __init__(self):
        super().__init__('mavlink_mission')
        
        # MAVLink connection parameters (UDP for SITL)
        self.declare_parameter('mavlink_connection', 'udp:127.0.0.1:14580')
        self.mavlink_connection = self.get_parameter('mavlink_connection').get_parameter_value().string_value
        
        # Create subscriber
        self.waypoints_sub = self.create_subscription(
            Float32MultiArray,
            '/mission/waypoints',
            self.waypoints_callback,
            10
        )
        
        # Create publishers
        self.mission_count_pub = self.create_publisher(
            UInt16,
            '/mission/count',
            10
        )
        self.current_waypoint_pub = self.create_publisher(
            UInt16,
            '/mission/current_waypoint',
            10
        )
        
        # MAVLink connection
        self.master = None
        self.upload_lock = threading.Lock()
        self.is_connected = False
        self.mission_count_ = 0
        self.monitoring_active = False
        
        # Start thread to monitor current waypoint
        threading.Thread(target=self.monitor_current_waypoint, daemon=True).start()
        
        # Connect to MAVLink
        self.connect_mavlink()
        
        self.get_logger().info(f"Mavlink Mission started, connection: {self.mavlink_connection}")
    
    def connect_mavlink(self):
        """连接到MAVLink UDP（适配模拟器）"""
        try:
            # 关闭旧连接（如果存在）
            if self.master is not None:
                try:
                    self.master.close()
                except:
                    pass
            
            self.master = mavutil.mavlink_connection(
                self.mavlink_connection,
                autoreconnect=True,  # 启用自动重连
                source_system=255
            )
            
            def wait_heartbeat():
                try:
                    if self.master is None:
                        self.get_logger().error("MAVLink master is None, cannot wait for heartbeat")
                        self.is_connected = False
                        return
                    
                    # 带超时的心跳等待（避免阻塞）
                    self.master.wait_heartbeat(timeout=10)
                    self.is_connected = True
                    self.get_logger().info("MAVLink heartbeat received, connected to PX4 SITL")
                except Exception as e:
                    self.get_logger().error(f"Heartbeat failed: {e}")
                    self.is_connected = False
                    # 失败后尝试重连
                    threading.Timer(5, self.connect_mavlink).start()
            
            threading.Thread(target=wait_heartbeat, daemon=True).start()
        except Exception as e:
            self.get_logger().error(f"MAVLink UDP connection failed: {e}")
            self.is_connected = False
            self.master = None
            # 失败后尝试重连
            threading.Timer(5, self.connect_mavlink).start()
    
    def waypoints_callback(self, msg):
        """接收航点数据并上传到PX4"""
        if len(msg.data) % 10 != 0:
            self.get_logger().error(
                f"Invalid waypoint format! Expected multiple of 10 parameters (with optional speed), got {len(msg.data)}"
            )
            return

        # Parse waypoints (now each waypoint has 10 floats: 9 standard fields + speed)
        num_waypoints = len(msg.data) // 10
        waypoints = []
        for i in range(num_waypoints):
            wp = [
                msg.data[i * 10 + 0],  # frame
                msg.data[i * 10 + 1],  # command
                msg.data[i * 10 + 2],  # lat
                msg.data[i * 10 + 3],  # lon
                msg.data[i * 10 + 4],  # alt
                msg.data[i * 10 + 5],  # param1
                msg.data[i * 10 + 6],  # param2
                msg.data[i * 10 + 7],  # param3
                math.nan if math.isnan(msg.data[i * 10 + 8]) else msg.data[i * 10 + 8],  # param4
                math.nan if math.isnan(msg.data[i * 10 + 9]) else msg.data[i * 10 + 9]   # speed (optional semantics)
            ]
            waypoints.append(wp)
        
        self.mission_count_ = len(waypoints)
        self.get_logger().info(f"Received {len(waypoints)} waypoints, uploading...")
        threading.Thread(target=self.upload_mission_thread, args=(waypoints,), daemon=True).start()
    
    def upload_mission_thread(self, waypoints):
        """在单独线程中上传任务（修复mission_type兼容）"""
        with self.upload_lock:
            if not self.is_connected or self.master is None:
                self.get_logger().error("MAVLink not connected, abort upload")
                return
            
            try:
                # Clear existing mission
                self.master.mav.mission_clear_all_send(
                    self.master.target_system,
                    self.master.target_component,
                    mavutil.mavlink.MAV_MISSION_TYPE_MISSION
                )
                
                # Wait for acknowledgment (timeout 3s)
                timeout = time.time() + 3.0
                clear_ack = False
                while time.time() < timeout:
                    msg = self.master.recv_match(type='MISSION_ACK', blocking=False, timeout=0.1)
                    if msg and msg.type == mavutil.mavlink.MAV_MISSION_ACCEPTED:
                        clear_ack = True
                        break
                    elif msg and msg.type != mavutil.mavlink.MAV_MISSION_ACCEPTED:
                        self.get_logger().error(f"Mission clear failed: {msg.type}")
                        return
                
                if not clear_ack:
                    self.get_logger().warning("No mission clear ack, continue anyway")
                
                # Send mission count
                self.master.mav.mission_count_send(
                    self.master.target_system,
                    self.master.target_component,
                    len(waypoints),
                    mavutil.mavlink.MAV_MISSION_TYPE_MISSION
                )
                
                # Wait for mission request (修复mission_type检查：增加hasattr判断)
                timeout = time.time() + 3.0
                request_received = False
                while time.time() < timeout:
                    msg = self.master.recv_match(type='MISSION_REQUEST', blocking=False, timeout=0.1)
                    if msg:
                        # 兼容MAVLink 1/2：先判断是否有mission_type属性
                        if hasattr(msg, 'mission_type') and msg.mission_type == mavutil.mavlink.MAV_MISSION_TYPE_MISSION:
                            request_received = True
                            break
                        elif not hasattr(msg, 'mission_type'):
                            # MAVLink 1无mission_type，直接认为是任务请求
                            request_received = True
                            break
                if not request_received:
                    self.get_logger().error("Mission request timeout")
                    return
                
                # Send waypoints
                for i, wp in enumerate(waypoints):
                    self.get_logger().info(f"Uploading waypoint {i+1}/{len(waypoints)}")
                    self.master.mav.mission_item_send(
                        self.master.target_system,
                        self.master.target_component,
                        i, int(wp[0]), int(wp[1]), 0, 1,
                        wp[5], wp[6], wp[7], wp[8],
                        wp[2], wp[3], wp[4],
                        mavutil.mavlink.MAV_MISSION_TYPE_MISSION
                    )
                    
                    # Wait for next request or final acknowledgment
                    timeout = time.time() + 3.0
                    next_request = False
                    while time.time() < timeout:
                        msg = self.master.recv_match(
                            type=['MISSION_REQUEST', 'MISSION_ACK'],
                            blocking=False, timeout=0.1
                        )
                        if msg:
                            msg_type = msg.get_type()
                            if msg_type == 'MISSION_REQUEST':
                                if msg.seq == i + 1:
                                    next_request = True
                                    break
                                elif msg.seq < i + 1:
                                    # 重发之前的航点
                                    self.get_logger().info(f"Resend waypoint {msg.seq}")
                                    resend_wp = waypoints[msg.seq]
                                    self.master.mav.mission_item_send(
                                        self.master.target_system,
                                        self.master.target_component,
                                        msg.seq, int(resend_wp[0]), int(resend_wp[1]), 0, 1,
                                        resend_wp[5], resend_wp[6], resend_wp[7], resend_wp[8],
                                        resend_wp[2], resend_wp[3], resend_wp[4],
                                        mavutil.mavlink.MAV_MISSION_TYPE_MISSION
                                    )
                            elif msg_type == 'MISSION_ACK':
                                if msg.type == mavutil.mavlink.MAV_MISSION_ACCEPTED:
                                    self.get_logger().info("Mission uploaded successfully")
                                    # Publish mission count
                                    count_msg = UInt16()
                                    count_msg.data = len(waypoints)
                                    self.mission_count_pub.publish(count_msg)
                                    return
                                else:
                                    self.get_logger().error(f"Upload failed: {msg.type}")
                                    return
                
                # Wait for final acknowledgment
                timeout = time.time() + 3.0
                while time.time() < timeout:
                    msg = self.master.recv_match(type='MISSION_ACK', blocking=False, timeout=0.1)
                    if msg:
                        if msg.type == mavutil.mavlink.MAV_MISSION_ACCEPTED:
                            self.get_logger().info("Mission uploaded successfully")
                            count_msg = UInt16()
                            count_msg.data = len(waypoints)
                            self.mission_count_pub.publish(count_msg)
                            return
                        else:
                            self.get_logger().error(f"Final upload failed: {msg.type}")
                            return
                
                self.get_logger().warning("No final ack, mission may be uploaded")
                
            except Exception as e:
                self.get_logger().error(f"Upload error: {e}")
                import traceback
                self.get_logger().error(f"Traceback: {traceback.format_exc()}")
    
    def monitor_current_waypoint(self):
        """监控当前航点"""
        current_seq = 0
        self.monitoring_active = True
        while self.monitoring_active:
            try:
                if self.is_connected and self.master is not None:
                    # Listen for MISSION_CURRENT
                    msg_current = self.master.recv_match(type='MISSION_CURRENT', blocking=False, timeout=0.1)
                    if msg_current:
                        current_seq = msg_current.seq
                        current_wp_msg = UInt16()
                        current_wp_msg.data = current_seq
                        self.current_waypoint_pub.publish(current_wp_msg)
                    
                    # Listen for MISSION_ITEM_REACHED
                    msg_reached = self.master.recv_match(type='MISSION_ITEM_REACHED', blocking=False, timeout=0.1)
                    if msg_reached:
                        reached_seq = msg_reached.seq
                        current_seq = reached_seq + 1
                        current_wp_msg = UInt16()
                        current_wp_msg.data = current_seq
                        self.current_waypoint_pub.publish(current_wp_msg)
                        self.get_logger().info(f"Waypoint {reached_seq} reached, next: {current_seq}")
                else:
                    # Reconnect if not connected
                    if self.master is None:
                        self.connect_mavlink()
                
                time.sleep(0.1)
            except Exception as e:
                self.get_logger().warning(f"Monitor error: {e}")
                time.sleep(1.0)

def main(args=None):
    rclpy.init(args=args)
    node = MavlinkMission()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down mavlink_mission node...")
    except Exception as e:
        node.get_logger().error(f"Node error: {e}")
    finally:
        node.monitoring_active = False
        # Close MAVLink connection
        if node.master is not None:
            try:
                node.master.close()
            except:
                pass
        # Destroy node
        try:
            node.destroy_node()
        except:
            pass
        # Shutdown rclpy
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
