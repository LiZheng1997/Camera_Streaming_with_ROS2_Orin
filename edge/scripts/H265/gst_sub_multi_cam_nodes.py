#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib
import numpy as np
import sys

class GstreamerH265Subscriber(Node):
    def __init__(self, port=5000, camera_name='camera1'):
        super().__init__(f'gstreamer_h265_subscriber_{camera_name}')
        
        Gst.init(None)
        
        # H.265 解码管道
        pipeline_str = (
            f"udpsrc port={port} "
            f"caps=\"application/x-rtp, media=video, clock-rate=90000, encoding-name=H265, payload={96+(port-5000)}\" ! "
            "rtph265depay ! "
            "h265parse ! "
            "nvv4l2decoder enable-max-performance=1 ! "  # 硬件解码 H.265
            "nvvidconv ! "
            "videoconvert ! "
            "video/x-raw, format=BGR ! "
            "appsink name=sink emit-signals=true sync=false max-buffers=2 drop=true"
        )
        
        self.get_logger().info(f'Pipeline: {pipeline_str}')
        
        try:
            self.pipeline = Gst.parse_launch(pipeline_str)
            self.appsink = self.pipeline.get_by_name('sink')
            self.appsink.connect('new-sample', self.on_new_sample)
            
            self.publisher = self.create_publisher(Image, f'{camera_name}/image_raw', 10)
            self.bridge = CvBridge()
            
            self.frame_count = 0
            self.last_log_time = self.get_clock().now()
            
            # 启动管道
            ret = self.pipeline.set_state(Gst.State.PLAYING)
            if ret == Gst.StateChangeReturn.FAILURE:
                self.get_logger().error('Unable to set pipeline to PLAYING state')
                sys.exit(1)
            
            self.get_logger().info(f'H.265 subscriber started for {camera_name} on port {port}')
            
        except Exception as e:
            self.get_logger().error(f'Failed to initialize: {str(e)}')
            sys.exit(1)
        
    def on_new_sample(self, sink):
        try:
            sample = sink.emit('pull-sample')
            if sample:
                buffer = sample.get_buffer()
                caps = sample.get_caps()
                
                height = caps.get_structure(0).get_value('height')
                width = caps.get_structure(0).get_value('width')
                
                success, map_info = buffer.map(Gst.MapFlags.READ)
                if success:
                    # BGR 格式，直接读取
                    img_array = np.ndarray(
                        shape=(height, width, 3),
                        dtype=np.uint8,
                        buffer=map_info.data
                    )
                    
                    # 发布图像
                    msg = self.bridge.cv2_to_imgmsg(img_array, encoding='bgr8')
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.header.frame_id = 'camera'
                    self.publisher.publish(msg)
                    
                    buffer.unmap(map_info)
                    
                    # 统计帧率
                    self.frame_count += 1
                    current_time = self.get_clock().now()
                    duration = (current_time - self.last_log_time).nanoseconds / 1e9
                    
                    if duration >= 2.0:
                        fps = self.frame_count / duration
                        self.get_logger().info(f'Receiving at {fps:.1f} FPS')
                        self.frame_count = 0
                        self.last_log_time = current_time
                    
        except Exception as e:
            self.get_logger().error(f'Error in on_new_sample: {str(e)}')
                
        return Gst.FlowReturn.OK
    
    def __del__(self):
        if hasattr(self, 'pipeline'):
            self.pipeline.set_state(Gst.State.NULL)

def main(args=None):
    rclpy.init(args=args)
    
    # 可以创建多个节点接收多路摄像头
    nodes = []
    ports = [5000, 5001, 5002, 5003]
    camera_names = ['CameraFrontLeft', 'CameraRear', 'CameraFrontRight', 'CameraFront']
    
    for port, name in zip(ports, camera_names):
        node = GstreamerH265Subscriber(port=port, camera_name=name)
        nodes.append(node)
    
    # 多线程 spin
    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    
    for node in nodes:
        executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        for node in nodes:
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()