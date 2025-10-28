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
import cv2

class GstreamerH264Subscriber(Node):
    def __init__(self):
        super().__init__('gstreamer_h264_subscriber')
        
        Gst.init(None)
        
        # 让 videoconvert 完成 YUV → BGR 转换
        pipeline_str = (
            "udpsrc port=5000 "
            "caps=\"application/x-rtp, media=video, clock-rate=90000, encoding-name=H264, payload=97\" ! "
            "rtph264depay ! "
            "h264parse ! "
            "nvv4l2decoder enable-max-performance=1 ! "
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
            
            self.publisher = self.create_publisher(Image, 'CameraFrontLeft/image_raw', 10)
            self.bridge = CvBridge()
            
            self.frame_count = 0
            self.last_log_time = self.get_clock().now()
            
            # 启动管道
            ret = self.pipeline.set_state(Gst.State.PLAYING)
            if ret == Gst.StateChangeReturn.FAILURE:
                self.get_logger().error('Unable to set pipeline to PLAYING state')
                sys.exit(1)
            
            self.get_logger().info('GStreamer H.264 subscriber started successfully')
            
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
            import traceback
            traceback.print_exc()
                
        return Gst.FlowReturn.OK
    
    def __del__(self):
        if hasattr(self, 'pipeline'):
            self.pipeline.set_state(Gst.State.NULL)

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = GstreamerH264Subscriber()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
