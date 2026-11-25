"""OpenSeeFace messages to calibrated, independently rendered side displays."""
from array import array
import json
from pathlib import Path
import time

import rclpy
from rclpy.clock import Clock
from rclpy.clock_type import ClockType
from rclpy.node import Node
from interface_pkg.msg import Image, OSFData
from interface_pkg.srv import ControlRequest

from prootface.offscreen import OffscreenRenderer
from prootface.ros_input import measurement_from_ros
from prootface.runtime import RenderWorker
from prootface.server import Application, start_http
from prootface.timing import ANIMATION_FPS


class ProotfaceNode(Node):
    def __init__(self, **kwargs):
        super().__init__('prootface_node', **kwargs)
        self.worker=None;self.http=None;self.http_thread=None;self._closed=False
        defaults=dict(character_path='',profile_path='~/.local/share/prootface/calibration.json',
                      style_path='',face_id=0,side_width=256,side_height=144,
                      presentation_fps=24,egl_platform='auto',web_host='127.0.0.1',
                      web_port=8080,input_topic='osf_data')
        for key,value in defaults.items():self.declare_parameter(key,value)
        value=lambda key:self.get_parameter(key).value
        rate=value('presentation_fps')
        if rate not in (6,12,24):raise ValueError('presentation_fps must be 6, 12 or 24')
        self.stride=ANIMATION_FPS//rate
        character=Path(value('character_path')).expanduser()
        if not character.is_file():raise ValueError('Set character_path to the private compiled character JSON')
        self.app=Application(Path(value('profile_path')).expanduser(),value('face_id'))
        self.app.mode='live'
        if value('style_path'):
            self.app.command('/api/style',json.loads(Path(value('style_path')).expanduser().read_text()))
        try:
            self.worker=RenderWorker(lambda:OffscreenRenderer(character,value('side_width'),
                                    value('side_height'),value('egl_platform')))
            self.get_logger().info('Offscreen graphics: '+json.dumps(self.worker.info))
            if value('web_port')>=0:
                self.http,self.http_thread=start_http(self.app,value('web_host'),value('web_port'),character)
                self.get_logger().info(f'Calibration UI: http://{value("web_host")}:{self.http.server_port}')
            self.publisher=self.create_publisher(Image,'request_screen_display',1)
            self.subscription=self.create_subscription(OSFData,value('input_topic'),self.on_measurement,1)
            self.control=self.create_client(ControlRequest,'request_screen_control')
            self.sender=self.get_fully_qualified_name();self.has_grant=False;self.request=None
            self.last_request=-1e9;self.request_finished=False;self.rejected=0;self.published=0
            self.timer=self.create_timer(1/ANIMATION_FPS,self.tick,
                                         clock=Clock(clock_type=ClockType.STEADY_TIME))
        except BaseException:
            self.destroy_node();raise

    def on_measurement(self,message):
        try:
            self.app.ingest(measurement_from_ros(message,time.monotonic()))
        except ValueError:
            with self.app.lock:self.app.rejected+=1
            self.rejected+=1

    def request_control(self,now):
        if self.request_finished:return
        if self.request is not None:
            if not self.request.done():return
            try:
                response=self.request.result();self.has_grant=bool(response.success)
                self.request_finished=True
                if not self.has_grant:self.get_logger().error('Screen control denied: '+response.message)
            except Exception as exc:
                self.get_logger().warning('Screen control request failed: '+str(exc))
            self.request=None
            return
        if now-self.last_request>=1 and self.control.service_is_ready():
            message=ControlRequest.Request();message.sender=self.sender
            self.request=self.control.call_async(message);self.last_request=now

    def tick(self):
        now=time.monotonic();self.request_control(now);self.app.tick(now)
        completed=self.worker.poll()
        if completed is not None and self.has_grant and now-completed.pose_time<=.25:
            frame=Image();frame.sender=self.sender
            frame.height,frame.width=completed.pixels.shape[:2]
            frame.flip_second_screen=False
            frame.data=array('B',completed.pixels.tobytes())
            self.publisher.publish(frame);self.published+=1
        with self.app.lock:
            sequence=self.app.sequence;pose=dict(self.app.snapshot['pose'])
        if (sequence-1)%self.stride==0:self.worker.submit(sequence,now,pose)

    def destroy_node(self):
        if self._closed:return
        self._closed=True
        if hasattr(self,'timer'):self.timer.cancel()
        if hasattr(self,'app'):self.app.stop.set()
        if self.http is not None:
            self.http.shutdown();self.http.server_close();self.http_thread.join(2)
        if self.worker is not None:self.worker.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args);node=None
    try:
        node=ProotfaceNode();rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
