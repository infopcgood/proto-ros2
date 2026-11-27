"""Real ROS messages, UDP receiver, screen ownership, EGL and calibration HTTP.

Hardware output is a recording smileds stub. No user character is used: the
fixture consists of two original colored triangles representing opposite sides.
"""
from array import array
import json
import os
from pathlib import Path
import signal
import socket
import struct
import subprocess
import sys
import tempfile
import time
from types import SimpleNamespace
import unittest
import urllib.error
import urllib.request
from unittest.mock import patch

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from interface_pkg.msg import FacialExpression, Image
from interface_pkg.srv import ControlRequest
from PIL import Image as PILImage

from prootface_pkg.osf_input import OSFInput
from prootface_pkg.node import ProotfaceNode
from prootface_pkg.preview_display import PreviewDisplay
from prootface_pkg.atlas_display import AtlasDisplay
from screen_dispatcher_pkg.screen_dispatcher import ScreenDispatcherNode


FEATURES=('eye_left','eye_right','eyebrow_steepness_left','eyebrow_updown_left',
          'eyebrow_quirk_left','eyebrow_steepness_right','eyebrow_updown_right',
          'eyebrow_quirk_right','mouth_corner_updown_left','mouth_corner_inout_left',
          'mouth_corner_updown_right','mouth_corner_inout_right','mouth_open','mouth_wide')


def packet(face_id=0,gap=25,**features):
    header=struct.pack('<di4fBf4f3f3f',1000.,face_id,640.,480.,1.,1.,1,.01,0.,0.,0.,1.,0.,0.,0.,0.,0.,0.)
    points=[(0.,0.)]*68;points[58]=(100.,200.);points[62]=(200.,200.)
    for a,b,x in [(59,65,120.),(60,64,150.),(61,63,180.)]:
        points[a]=(x,200.-gap/2);points[b]=(x,200.+gap/2)
    values={k:0. for k in FEATURES};values.update(features)
    return (header+struct.pack('<68f',*([1.]*68))+struct.pack('<136f',*(v for p in points for v in p))
            +struct.pack('<210f',*([0.]*210))+struct.pack('<14f',*(values[k] for k in FEATURES)))


def fixture(root, expressions=False):
    points=[[-.6,-.6,.8],[.9,-.6,.8],[-.6,.6,.8],[-.6,-.6,-.8],[.9,-.6,-.8],[-.6,.6,-.8]]
    rig=dict(schema=2,positions=[v for p in points for v in p],uvs=[.25,.5]*3+[.75,.5]*3,
             parts=[dict(name='test',textured=True,indices=[0,1,2,3,4,5])],
             targets={'open':dict(indices=[0,1,2],deltas=[0,-.2,0]*3)},
             view=dict(zoom=1,offset=[0,0]),anatomical_left_sign=1)
    if expressions:
        rig['expression_sets'] = {}
        for name, sign in [('happy', 1), ('angry', -1)]:
            bank = {}
            for phase in ('rest', 'open', 'round', 'wide', 'eye_left', 'eye_right'):
                key = 'expr_' + name + '_' + phase
                rig['targets'][key] = dict(indices=[0],deltas=[0,sign*.05,0])
                bank[phase] = key
            rig['expression_sets'][name] = bank
    path=root/'character.json';path.write_text(json.dumps(rig))
    image=PILImage.new('RGB',(2,1));image.putdata([(255,0,0),(0,255,0)]);image.save(path.with_suffix('.png'))
    return path


def parameters(**values):return [Parameter(k,value=v) for k,v in values.items()]


class ROSPipelineTests(unittest.TestCase):
    def setUp(self):
        rclpy.init();self.executor=SingleThreadedExecutor();self.nodes=[]
        self.temp=tempfile.TemporaryDirectory();self.root=Path(self.temp.name)
        self.socket=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)

    def add(self,node):
        self.nodes.append(node);self.executor.add_node(node);return node

    def spin_until(self,predicate,timeout=10,payload=None,address=None):
        deadline=time.monotonic()+timeout
        next_send = 0
        while time.monotonic()<deadline:
            now = time.monotonic()
            if payload is not None and now >= next_send:
                self.socket.sendto(payload, address)
                next_send = now + 1/30
            self.executor.spin_once(timeout_sec=.01)
            if predicate():return True
        return False

    def post(self,node,path,data):
        request=urllib.request.Request(f'http://127.0.0.1:{node.http.server_port}'+path,
                 json.dumps(data).encode(),{'Content-Type':'application/json'})
        try:
            with urllib.request.urlopen(request,timeout=3) as response:
                return json.load(response)
        except urllib.error.HTTPError as exc:
            self.fail(f'{path}: HTTP {exc.code}: {exc.read().decode()}')

    def tearDown(self):
        for node in reversed(self.nodes):
            self.executor.remove_node(node);node.destroy_node()
        self.executor.shutdown();self.socket.close()
        if rclpy.ok():rclpy.shutdown()
        self.temp.cleanup()

    def test_package_owned_preview_checks_frames_without_a_global_param_server(self):
        shown, resized, cleared = [], [], []
        cv = SimpleNamespace(INTER_NEAREST=0,
            resize=lambda frame, size, interpolation: resized.append(size) or frame,
            imshow=lambda name, frame: shown.append(frame.copy()), waitKey=lambda delay: None,
            destroyAllWindows=lambda: cleared.append(True))
        with patch.dict(sys.modules, {'cv2': cv}):
            preview = self.add(PreviewDisplay(parameter_overrides=parameters(
                expected_width=2, expected_height=1)))
            frame = Image()
            frame.width, frame.height = 2, 1
            frame.data = array('B', [1, 2, 3, 4, 5, 6])
            preview.on_frame(frame)
            self.assertEqual(resized, [(4, 2)])
            self.assertEqual(shown[0].tobytes(), bytes(frame.data))
            frame.flip_second_screen = True
            preview.on_frame(frame)
            self.assertEqual(preview.rejected, 1)
            preview.destroy_node()
            self.assertEqual(cleared, [True])

    def test_multi_face_burst_retains_selected_face_and_parameters_are_read_only(self):
        receiver = self.add(OSFInput(parameter_overrides=parameters(udp_port=0)))
        node = self.add(ProotfaceNode(parameter_overrides=parameters(
            character_path=str(fixture(self.root)), profile_path=str(self.root/'profile.json'),
            side_width=32, side_height=32, web_port=-1)))
        self.assertTrue(self.spin_until(lambda: receiver.pub.get_subscription_count() > 0))
        result = node.set_parameters(parameters(presentation_fps=6))[0]
        self.assertFalse(result.successful)
        self.assertEqual(node.get_parameter('presentation_fps').value, 24)
        self.socket.sendto(b''.join(packet(face_id=i) for i in range(8)), receiver.socket.getsockname())
        # Let the receiver publish the whole burst before the executor filters it.
        time.sleep(.1)
        self.assertTrue(self.spin_until(lambda: node.app.received == 1, 3))
        self.assertEqual(node.app.latest.face_id, 0)

    def test_external_button_topic_latches_expression_while_speech_continues(self):
        receiver = self.add(OSFInput(parameter_overrides=parameters(udp_port=0)))
        node = self.add(ProotfaceNode(parameter_overrides=parameters(
            character_path=str(fixture(self.root, expressions=True)),
            profile_path=str(self.root/'profile.json'), side_width=32, side_height=32,
            web_port=-1, expression_topic='test_expression_buttons')))
        button = self.add(Node('expression_button_test'))
        publisher = button.create_publisher(FacialExpression, 'test_expression_buttons', 10)
        self.assertTrue(self.spin_until(lambda: publisher.get_subscription_count() > 0))
        command = FacialExpression()
        command.type, command.expression, command.override_time = 'emotion', 'angry', 0.
        publisher.publish(command)
        address = receiver.socket.getsockname()
        self.assertTrue(self.spin_until(lambda: node.app.snapshot.get('pose', {}).get(
            'blend_expr_angry_open', 0) > .5, 3, packet(mouth_open=.9), address))
        self.assertEqual(node.app.mode, 'live')
        self.assertEqual(node.app.snapshot['expression']['parts']['mouth']['selected'], 'angry')
        # No more button messages; the selected bank persists through a closure.
        self.assertTrue(self.spin_until(lambda: node.app.snapshot['pose']['aperture'] == 0,
                                       3, packet(gap=0), address))
        self.assertEqual(node.app.snapshot['pose']['blend_expr_angry_open'], 0)
        self.assertEqual(node.app.snapshot['pose']['blend_expr_angry_rest'], 1)
        # A separate eye command does not replace the mouth's selected bank.
        command.type, command.expression = 'eyes', 'happy'
        publisher.publish(command)
        self.assertTrue(self.spin_until(lambda: node.app.snapshot['expression']['parts']['eyes']['selected'] == 'happy'))
        self.assertEqual(node.app.snapshot['expression']['parts']['mouth']['selected'], 'angry')
        command.type, command.expression = 'emotion', 'unsupported'
        publisher.publish(command)
        self.assertTrue(self.spin_until(lambda: node.expression_rejected == 1))
        self.assertEqual(node.app.snapshot['expression']['parts']['mouth']['selected'], 'angry')

    def test_panel_watchdog_uses_wall_progress_with_simulated_ros_time_paused(self):
        clears = []
        driver = SimpleNamespace(leds_init=lambda *args: None, leds_set=lambda data: None,
                                 leds_send=lambda: None, leds_clear=lambda: clears.append(True))
        mapping = dict(schema=1, side_width=1, side_height=1, leds_per_strip=1,
                       num_strips=1, pixels=[['left', 0, 0]])
        path = self.root/'map.json'
        path.write_text(json.dumps(mapping))
        with patch.dict(sys.modules, {'smileds': driver}):
            display = self.add(AtlasDisplay(parameter_overrides=parameters(
                panel_map=str(path), driver_strips=1, stale_timeout=.05, use_sim_time=True)))
            frame = Image()
            frame.width, frame.height = 2, 1
            frame.data = array('B', bytes(6))
            display.on_frame(frame)
            self.assertTrue(self.spin_until(lambda: bool(clears), 2))

    def test_udp_to_two_view_led_bytes_calibration_and_ownership(self):
        calls=[];clears=[]
        driver=SimpleNamespace(leds_init=lambda *args:None,leds_set=lambda data:calls.append(data),
                               leds_send=lambda:None,leds_clear=lambda:clears.append(True))
        with patch.dict(sys.modules,{'smileds':driver}):
            receiver=self.add(OSFInput(parameter_overrides=parameters(udp_port=0)))
            dispatcher=self.add(ScreenDispatcherNode())
            node=self.add(ProotfaceNode(parameter_overrides=parameters(
                character_path=str(fixture(self.root)),profile_path=str(self.root/'profile.json'),
                side_width=64,side_height=64,presentation_fps=12,web_port=0)))
            mapping=dict(schema=1,side_width=64,side_height=64,leds_per_strip=1,num_strips=8,
                         pixels=[['left',32,32],['right',32,32],None,None,None,None,None,None])
            map_path=self.root/'map.json';map_path.write_text(json.dumps(mapping))
            display=self.add(AtlasDisplay(parameter_overrides=parameters(panel_map=str(map_path))))
            observer=self.add(Node('pipeline_test_observer'));frames=[]
            observer.create_subscription(Image,'screen_display',lambda frame:frames.append(frame),1)
            source=observer.create_publisher(Image,'request_screen_display',1)
            address=receiver.socket.getsockname();payload=packet(mouth_open=.8)
            self.socket.sendto(b'bad',address)
            self.assertTrue(self.spin_until(lambda:node.app.received>2 and len(calls)>2,15,payload,address))
            self.assertEqual((frames[-1].width,frames[-1].height),(128,64))
            self.assertFalse(frames[-1].flip_second_screen)
            self.assertEqual(calls[-1][:6],bytes([255,0,0,0,255,0]))
            self.assertEqual(calls[-1][6:],bytes(18));self.assertGreater(receiver.rejected,0)
            self.assertEqual(dispatcher.get_parameter('dominating_node_name').value,node.sender)
            # Multiple records in one datagram are decoded without truncation.
            self.socket.sendto(packet(face_id=7)+payload,address)
            for stage,values,gap in [('neutral',{},1),('open',{'mouth_open':.9},30),
                    ('wide',{'mouth_wide':.9},1),('narrow',{'mouth_wide':-.9},1),
                    ('smile',{'mouth_corner_updown_left':.8,'mouth_corner_updown_right':.8},1)]:
                self.post(node,'/api/calibration/start',{'stage':stage})
                self.assertTrue(self.spin_until(lambda:node.app.session.status()['counts'][stage]>=8,
                                               5,packet(gap=gap,**values),address))
            self.post(node,'/api/calibration/save',{})
            self.assertTrue(node.app.profile.calibrated)
            self.assertTrue(json.loads((self.root/'profile.json').read_text())['calibrated'])
            self.assertTrue(self.spin_until(lambda:not node.app.snapshot['tracking'],3))
            # A different producer can own the screen; the face node does not
            # periodically steal it back or bypass the dispatcher.
            client=observer.create_client(ControlRequest,'request_screen_control')
            self.assertTrue(client.wait_for_service(timeout_sec=3))
            request=ControlRequest.Request();request.sender='test_override';future=client.call_async(request)
            self.assertTrue(self.spin_until(future.done));self.assertTrue(future.result().success)
            sentinel=Image();sentinel.sender='test_override';sentinel.width=128;sentinel.height=64
            sentinel.flip_second_screen=False;sentinel.data=array('B',bytes([1,2,3])*(128*64))
            deadline=time.monotonic()+3
            while time.monotonic()<deadline and (not calls or calls[-1][:3]!=bytes([3,2,1])):
                source.publish(sentinel);self.executor.spin_once(timeout_sec=.03)
            self.assertEqual(calls[-1][:3],bytes([3,2,1]))
            self.assertEqual(dispatcher.get_parameter('dominating_node_name').value,'test_override')
            before=display.rejected;sentinel.flip_second_screen=True
            source.publish(sentinel)
            self.assertTrue(self.spin_until(lambda:display.rejected>before))
            self.assertEqual(node.rejected,0)

    def test_installed_launch_starts_real_pipeline(self):
        character=fixture(self.root)
        probe=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);probe.bind(('127.0.0.1',0))
        port=probe.getsockname()[1];probe.close()
        probe=socket.socket(socket.AF_INET,socket.SOCK_STREAM);probe.bind(('127.0.0.1',0))
        web_port=probe.getsockname()[1];probe.close()
        observer=self.add(Node('launch_test_observer'));frames=[]
        observer.create_subscription(Image,'screen_display',lambda frame:frames.append(frame),1)
        log_path=self.root/'launch.log'
        with log_path.open('w') as log:
            process=subprocess.Popen(['ros2','launch','prootface_pkg','face.launch.py',
                'character:='+str(character),'profile:='+str(self.root/'profile.json'),
                'backend:=none','side_width:=64','side_height:=64','presentation_fps:=6',
                'web_port:='+str(web_port),'udp_port:='+str(port),
                'input_topic:=launch_test_osf'],stdout=log,stderr=subprocess.STDOUT)
            try:
                success=self.spin_until(lambda:bool(frames) or process.poll() is not None,
                    30,packet(mouth_open=.8),('127.0.0.1',port))
                self.assertTrue(success and frames,log_path.read_text())
                self.assertEqual((frames[-1].width,frames[-1].height),(128,64))
                self.assertFalse(frames[-1].flip_second_screen)
                with urllib.request.urlopen(f'http://127.0.0.1:{web_port}/api/state',timeout=3) as response:
                    state=json.load(response)
                # Frames can be rendered at rest even without input. Verify the
                # installed launch actually connects the remapped receiver.
                if not state['received']:
                    self.spin_until(lambda:False,.4,packet(mouth_open=.8),('127.0.0.1',port))
                    with urllib.request.urlopen(f'http://127.0.0.1:{web_port}/api/state',timeout=3) as response:
                        state=json.load(response)
                self.assertGreater(state['received'],0,log_path.read_text())
            finally:
                if process.poll() is None:process.send_signal(signal.SIGINT)
                try:process.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    process.kill();process.wait();self.fail('Launch did not shut down: '+log_path.read_text())


if __name__=='__main__':unittest.main()
