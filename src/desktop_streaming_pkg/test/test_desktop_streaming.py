"""Exercise the node contract with fake ROS services and desktop capture."""

import importlib.util
from pathlib import Path
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import Mock, patch


class DesktopStreamingTests(unittest.TestCase):

    def setUp(self):
        # Load a private module with scoped API doubles; never replace installed
        # ROS modules for other tests in the workspace.
        modules = {
            name: ModuleType(name) for name in [
                'mss', 'mss.exception', 'rclpy', 'rclpy.executors',
                'rclpy.node', 'rcl_interfaces', 'rcl_interfaces.msg',
                'rcl_interfaces.srv', 'interface_pkg', 'interface_pkg.msg',
                'interface_pkg.srv',
            ]
        }
        modules['mss'].MSS = Mock()
        modules['mss.exception'].ScreenShotError = type(
            'ScreenShotError', (Exception,), {},
        )
        self.ros = modules['rclpy']
        for name in ['init', 'spin', 'spin_until_future_complete', 'try_shutdown']:
            setattr(self.ros, name, Mock())
        modules['rclpy.executors'].ExternalShutdownException = type(
            'ExternalShutdownException', (Exception,), {},
        )
        modules['rclpy.node'].Node = object
        modules['rcl_interfaces.msg'].ParameterDescriptor = SimpleNamespace
        modules['rcl_interfaces.msg'].ParameterType = SimpleNamespace(
            PARAMETER_INTEGER=2,
        )
        modules['rcl_interfaces.srv'].GetParameters = SimpleNamespace(
            Request=SimpleNamespace,
        )
        modules['interface_pkg.msg'].Image = SimpleNamespace
        modules['interface_pkg.srv'].ControlRequest = SimpleNamespace(
            Request=SimpleNamespace,
        )
        path = (Path(__file__).parents[1] / 'desktop_streaming_pkg' /
                'desktop_streaming.py')
        spec = importlib.util.spec_from_file_location('desktop_node_under_test', path)
        self.module = importlib.util.module_from_spec(spec)
        with patch.dict('sys.modules', modules):
            spec.loader.exec_module(self.module)

        self.node = self.module.DesktopStreamingNode.__new__(
            self.module.DesktopStreamingNode,
        )
        self.node.capture = None
        self.node.timer = None
        self.node.get_name = Mock(return_value='desktop_streaming_node')
        self.node.get_logger = Mock(return_value=Mock())
        self.overrides = {}
        self.node.declare_parameter = Mock(side_effect=lambda name, value, desc:
                                           SimpleNamespace(value=self.overrides.get(name, value)))
        self.node.create_publisher = Mock(return_value=Mock())
        self.node.create_timer = Mock(return_value=Mock())
        self.node.destroy_client = Mock()
        self.capture = self.module.mss.MSS.return_value
        self.capture.monitors = [{'width': 2}, {'width': 2}, {'width': 2}]
        self.capture.grab.return_value = SimpleNamespace(
            width=2, height=1, bgra=bytes([0, 0, 255, 255]) * 2,
        )
        self.dimensions = SimpleNamespace(values=[
            SimpleNamespace(type=2, integer_value=2),
            SimpleNamespace(type=2, integer_value=1),
        ])
        self.control = SimpleNamespace(success=True, message='')
        self.node.call_service = Mock(side_effect=[self.dimensions, self.control])
        session = patch.dict('os.environ', {'XDG_SESSION_TYPE': 'x11'})
        session.start()
        self.addCleanup(session.stop)

    def test_start_and_publish_match_dispatcher_contract(self):
        self.overrides = {'monitor': 2, 'fps': 15.0, 'flip_second_screen': True}
        self.node.start()
        self.capture.grab.assert_called_once_with(self.capture.monitors[2])
        calls = self.node.call_service.call_args_list
        self.assertEqual(calls[0].args[1], '/param_server_node/get_parameters')
        self.assertEqual(calls[0].args[2].names, ['DISPLAY_WIDTH', 'DISPLAY_HEIGHT'])
        self.assertEqual(calls[1].args[1], 'request_screen_control')
        self.assertEqual(calls[1].args[2].sender, 'desktop_streaming_node')
        self.node.create_timer.assert_called_once_with(1 / 15.0, self.node.publish_frame)
        self.node.create_publisher.assert_called_once_with(
            self.module.Image, 'request_screen_display', 1,
        )
        self.node.publish_frame()
        message = self.node.publisher.publish.call_args.args[0]
        self.assertEqual(message.sender, calls[1].args[2].sender)
        self.assertEqual((message.width, message.height), (2, 1))
        self.assertTrue(message.flip_second_screen)
        self.assertEqual(bytes(message.data), bytes([0, 0, 255]) * 2)

    def test_combined_monitor_is_supported(self):
        self.overrides['monitor'] = 0
        self.node.start()
        self.capture.grab.assert_called_once_with(self.capture.monitors[0])

    def test_invalid_rates_and_timeouts_fail_before_services(self):
        for name in ['fps', 'service_timeout']:
            for value in [0.0, -1.0, float('nan'), float('inf')]:
                with self.subTest(name=name, value=value):
                    self.overrides = {name: value}
                    with self.assertRaisesRegex(ValueError, name):
                        self.node.start()
        self.node.call_service.assert_not_called()

    def test_invalid_monitor_does_not_request_control(self):
        self.overrides['monitor'] = 3
        with self.assertRaisesRegex(ValueError, 'Monitor 3 is unavailable'):
            self.node.start()
        self.assertEqual(self.node.call_service.call_count, 1)
        self.node.close()
        self.capture.close.assert_called_once()

    def test_invalid_dimensions_do_not_open_capture(self):
        self.dimensions.values[0].integer_value = 0
        with self.assertRaisesRegex(ValueError, 'DISPLAY_WIDTH'):
            self.node.start()
        self.module.mss.MSS.assert_not_called()

    def test_capture_failure_does_not_take_screen_control(self):
        self.capture.grab.side_effect = self.module.ScreenShotError('no display')
        with self.assertRaisesRegex(RuntimeError, 'Cannot capture the desktop'):
            self.node.start()
        self.assertEqual(self.node.call_service.call_count, 1)
        self.node.create_timer.assert_not_called()

    def test_control_denial_does_not_start_timer(self):
        self.control.success = False
        self.control.message = 'busy'
        with self.assertRaisesRegex(RuntimeError, 'denied: busy'):
            self.node.start()
        self.node.create_timer.assert_not_called()

    def test_close_is_idempotent(self):
        self.node.start()
        timer = self.node.timer
        self.node.close()
        self.node.close()
        timer.cancel.assert_called_once()
        self.capture.close.assert_called_once()

    def test_service_discovery_and_response_timeouts(self):
        self.node.service_timeout = 0.1
        client = Mock()
        self.node.create_client = Mock(return_value=client)
        call = self.module.DesktopStreamingNode.call_service
        client.wait_for_service.return_value = False
        with self.assertRaisesRegex(RuntimeError, 'Service unavailable'):
            call(self.node, object, '/test', object())
        client.call_async.assert_not_called()
        self.node.destroy_client.assert_called_once_with(client)

        self.node.destroy_client.reset_mock()
        client.wait_for_service.return_value = True
        client.call_async.return_value.done.return_value = False
        with self.assertRaisesRegex(RuntimeError, 'response timed out'):
            call(self.node, object, '/test', object())
        client.call_async.return_value.cancel.assert_called_once()
        self.node.destroy_client.assert_called_once_with(client)

    def test_completed_service_response_is_returned(self):
        self.node.service_timeout = 0.1
        client = Mock()
        self.node.create_client = Mock(return_value=client)
        client.call_async.return_value.result.return_value = self.dimensions
        result = self.module.DesktopStreamingNode.call_service(
            self.node, object, '/test', object(),
        )
        self.assertIs(result, self.dimensions)
        self.node.destroy_client.assert_called_once_with(client)

    def test_main_cleans_up_startup_and_capture_errors(self):
        for method in ['start', 'spin']:
            with self.subTest(method=method):
                node = Mock()
                self.ros.spin.side_effect = None
                self.ros.try_shutdown.reset_mock()
                target = node.start if method == 'start' else self.ros.spin
                target.side_effect = RuntimeError('capture failed')
                with patch.object(self.module, 'DesktopStreamingNode', return_value=node):
                    with self.assertRaisesRegex(RuntimeError, 'capture failed'):
                        self.module.main([])
                node.close.assert_called_once()
                node.destroy_node.assert_called_once()
                self.ros.try_shutdown.assert_called_once()


if __name__ == '__main__':
    unittest.main()
