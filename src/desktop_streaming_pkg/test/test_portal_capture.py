"""Check portal lifecycle, backend selection, and raw video buffer handling."""

import os
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from desktop_streaming_pkg import portal_capture as portal
from desktop_streaming_pkg.frame_utils import prepare_frame


class Variant:
    def __init__(self, signature, value):
        self.signature, self.value = signature, value

    def unpack(self):
        return self.value


class Context:
    def __init__(self):
        self.events = []

    def pending(self):
        return bool(self.events)

    def iteration(self, blocking):
        self.events.pop(0)()


class BackendAndPixelsTests(unittest.TestCase):

    def test_auto_selection_and_explicit_overrides(self):
        with patch.object(portal.sys, 'platform', 'linux'):
            with patch.dict(os.environ, {'XDG_SESSION_TYPE': 'x11', 'WAYLAND_DISPLAY': ''}):
                self.assertEqual(portal.select_backend('auto'), 'mss')
                self.assertEqual(portal.select_backend('portal'), 'portal')
            for env in [{'XDG_SESSION_TYPE': 'wayland'}, {'WAYLAND_DISPLAY': 'wayland-0'}]:
                with patch.dict(os.environ, env, clear=True):
                    self.assertEqual(portal.select_backend('auto'), 'portal')
                    with self.assertRaisesRegex(ValueError, 'backend:=portal'):
                        portal.select_backend('mss')
        with patch.object(portal.sys, 'platform', 'win32'):
            self.assertEqual(portal.select_backend('auto'), 'mss')
            with self.assertRaisesRegex(ValueError, 'only available on Linux'):
                portal.select_backend('portal')
        with self.assertRaisesRegex(ValueError, 'auto, mss, or portal'):
            portal.select_backend('invalid')

    def test_row_padding_offset_and_bgr_order(self):
        red = bytes([0, 0, 255, 0])
        blue = bytes([255, 0, 0, 0])
        shot = portal.unpack_bgrx(b'HEAD' + red + b'PAD!' + blue, 1, 2, 8, 4)
        self.assertEqual(shot.bgra, red + blue)
        self.assertEqual(prepare_frame(shot, 1, 2), bytes([0, 0, 255, 255, 0, 0]))

    def test_invalid_buffer_layout_is_rejected(self):
        for width, height, stride, offset in [
            (0, 1, 4, 0), (1, 0, 4, 0), (1, 1, -4, 0),
            (2, 1, 4, 0), (1, 1, 4, -1), (1, 3, 4, 0),
        ]:
            with self.subTest(layout=(width, height, stride, offset)):
                with self.assertRaises(ValueError):
                    portal.unpack_bgrx(bytes(8), width, height, stride, offset)


class PortalTests(unittest.TestCase):

    def setUp(self):
        self.capture = portal.PortalCapture(timeout=0.1)
        self.capture.context = Context()
        self.capture.Gio = SimpleNamespace(
            DBusSignalFlags=SimpleNamespace(NONE=0),
            DBusCallFlags=SimpleNamespace(NONE=0),
        )
        self.capture.GLib = SimpleNamespace(
            Variant=Variant, VariantType=SimpleNamespace(new=lambda x: x),
            Error=RuntimeError,
        )
        self.capture.bus = Mock()
        self.capture.bus.get_unique_name.return_value = ':1.42'
        self.callbacks = []
        self.capture.bus.signal_subscribe.side_effect = self.subscribe
        self.code = 0
        self.response_result = {'session_handle': '/session/test'}
        self.emit_response = True
        self.alternate_path = False
        self.introspection = f'<node><interface name="{portal.SCREENCAST}"/></node>'
        self.capture.bus.call_sync.side_effect = self.call

    def subscribe(self, *args):
        self.callbacks.append(args[6])
        return len(self.callbacks)

    def call(self, *args):
        if args[3] == 'Introspect':
            return Variant('(s)', (self.introspection,))
        if args[3] == 'Close':
            return Variant('()', ())
        options = args[4].unpack()[-1]
        token = options['handle_token'].unpack()
        path = f'{portal.OBJECT_PATH}/request/1_42/{token}'
        if self.alternate_path:
            path = '/legacy/request/1'
        if self.emit_response:
            callback = self.callbacks[-1]
            def respond():
                callback(None, None, path, None, None,
                         Variant('(ua{sv})', (self.code, self.response_result)), None)
            self.capture.context.events.append(respond)
        return Variant('(o)', (path,))

    def test_response_subscription_precedes_call_and_accepts_legacy_path(self):
        for legacy in [False, True]:
            with self.subTest(legacy=legacy):
                self.alternate_path = legacy
                result = self.capture._request('CreateSession')
                self.assertEqual(result, self.response_result)
        self.assertEqual(self.capture.bus.signal_unsubscribe.call_count, 2)

    def test_cancelled_or_failed_requests_do_not_proceed(self):
        for code, message in [(1, 'cancelled'), (2, 'failed')]:
            self.code = code
            with self.assertRaisesRegex(RuntimeError, message):
                self.capture._request('Start', '(osa{sv})', ('/session/test', ''))

    def test_request_timeout_closes_dialog_and_unsubscribes(self):
        self.emit_response = False
        self.capture.timeout = 0.0
        with self.assertRaisesRegex(RuntimeError, 'timed out'):
            self.capture._request('Start', '(osa{sv})', ('/session/test', ''))
        self.assertEqual(self.capture.bus.call_sync.call_args.args[3], 'Close')
        self.capture.bus.signal_unsubscribe.assert_called_once()

    def test_shutdown_while_selecting_closes_dialog(self):
        self.emit_response = False
        self.capture.is_running = lambda: False
        with self.assertRaises(KeyboardInterrupt):
            self.capture._request('Start', '(osa{sv})', ('/session/test', ''))
        self.assertEqual(self.capture.bus.call_sync.call_args.args[3], 'Close')

    def test_session_revocation_is_observed(self):
        self.capture._session_closed()
        with self.assertRaisesRegex(RuntimeError, 'ended screen sharing'):
            self.capture._pump_events()

    def test_pipeline_uses_portal_fd_and_prefers_stream_serial(self):
        gst = Mock()
        self.capture.Gst = gst
        self.capture.fd = 53
        pipeline = gst.parse_launch.return_value
        source = pipeline.get_by_name.return_value
        self.capture._start_pipeline(42, {'pipewire-serial': 9876})
        source.set_property.assert_any_call('fd', 53)
        source.set_property.assert_any_call('target-object', '9876')
        source.reset_mock()
        self.capture._start_pipeline(42, {})
        source.set_property.assert_any_call('path', '42')

    def test_start_requests_a_monitor_and_keeps_session_alive(self):
        gio, glib, gst = self.capture.Gio, self.capture.GLib, Mock()
        context = Mock()
        glib.MainContext = SimpleNamespace(new=lambda: context)
        gio.BusType = SimpleNamespace(SESSION=0)
        gio.bus_get_sync = Mock(return_value=self.capture.bus)
        self.capture._request = Mock(side_effect=[
            {'session_handle': '/session/approved'}, {},
            {'streams': [(42, {'pipewire-serial': 9876})]},
        ])
        self.capture._start_pipeline = Mock()
        self.capture.grab = Mock(return_value=object())
        fd = os.open(os.devnull, os.O_RDONLY)
        self.capture.bus.call_with_unix_fd_list_sync.return_value = (
            Variant('(h)', (0,)), SimpleNamespace(get=lambda index: fd),
        )
        with patch.object(portal, 'load_runtime', return_value=(gio, glib, gst, Mock())):
            self.capture.start()
        calls = self.capture._request.call_args_list
        self.assertEqual([call.args[0] for call in calls],
                         ['CreateSession', 'SelectSources', 'Start'])
        self.assertEqual(calls[1].args[3]['types'].unpack(), 1)
        self.assertFalse(calls[1].args[3]['multiple'].unpack())
        self.capture._start_pipeline.assert_called_once_with(42, {'pipewire-serial': 9876})
        self.assertEqual(self.capture.session, '/session/approved')
        self.assertEqual(self.capture.fd, fd)
        context.pop_thread_default.assert_called_once()
        self.capture.close()
        with self.assertRaises(OSError):
            os.fstat(fd)

    def test_start_failure_closes_session_and_pops_context(self):
        gio, glib, gst = self.capture.Gio, self.capture.GLib, Mock()
        context = Mock()
        glib.MainContext = SimpleNamespace(new=lambda: context)
        gio.BusType = SimpleNamespace(SESSION=0)
        bus = self.capture.bus
        gio.bus_get_sync = Mock(return_value=bus)
        self.capture._request = Mock(side_effect=[
            {'session_handle': '/session/approved'}, {}, RuntimeError('cancelled'),
        ])
        with patch.object(portal, 'load_runtime', return_value=(gio, glib, gst, Mock())):
            with self.assertRaisesRegex(RuntimeError, 'cancelled'):
                self.capture.start()
        self.assertEqual(bus.call_sync.call_args.args[1:4],
                         ('/session/approved', portal.SESSION, 'Close'))
        context.pop_thread_default.assert_called_once()
        self.assertIsNone(self.capture.bus)

    def test_missing_screencast_fails_before_session_creation(self):
        self.introspection = '<node><interface name="org.freedesktop.portal.Settings"/></node>'
        gio, glib, gst = self.capture.Gio, self.capture.GLib, Mock()
        context = Mock()
        glib.MainContext = SimpleNamespace(new=lambda: context)
        gio.BusType = SimpleNamespace(SESSION=0)
        bus = self.capture.bus
        gio.bus_get_sync = Mock(return_value=bus)
        self.capture._request = Mock()
        with patch.object(portal, 'load_runtime', return_value=(gio, glib, gst, Mock())):
            with patch.dict(os.environ, {'XDG_CURRENT_DESKTOP': 'ubuntu:GNOME'}):
                with self.assertRaisesRegex(RuntimeError, 'does not expose') as error:
                    self.capture.start()
        self.assertIn('ubuntu:GNOME', str(error.exception))
        self.assertIn('xdg-desktop-portal-gnome', str(error.exception))
        self.capture._request.assert_not_called()
        bus.call_with_unix_fd_list_sync.assert_not_called()
        bus.signal_subscribe.assert_not_called()
        self.assertEqual(bus.call_sync.call_count, 1)
        self.assertIsNone(self.capture.session)
        self.assertIsNone(self.capture.bus)
        context.pop_thread_default.assert_called_once()

    def test_introspection_transport_error_is_preserved(self):
        error = RuntimeError('Session bus disconnected')
        self.capture.bus.call_sync.side_effect = error
        with self.assertRaises(RuntimeError) as raised:
            self.capture._require_screencast()
        self.assertIs(raised.exception, error)

    def test_sample_layout_is_copied_before_unmapping(self):
        gst = SimpleNamespace(
            MessageType=SimpleNamespace(ERROR=1, EOS=2),
            MapFlags=SimpleNamespace(READ=1),
        )
        self.capture.Gst = gst
        self.capture.pipeline = Mock()
        self.capture.pipeline.get_bus.return_value.timed_pop_filtered.return_value = None
        self.capture.sink = Mock()
        sample = self.capture.sink.emit.return_value
        buffer = sample.get_buffer.return_value
        buffer.map.return_value = (True, SimpleNamespace(data=bytes(range(16))))
        self.capture.GstVideo = SimpleNamespace(
            VideoInfo=SimpleNamespace(new_from_caps=lambda caps: SimpleNamespace(
                width=1, height=2, stride=[4], offset=[0],
            )),
            buffer_get_video_meta=lambda buf: SimpleNamespace(stride=[8], offset=[4]),
        )
        shot = self.capture.grab()
        self.assertEqual(shot.bgra, bytes(range(4, 8)) + bytes(range(12, 16)))
        buffer.unmap.assert_called_once()
        self.capture.sink.emit.return_value = None
        self.assertIs(self.capture.grab(), shot)  # A static desktop keeps its last frame.

    def test_close_releases_fd_and_session_once(self):
        self.capture.Gst = SimpleNamespace(State=SimpleNamespace(NULL=0))
        self.capture.pipeline = Mock()
        pipeline = self.capture.pipeline
        self.capture.session = '/session/test'
        self.capture.session_subscription = 4
        bus = self.capture.bus
        fd = self.capture.fd = os.open(os.devnull, os.O_RDONLY)
        self.capture.close()
        self.capture.close()
        pipeline.set_state.assert_called_once_with(0)
        bus.signal_unsubscribe.assert_called_once_with(4)
        self.assertEqual(bus.call_sync.call_args.args[1:4],
                         ('/session/test', portal.SESSION, 'Close'))
        with self.assertRaises(OSError):
            os.fstat(fd)


if __name__ == '__main__':
    unittest.main()
