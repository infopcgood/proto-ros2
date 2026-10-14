"""Capture an approved Wayland monitor through XDG ScreenCast and PipeWire."""

import os
import sys
import time
from types import SimpleNamespace
import uuid
from xml.etree import ElementTree


BUS_NAME = 'org.freedesktop.portal.Desktop'
OBJECT_PATH = '/org/freedesktop/portal/desktop'
SCREENCAST = 'org.freedesktop.portal.ScreenCast'
REQUEST = 'org.freedesktop.portal.Request'
SESSION = 'org.freedesktop.portal.Session'


def select_backend(backend):
    """Choose the portal on Wayland without trying XWayland screen capture."""
    if backend not in ('auto', 'mss', 'portal'):
        raise ValueError('backend must be auto, mss, or portal.')
    linux = sys.platform.startswith('linux')
    wayland = linux and (
        os.environ.get('XDG_SESSION_TYPE', '').lower() == 'wayland' or
        bool(os.environ.get('WAYLAND_DISPLAY'))
    )
    if backend == 'auto':
        backend = 'portal' if wayland else 'mss'
    if backend == 'portal' and not linux:
        raise ValueError('The portal capture backend is only available on Linux.')
    if backend == 'mss' and wayland:
        raise ValueError('MSS cannot capture native Wayland desktops. '
                         'Use --ros-args -p backend:=portal instead.')
    return backend


def unpack_bgrx(data, width, height, stride, offset=0):
    """Copy padded GStreamer rows to tightly packed MSS-compatible pixels."""
    row_bytes = width * 4
    if width <= 0 or height <= 0 or offset < 0 or stride < row_bytes:
        raise ValueError('Invalid PipeWire frame dimensions or row stride.')
    required = offset + (height - 1) * stride + row_bytes
    if required > len(data):
        raise ValueError('PipeWire frame buffer is shorter than its video layout.')
    pixels = b''.join(
        data[offset + row * stride:offset + row * stride + row_bytes]
        for row in range(height)
    )
    return SimpleNamespace(width=width, height=height, bgra=pixels)


def load_runtime():
    """Load optional system libraries only when the portal backend is used."""
    try:
        import gi
        gi.require_version('Gio', '2.0')
        gi.require_version('Gst', '1.0')
        gi.require_version('GstVideo', '1.0')
        from gi.repository import Gio, GLib, Gst, GstVideo
    except (ImportError, ValueError) as exc:
        raise RuntimeError(
            'Wayland capture needs PyGObject and GStreamer. On Ubuntu/Debian, '
            'install python3-gi python3-gst-1.0 gir1.2-gst-plugins-base-1.0 '
            'gstreamer1.0-pipewire gstreamer1.0-plugins-base. A virtual '
            'environment must expose system site packages.'
        ) from exc
    return Gio, GLib, Gst, GstVideo


class PortalCapture:
    """Keep the approved portal session and its PipeWire connection alive."""

    def __init__(self, timeout=120.0, frame_timeout=10.0, is_running=lambda: True):
        self.timeout = timeout
        self.frame_timeout = frame_timeout
        self.is_running = is_running
        self.bus = None
        self.context = None
        self.session = None
        self.session_subscription = None
        self.pipeline = None
        self.fd = None
        self.last_frame = None
        self.closed = False

    def start(self):
        """Ask the user to share one monitor and wait for the first video frame."""
        self.Gio, self.GLib, self.Gst, self.GstVideo = load_runtime()
        self.Gst.init(None)
        if self.Gst.ElementFactory.find('pipewiresrc') is None:
            raise RuntimeError('Missing pipewiresrc; install gstreamer1.0-pipewire.')
        self.context = self.GLib.MainContext.new()
        self.context.push_thread_default()
        try:
            self.bus = self.Gio.bus_get_sync(self.Gio.BusType.SESSION, None)
            self._require_screencast()
            sender = self.bus.get_unique_name()[1:].replace('.', '_')
            token = 'desktop_' + uuid.uuid4().hex
            # Predict the session path so even an interrupted create can close it.
            self.session = f'{OBJECT_PATH}/session/{sender}/{token}'
            result = self._request('CreateSession', options={
                'session_handle_token': self.GLib.Variant('s', token),
            })
            self.session = result['session_handle']
            self.session_subscription = self.bus.signal_subscribe(
                BUS_NAME, SESSION, 'Closed', self.session, None,
                self.Gio.DBusSignalFlags.NONE, self._session_closed, None,
            )
            self._request('SelectSources', '(oa{sv})', (self.session,), {
                'types': self.GLib.Variant('u', 1),  # MONITOR
                'multiple': self.GLib.Variant('b', False),
            })
            result = self._request('Start', '(osa{sv})', (self.session, ''))
            streams = result.get('streams', [])
            if len(streams) != 1:
                raise RuntimeError('The portal did not return one selected monitor.')
            node_id, properties = streams[0]
            reply, fd_list = self.bus.call_with_unix_fd_list_sync(
                BUS_NAME, OBJECT_PATH, SCREENCAST, 'OpenPipeWireRemote',
                self.GLib.Variant('(oa{sv})', (self.session, {})),
                self.GLib.VariantType.new('(h)'), self.Gio.DBusCallFlags.NONE,
                10000, None, None,
            )
            # UnixFDList.get() duplicates the descriptor; this instance owns it.
            self.fd = fd_list.get(reply.unpack()[0])
            self._start_pipeline(node_id, properties)
            deadline = time.monotonic() + self.frame_timeout
            while self.grab() is None:
                if time.monotonic() >= deadline:
                    raise RuntimeError('Timed out waiting for a PipeWire video frame.')
                time.sleep(0.01)
        except BaseException:
            self.close()
            raise
        finally:
            self.context.pop_thread_default()

    def _require_screencast(self):
        """Explain an unavailable desktop backend before creating a session."""
        reply = self.bus.call_sync(
            BUS_NAME, OBJECT_PATH, 'org.freedesktop.DBus.Introspectable',
            'Introspect', None, self.GLib.VariantType.new('(s)'),
            self.Gio.DBusCallFlags.NONE, 10000, None,
        )
        interfaces = ElementTree.fromstring(reply.unpack()[0]).findall('interface')
        if not any(interface.get('name') == SCREENCAST for interface in interfaces):
            desktop = os.environ.get('XDG_CURRENT_DESKTOP') or 'unset'
            raise RuntimeError(
                f'The desktop portal does not expose {SCREENCAST} '
                f'(XDG_CURRENT_DESKTOP={desktop!r}). Install and enable the '
                'ScreenCast backend for your actual desktop: '
                'xdg-desktop-portal-gnome for GNOME, xdg-desktop-portal-kde '
                'for KDE Plasma, or xdg-desktop-portal-wlr for Sway. '
                'The generic xdg-desktop-portal package alone is insufficient. '
                'If the backend is installed, check its user service logs and '
                'portals.conf selection. Log out and back in after setup, then '
                'run from a terminal inside the graphical desktop session. '
                'See the package README troubleshooting instructions.'
            )

    def _request(self, method, signature='(a{sv})', args=(), options=None):
        """Subscribe before making a portal call, then await its Response signal."""
        token = 'desktop_' + uuid.uuid4().hex
        sender = self.bus.get_unique_name()[1:].replace('.', '_')
        paths = {f'{OBJECT_PATH}/request/{sender}/{token}'}
        response = []

        def received(connection, sender_name, path, interface, signal, params, user_data):
            if path in paths:
                response.append(params.unpack())

        subscription = self.bus.signal_subscribe(
            BUS_NAME, REQUEST, 'Response', None, None,
            self.Gio.DBusSignalFlags.NONE, received, None,
        )
        handle = next(iter(paths))
        try:
            opts = dict(options or {})
            opts['handle_token'] = self.GLib.Variant('s', token)
            reply = self.bus.call_sync(
                BUS_NAME, OBJECT_PATH, SCREENCAST, method,
                self.GLib.Variant(signature, (*args, opts)),
                self.GLib.VariantType.new('(o)'), self.Gio.DBusCallFlags.NONE,
                10000, None,
            )
            handle = reply.unpack()[0]
            paths.add(handle)  # Also accept older portals with a different path.
            deadline = time.monotonic() + self.timeout
            while not response:
                self._pump_events()
                if response:
                    break
                if time.monotonic() >= deadline:
                    raise RuntimeError(f'Portal {method} timed out; select a screen '
                                       'or increase portal_timeout.')
                time.sleep(0.01)
            code, result = response[0]
            if code == 1:
                raise RuntimeError('Screen sharing was cancelled in the portal dialog.')
            if code != 0:
                raise RuntimeError(f'Portal {method} failed (response {code}).')
            return result
        finally:
            self.bus.signal_unsubscribe(subscription)
            if not response:
                self._close_remote(handle, REQUEST)

    def _start_pipeline(self, node_id, properties):
        self.pipeline = self.Gst.parse_launch(
            'pipewiresrc name=desktop_source do-timestamp=true ! '
            'queue max-size-buffers=1 max-size-bytes=0 max-size-time=0 '
            'leaky=downstream ! videoconvert ! video/x-raw,format=BGRx ! '
            'appsink name=desktop_frames max-buffers=1 drop=true sync=false'
        )
        source = self.pipeline.get_by_name('desktop_source')
        source.set_property('fd', self.fd)
        serial = properties.get('pipewire-serial')
        if serial is not None and source.find_property('target-object') is not None:
            source.set_property('target-object', str(serial))
        else:
            source.set_property('path', str(node_id))
        self.sink = self.pipeline.get_by_name('desktop_frames')
        if self.pipeline.set_state(self.Gst.State.PLAYING) == self.Gst.StateChangeReturn.FAILURE:
            raise RuntimeError('Could not start the PipeWire GStreamer pipeline.')

    def _session_closed(self, *args):
        self.closed = True

    def _pump_events(self):
        while self.context.pending():
            self.context.iteration(False)
        if not self.is_running():
            raise KeyboardInterrupt
        if self.closed:
            raise RuntimeError('The desktop portal ended screen sharing.')

    def grab(self, monitor=None):
        """Return the latest approved frame, including for an unchanged desktop."""
        self._pump_events()
        bus = self.pipeline.get_bus()
        message = bus.timed_pop_filtered(
            0, self.Gst.MessageType.ERROR | self.Gst.MessageType.EOS,
        )
        if message is not None:
            if message.type == self.Gst.MessageType.ERROR:
                error, debug = message.parse_error()
                raise RuntimeError(f'PipeWire capture failed: {error.message}')
            raise RuntimeError('PipeWire screen capture ended.')
        sample = self.sink.emit('try-pull-sample', 0)
        if sample is not None:
            info = self.GstVideo.VideoInfo.new_from_caps(sample.get_caps())
            buffer = sample.get_buffer()
            mapped, mapping = buffer.map(self.Gst.MapFlags.READ)
            if not mapped:
                raise RuntimeError('Could not map the PipeWire video frame.')
            try:
                meta = self.GstVideo.buffer_get_video_meta(buffer)
                layout = meta if meta is not None else info
                self.last_frame = unpack_bgrx(
                    mapping.data, info.width, info.height,
                    layout.stride[0], layout.offset[0],
                )
            finally:
                buffer.unmap(mapping)
        return self.last_frame

    def _close_remote(self, path, interface):
        try:
            self.bus.call_sync(
                BUS_NAME, path, interface, 'Close', None, None,
                self.Gio.DBusCallFlags.NONE, 2000, None,
            )
        except self.GLib.Error:
            pass  # The compositor may already have closed the request/session.

    def close(self):
        """Release the pipeline before closing its FD and portal session."""
        try:
            if self.pipeline is not None:
                pipeline, self.pipeline = self.pipeline, None
                pipeline.set_state(self.Gst.State.NULL)
        finally:
            try:
                if self.fd is not None:
                    fd, self.fd = self.fd, None
                    os.close(fd)
            finally:
                if self.bus is not None:
                    if self.session_subscription is not None:
                        self.bus.signal_unsubscribe(self.session_subscription)
                        self.session_subscription = None
                    if self.session is not None:
                        self._close_remote(self.session, SESSION)
                        self.session = None
                    self.bus = None
                self.last_frame = None
