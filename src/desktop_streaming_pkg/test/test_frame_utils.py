"""Check actual pixel conversion and display sizing without desktop hardware."""

from types import SimpleNamespace
import unittest

from desktop_streaming_pkg.frame_utils import prepare_frame


def screenshot(width, height, pixel=(10, 20, 30, 255)):
    return SimpleNamespace(
        width=width, height=height, bgra=bytes(pixel) * width * height,
    )


class FrameTests(unittest.TestCase):

    def test_bgr_order_alpha_removal_and_pixel_order(self):
        shot = screenshot(2, 1)
        shot.bgra = bytes([0, 0, 255, 0, 255, 0, 0, 255])
        self.assertEqual(
            prepare_frame(shot, 2, 1), bytes([0, 0, 255, 255, 0, 0]),
        )

    def test_wide_desktop_has_centered_letterbox(self):
        data = prepare_frame(screenshot(4, 2), 4, 4)
        row = bytes([10, 20, 30]) * 4
        self.assertEqual(data, bytes(12) + row + row + bytes(12))

    def test_tall_desktop_has_centered_pillarbox(self):
        data = prepare_frame(screenshot(2, 4), 4, 4)
        row = bytes(3) + bytes([10, 20, 30]) * 2 + bytes(3)
        self.assertEqual(data, row * 4)

    def test_stretch_fills_the_entire_output(self):
        data = prepare_frame(screenshot(4, 2), 3, 5, False)
        self.assertEqual(data, bytes([10, 20, 30]) * 15)

    def test_downsampling_produces_display_sized_bgr(self):
        data = prepare_frame(screenshot(1920, 1080), 64, 32, False)
        self.assertEqual(len(data), 64 * 32 * 3)
        self.assertEqual(data, bytes([10, 20, 30]) * 64 * 32)

    def test_odd_display_dimensions_have_exact_byte_count(self):
        data = prepare_frame(screenshot(16, 9), 7, 5)
        self.assertEqual(len(data), 7 * 5 * 3)

    def test_extreme_aspect_ratio_keeps_at_least_one_pixel(self):
        for width, height in [(20, 1), (1, 20)]:
            with self.subTest(width=width, height=height):
                self.assertEqual(
                    prepare_frame(screenshot(width, height), 1, 1),
                    bytes([10, 20, 30]),
                )

    def test_invalid_display_dimensions(self):
        for width, height in [(0, 32), (64, 0), (-1, 32), (64, -1)]:
            with self.subTest(width=width, height=height):
                with self.assertRaisesRegex(ValueError, 'must be positive'):
                    prepare_frame(screenshot(2, 2), width, height)


if __name__ == '__main__':
    unittest.main()
