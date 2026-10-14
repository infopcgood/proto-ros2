"""Convert MSS screenshots to the packed BGR format used by the displays."""

from PIL import Image


def prepare_frame(screenshot, width, height, preserve_aspect_ratio=True):
    """Resize an MSS screenshot and return exactly width * height * 3 bytes."""
    if width <= 0 or height <= 0:
        raise ValueError('Display width and height must be positive.')
    frame = Image.frombytes(
        'RGB', (screenshot.width, screenshot.height), screenshot.bgra,
        'raw', 'BGRX',
    )
    if preserve_aspect_ratio:
        scale = min(width / frame.width, height / frame.height)
        size = (
            max(1, round(frame.width * scale)),
            max(1, round(frame.height * scale)),
        )
        resized = frame.resize(size, Image.Resampling.LANCZOS)
        frame = Image.new('RGB', (width, height), (0, 0, 0))
        frame.paste(resized, ((width - size[0]) // 2, (height - size[1]) // 2))
    else:
        frame = frame.resize((width, height), Image.Resampling.LANCZOS)
    return frame.tobytes('raw', 'BGR')
