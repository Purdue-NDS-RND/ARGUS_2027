"""Which stamp a detection carries. No ROS imports."""

from __future__ import annotations

from typing import Any


def detection_stamp(image_stamp: Any, now_stamp: Any, stamp_from_image: Any) -> Any:
    """The stamp for a detection made from one image.

    With the `features.stamp_from_image` node flag off (the default, and what
    flew in 2026) it is the detector's clock when the result is published.
    With it on it is the image's own stamp, so a consumer can tell how old
    the picture behind a detection is. An image with no stamp (zero) still
    gets the clock: a zero stamp would read as decades old.
    """
    if stamp_from_image is not True or image_stamp is None:
        return now_stamp
    if not (image_stamp.sec or image_stamp.nanosec):
        return now_stamp
    return image_stamp
