"""Stand-ins for the vision_msgs 3D detection messages, shared by the estimator's tests. No ROS, and no imports of the code under test."""


class Record:
    """A message stand-in that has only the fields it was made with."""

    def __init__(self, **fields):
        for name, value in fields.items():
            object.__setattr__(self, name, value)

    def __setattr__(self, name, value):
        if name not in self.__dict__:
            raise AttributeError(f"no field '{name}'")
        object.__setattr__(self, name, value)


def _header():
    return Record(stamp=Record(sec=0, nanosec=0), frame_id="")


def _pose():
    return Record(position=Record(x=0.0, y=0.0, z=0.0), orientation=Record(x=0.0, y=0.0, z=0.0, w=1.0))


class FakeMessages:
    """vision_msgs 4.1 (ROS 2 Humble): the fields of the 3D detection messages."""

    @staticmethod
    def Detection3DArray():
        return Record(header=_header(), detections=[])

    @staticmethod
    def Detection3D():
        return Record(header=_header(), results=[], bbox=Record(center=_pose(), size=Record(x=0.0, y=0.0, z=0.0)), id="")

    @staticmethod
    def ObjectHypothesisWithPose():
        return Record(hypothesis=Record(class_id="", score=0.0), pose=Record(pose=_pose(), covariance=[0.0] * 36))
