import math

import pytest
import yaml

from irl_robot_estimation.camera_tf import load_camera_extrinsics


def test_load_saved_pose_without_using_optical_pose(tmp_path):
    path = tmp_path / "extrinsics.yaml"
    path.write_text(yaml.safe_dump({
        "parent_frame": "base_link", "child_frame": "camera_link",
        "translation_m": [0.1, 0.2, 0.6], "rotation_xyzw": [0, 0, 0, 2],
        "base_to_optical": {"translation_m": [9, 9, 9]},
    }))
    data = load_camera_extrinsics(str(path))
    assert data["translation_m"] == [0.1, 0.2, 0.6]
    assert data["rotation_xyzw"] == [0, 0, 0, 1]


@pytest.mark.parametrize("replacement", [
    {"parent_frame": "camera_link"},
    {"child_frame": ""},
    {"translation_m": [0, 0]},
    {"translation_m": [math.nan, 0, 0]},
    {"rotation_xyzw": [0, 0, 0, 0]},
])
def test_reject_invalid_transform(tmp_path, replacement):
    data = {
        "parent_frame": "base_link", "child_frame": "camera_link",
        "translation_m": [0, 0, 0], "rotation_xyzw": [0, 0, 0, 1],
    }
    data.update(replacement)
    path = tmp_path / "extrinsics.yaml"
    path.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError):
        load_camera_extrinsics(str(path))
