"""The digit reader's evaluation tool (LOC-6), on a handful of markers."""

import json
import sys
from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy as np
import pytest

TOOLS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(TOOLS))

import digit_reader_eval as tool  # noqa: E402


def steady(report):
    return {key: value for key, value in report.items() if key != "ms_a_crop"}  # the time is the machine's, not the seed's


def test_a_small_run_counts_answers_and_abstentions_and_repeats_with_its_seed():
    report = tool.evaluate(count=40, seed=5)
    assert report["count"] == 40 and report["answered"] == report["right"] + report["wrong"] and report["answered"] + sum(report["abstained"].values()) == 40
    assert (report["unreadable"], report["unreadable_answered"]) == (0, 0)
    assert report["coverage"] == report["answered"] / 40 and report["precision"] >= 0.97
    assert (report["classifier"], report["layout"]) == ("template (stand-in)", "x_2027") and "115 to 143 px" in report["source"]
    assert steady(report) == steady(tool.evaluate(count=40, seed=5))
    # This year's marker does not say which way up: its 6s and 9s are left, and counted apart.
    assert report["six_nine"] > 0 and report["six_nine_left"] == report["abstained"]["a 6 or a 9"] and report["coverage_but_six_nine"] > report["coverage"]
    assert report["ms_a_crop"]["median"] > 0.0 and report["ms_a_crop"]["p95"] >= report["ms_a_crop"]["median"]
    lines = tool.summary(report)
    assert "precision" in lines[0] and "coverage" in lines[0] and "stand-in" in lines[1] and any("6 or 9" in line for line in lines) and "time a crop" in lines[-1]
    json.dumps(report)


def test_last_years_plate_has_a_cue_and_its_sixes_and_nines_are_answered():
    report = tool.evaluate(count=40, seed=5, layout="plate_2026")
    assert report["layout"] == "plate_2026" and report["six_nine"] > 0 and report["six_nine_left"] == 0 and report["coverage"] >= 0.9
    assert not any("6 or 9" in line for line in tool.summary(report))


def test_the_lines_are_the_specs_and_the_verdict_holds_both():
    assert (tool.MIN_PRECISION, tool.MIN_COVERAGE) == (0.99, 0.90)
    assert tool.verdict({"precision": 0.995, "coverage": 0.93}) and not tool.verdict({"precision": 0.98, "coverage": 0.99}) and not tool.verdict({"precision": 1.0, "coverage": 0.85})


def test_the_crop_is_the_markers_box_with_the_pipelines_margin():
    image = np.zeros((300, 300, 3), dtype=np.uint8)
    crop = tool.crop_of(image, [[100, 100], [200, 100], [200, 200], [100, 200]])
    assert crop.origin == (85, 85) and crop.image.shape == (130, 130, 3) and tool.CROP_MARGIN == 0.15
    edge = tool.crop_of(image, [[-10, 10], [90, 10], [90, 110], [-10, 110]])
    assert edge.origin == (0, 0)  # cut to the picture


def test_the_command_line_fails_a_run_under_the_lines(tmp_path, capsys):
    out = tmp_path / "report.json"
    code = tool.main(["--n", "30", "--seed", "2", "--mpp", "0.03", "--report", str(out)])  # 20 px markers: under the reader's 24 px
    assert code == 1 and json.loads(out.read_text())["coverage"] < 0.5 and "FAIL" in capsys.readouterr().out


def crop_set(folder, count=12, seed=3, digits=None):
    """A labelled crop set as the field day's will be: crop images and a labels.csv. Generated, not recorded."""
    folder.mkdir()
    lines = ["file,digit"]
    for index, (crop, digit) in enumerate(tool.synthetic_crops(count, seed, tool.DEFAULT_MPP, "x_2027")):
        if digits is not None and digit not in digits:
            continue
        cv2.imwrite(str(folder / f"crop_{index:03d}.png"), crop.image)
        lines.append(f"crop_{index:03d}.png,{digit}")
    (folder / tool.LABELS_FILE).write_text("\n".join(lines) + "\n")
    return len(lines) - 1


def test_a_labelled_crop_set_is_scored_the_way_the_field_days_will_be(tmp_path, capsys):
    written = crop_set(tmp_path / "day", digits={0, 1, 2, 3, 4, 5, 7, 8})
    report = tool.evaluate(crops=tmp_path / "day")
    assert report["count"] == written and report["source"] == "labelled crops (day)" and report["wrong"] == 0 and report["coverage"] >= 0.9
    assert tool.main(["--crops", str(tmp_path / "day")]) == 0 and "labelled crops (day)" in capsys.readouterr().out
    # A set that cannot be read is said, not scored.
    assert tool.main(["--crops", str(tmp_path / "nowhere")]) == 2 and "is not a file" in capsys.readouterr().err
    (tmp_path / "day" / tool.LABELS_FILE).write_text("file,digit\ncrop_000.png,12\n")
    assert tool.main(["--crops", str(tmp_path / "day")]) == 2 and "0 to 9" in capsys.readouterr().err
    (tmp_path / "day" / tool.LABELS_FILE).write_text("file,digit\nmissing.png,3\n")
    assert tool.main(["--crops", str(tmp_path / "day")]) == 2 and "not an image" in capsys.readouterr().err
    (tmp_path / "day" / tool.LABELS_FILE).write_text("file,digit\n")
    assert tool.main(["--crops", str(tmp_path / "day")]) == 2 and "no crops" in capsys.readouterr().err


def test_the_trained_classifier_is_scored_through_the_same_reader(tmp_path, monkeypatch, capsys):
    """`--model` is an ONNX file run on the CPU. Here ONNX Runtime is stood in for by a session that always answers 3."""

    class Session:
        def get_inputs(self):
            return [SimpleNamespace(name="image")]

        def run(self, outputs, feeds):
            assert feeds["image"].shape == (1, 1, 64, 64)
            scores = np.full((1, 10), -4.0, dtype=np.float32)
            scores[0, 3] = 6.0
            return [scores]

    made = []
    monkeypatch.setitem(sys.modules, "onnxruntime", SimpleNamespace(InferenceSession=lambda path, providers: made.append((path, providers)) or Session()))
    model = tmp_path / "digits.onnx"
    model.write_bytes(b"stand-in")
    written = crop_set(tmp_path / "day", count=10)
    report = tool.evaluate(crops=tmp_path / "day", model=model)
    assert made == [(str(model), ["CPUExecutionProvider"])] and report["classifier"] == "onnx"
    assert report["answered"] == report["count"] == written and set(report["confusions"]) <= {f"{digit} read as 3" for digit in range(10)}
    # No model file: said, exit code 2.
    assert tool.main(["--model", str(tmp_path / "missing.onnx")]) == 2 and "is not a file" in capsys.readouterr().err


def test_a_crop_labelled_unreadable_is_not_asked_for_and_an_answer_on_it_is_a_wrong_one(tmp_path, capsys):
    """DAT-08 labels a crop a digit or `unreadable`. A field of grass is unreadable and the reader says nothing;
    a clear marker labelled unreadable (a labelling slip here, a reader's guess on the field day) is a wrong answer."""
    folder = tmp_path / "day"
    written = crop_set(folder, count=6, digits={0, 1, 2, 3, 4, 5, 7, 8})
    cv2.imwrite(str(folder / "grass.png"), np.full((180, 180, 3), (85, 120, 95), dtype=np.uint8))
    clear = next(crop for crop, digit in tool.synthetic_crops(20, 9, tool.DEFAULT_MPP, "x_2027") if digit == 3)
    cv2.imwrite(str(folder / "clear.png"), clear.image)
    with (folder / tool.LABELS_FILE).open("a") as labels:
        labels.write("grass.png,unreadable\nclear.png,Unreadable\n")
    report = tool.evaluate(crops=folder)
    assert report["count"] == written and (report["unreadable"], report["unreadable_answered"]) == (2, 1)
    assert report["wrong"] == 1 and report["confusions"] == {"unreadable read as 3": 1}
    assert report["coverage"] == (report["answered"] - 1) / written and report["precision"] == report["right"] / report["answered"] < 1.0
    assert any("unreadable: 2 crops labelled so, 1 of them answered" in line for line in tool.summary(report))
