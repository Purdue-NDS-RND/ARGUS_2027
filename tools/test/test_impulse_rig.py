"""OPS-07: the impulse rig's analysis (tools/impulse_rig.py), on set-downs generated here. No ROS."""

import math
import sys
from pathlib import Path

import pytest

TOOLS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(TOOLS))

import impulse_rig as rig  # noqa: E402


def set_down(mass_kg=0.5, descent_mps=0.12, contact_s=0.03, rate_hz=2000.0, before_s=0.2, after_s=0.5, carried=0.0, ripple=0.0):
    """A set-down as a load cell under the plate sees it: nothing, a half-sine contact that takes the package's momentum, its weight.

    The pulse's area over the weight is the momentum (mass times descent speed). `carried` is the share of the
    momentum something else took (a line still carrying); `ripple` a ringing of the plate, as a share of the weight.
    """
    weight = mass_kg * rig.G
    peak_over = math.pi * mass_kg * descent_mps * (1.0 - carried) / (2.0 * contact_s)
    times, forces = [], []
    for index in range(int((before_s + contact_s + after_s) * rate_hz)):
        t = index / rate_hz
        tau = t - before_s
        if tau < 0:
            force = 0.0
        elif tau < contact_s:
            force = weight + peak_over * math.sin(math.pi * tau / contact_s)
        else:
            force = weight * (1.0 + ripple * math.exp(-tau * 40.0) * math.sin(2 * math.pi * 60.0 * tau))
        times.append(t)
        forces.append(force)
    return times, forces


def never_quiet():
    """A plate that goes on ringing by a fifth of the weight: the package is never at rest on it."""
    times, forces = set_down()
    weight = 0.5 * rig.G
    return times, [force if t < 0.23 else weight * (1.0 + 0.2 * math.sin(2 * math.pi * 60.0 * t)) for t, force in zip(times, forces)]


def write(path, times, values, column="force_n"):
    path.write_text(f"t_s,{column}\n" + "".join(f"{t:.5f},{value:.5f}\n" for t, value in zip(times, values)))
    return path


def test_a_clean_set_downs_impulse_is_the_packages_momentum_and_its_peak_is_the_pulses():
    trial = rig.analyse("one", *set_down(), descent_mps=0.12)
    assert trial.impulse_ns == pytest.approx(0.5 * 0.12, rel=0.02) and trial.momentum_kg == pytest.approx(0.5, rel=0.02)
    assert trial.weight_n == pytest.approx(0.5 * rig.G, rel=1e-3) and trial.peak_n == pytest.approx(0.5 * rig.G + math.pi * 0.5 * 0.12 / 0.06, rel=0.01)
    assert trial.contact_s == pytest.approx(0.03, abs=0.004) and trial.samples >= 50 and trial.rate_hz == pytest.approx(2000.0, rel=1e-3)
    # A plate that rings after the touch: the contact lasts until it is quiet, and the momentum is the same.
    ringing = rig.analyse("ring", *set_down(ripple=0.3), descent_mps=0.12)
    assert ringing.contact_s > 0.035 and ringing.impulse_ns == pytest.approx(0.06, rel=0.05)
    # Twice the speed: twice the impulse, and twice the peak over the weight.
    fast = rig.analyse("fast", *set_down(descent_mps=0.24), descent_mps=0.24)
    assert fast.impulse_ns == pytest.approx(2 * trial.impulse_ns, rel=0.02) and fast.peak_n - fast.weight_n == pytest.approx(2 * (trial.peak_n - trial.weight_n), rel=0.02)


@pytest.mark.parametrize(
    "recording, word",
    [
        (set_down(rate_hz=80.0), "too slow to see the peak"),  # a hobby amplifier at 80 samples a second
        (never_quiet(), "never settles"),
        (set_down(after_s=0.05), "nothing is resting on the plate at the end"),  # stopped before the package was at rest
        (set_down(before_s=0.0), "must start with the plate empty"),
        (([0.0, 0.1, 0.2], [0.0, 1.0, 1.0]), "is not a recording of a set-down"),
        (([index / 2000.0 for index in range(2000)], [0.0] * 2000), "nothing is resting on the plate"),
    ],
)
def test_a_recording_that_cannot_give_the_answer_is_refused_with_why(recording, word):
    with pytest.raises(rig.RigError, match=word):
        rig.analyse("bad", *recording, descent_mps=0.12)


def test_the_fits_give_the_set_down_speed_its_numbers_and_a_limit_gives_the_fastest_descent():
    trials = [rig.analyse(f"run_{index}", *set_down(descent_mps=speed), descent_mps=speed) for index, speed in enumerate((0.08, 0.12, 0.20))]
    found = rig.summary(trials, package_kg=0.5)
    assert found["impulse_per_mps"] == pytest.approx(0.5, rel=0.02) and found["peak_over_weight_per_mps"] == pytest.approx(math.pi * 0.5 / 0.06, rel=0.02)
    assert found["impulse_at"] == pytest.approx(0.06, rel=0.02) and found["max_descent_mps"] is None and found["warnings"] == []
    assert "No limit was given" in rig.report(trials, found)
    by_impulse = rig.summary(trials, package_kg=0.5, limit_ns=0.10)
    assert by_impulse["max_descent_mps"] == pytest.approx(0.8 * 0.10 / 0.5, rel=0.03) and by_impulse["limited_by"] == "impulse" and by_impulse["impulse_share_at"] == pytest.approx(0.6, rel=0.02)
    assert "The set-down at 0.12 m/s is under it" in rig.report(trials, by_impulse)
    # Both limits given: the stricter one sets the speed.
    both = rig.summary(trials, package_kg=0.5, limit_ns=0.10, limit_peak_n=6.0)
    assert both["limited_by"] == "peak force" and both["max_descent_mps"] == pytest.approx(0.8 * (6.0 - 0.5 * rig.G) / (math.pi * 0.5 / 0.06), rel=0.03)
    assert "IS OVER IT: slow the set-down" in rig.report(trials, both)
    with pytest.raises(rig.RigError, match="no more than the package's own weight"):
        rig.summary(trials, limit_peak_n=4.0)
    # The worst trial sets the speed, not the average: one hard set-down among soft ones.
    hard = trials + [rig.analyse("hard", *set_down(descent_mps=0.12, contact_s=0.01), descent_mps=0.12)]
    assert rig.summary(hard, limit_peak_n=9.0)["max_descent_mps"] < 0.5 * rig.summary(trials, limit_peak_n=9.0)["max_descent_mps"]


def test_set_downs_that_are_not_clean_and_a_plate_that_reads_wrong_are_warned_about():
    carrying = [rig.analyse("line", *set_down(carried=0.5), descent_mps=0.12)]
    found = rig.summary(carrying, package_kg=0.5)
    assert any("the impulse is that of 0.25 kg at 0.12 m/s, and the package is 0.50 kg" in warning for warning in found["warnings"])
    assert any("fewer than three descent speeds" in warning for warning in found["warnings"])
    heavy = rig.summary([rig.analyse("x", *set_down(mass_kg=0.6), descent_mps=0.12)], package_kg=0.5)
    assert any("the plate reads 0.60 kg at rest, and the package is 0.50 kg" in warning for warning in heavy["warnings"])
    with pytest.raises(rig.RigError, match="no trials"):
        rig.summary([])


def gentle(mass_kg=0.5, transfer_s=0.05, rate_hz=2000.0, before_s=0.2, after_s=0.5):
    """A set-down in which the vehicle holds the package until it stands: the weight comes on in a straight line, and nothing knocks."""
    weight = mass_kg * rig.G
    times = [index / rate_hz for index in range(int((before_s + transfer_s + after_s) * rate_hz))]
    return times, [0.0 if t < before_s else weight * min(1.0, (t - before_s) / transfer_s) for t in times]


def test_set_downs_that_never_knock_bound_no_speed_and_are_not_called_over_the_limit(tmp_path, capsys):
    """The spec audit: these gave a negative "fastest descent", "IS OVER IT" and exit code 1, and a slope of exactly nothing a traceback."""
    files = [write(tmp_path / f"soft_{index}.csv", *gentle()) for index in range(3)]
    trials = [f"--trial={path}:{speed}" for path, speed in zip(files, (0.08, 0.12, 0.20))]
    for limit in (["--limit-ns", "0.05"], ["--limit-peak-n", "9.0"], ["--limit-ns", "0.05", "--limit-peak-n", "9.0"]):
        assert rig.main(trials + limit) == 0
        out = capsys.readouterr().out
        assert "The limit bounds no descent speed at the speeds tried" in out and "IS OVER IT" not in out and "Fastest descent" not in out
        assert "the weight came on without a knock" in out
    found = rig.summary([rig.analyse("soft", *gentle(), descent_mps=0.12)], limit_ns=0.05, limit_peak_n=9.0)
    assert found["max_descent_mps"] is None and found["limit_given"] and 0.0 <= found["impulse_share_at"] and found["peak_share_at"] == pytest.approx(0.5 * rig.G / 9.0, rel=0.01)
    # One knocking trial among them is what bounds the speed.
    mixed = rig.summary([rig.analyse("soft", *gentle(), descent_mps=0.12), rig.analyse("knock", *set_down(descent_mps=0.2), descent_mps=0.2)], limit_ns=0.05)
    assert mixed["max_descent_mps"] == pytest.approx(0.8 * 0.05 / 0.5, rel=0.05)


def test_the_command_reads_load_cell_recordings_and_its_exit_code_says_over_the_limit(tmp_path, capsys):
    files = [write(tmp_path / f"run_{index}.csv", *set_down(descent_mps=speed)) for index, speed in enumerate((0.08, 0.12, 0.20))]
    trials = [f"--trial={path}:{speed}" for path, speed in zip(files, (0.08, 0.12, 0.20))]
    assert rig.main(trials + ["--package-kg", "0.5"]) == 0
    out = capsys.readouterr().out
    assert "| run_1.csv | 0.12 | 0.0" in out and "No limit was given" in out and "WARNING" not in out
    assert rig.main(trials + ["--package-kg", "0.5", "--limit-ns", "0.10"]) == 0 and "Fastest descent under the limit (0.1 N s, by impulse" in capsys.readouterr().out
    assert rig.main(trials + ["--limit-ns", "0.05"]) == 1 and "IS OVER IT" in capsys.readouterr().out
    # An accelerometer's recording is not read (a free plate's acceleration sums to nothing and shows no weight to rest at).
    times, forces = set_down()
    plate = write(tmp_path / "plate.csv", times, [force / 1.2 for force in forces], column="accel_mps2")
    for arguments, word in (([f"--trial={plate}:0.12"], "expected a header with t_s and force_n"), ([], "give at least one --trial"),
                            ([f"--trial={files[0]}:fast"], "expected FILE:SPEED"), ([f"--trial={tmp_path / 'none.csv'}:0.1"], "No such file")):  # fmt: skip
        assert rig.main(arguments) == 2 and word in capsys.readouterr().err
    with pytest.raises(SystemExit):
        rig.main([f"--trial={plate}:0.12", "--accelerometer"])
    capsys.readouterr()
    backwards = write(tmp_path / "back.csv", [0.0, 0.2, 0.1], [0.0, 1.0, 1.0])
    assert rig.main([f"--trial={backwards}:0.1"]) == 2 and "the times do not go forward" in capsys.readouterr().err
