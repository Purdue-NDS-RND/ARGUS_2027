# The impulse rig (OPS-07)

The delivery's score will depend on a contact impulse limit that the organisers have not yet given (question 9: the limit, and how they measure it). This is how the team measures its own set-down so that the descent speed can be set the day the limit is known. The analysis is `tools/impulse_rig.py`; what is here is the rig and the procedure, and neither has been built or run yet.

## The rig

- A stiff plate about 0.4 m square (plywood 12 mm or aluminium), the size of the delivery target's centre.
- **A load cell under the plate:** one bar-type load cell rated 5 to 10 kg, on a base that does not rock. The amplifier must be read at 1000 samples a second or more: a touch lasts 10 to 50 ms, and the common hobby amplifier at 80 samples a second gives two or three samples of it (the tool refuses such a recording). Calibrate with two known masses; zero with the plate empty.
- An accelerometer on a free plate is not a rig the tool reads: such a plate starts and ends at rest, so its acceleration sums to nothing over the touch, and it never shows the package's weight. If only an accelerometer is to hand, it needs its own analysis (the plate's peak acceleration against descent speed), which is not built.
- A recording is a CSV with a header, `t_s,force_n` (newtons above the empty plate). It starts with the plate empty and runs until the package has rested for half a second.
- A set-down in which the weight comes on without a knock (the vehicle holds the package until it stands, which is what the delivery does) bounds no speed: the tool says the limit does not bind at the speeds tried, and faster trials find where it does.

## The procedure

1. Weigh the package. Zero the plate.
2. Set the package down on the plate with the aircraft, as the delivery does, at three or more descent speeds around the planned one (0.12 m/s): for example 0.08, 0.12 and 0.20 m/s, three set-downs each. The speed is the one the flight controller's log shows at the touch, not the one asked for.
3. `python3 tools/impulse_rig.py --trial <file>:<speed> ... --package-kg <mass>`; with the limit, `--limit-ns <N s>` or `--limit-peak-n <N>`.
4. Put the table in `docs/results/YYYY-MM-DD-impulse-rig.md` with the command.

## Reading the result

- **Impulse** is the momentum the plate took from the package: for a clean set-down, the package's mass times its speed. The "momentum's mass" column should be the package's mass; when it is not, the tool warns: the line was still carrying part of the weight, the vehicle pushed, or the speed is not the one flown.
- **Peak force** depends on how hard the contact is (the package's and the plate's stiffness), which the impulse does not. If the organisers limit a peak, their surface matters: ask what it is.
- **Fastest descent under the limit** uses the worst trial and keeps a fifth in hand. The command's exit code is 1 when 0.12 m/s (or `--at`) is over it.
- Heavier packages earn 0.25 points per 500 g and raise both numbers in proportion: the package's weight is the team's decision once the limit is known.

The set-down speed itself is a tuned value of the delivery mission; changing it is a tuning change like any other.
