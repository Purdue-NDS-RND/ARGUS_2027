# The digit reader on synthetic markers (LOC-6)

2026-11-20, on `d0e3997` plus the LOC-6 change (uncommitted when run; it is the commit this file comes with).

This is not the spec's test. The spec's line (at least 99% precision while answering at least 90% of crops) is on
a held-out field day with the trained classifier, and both wait for training data. Here the classifier is the
template stand-in, on the synthetic markers it was written for (`tools/synth_targets.py`, drawn from the same
shapes as the sim's models). What it shows is the reader's own steps (finding the plate's corners, the warp,
which way up, abstaining) at a marker size, and what a crop costs.

## Commands

```bash
python3 tools/digit_reader_eval.py --n 1000 --seed 1                      # this year's marker, 4K scale: 4.2 to 5.2 mm a pixel
python3 tools/digit_reader_eval.py --n 300 --seed 1 --mpp 0.0067
python3 tools/digit_reader_eval.py --n 300 --seed 1 --mpp 0.01
python3 tools/digit_reader_eval.py --n 300 --seed 1 --mpp 0.0136          # the sim's 1280 x 720 camera from 35 ft
python3 tools/digit_reader_eval.py --n 1000 --seed 1 --layout plate_2026  # last year's plate, 4K scale
```

Each marker: a random digit, any rotation, up to 12 degrees of tilt each way, blur up to 1 px, noise up to 4
levels, brightness 0.75 to 1.2, on a noisy field; the crop is the marker's box and 15% a side, as the pipeline
cuts it.

## Results

| Marker | Px across | Crops | Answered (coverage) | Right of answered | 6s and 9s left as "6 or 9" | Other digits answered | Lines met |
| --- | --- | --- | --- | --- | --- | --- | --- |
| 2027 (digit on the X) | 115 to 143 (the 4K frame) | 1000 | 782 (78.2%) | 782 (100.0%) | 209 of 211 | 99.1% | no: coverage |
| 2027 | 90 | 300 | 236 (78.7%) | 236 (100.0%) | 59 of 60 | 98.3% | no: coverage |
| 2027 | 60 | 300 | 227 (75.7%) | 226 (99.6%) | 59 of 60 | 94.6% | no: coverage |
| 2027 | 44 (the sim's 720p frame) | 300 | 202 (67.3%) | 202 (100.0%) | 52 of 60 | 84.2% | no: coverage |
| 2026 plate (digit to one side) | 117 to 145 | 1000 | 965 (96.5%) | 965 (100.0%) | none: it has a cue | | yes |

```
FAIL: precision 100.0% (782 of 782 answers right), coverage 78.2% (782 of 1000 crops answered); lines 99% and 90%
source: synthetic, seed 1, markers 115 to 143 px across; marker layout x_2027; classifier: template (stand-in)
6 or 9: 209 of 211 such crops left as 'a 6 or a 9' (this layout does not say which way up); of the other 789 crops, 99.1% answered
abstained: 209 a 6 or a 9, 9 probability
time a crop: median 2.65 ms, 95th percentile 2.96 ms (this machine)
```

```
PASS: precision 100.0% (965 of 965 answers right), coverage 96.5% (965 of 1000 crops answered); lines 99% and 90%
source: synthetic, seed 1, markers 117 to 145 px across; marker layout plate_2026; classifier: template (stand-in)
abstained: 21 probability, 8 too small or not a square, 6 digit's side
time a crop: median 2.86 ms, 95th percentile 3.19 ms (this machine)
```

## Reading them

- This year's marker is under the coverage line for one reason: its model has nothing that says which way up it
  is, a 6 is a 9 turned half round, and the reader answers neither from one crop. Two crops in ten are a 6 or a
  9. Of the other eight digits' crops 99.1% are answered at the 4K frame's scale, and no answer in 782 is wrong.
  Whether the rules' marker has a 6/9 mark is organiser question 8.
- Last year's plate, whose digit sits to one side, meets both lines on all ten digits: the reader's steps are
  not what holds this year's back.
- Coverage falls with the marker's size. At the sim's 44 px, 84% of the answerable crops are answered.
- A crop takes 2.6 ms on the build machine (an x86 laptop, the stand-in classifier, no lens model). The node
  allows a frame's crops 30 ms.
- A single reading is one vote: LOC-8 sums a track's readings over about 25 sightings.

## Not shown

- Real markers, a real field, the trained classifier, ONNX Runtime on the Jetson's CPU, the time a crop takes
  there: the data hold and the bench. The same tool scores them: `--crops <folder> --model <file>`.
- Lens distortion in these numbers: the synthetic pictures are an ideal pinhole's. A unit test bends a marker
  with a strong lens and checks that the camera model straightens it.
