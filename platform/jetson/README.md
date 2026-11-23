# The flight computer: Jetson Orin Nano Super, JetPack 6.2

`setup.sh` takes a freshly flashed JetPack 6.2 to flight state and says, step by step, whether it got there. It can be run again at any time: a step that is already done is left alone.

```
sudo platform/jetson/setup.sh --check     # the checklist, changes nothing
sudo platform/jetson/setup.sh --apply     # do what is missing, then the checklist
```

| Step | Flight state | How it is checked |
| --- | --- | --- |
| `platform` | Jetson Linux 36.4.x on an Orin Nano | `/etc/nv_tegra_release`, the device-tree model |
| `power_mode` | MAXN SUPER | `nvpmodel -q` |
| `jetson_clocks` | `argus-jetson-clocks.service` runs `jetson_clocks` at boot | the unit file and `systemctl is-enabled` |
| `fan` | nvfancontrol's `cool` profile | `/etc/nvfancontrol.conf` |
| `log_disk` | the NVMe filesystem labelled `argus-logs` mounted on `/mnt/argus-logs` (a plain folder when the Jetson boots from the NVMe) | `/etc/fstab` and `findmnt` |
| `ros` | ROS 2 Humble, MAVROS, the packages `src/` depends on, the geoid model | `dpkg-query`, `/opt/ros/humble`, the geoid file |
| `onnxruntime` | ONNX Runtime with a GPU provider | `python3 -c "import onnxruntime ..."` |
| `udev` | the flight controller is `/dev/argus/pixhawk`; the flight user is in `dialout` | the rule file, `id -nG` |

Exit code 0 means every step is ok. A whole `--apply` leaves its checklist in `/var/lib/argus/setup-checklist.txt` with the date and the repo's commit.

## First run on a new Jetson (bench)

1. Flash JetPack 6.2 with SDK Manager, finish the first-boot setup, clone this repo.
2. `sudo platform/jetson/setup.sh --check` and read what is missing.
3. The log disk. If the Jetson boots from the SD card, make one partition on the NVMe (`sudo parted /dev/nvme0n1 -- mklabel gpt mkpart logs ext4 0% 100%`) and pass it once: `sudo platform/jetson/setup.sh --apply --log-device /dev/nvme0n1p1`. The script formats only a partition with no filesystem on it, never the boot disk. If the Jetson boots from the NVMe there is nothing to do: `/mnt/argus-logs` is a folder.
4. `sudo platform/jetson/setup.sh --apply`. If `power_mode` says to reboot, reboot and run it again.
5. Plug in the flight controller: `ls -l /dev/argus/pixhawk` must point at its first serial port, and `--check` then says "is present". If it does not appear, `udevadm info -a /dev/ttyACM0 | grep -m3 -E 'idVendor|idProduct|serial'` gives the ids to add to `PIXHAWK_USB_VENDORS` in `setup.conf`. To pin the name to that one board, run `--apply --only udev` with `ARGUS_PIXHAWK_SERIAL=<its serial>` set (use `sudo -E`), and always run with it afterwards.
6. Correct anything in `setup.conf` marked "bench" that turned out different, in a pull request.
7. Log out and in again (the `dialout` group), then start MAVROS on `/dev/argus/pixhawk:115200`.

## The backup image

Once the checklist is 8 of 8 and one flight has been made on it, keep an image of the disk the Jetson boots from, so a dead card on competition day is ten minutes and not a day.

- SD card: power off, put the card in the ground laptop's reader, find it with `lsblk`, then
  `sudo dd if=/dev/sdX bs=4M status=progress | zstd -T0 > argus-orin-$(date +%F).img.zst && sha256sum argus-orin-*.img.zst > argus-orin.sha256`.
  Restore with `zstd -dc argus-orin-<date>.img.zst | sudo dd of=/dev/sdX bs=4M status=progress` onto a card of the same size or larger.
- NVMe boot: with the Jetson in recovery mode, NVIDIA's `tools/backup_restore/l4t_backup_restore.sh -b` from the Jetson Linux 36.4 host package.

Write the image's date, checksum and the commit from `/var/lib/argus/setup-checklist.txt` in the team drive beside it. The image is tens of gigabytes and holds the Jetson's keys: it never goes in this repo.

## Tests

`platform/jetson/test/test_setup.py` runs the script against a fake Jetson: a temporary root (`ARGUS_SYSROOT`) and stand-in system commands (`ARGUS_SETUP_BIN`) that record what they were asked. With `ARGUS_SYSROOT` set the script refuses to run without the stand-ins.
