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

## Boot services and the shared clock (PLT-02)

`services.sh` is the second script, with the same `--check` and `--apply`:

```
sudo platform/jetson/services.sh --apply --ground-ip <the laptop's address>          # on the Jetson
sudo platform/jetson/services.sh --apply --role ground --allow <network>/<bits>      # on the ground laptop
python3 platform/jetson/clock_sync.py --source <the laptop's address>                # CLOCK OK / CLOCK NOT OK
```

| Step | State | Where |
| --- | --- | --- |
| `boot_env` | what starts at boot | `/etc/argus/boot.env`, written once and never rewritten |
| `mavros_service` | `argus-mavros.service` enabled | waits for `/dev/argus/pixhawk`, then starts MAVROS |
| `camera_service` | `argus-camera.service` enabled | starts `argus_camera` when `boot.env` names a sensor mode |
| `chrony` | the Jetson follows the ground laptop's clock | `/etc/chrony/conf.d/argus.conf` |

What ships in `boot.env` is the 2026 start-up: `ARGUS_BOOT_MAVROS=apm` (MAVROS's own `apm.launch`, on the fixed device name) and `ARGUS_BOOT_CAMERA=off` (the 2026 launch files start their own grabber, and two readers cannot share the camera). For the 2027 stack set `ARGUS_BOOT_MAVROS=argus` and `ARGUS_BOOT_CAMERA=<mode>`, then `sudo systemctl restart argus-mavros argus-camera`. With MAVROS started at boot, do not also run `mavros_boot.sh`.

No unit starts a mission: missions start only through the GO launcher. The units run `boot/argus-boot.sh`, which knows `mavros` and `camera` and nothing else.

The clock: the laptop serves its own clock (`local stratum 8`, so it works with no internet) and the Jetson follows it. The Jetson's clock may be stepped only in chrony's first three updates after it starts; after that it is slewed, so time does not jump under a running mission. Bring the link up and see `CLOCK OK` before arming, not after. `clock_sync.py` passes within 20 ms of the laptop by default (`--max-offset-ms`).

The ground laptop needs chrony installed first (`sudo apt-get install chrony`; it replaces systemd-timesyncd), and its `/etc/chrony/chrony.conf` must read `/etc/chrony/conf.d` (Ubuntu's does: `confdir /etc/chrony/conf.d`).

On the bench, each with nothing typed after the reboot:

1. Reboot with the flight controller plugged in: `systemctl status argus-mavros` is active and `ros2 topic echo /mavros/state --once` says `connected: true`.
2. Unplug the flight controller, wait ten seconds, plug it in again: `/mavros/state` says connected again within a minute. If it does not, MAVROS kept running without its port and systemd saw no failure to restart on (`Restart=on-failure`): write that down for the lead; the remedy is to bind the unit to the device (`BindsTo=dev-argus-pixhawk.device` with a `SYSTEMD_WANTS` udev rule), which needs the real hardware to try.
3. Set `ARGUS_BOOT_CAMERA=imx519_720p60` in `/etc/argus/boot.env` and reboot: `systemctl status argus-camera` is active and `ros2 topic hz /argus/camera/down/image_raw` shows frames. Set it back to `off` before flying the 2026 launch files.
4. After a reboot, `pgrep -af mission` finds nothing: no mission or executor is running.
5. `python3 platform/jetson/clock_sync.py --source <the laptop's address>` over the link the field will use says `CLOCK OK`; write the offset in `docs/results/`.

## Tests

`platform/jetson/test/test_setup.py` and `test_services.py` run the scripts against a fake Jetson: a temporary root (`ARGUS_SYSROOT`) and stand-in system commands (`ARGUS_SETUP_BIN`) that record what they were asked. With `ARGUS_SYSROOT` set a script refuses to run without the stand-ins.
