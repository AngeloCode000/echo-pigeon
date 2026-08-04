# Setup Guide

Clone-to-running instructions for the echo-pigeon radar tracking pipeline, covering native Ubuntu 22.04 and Windows 11 + WSL2. The simulation pipeline (Phase 0) needs no hardware at all; the hardware sections apply once a TI IWR6843ISK-ODS board is on hand.

## 1. Prerequisites

| Requirement | Version | Notes |
|---|---|---|
| Ubuntu | 22.04 (Jammy) | Native or inside WSL2 |
| ROS 2 | Humble | The only distro this workspace is tested against |
| Python | 3.10 | Ships with Ubuntu 22.04 |
| numpy | **1.x** (apt: 1.21) | See the numpy warning below |
| pyserial | 3.5 | Hardware phase only |
| Hardware | TI IWR6843ISK-ODS | Phase 1+ only — the **-ODS** variant, not the standard ISK |

> **numpy warning:** install numpy from apt (`python3-numpy`), not pip. ROS 2 Humble's binary packages are built against numpy 1.x; a pip-installed numpy 2.x will break message serialization with cryptic `_ARRAY_API` import errors. If you must use pip, pin `numpy<2`.

## 2. Common install (native Ubuntu and WSL2)

Install ROS 2 Humble if you don't have it ([official instructions](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)):

```bash
sudo apt update && sudo apt install -y software-properties-common curl
sudo add-apt-repository universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install -y ros-humble-desktop
```

Install the project dependencies, clone, and build:

```bash
sudo apt install -y python3-numpy python3-serial python3-pytest \
    python3-colcon-common-extensions ros-humble-rviz2

git clone <your-fork-url> echo-pigeon
cd echo-pigeon/ros2_ws

source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Add the two `source` lines to `~/.bashrc` so every new shell is ready:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source ~/echo-pigeon/ros2_ws/install/setup.bash" >> ~/.bashrc
```

Run the unit tests to confirm the toolchain:

```bash
cd ~/echo-pigeon/ros2_ws
python3 -m pytest \
    src/target_tracker/test/test_coordinates.py \
    src/target_tracker/test/test_ekf.py \
    src/target_tracker/test/test_track_manager.py \
    src/radar_simulator/test/test_trajectories.py \
    src/radar_simulator/test/test_measurement_model.py \
    src/radar_preprocessor/test/test_filters.py \
    src/radar_preprocessor/test/test_clustering.py \
    src/data_logger/test/test_csv_writer.py \
    src/ti_radar_driver/test/test_tlv_parser.py \
    src/ti_radar_driver/test/test_cfg_loader.py -q
```

All tests are pure Python (no ROS runtime needed) and should pass in under a second.

## 3. Run the simulation (Phase 0 — no hardware)

```bash
ros2 launch radar_bringup sim.launch.py
```

What you should see in RViz:

- **Red cube** — ground-truth drone position flying a figure-eight ~8 m in front of the radar origin.
- **Blue points** — noisy filtered detections, occasionally dropping out or showing clutter blips.
- **Green sphere + arrow + label** — the confirmed track following the truth, with its ID, confidence, and a translucent 2-sigma covariance ellipsoid.
- **Yellow spheres** — short-lived tentative tracks spawned by clutter; they never confirm and vanish quickly.

Useful variations:

```bash
ros2 launch radar_bringup sim.launch.py trajectory:=circle
ros2 launch radar_bringup sim.launch.py rviz:=false   # headless
```

All tuning knobs live in `ros2_ws/src/radar_bringup/config/sim_params.yaml` — trajectory geometry, noise sigmas, detection drop probability, clutter rate, tracker gating and confirmation thresholds. Note the simulator's noise sigmas and the tracker's measurement sigmas describe the same physical quantities: change them together.

## 4. WSL2-specific setup

### 4.1 RViz under WSLg

Windows 11's WSLg runs GUI apps out of the box — `rviz2` should just open a window. If it crashes or renders black, force software rendering:

```bash
export LIBGL_ALWAYS_SOFTWARE=1
ros2 launch radar_bringup sim.launch.py
```

### 4.2 USB serial passthrough with usbipd-win (hardware phase)

WSL2 has no native USB. The radar's XDS110 USB interface must be forwarded with [usbipd-win](https://github.com/dorssel/usbipd-win).

In **Windows PowerShell (administrator)**, one time:

```powershell
winget install usbipd
usbipd list                          # find the XDS110 BUSID, e.g. 2-3
usbipd bind --busid 2-3              # one-time, persists across reboots
```

Each session (plain PowerShell is fine after the bind):

```powershell
usbipd attach --wsl --busid 2-3
```

Then verify inside WSL:

```bash
ls /dev/ttyACM*        # expect /dev/ttyACM0 and /dev/ttyACM1
dmesg | tail           # should show cdc_acm registering both ports
```

**Gotchas:**

- The attachment is lost whenever the board re-enumerates — reset button, reflash, or power cycle. Re-run `usbipd attach` afterward. The driver node reconnects by itself once the ports reappear.
- Flash firmware from Windows (UniFlash is a Windows GUI anyway); flashing through usbipd is flaky.
- usbipd adds a little latency but has no trouble with the 921600-baud data stream.

## 5. Serial permissions (both platforms)

```bash
sudo usermod -aG dialout $USER
```

Log out and back in (or `wsl --shutdown` from Windows and reopen). Without this you'll get `Permission denied: '/dev/ttyACM0'`.

Optional — stable device names via udev rule (`/etc/udev/rules.d/99-ti-radar.rules`):

```text
SUBSYSTEM=="tty", ATTRS{idVendor}=="0451", ATTRS{idProduct}=="bef3", SYMLINK+="ti_radar_%n"
```

## 6. Flashing the radar (one-time, when hardware arrives)

1. Install [TI UniFlash](https://www.ti.com/tool/UNIFLASH) and download the **mmWave SDK 3.x out-of-box demo** binary for the IWR6843 (`xwr68xx_mmw_demo.bin`), which ships with the [mmWave SDK](https://www.ti.com/tool/MMWAVE-SDK).
2. Set the SOP jumpers to **flashing mode** (SOP0=1, SOP1=0, SOP2=1 on the ISK-ODS — check the silkscreen), connect USB, press reset.
3. In UniFlash pick the XDS110 COM port (the *application/user* port), load `xwr68xx_mmw_demo.bin`, and flash.
4. Set SOP back to **functional mode** (SOP0=1, SOP1=0, SOP2=0), press reset.

Do this from Windows if you're on WSL2, then attach the device with usbipd.

> **Antenna variant matters:** the shipped chirp config is for the **ODS** antenna layout. The demo binary is the same for ISK and ISK-ODS, but a non-ODS config on an ODS board produces garbage angles.

## 7. Hardware launch (Phase 1)

```bash
ros2 launch radar_bringup hardware.launch.py
# or with explicit ports / custom chirp config:
ros2 launch radar_bringup hardware.launch.py \
    cli_port:=/dev/ttyACM0 data_port:=/dev/ttyACM1 \
    cfg_file:=/absolute/path/to/custom.cfg
```

The driver sends `config/iwr6843_ods_default.cfg` (10 fps, ~9 m max range, SNR side-info enabled) over the CLI UART, starts the sensor, and streams parsed detections into the same pipeline the simulator used. A reference copy of the config lives at the repo root under `config/`; the copy that is actually loaded is installed with the `ti_radar_driver` package.

First-contact checklist (per `docs/test_plan.md` — do these in order, not with a drone first):

1. Launch with just the room in view — expect sparse detections after the static-clutter filter.
2. **Walk in front of the radar at 2–5 m** — you should see blue detection points on yourself in RViz and a green confirmed track following you within a second.
3. Moving metal plate, then bicycle, then hand-carried drone (see the test plan's progression).
4. For **hover** tests, set `enable_static_clutter_filter: false` in `hardware_params.yaml` — a stationary hovering drone has near-zero radial velocity and would be filtered out.

Troubleshooting is in section 9.

## 8. Data logging and replay

Every launch writes CSVs to `~/echo_pigeon_logs/run_YYYYmmDD_HHMMSS/`:

- `detections.csv` — one row per raw detection point (spherical coordinates + SNR).
- `tracks.csv` — one row per track update (position, velocity, age, hit/miss counts, confidence, covariance diagonals).

For full-fidelity capture and hardware-free replay, record a bag alongside:

```bash
ros2 bag record /radar/detections /tracks
# later, without radar or simulator — the preprocessor, tracker, and RViz
# run against the recording:
ros2 launch radar_bringup hardware.launch.py rviz:=true &   # driver will just retry
ros2 bag play <bag_dir>
```

(Or start the nodes individually; any `radar/detections` publisher feeds the pipeline.)

## 9. Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| `No such file or directory: '/dev/ttyACM0'` | Device not attached (WSL: usbipd detached after reset) | `usbipd attach --wsl --busid <id>`; check `ls /dev/ttyACM*` |
| `Permission denied: '/dev/ttyACM0'` | Not in `dialout` group | Section 5, then re-login |
| Driver logs `radar rejected '...': Error ...` | Wrong firmware or cfg/firmware mismatch | Reflash the out-of-box demo; confirm SDK 3.x; try TI's demo visualizer to sanity-check the board |
| Driver connects but publishes nothing | Wrong port order (CLI vs data swapped) | Swap `cli_port`/`data_port` launch args |
| No detections on a person walking | Range/SNR thresholds too tight | Lower `min_snr_db`, raise `max_range_m` in `hardware_params.yaml` |
| Hovering drone invisible | Static-clutter (zero-doppler) filter | Set `enable_static_clutter_filter: false` |
| RViz blank / crashes in WSL | GPU passthrough issue | `export LIBGL_ALWAYS_SOFTWARE=1` |
| `ImportError ... _ARRAY_API` or numpy crash | pip numpy 2.x shadowing apt numpy 1.x | `pip uninstall numpy`; use `python3-numpy` from apt |
| Nodes start but no `/tracks` | Forgot to `source install/setup.bash` in this shell | Source it, or add to `~/.bashrc` |
