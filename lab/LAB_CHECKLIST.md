# Lab checklist: connecting the real Unitree Go2

The cable route was tested against the simulator and against fake robot services that answer like the dog,
straight from the Mac and through the Ubuntu VM. The Wi-Fi route was only tested without a dog (it gives clear
messages when the dog cannot be reached); its library is the one many Go2 owners use, but it has never talked to
this dog. The steps marked **real robot** still need the dog.

## 0. Before you go
- Charge the Mac. Pack a **USB-C to Ethernet adapter** and an **Ethernet cable**.
- Double-click **机器狗控制面板** on the Desktop (the app with the Go2 icon, made by `bash tools/make_panel_app.sh`;
  `ENGR6010/机器狗控制面板.command` is the backup launcher). The first time, macOS asks to let it use the Desktop
  folder, the local network and UTM: click **Allow** each time, at home, before the lab.
  The real dog does **not** need the VM; the Mac talks to it directly with the official Unitree SDK
  (`.venv_robot`, made by `bash lab/setup_mac_sdk.sh`).

## 1. First question in the lab: which Go2 is it?
Look in the Unitree app (Device / About) or ask the instructor for the **model** and **firmware version**.

| Model | **Cable** (official SDK) | **Wi-Fi** (WebRTC, like the phone app) |
|---|---|---|
| **Go2 EDU** | yes (best: the Mac keeps its internet on Wi-Fi) | yes |
| Go2 AIR / PRO | **no** (SDK is EDU-only without custom firmware) | yes |

Pick the connection at the top of card 1 in the panel: **Cable | Wi-Fi | VM (backup)**.

**Firmware 1.1.15 or newer on Wi-Fi needs the dog's AES key.** Get it **at home, with internet**, using the Unitree
account the dog is bound to (ask the instructor; add `--region cn` for a Chinese account):
```
.venv_wifi/bin/unitree-fetch-aes-key --email <account email> --device-type Go2 --sn <dog serial> -q > lab/.go2_aes_key
```
(The password is asked for. `--device-type Go2` matters: the default is G1. On 2026-09-28 someone reported that the
Unitree cloud was blocking these logins; if it fails, ask the instructor.) The file stays on this Mac only.

## 2a. Connect by cable (EDU, about 2 minutes)
1. Plug the adapter into the Mac and the cable into the Go2's Ethernet port. Turn the dog on and wait about 1 minute.
2. Panel **1-1 Connect to the dog**. macOS asks for the **Mac password** once: the script gives the adapter the
   address 192.168.123.222 (Wi-Fi is not touched) and pings the dog at 192.168.123.161.
   You want `ROBOT REACHABLE`. The top right then says `robot ✓ (Mac en7)` (the card name can differ).
3. If macOS asks whether Terminal may find devices on the local network, click **Allow**.
4. No reply? Check the cable, wait for the dog to finish booting, and turn off any VPN (快帆, Tailscale).

**Backup route through the VM** (only if the cable route does not work): choose *VM (backup)* at the top of
card 1, click *Start the VM*, then **1-1** again. It restarts the VM once to give it a network card on the cable
(about 30 s), then sets that card to 192.168.123.223 (not .222, so it never clashes with the Mac's adapter).
All other buttons then run inside Ubuntu.

## 2b. Connect by Wi-Fi (any Go2)
1. **Close the Unitree phone app.** The dog accepts only one Wi-Fi client at a time.
2. In the panel choose **Wi-Fi** at the top of card 1.
3. Click the Wi-Fi icon at the top right of the Mac screen and join the dog's network (its name usually starts with
   Go2; the password comes from the instructor or the dog's label). The Mac gets an address 192.168.12.x.
4. Panel **1-1 Connect to the dog**: it checks the Mac is on the dog's Wi-Fi and the dog answers, and says whether
   the AES key is there and whether the Mac still has internet. You want `DOG REACHABLE over Wi-Fi`.
5. Click **one button at a time** and wait for `done` (each Wi-Fi command first connects, which takes a few seconds).
   STOP first ends the running command, which sends StopMove over its own connection, then sends StopMove again.

**Internet while on the dog's Wi-Fi.** The panel does not need internet; only asking Claude or looking things up does.
To keep internet anyway: connect the **iPhone with a USB cable** and turn on **Personal Hotspot**. This Mac already
lists *iPhone USB* above *Wi-Fi* in its network order, so the internet goes through the phone while Wi-Fi talks to
the dog.

**Other way: the dog on a router.** If the instructor puts the dog on a normal Wi-Fi router (Unitree app, STA mode)
and the Mac joins the same router, write the dog's address on that router into `lab/.go2_wifi_ip`
(for example `echo 192.168.1.23 > lab/.go2_wifi_ip`); the panel then uses it instead of the hotspot. Campus Wi-Fi
usually blocks devices from talking to each other, so this needs a small router or a phone hotspot.
Delete the file to go back to the dog's hotspot.

## 3. Read-only (safe, the robot does not move)
- **1-2 Read robot state**: battery, tilt, body height, sport mode, leg angles, foot forces, about 500 messages/s on
  the cable (fewer on Wi-Fi, which sends the lower-rate copies). Do this first, every time.
- **1-3 Take a photo**: one front-camera image, opens on the Mac, saved in `lab/data/photos/`.
- **1-4 Record data 20 s**: camera frames + state for Question 4, saved in `lab/data/real_run_<time>/`
  (`wifi_run_<time>/` on Wi-Fi, same files and columns).

## 4. Motion (**real robot**, only with the instructor)
- Clear 2 m around the dog. Someone holds the **remote**: **L2 + B** = damping (soft emergency stop).
- Tick **Allow motion** in the panel. Every motion button asks for confirmation.
- Order for a first session: **1-5 Stand up -> 1-6 Balance stand -> 1-7 Walk 0.3 m/s 3 s -> 1-9 Lie down**.
  1-7 and 1-8 check the dog's mode first: standing but not balanced -> they send BalanceStand first (right after
  Stand up the joints are locked and Move does little); lying -> they refuse. They print how far the dog moved.
  Everything the dark box shows is also saved in `lab/data/logs/panel_<date>.log`.
  Click each button once and wait for `done`; do not click a motion button again if it failed.
- **STOP moving** is always available. It first ends any walk command still running, then sends StopMove
  (about 1 s from the Mac by cable; a few seconds on Wi-Fi, where the remote's **L2 + B** is faster).
- Safe way to finish: StopMove -> **Lie down** -> wait 3 s -> **Damp**. Damp on a standing dog makes it drop.
- Never use flips, jumps, hand stand or the low-level stand example on the real dog.
- If sport commands return code 3104 (no answer) or the script says sport mode is released, stop and ask the
  instructor.

## 5. After the lab
- The photos and recordings are on the Mac in `lab/data/`. Run the Question 4 analysis on them.
- The adapter keeps the address 192.168.123.222. To use it on a normal network later:
  System Settings -> Network -> the USB LAN adapter -> Details -> TCP/IP -> Configure IPv4: Using DHCP.

## Command line (what the panel runs)
On the Mac (`en7` = the adapter; see `ifconfig`):
```
bash lab/mac_net_up.sh
.venv_robot/bin/python lab/real_state.py en7 10
.venv_robot/bin/python lab/real_photo.py en7 front.jpg
.venv_robot/bin/python lab/real_sport.py en7 standup        # asks Enter; add --dry-run to test without a robot
.venv_robot/bin/python lab/collect_real.py en7 --secs 20 --fps 5
```
Over Wi-Fi (`ap` = the Mac is on the dog's hotspot; or the dog's IP on a shared router):
```
bash lab/setup_wifi_sdk.sh                                   # once: makes .venv_wifi
bash lab/wifi_check.sh
.venv_wifi/bin/python lab/wifi_state.py ap 10
.venv_wifi/bin/python lab/wifi_photo.py ap front.jpg
.venv_wifi/bin/python lab/wifi_sport.py ap standup           # asks Enter; add --dry-run to test without a robot
.venv_wifi/bin/python lab/wifi_record.py ap --secs 20 --fps 5
```
Inside the VM (backup route, `enp0s2` = the bridged card):
```
bash ~/q2/lab/robot_net_up.sh enp0s2
python3 ~/q2/lab/real_state.py enp0s2 10
python3 ~/q2/lab/real_sport.py enp0s2 standup
```
