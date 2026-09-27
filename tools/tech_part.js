// ---------------------------------------------------------------- technical note (Questions 2-4)
const FT = process.env.FT || "/Users/xiaogui/Desktop/2026fall/ENGR6010/firsttest";

function pngSize(p) {
  const b = fs.readFileSync(p);
  return { w: b.readUInt32BE(16), h: b.readUInt32BE(20) };
}
function fig(rel, caption, widthIn = 6.5) {
  const p = path.join(FT, rel);
  const { w, h } = pngSize(p);
  const W = Math.round(widthIn * 96);
  const H = Math.round(W * h / w);
  return [
    new Paragraph({ alignment: AlignmentType.CENTER, keepNext: true, spacing: { before: 80, after: 40 },
      children: [new ImageRun({ type: "png", data: fs.readFileSync(p), transformation: { width: W, height: H } })] }),
    new Paragraph({ alignment: AlignmentType.JUSTIFIED, spacing: { after: 140 },
      children: runs(caption, { size: SMALL, italics: true }) }),
  ];
}
function tbl(caption, head, rowsIn, widths) {
  const tot = widths.reduce((a, b) => a + b, 0);
  const brd = { style: BorderStyle.SINGLE, size: 4, color: "808080" };
  const bs = { top: brd, bottom: brd, left: brd, right: brd };
  const nRows = rowsIn.length + 1;
  // keepNext on every row but the last keeps the whole table on one page
  const mk = (t, i, h, ri) => new TableCell({
    borders: bs, width: { size: widths[i], type: WidthType.DXA },
    shading: h ? { fill: "E7E6E6", type: ShadingType.CLEAR, color: "auto" } : undefined,
    margins: { top: 30, bottom: 30, left: 70, right: 70 },
    children: [new Paragraph({ spacing: { after: 0 }, keepNext: ri < nRows - 1, keepLines: true,
      children: runs(String(t), { size: SMALL, bold: h }) })],
  });
  return [
    new Paragraph({ keepNext: true, spacing: { before: 100, after: 50 }, alignment: AlignmentType.CENTER,
      children: runs(caption, { size: SMALL }) }),
    new Table({ width: { size: tot, type: WidthType.DXA }, columnWidths: widths,
      rows: [head, ...rowsIn].map((r, ri) => new TableRow({ cantSplit: true,
        children: r.map((t, i) => mk(t, i, ri === 0, ri)) })) }),
    new Paragraph({ spacing: { after: 60 }, children: [] }),
  ];
}
const B = (t) => new Paragraph({ children: runs(t, { size: BODY }), alignment: AlignmentType.JUSTIFIED,
  spacing: { after: 60 }, indent: { left: 360, hanging: 240 } });
const CODE = (t) => new Paragraph({ spacing: { after: 20 }, indent: { left: 360 },
  children: [new TextRun({ text: t, font: "Courier New", size: 18 })] });

const techTitle = [
  new Paragraph({ children: [new PageBreak()] }),
  new Paragraph({ alignment: AlignmentType.CENTER, spacing: { after: 100 },
    children: [new TextRun({ text: "Technical Note: Getting Ready to Work with the Unitree Go2", bold: true, size: 32, font: FONT })] }),
  new Paragraph({ alignment: AlignmentType.CENTER, spacing: { after: 0 },
    children: [new TextRun({ text: "Harry He", size: BODY, font: FONT })] }),
  new Paragraph({ alignment: AlignmentType.CENTER, spacing: { after: 160 },
    children: [new TextRun({ text: "ENGR 6010 AI in Robotics, First Test, Questions 2 to 4    Instructor: Dr. Hongbo Zhang", size: 20, font: FONT, italics: true })] }),
];

const techSummary = [
  H1("Summary"),
  P("I set up an Ubuntu 22.04 virtual machine on my Mac, installed the Unitree SDK, and ran three SDK examples against a simulated Go2. I ran Go2Py, one of the five demo projects, with its trained walking controller. I built a small test field in the simulator, recorded 170 camera frames with exact labels and depth, and ran the five methods from Question 1 on them. Finally I ran a Go2 walking policy from unitree_rl_gym in MuJoCo."),
  P("Along the way I found four problems and fixed them. Go2Py's front camera looks out of the dog's side. The simulator's default time step makes the standing dog slide away. A sim-to-sim script makes the dog fall when it turns. OpenCV's Farneback flow returns almost zero on weak texture. Each one is described below with the measurement that showed it. Table 1 shows what is finished and what still needs the real robot in the lab."),
  ...tbl("**Table 1.** What I did for each question.",
    ["Question", "What ran", "Main result", "Still needs the lab"],
    [["Q2  Linux and SDK", "Ubuntu 22.04 VM in UTM, CycloneDDS 0.10.5, unitree_sdk2py 1.0.1, three examples on unitree_mujoco", "the simulated dog stands up (body 0.117 m to 0.384 m)", "sport-mode examples on the real dog"],
     ["Q3  Dog apps", "Go2Py MuJoCo simulator with its walk-these-ways RL controller", "follows forward, turn, sideways and backward commands within 0.01 m/s", "the four ROS 2 / WebRTC projects"],
     ["Q4  Data and simulation", "170 synthetic camera frames, the five methods, unitree_rl_gym Go2 policy in MuJoCo", "Sections 3.2 to 3.4", "real camera data (script ready)"]],
    [1500, 3100, 2800, 1960]),
];

const q2 = [
  H1("1  Question 2: Getting the Dog Environment Ready"),
  H2("1.1  Linux platform"),
  P("The course guide recommends Ubuntu 20.04 or 22.04, and a virtual machine for a Mac. My Mac has an Apple M4 Pro chip, so I used UTM with the ARM64 version of Ubuntu 22.04. Instead of clicking through the installer, I used the Ubuntu cloud image and a small cloud-init file. On first boot it creates my user, adds my SSH key and installs the build tools. The VM has 4 cores, 8 GB of memory and a 32 GB disk. The whole setup is one script (q2_sdk/setup_vm.sh), so it can be repeated."),
  P("It took three tries to boot correctly. Both problems were in how the configuration disk reached Ubuntu."),
  B("1.  UTM created the virtual CD drive but did not insert my configuration image, so Ubuntu started with default settings and my user did not exist."),
  B("2.  My image, made with hdiutil -hfs, started with an Apple partition map. Linux read it as a partitioned disk and never saw the “cidata” label. A plain ISO 9660 image attached as a second disk worked."),
  H2("1.2  Unitree SDK"),
  P("unitree_sdk2_python talks to the robot through CycloneDDS 0.10. There is no ready-made CycloneDDS package for Linux on ARM, so I built the C library from source (0.10.5) and pointed the Python package to it with CYCLONEDDS_HOME. Then pip3 install -e . failed with “can't create or remove files in install directory”. Ubuntu 22.04 ships setuptools 59, which falls back to an old install method that writes into /usr/local. Upgrading pip and setuptools first and installing with --user fixed it."),
  H2("1.3  Three examples"),
  P("Without the robot I used unitree_mujoco, Unitree's own simulator. It sends and receives the same DDS messages as the real Go2, so the SDK examples run against it with small changes."),
  B("1.  helloworld publisher and subscriber. 12 messages were sent and 12 were received with the same time stamps, so DDS works inside the VM."),
  B("2.  go2_stand_example.py. I changed three things and left the control code alone. It uses the simulator's DDS domain 1 on the loopback interface. SportClient and MotionSwitcher are removed, because these services run on the robot and do not exist in the simulator. The “press Enter” line is removed too. The dog stood up from 0.117 m to 0.384 m and lowered again, as the example intends (Figure 1)."),
  B("3.  wireless_controller.py. I changed it to the Go2 message type and the simulator's topic. It printed 1,933 remote-controller states read from rt/lowstate."),
  P("A separate recorder saved 9,932 rt/lowstate messages at about 495 Hz during the run. During the stand the body moved less than 2 cm sideways or forward."),
  ...fig("q2_sdk/results/q2_stand_example.png", "**Figure 1.** The stand example in the simulator. Top: screenshots from the VM. Bottom: body height and three of the twelve motor angles, read through the SDK while the example ran."),
  H2("1.4  A problem I found: the standing dog slid away"),
  P("The first time, the dog stood up and then disappeared from the screen. The recorded position showed it sliding backwards at about 1 m/s while every joint target was fixed. I repeated the example's joint targets directly in MuJoCo, with no DDS and no threads, and changed only the physics time step (Figure 2). With the simulator's default step of 5 ms the body moved 3.01 m in 6 s. With 2 ms or 1 ms it moved 0.01 m. The stand example uses stiff gains (kp = 60, kd = 5), and a 5 ms contact step is too coarse for them. The real robot's control period is 2 ms, so I set the simulator to 2 ms."),
  ...fig("q2_sdk/results/q2_timestep.png", "**Figure 2.** The same joint targets with three physics time steps. Only the 5 ms step, the simulator's default, makes the body slide.", 5.2),
];

const q3 = [
  H1("2  Question 3: Getting Familiar with the Dog Apps"),
  H2("2.1  Review of the five demos"),
  P("I read each project to see what it does and what it needs (Table 2). Four of the five need the real robot or an NVIDIA GPU. They also show where the course is going. go2_robot and the SLAM and Nav2 demo are the mapping and navigation parts of the building-delivery robot I proposed in my earlier Unitree assignment."),
  ...tbl("**Table 2.** The five demo projects.",
    ["Project", "What it does", "What it needs", "Ran here"],
    [["go2_robot", "ROS 2 driver: robot model, odometry, point cloud, joint states, rviz, velocity commands", "real Go2, ROS 2 Humble", "no"],
     ["go2-webrtc", "web page that connects to the Go2 over WebRTC, sends commands and shows its video", "real Go2 on the same network and its login token", "no"],
     ["unitree-go2-slam-nav2", "SLAM mapping and Nav2 autonomous navigation demos", "real Go2 with LiDAR and a depth camera, ROS 2", "no"],
     ["Go2-Dynamic-Inspection", "“Go2 Planner Suite”: CUDA-accelerated navigation with a Gazebo simulation", "ROS 2 Humble and an NVIDIA GPU", "no"],
     ["Go2Py", "Python interface for the Go2, MuJoCo simulator and trained RL walking controllers", "nothing extra for simulation", "yes"]],
    [1900, 3800, 2660, 1000]),
  H2("2.2  Running Go2Py"),
  P("Two things stopped Go2Py on my Mac. I fixed both without editing its files. Its walk-these-ways checkpoint was saved with GPU tensors, which cannot load without CUDA, so a small module maps them to the CPU before Go2Py is imported. Its “front_camera” points out of the dog's left side, not forward. The demo does not need the camera, and I turned it forward for Question 4."),
  P("The trained controller followed every command closely (Figure 3). Commanded at 0.6 m/s forward it walked at 0.60 m/s, and it also matched 0.8 rad/s turning, 0.3 m/s sideways and 0.3 m/s backward within 0.01. Its body never dropped below 0.23 m. The same run can be watched live with mjpython demo_walk.py --view."),
  ...fig("q3_go2py/results/q3_go2py_walk.png", "**Figure 3.** Go2Py's walk-these-ways controller in MuJoCo. Top: the tracking camera. Bottom: commanded and actual speeds, and the path seen from above.", 6.0),
];

const q4 = [
  H1("3  Question 4: Dog Data and Simulation"),
  H2("3.1  How I plan to collect real data"),
  P("The suggested repository, unitree_go2_create_dataset, records the real robot's joint states and foot forces from the lf/lowstate topic into a ROS 2 bag. It does not record images, and it does not make synthetic data. “synthetic” is only the name of one of its topics. So I wrote my own collector, lab/collect_real.py. It only listens and never sends a command. It saves front camera frames through VideoClient, the 12 joint angles and speeds, foot forces, IMU and battery from rt/lowstate, and the robot's own velocity estimate. This is the same kind of record my simulator makes, so the same analysis can run on real frames. The lab steps are in lab/LAB_CHECKLIST.md."),
  H2("3.2  Synthetic data"),
  P("Go2Py's default scene is an empty dark floor, so I built a test field on top of its Go2 model (Figure 4). It has soil with texture, sunlight, ten weeds and five objects chosen to test the methods. The red box has strong color. The gray box has almost none, to test hue. A box the color of the soil was meant to fool GrabCut. The dog walked a fixed route with Go2Py's controller: forward, a fast turn left and back, a sidestep, and forward again. Every 0.1 s the front camera saved an image, the true label of every pixel and the depth, 170 frames in total."),
  P("The objects do not move and the camera pose is known exactly. So the true optical flow of every pixel can be computed from the depth and the two camera poses. My earlier assignments never had this ground truth. As a check, the DIS flow method agrees with it to about 2 pixels on slow motion."),
  ...fig("q4_sim/results/q4_data.png", "**Figure 4.** The synthetic data at four moments of the run. Every image comes with the true object under each pixel and its depth."),
  H2("3.3  The five methods on the synthetic data"),
  P("**Color space.** I scored each channel alone on how well it separates weed pixels from soil pixels, using the AUC (1.0 is perfect). Lab a, Cr and hue reached 1.000. The green channel G reached only 0.741, because brown soil also has a lot of green. Lightness L was 0.544, almost useless. When I moved every pixel by one RGB unit, the hue of the red box changed by 1.1 degrees and the hue of the gray box by 27.2 degrees. This is the same warning as in the color review, and the same result as my Reading 2."),
  P("**Gaussian pyramid.** I tracked 20,766 corners with pyramidal Lucas-Kanade and compared each result with the true flow (Table 3). Without a pyramid, the share tracked within 1 pixel fell from 62% for motion under 2 pixels to 7% at 20 to 40 pixels. Above 40 pixels, which is the fast turn, it was 0%. With three extra levels it stayed at 60 to 67% up to 20 pixels and still reached 30% above 40 pixels. It is my Homework 1 result, an error of 1.77 pixels for a 5-pixel shift, seen again on camera images."),
  ...tbl("**Table 3.** Share of corners tracked to within 1 pixel of the true position, by true motion between frames.",
    ["Pyramid levels", "0-2 px", "2-5 px", "5-10 px", "10-20 px", "20-40 px", "40-90 px"],
    [["none", "62%", "55%", "43%", "28%", "7%", "0%"],
     ["1 extra", "61%", "61%", "60%", "59%", "26%", "8%"],
     ["2 extra", "61%", "60%", "61%", "66%", "45%", "18%"],
     ["3 extra", "61%", "60%", "61%", "67%", "49%", "30%"],
     ["corners", "689", "4,303", "5,011", "3,878", "2,514", "4,001"]],
    [1860, 1250, 1250, 1250, 1250, 1250, 1250]),
  P("**Optical flow.** DIS had a mean error of 2.3 pixels when walking forward, 12.9 pixels in the fast turn and 2.0 pixels in the sidestep (Table 4, Figure 5). Farneback failed almost completely, with errors close to the size of the motion itself. To find out why, I moved one frame by a known (3, 2) pixels. On the soil Farneback returned (0.01, 0.01), while DIS returned (3.01, 2.00). The OpenCV source explains it. Each Farneback update divides by the determinant of a 2 by 2 matrix plus 0.001 (optflowgf.cpp: idet = 1/(g11 g22 − g12^{2} + 1e-3)). The simulated soil changes by only about half a gray level per pixel, so the determinant is tiny and the 0.001 pushes every update toward zero. When I multiplied the soil's contrast by 2, 4 and 8, the estimate of the 3-pixel shift grew from 0.16 to 2.18 to 2.94 pixels. It is the same matrix as A^{T}A in Homework 1."),
  ...tbl("**Table 4.** Mean end-point error of dense optical flow against the true flow, in pixels.",
    ["Motion", "True motion (median)", "Farneback", "Farneback + CLAHE", "DIS"],
    [["forward", "6.8", "9.30", "8.33", "2.31"],
     ["fast turn", "58.4", "53.73", "54.23", "12.90"],
     ["sidestep", "14.1", "12.82", "11.94", "1.96"],
     ["time per frame pair", "", "29 ms", "30 ms", "12 ms"]],
    [2100, 2100, 1700, 1860, 1600]),
  ...fig("q4_sim/results/q4_flow.png", "**Figure 5.** True flow and two estimates for a forward step (top) and the fast turn (bottom). Hue is direction and brightness is speed. Farneback's output is almost black on the soil, which means almost no motion."),
  P("**GrabCut.** I gave GrabCut a rectangle 15% larger than each object (Figure 6). The IoU was 0.98 for the blue cylinder, 0.97 for the soil box and 0.96 for the red box. It was 0.92 for the ball, 0.88 for the gray box and 0.67 for the weed. The soil box did not fool it. Its sides were in shade, which gave a contrast-to-noise ratio of 3.6, above the limit of about 2.2 I measured in Reading 2. All six objects were above that limit. GrabCut never missed a pixel of any object. Every error was extra pixels, and they were shadows. The extra soil under the weed had a brightness of 86, while the rest of the soil in its rectangle had 136. The shadow is inside the rectangle but not outside it, so the background model never learned it. That is the third failure case in Section 5 of the GrabCut paper."),
  ...fig("q4_sim/results/q4_grabcut.png", "**Figure 6.** GrabCut on each object. Top: the image inside the rectangle. Bottom: GrabCut's result, with the background darkened and the true outline in yellow. The extra pixels are the shadows."),
  P("**Supervised learning.** I trained six classifiers from Kotsiantis's review to label weed pixels (Table 5). They used the 25 pixel features from Reading 3 and the simulator's exact labels. Training used 88 frames and testing used 60 later frames seen from new viewpoints. All six reached an F1 between 0.989 and 0.999. Naive Bayes was lowest. k-nearest neighbors needed about 3 s per image, against 9 ms for the decision tree. Both match the accuracy and speed stars in the review's Table 4. At its strongest penalty the lasso kept only three channels, ExG, R and Lab a, and still reached F1 0.986. Lab a was kept at every penalty, as in Reading 3. On real photos with weak labels, my best pixel F1 in Reading 3 was 0.786. The gap shows how much easier clean synthetic images with exact labels are."),
  ...tbl("**Table 5.** Weed-pixel classifiers on 36,000 test pixels from later frames.",
    ["Classifier", "Family (Kotsiantis)", "F1", "Time per 640x480 image"],
    [["Lasso logistic regression", "statistical / linear", "0.998", "13 ms"],
     ["Decision tree", "logic-based", "0.999", "9 ms"],
     ["Naive Bayes", "statistical", "0.989", "46 ms"],
     ["k-nearest neighbors", "instance-based", "0.999", "3,064 ms"],
     ["Linear SVM", "support vector machine", "0.999", "20 ms"],
     ["Neural network (MLP)", "perceptron-based", "0.999", "56 ms"]],
    [2800, 2600, 1100, 2860]),
  H2("3.4  Dog simulation with unitree_rl_gym"),
  P("Training in unitree_rl_gym needs Isaac Gym, which runs only on Linux with an NVIDIA GPU. The official repository also has no Go2 policy, only humanoids. A pull request (#62, fork UESTC-REVERIE/unitree_rl_gym_go2) adds a Go2 policy trained in Isaac Gym and deploy_go2.py, which runs it in MuJoCo. Running a trained policy does not need Isaac Gym, so it works on my Mac."),
  P("Reading deploy_go2.py, I noticed that it gives the policy the body's linear velocity in world coordinates, while Isaac Gym trains with the velocity in the robot's own body frame. The two are the same only while the robot faces its starting direction. I ran the same commands twice, once with the original code and once with the velocity turned into the body frame (Table 6, Figure 7). Both stood and walked forward with an error of about 0.04 m/s. When the command asked the dog to turn while walking, the original version's turning error rose to 1.0 rad/s. The dog fell to 0.133 m and never got up. With the body-frame velocity the errors stayed between 0.05 and 0.07, and the body never went below 0.27 m."),
  ...tbl("**Table 6.** RMS tracking error of the unitree_rl_gym Go2 policy, 1 s after each new command.",
    ["Phase", "Velocity given in", "Forward (m/s)", "Sideways (m/s)", "Turn (rad/s)", "Lowest body (m)"],
    [["walk forward 0.5 m/s", "world frame (original)", "0.038", "0.054", "0.047", "0.308"],
     ["walk forward 0.5 m/s", "body frame (fixed)", "0.041", "0.042", "0.043", "0.311"],
     ["walk 0.5 m/s + turn 0.6 rad/s", "world frame (original)", "0.543", "0.502", "1.003", "0.133 (fell)"],
     ["walk 0.5 m/s + turn 0.6 rad/s", "body frame (fixed)", "0.048", "0.057", "0.062", "0.309"]],
    [2500, 2060, 1200, 1200, 1100, 1300]),
  ...fig("q4_sim/results/q4_rl_gym_go2.png", "**Figure 7.** The same commands with the velocity given in the world frame (red, as in deploy_go2.py) and in the body frame (blue). The red run falls soon after the turn starts."),
  H2("3.5  What I learned"),
  P("Most of my time went into problems that no paper mentions. A camera pointing the wrong way, a time step that is too coarse and a velocity in the wrong frame all looked like the method failing. In each case one measurement against a known answer found the real cause. Simulation made that possible, because it gives exact labels, depth and poses. It also showed its limits. The synthetic soil was too smooth for Farneback, and the weeds were too easy for every classifier. The next step is to collect real frames with lab/collect_real.py and run the same analysis on them."),
];

const techRefsList = [
  "Unitree Robotics, “unitree_sdk2_python,” GitHub. https://github.com/unitreerobotics/unitree_sdk2_python",
  "Unitree Robotics, “unitree_mujoco,” GitHub. https://github.com/unitreerobotics/unitree_mujoco",
  "Eclipse Foundation, “Cyclone DDS,” releases/0.10.x. https://github.com/eclipse-cyclonedds/cyclonedds",
  "R. Khorrambakht and B. Dai, “Go2Py,” GitHub. https://github.com/machines-in-motion/Go2Py",
  "Unitree Robotics, “unitree_rl_gym,” GitHub, and pull request #62 (fork UESTC-REVERIE/unitree_rl_gym_go2). https://github.com/unitreerobotics/unitree_rl_gym",
  "S. Zhao, “unitree_go2_create_dataset,” GitHub. https://github.com/ShijieZhao36/unitree_go2_create_dataset",
  "OpenCV, “optflowgf.cpp,” Farneback optical flow source, 4.x branch. https://github.com/opencv/opencv/blob/4.x/modules/video/src/optflowgf.cpp",
  "Demo projects reviewed in Table 2: github.com/Unitree-Go2-Robot/go2_robot, github.com/tfoldi/go2-webrtc, github.com/h-naderi/unitree-go2-slam-nav2, github.com/YasiruDEX/Go2-Dynamic-Inspection.",
];
const techRefs = [
  H1("References"),
  P("The papers behind the five methods are listed in the review paper (Question 1). The software used here:", { after: 60 }),
  ...techRefsList.map((r, i) => new Paragraph({
    children: [new TextRun({ text: `[${i + 1}]\t${r}`, font: FONT, size: SMALL })],
    tabStops: [{ type: TabStopType.LEFT, position: 400 }],
    indent: { left: 400, hanging: 400 },
    spacing: { after: 30 },
  })),
];

// ---------------------------------------------------------------- document
const doc = new Document({
  creator: "Harry He",
  title: "ENGR 6010 First Test - Harry He",
  styles: { default: { document: { run: { font: FONT, size: BODY } } } },
  sections: [{
    properties: { page: { size: { width: 12240, height: 15840 },
      margin: { top: 1440, bottom: 1440, left: 1440, right: 1440, header: 720, footer: 720 } } },
    headers: { default: new Header({ children: [new Paragraph({ alignment: AlignmentType.RIGHT,
      children: [new TextRun({ text: "ENGR 6010 First Test – Harry He", size: 16, font: FONT, color: "666666" })] })] }) },
    footers: { default: new Footer({ children: [new Paragraph({ alignment: AlignmentType.CENTER,
      children: [new TextRun({ children: [PageNumber.CURRENT], size: 18, font: FONT })] })] }) },
    children: [...title, ...abstract, ...intro, ...review, ...compare, ...conclusion, ...references,
               ...techTitle, ...techSummary, ...q2, ...q3, ...q4, ...techRefs],
  }],
});
Packer.toBuffer(doc).then((buf) => { fs.writeFileSync(OUT, buf); console.log("wrote", OUT); });
