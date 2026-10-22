/* Every word the panel shows, in both languages, in one place.
 *
 * The button labels already come from control_panel.ACTIONS, which carries
 * its own en/zh pair for each action. This file covers everything around
 * them - headings, hints, units, placeholders, error text - which used to
 * be written straight into the markup in Chinese, so switching to EN left
 * most of the panel in Chinese.
 *
 * L.t("key") returns the current language's string. {0} {1} are filled from
 * the extra arguments. L.lang is the current choice and survives a reload.
 */
const L = (() => {
  const S = {
    // ---- top bar
    title:        ["Go2 控制面板",            "Go2 control panel"],
    connected:    ["已连上",                  "Connected"],
    noAnswer:     ["网卡 {0},狗没回应",      "Interface {0}, no answer"],
    notConnected: ["未连接",                  "Not connected"],
    telemetry:    ["遥测",                    "Telemetry"],
    on:           ["开",                      "on"],
    cState:       ["状态",                    "state"],
    cCloud:       ["点云",                    "cloud"],
    cFrame:       ["画面",                    "frames"],
    mode:         ["模式",                    "mode"],

    // ---- camera view
    camera:       ["前置摄像头",              "Front camera"],
    nFrames:      ["{0} 帧",                  "{0} frames"],
    waitFrame:    ["等待画面",                "waiting for video"],
    camEmpty:     ["摄像头画面会出现在这里",  "The camera picture appears here"],
    camEmpty2:    ["先连上狗(右边 1-1)",     "connect the dog first (1-1)"],

    // ---- lidar view
    lidar:        ["激光雷达",                "LiDAR"],
    nPoints:      ["{0} 点 · 颜色按高度",     "{0} points · coloured by height"],
    waitCloud:    ["等待点云",                "waiting for a sweep"],
    orbit:        ["拖动旋转 · 滚轮缩放",     "drag to orbit · scroll to zoom"],
    topDown:      ["俯视图 · 从上往下看",     "top-down view"],
    cloudHint:    ["等待点云 — 连上狗并让它站起来,再用 1-2b 确认雷达在发",
                   "Waiting for a sweep - connect the dog, stand it up, "
                   + "then use 1-2b to check the LiDAR is publishing"],
    no3D:         ["这个浏览器关了显卡加速,没有 3D。用俯视图代替。",
                   "This browser has no WebGL (graphics acceleration is off). "
                   + "Showing a top-down view instead."],

    // ---- telemetry
    battery:      ["电池",                    "Battery"],
    volts:        ["电压",                    "Voltage"],
    amps:         ["电流",                    "Current"],
    forward:      ["前进",                    "Forward"],
    height:       ["身高",                    "Body height"],
    heading:      ["朝向",                    "Heading"],
    torques:      ["12 个关节扭矩 (Nm)",      "12 joint torques (Nm)"],
    footForce:    ["4 只脚受力",              "Foot forces"],

    // ---- buttons and log
    running:      ["运行中",                  "running"],
    moves:        ["会动",                    "moves"],
    logEmpty:     ["点右边的按钮,输出会实时出现在这里",
                   "Press a button; its output appears here as it runs"],
    clear:        ["清空",                    "Clear"],
    nLines:       ["{0} 行",                  "{0} lines"],

    // ---- sidebar
    estop:        ["紧急停止",                "Emergency stop"],
    stopMove:     ["■ 停止运动",              "■ STOP moving"],
    estopHint:    ["遥控器 L2 + B 永远比这个快,优先用遥控器。",
                   "The remote (L2 + B) is always faster than this button. "
                   + "Use the remote first."],
    safeHead:     ["只读 · 狗不会动",         "Read-only · the dog will not move"],
    moveHead:     ["会让狗动",                "These make the dog move"],
    moveHint:     ["按之前确认周围没人、脚下有空间。",
                   "Before pressing, check nobody is nearby and the dog has room."],
    recHead:      ["录过的记录与地图",        "Recordings and maps"],

    // ---- maps panel
    noSurvey:     ["还没有记录。用「扫一遍楼」录一次,回来跑 "
                   + "mapping/survey_map.py 建图。",
                   "No recordings yet. Record one with “Survey the "
                   + "building”, then run mapping/survey_map.py to build "
                   + "the map."],
    mapped:       ["已建图",                  "mapped"],
    rawOnly:      ["只有原始数据",            "raw only"],
    seconds:      ["{0} 秒",                  "{0} s"],
    cloudFrames:  ["{0} 帧点云",              "{0} sweeps"],
    photos:       ["{0} 张照片",              "{0} photos"],
    walked:       ["走了 {0} m",              "walked {0} m"],
    mapPoints:    ["地图 {0} 点",             "map {0} points"],
    icpOnly:      ["纯 ICP(狗没给里程计)",   "ICP only (no odometry)"],
    usedOdom:     ["用了板载里程计",          "used on-board odometry"],
    notMapped:    ["这份记录还没建图",        "This recording has no map yet"],
    mapInfo:      ["{0} 点 · 轨迹 {1} 个位姿", "{0} points · {1} poses"],
    pickSurvey:   ["选一份记录",              "Pick a recording"],
    redLine:      ["红线是狗走过的路",        "the red line is where the dog walked"],

    // ---- route and session
    route:        ["怎么连狗",                "How to reach the dog"],
    cable:        ["网线",                    "Cable"],
    wifi:         ["Wi-Fi",                   "Wi-Fi"],
    cableHint:    ["网线:全部功能,包括点云、建图、实时 3D。需要 EDU 版。",
                   "Cable: everything, including the point cloud, mapping "
                   + "and the live 3D view. Needs a Go2 EDU."],
    wifiHint:     ["Wi-Fi:走 WebRTC,和手机 App 一样。能动、能拍照、能录,"
                   + "但没有点云/建图/实时画面。狗一次只接一个 Wi-Fi 连接,"
                   + "先关掉手机 App。",
                   "Wi-Fi: WebRTC, the same path the phone app uses. It can "
                   + "move, photograph and record, but carries no point "
                   + "cloud, no mapping and no live view. The dog accepts "
                   + "one Wi-Fi client at a time, so close the phone app."],
    cableOnly:    ["只能走网线",              "cable only"],
    dogWifi:      ["狗的 Wi-Fi",              "dog Wi-Fi"],
    session:      ["这次测试的记录",          "This session"],
    sessionHint:  ["断网也在记。回来点「导出」,把那个文件给我,我就能看到发生了什么。",
                   "Recorded even with no internet. Press Export when you "
                   + "are back online and hand me the file."],
    nEvents:      ["{0} 条记录 · {1} 个动作 · {2} 个错误",
                   "{0} events · {1} actions · {2} errors"],
    exportBtn:    ["导出给 Claude",           "Export for Claude"],
    exported:     ["已导出:{0}",              "Exported: {0}"],

    // ---- failure box
    fatalHead:    ["面板出错了",              "The panel hit an error"],
    reload:       ["重新加载",                "Reload"],
    dismiss:      ["关掉这个提示",            "Dismiss"],
    scriptErr:    ["脚本出错",                "script error"],
    promiseErr:   ["有一步没完成",            "an operation did not finish"],
  };

  let lang = "zh";
  try {
    // Default to Chinese: this panel is built for one person, who reads
    // Chinese, and whose Chrome happens to be set to English. One click on
    // the EN button changes it for good.
    lang = localStorage.getItem("go2lang") || "zh";
  } catch { /* private window: fall back to the default */ }

  const api = {
    get lang() { return lang; },
    set(next) {
      lang = next;
      try { localStorage.setItem("go2lang", next); } catch {}
      document.documentElement.lang = next === "zh" ? "zh" : "en";
      document.title = api.t("title");
    },
    t(key, ...args) {
      const pair = S[key];
      if (!pair) return key;                      // loud rather than blank
      let s = pair[lang === "zh" ? 0 : 1];
      args.forEach((v, i) => { s = s.split("{" + i + "}").join(v); });
      return s;
    },
  };
  return api;
})();
