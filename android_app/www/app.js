// DiveWatch App - 主逻辑
// BLE: 优先 Capacitor 原生插件 (Android), 回退 Web Bluetooth
import { Chart, registerables } from "chart.js";
Chart.register(...registerables);

const SVC  = "1d14d6ee-fd63-4fa1-bfa4-8f47b42119f0";
const SSID = "f7bf3564-fb6d-4e53-88a4-5e37e0326063";
const PSK  = "984227f3-34fc-4045-a5d0-2c581f81a153";
const CMD  = "0c533cef-d6b0-4ba2-9f08-7d6cea1b3e7e";
const STAT = "3b1f9c5e-8a72-4d11-9b3a-7c4e2f1a5d80";

// ============ State ============
const S = {
  deviceId: null, native: false,    // native=true 用 Capacitor BLE
  ip: "", bat: null, depth: null, temp: null,
  liveDepth: [], otaFile: null,
  charts: {},
};

const $ = (id) => document.getElementById(id);
const enc = new TextEncoder(); const dec = new TextDecoder();

// ============ Capacitor 探测 ============
let BLE = null;          // @capacitor-community/bluetooth-le
let Prefs = null;        // @capacitor/preferences
// 静态 import 让 esbuild 把它打到 bundle 里
import { BleClient } from "@capacitor-community/bluetooth-le";
import { Preferences } from "@capacitor/preferences";
async function loadCap() {
  if (window.Capacitor?.isNativePlatform?.()) {
    try {
      BLE = BleClient;
      await BLE.initialize({ androidNeverForLocation: true });
      Prefs = Preferences;
      S.native = true;
      log("Capacitor BLE ready", "ok");
    } catch (e) {
      log("Cap BLE 初始化失败: " + e.message, "err");
    }
  }
}

// ============ Log ============
function log(msg, type) {
  const box = $("log"); if (!box) return;
  const div = document.createElement("div");
  if (type) div.className = type;
  const t = new Date().toLocaleTimeString("zh-CN", { hour12: false });
  div.textContent = `[${t}] ${msg}`;
  box.appendChild(div); box.scrollTop = box.scrollHeight;
}
let _toastT;
function toast(msg, type) {
  const el = $("toast"); el.textContent = msg;
  el.className = "toast show " + (type || "");
  clearTimeout(_toastT);
  _toastT = setTimeout(() => el.className = "toast", 2400);
}

// ============ Conn UI ============
function setConn(state, text) {
  const pill = $("btnConn"); pill.className = "pill " + (state === "offline" ? "" : state);
  $("connText").textContent = text;
  const online = (state === "online");
  $("btnDisc").disabled = !online;
  $("btnScan").disabled = (state === "busy");
  $("btnScan").textContent = online ? "已连接" : "扫描连接";
  ["btnWifiConn","btnWifiForget","btnReboot"].forEach(id => $(id).disabled = !online);
  updateOtaBtn();
}
function updateOtaBtn() {
  $("btnOta").disabled = !(S.otaFile && $("inpIp").value.trim());
}

// ============ BLE: scan + connect ============
async function scan() {
  setConn("busy", "扫描中…");
  try {
    if (S.native) {
      const dev = await BLE.requestDevice({
        services: [SVC],
        namePrefix: "DiveWatch",
      });
      S.deviceId = dev.deviceId;
      log("已选: " + (dev.name || dev.deviceId));
      await BLE.connect(S.deviceId, onDisconnect);
      await BLE.startNotifications(S.deviceId, SVC, STAT, (v) => {
        onStat(dec.decode(v));
      });
    } else {
      // Web Bluetooth fallback (开发用)
      const d = await navigator.bluetooth.requestDevice({
        filters: [{ namePrefix: "DiveWatch" }],
        optionalServices: [SVC],
      });
      d.addEventListener("gattserverdisconnected", onDisconnect);
      const server = await d.gatt.connect();
      const svc = await server.getPrimaryService(SVC);
      const ch  = await svc.getCharacteristic(STAT);
      await ch.startNotifications();
      ch.addEventListener("characteristicvaluechanged",
        (e) => onStat(dec.decode(e.target.value)));
      S.deviceId = d;
      S._svc = svc; // 暂存
    }
    setConn("online", "已连接");
    log("订阅 STAT 完成", "ok"); toast("已连接", "ok");
    $("aboutDev").textContent = "DiveWatch-OTA";
  } catch (e) {
    setConn("offline", "未连接");
    log("连接失败: " + e.message, "err"); toast("连接失败", "err");
  }
}
function onDisconnect() {
  setConn("offline", "未连接");
  log("设备已断开", "warn");
}
async function disconnect() {
  try {
    if (S.native) await BLE.disconnect(S.deviceId);
    else S.deviceId?.gatt?.disconnect();
  } catch (e) {}
}
async function bleWrite(charUuid, str) {
  const data = enc.encode(str);
  if (S.native) {
    const view = new DataView(data.buffer);
    await BLE.write(S.deviceId, SVC, charUuid, view);
  } else {
    const ch = await S._svc.getCharacteristic(charUuid);
    await ch.writeValueWithResponse(data);
  }
}

// ============ STAT handler + live data ============
function onStat(txt) {
  txt = txt.trim();
  log("STAT: " + txt);
  let m;
  if ((m = txt.match(/^WIFI_OK\s+([\d.]+)/))) {
    S.ip = m[1]; $("ipVal").textContent = S.ip;
    $("wifiVal").textContent = "已连";
    $("inpIp").value = S.ip;
    updateOtaBtn(); toast("WiFi 已连 " + S.ip, "ok");
  } else if (txt.startsWith("WIFI_FAIL")) {
    $("wifiVal").textContent = "失败"; toast("WiFi 失败", "err");
  }
  if ((m = txt.match(/bat=(\d+)/i)))   { S.bat = +m[1]; $("batPct").textContent = S.bat; }
  if ((m = txt.match(/depth=([\d.]+)/i))) {
    S.depth = +m[1];
    $("depthVal").innerHTML = S.depth.toFixed(1) + "<small>m</small>";
    pushLive(S.depth);
  }
  if ((m = txt.match(/temp=([\d.]+)/i))) {
    S.temp = +m[1];
    $("tempVal").innerHTML = S.temp.toFixed(1) + "<small>°</small>";
  }
  if ((m = txt.match(/ip=([\d.]+)/i))) {
    S.ip = m[1]; $("ipVal").textContent = S.ip; $("inpIp").value = S.ip;
    updateOtaBtn();
  }
}

function pushLive(d) {
  const now = Date.now();
  S.liveDepth.push({ t: now, d });
  if (S.liveDepth.length > 120) S.liveDepth.shift();
  $("liveCnt").textContent = S.liveDepth.length + " 点";
  if (S.charts.live) {
    S.charts.live.data.labels = S.liveDepth.map((_, i) => i);
    S.charts.live.data.datasets[0].data = S.liveDepth.map(p => p.d);
    S.charts.live.update("none");
  }
}

// ============ WiFi / 控制 ============
async function wifiConn() {
  const ssid = $("inpSsid").value.trim(), psk = $("inpPsk").value;
  if (!ssid) { toast("请输入 SSID", "err"); return; }
  try {
    await bleWrite(SSID, ssid);
    await bleWrite(PSK, psk);
    await bleWrite(CMD, "C");
    log("WiFi 配置已发送", "info"); toast("已发送, 等待连接…");
    await saveLocal("wifi-ssid", ssid);
  } catch (e) { log(e.message, "err"); toast("失败", "err"); }
}
async function wifiForget() {
  try { await bleWrite(CMD, "F"); log("CMD=F", "warn"); toast("已清除"); }
  catch (e) { log(e.message, "err"); }
}
async function reboot() {
  try { await bleWrite(CMD, "R"); log("CMD=R", "warn"); toast("重启中"); }
  catch (e) { log(e.message, "err"); }
}

// ============ OTA push (WiFi HTTP) ============
async function otaPush() {
  const ip = $("inpIp").value.trim();
  if (!ip || !S.otaFile) return;
  log(`推送 ${ip} (${S.otaFile.size} B)`, "info"); toast("上传中…");
  $("btnOta").disabled = true;
  try {
    const url = `http://${ip}/update`;
    const xhr = new XMLHttpRequest();
    const form = new FormData();
    form.append("update", S.otaFile, S.otaFile.name);
    xhr.upload.onprogress = (e) => {
      if (!e.lengthComputable) return;
      const pct = Math.round(e.loaded * 100 / e.total);
      $("otaBar").style.width = pct + "%";
      $("otaPct").textContent = pct + "%";
      $("otaSize").textContent = `${(e.loaded/1024).toFixed(0)}/${(e.total/1024).toFixed(0)} KB`;
    };
    xhr.onload = () => {
      if (xhr.status >= 200 && xhr.status < 300) {
        log("OTA 完成, 设备重启", "ok"); toast("OTA 完成", "ok");
      } else {
        log(`HTTP ${xhr.status}: ${xhr.responseText}`, "err");
        toast("OTA 失败 " + xhr.status, "err");
      }
      $("btnOta").disabled = false;
    };
    xhr.onerror = () => { log("网络错误", "err"); toast("网络错误", "err"); $("btnOta").disabled = false; };
    xhr.open("POST", url, true);
    xhr.send(form);
  } catch (e) { log(e.message, "err"); $("btnOta").disabled = false; }
}

// ============ 本地存储 (IndexedDB / Preferences) ============
async function saveLocal(k, v) {
  if (Prefs) await Prefs.set({ key: k, value: String(v) });
  else localStorage.setItem(k, String(v));
}
async function loadLocal(k) {
  if (Prefs) return (await Prefs.get({ key: k })).value;
  return localStorage.getItem(k);
}

// IndexedDB for dive logs
const DB_NAME = "divewatch", STORE = "dives";
function openDB() {
  return new Promise((res, rej) => {
    const r = indexedDB.open(DB_NAME, 1);
    r.onupgradeneeded = (e) => {
      const db = e.target.result;
      if (!db.objectStoreNames.contains(STORE)) {
        const s = db.createObjectStore(STORE, { keyPath: "id", autoIncrement: true });
        s.createIndex("ts", "ts");
      }
    };
    r.onsuccess = () => res(r.result);
    r.onerror = () => rej(r.error);
  });
}
async function addDive(dive) {
  const db = await openDB();
  return new Promise((res) => {
    const tx = db.transaction(STORE, "readwrite");
    tx.objectStore(STORE).add(dive).onsuccess = (e) => res(e.target.result);
  });
}
async function listDives() {
  const db = await openDB();
  return new Promise((res) => {
    const tx = db.transaction(STORE, "readonly");
    tx.objectStore(STORE).getAll().onsuccess = (e) => res(e.target.result || []);
  });
}
async function clearDives() {
  const db = await openDB();
  return new Promise((res) => {
    const tx = db.transaction(STORE, "readwrite");
    tx.objectStore(STORE).clear().onsuccess = () => res();
  });
}

// ============ 图表 ============
const chartCommon = {
  responsive: true, maintainAspectRatio: false,
  animation: false,
  scales: {
    x: { grid: { color: "#21262e", display: false }, ticks: { display: false } },
    y: { grid: { color: "#21262e" }, ticks: { color: "#5a5f68", font: { size: 10 } },
         reverse: false },
  },
  plugins: { legend: { display: false } },
};
function initCharts() {
  S.charts.live = new Chart($("liveChart"), {
    type: "line",
    data: { labels: [], datasets: [{
      data: [], borderColor: "#ff7a3a", backgroundColor: "rgba(255,122,58,0.15)",
      borderWidth: 2, fill: true, tension: 0.3, pointRadius: 0,
    }] },
    options: { ...chartCommon, scales: { ...chartCommon.scales,
      y: { ...chartCommon.scales.y, reverse: true } } },
  });
  S.charts.hist = new Chart($("histChart"), {
    type: "bar",
    data: { labels: [], datasets: [{
      data: [], backgroundColor: "#ff7a3a", borderRadius: 4,
    }] },
    options: chartCommon,
  });
}

// ============ Dive 日志渲染 ============
async function renderDives() {
  const dives = await listDives();
  $("diveCount").textContent = dives.length + " 条";
  const list = $("diveList");
  list.innerHTML = "";
  if (!dives.length) {
    list.innerHTML = '<div class="empty">暂无记录<br><small style="opacity:0.6">实时数据自动累积记录</small></div>';
  } else {
    for (const d of dives.slice().reverse()) {
      const item = document.createElement("div");
      item.className = "dive-item";
      const date = new Date(d.ts).toLocaleString("zh-CN");
      item.innerHTML = `<div>
        <div class="dive-date">${date}</div>
        <div class="dive-meta">${(d.minutes||0)}分 · ${d.points||0}点</div>
      </div><div class="dive-depth">${(d.maxDepth||0).toFixed(1)}<small style="font-size:12px;color:var(--text-dim)">m</small></div>`;
      list.appendChild(item);
    }
  }
  // 历史 chart: 最近 10 次最深
  const recent = dives.slice(-10);
  S.charts.hist.data.labels = recent.map((_, i) => "#" + (dives.length - recent.length + i + 1));
  S.charts.hist.data.datasets[0].data = recent.map(d => d.maxDepth || 0);
  S.charts.hist.update();
  // 主页统计
  $("totalDives").textContent = dives.length;
  $("maxDepth").textContent = dives.reduce((m, d) => Math.max(m, d.maxDepth || 0), 0).toFixed(1);
  $("totalMin").textContent = Math.round(dives.reduce((s, d) => s + (d.minutes || 0), 0));
}

// 自动收尾 live → 存为 dive (>30 点 & 静止 60s 视为一次潜水结束)
let lastPushTs = 0, autoSaveT;
function trackAutoSave() {
  setInterval(async () => {
    if (S.liveDepth.length < 30) return;
    const last = S.liveDepth[S.liveDepth.length - 1];
    if (Date.now() - last.t > 60_000) {
      // 一次潜水完成
      const max = Math.max(...S.liveDepth.map(p => p.d));
      const mins = (last.t - S.liveDepth[0].t) / 60000;
      await addDive({
        ts: S.liveDepth[0].t, maxDepth: max,
        minutes: Math.round(mins),
        points: S.liveDepth.length,
      });
      log(`已保存潜水: 最深 ${max.toFixed(1)}m, ${Math.round(mins)} 分`, "ok");
      S.liveDepth = []; renderDives(); pushLive(0);
    }
  }, 10000);
}

// ============ Tab 切换 ============
function switchPage(name) {
  for (const p of document.querySelectorAll(".page")) {
    p.classList.toggle("hidden", p.id !== "page-" + name);
  }
  for (const t of document.querySelectorAll(".tab")) {
    t.classList.toggle("active", t.dataset.page === name);
  }
  if (name === "log") renderDives();
}

// ============ Bind ============
function bind() {
  $("btnScan").onclick = scan;
  $("btnDisc").onclick = disconnect;
  $("btnConn").onclick = () => { if (S.deviceId) disconnect(); else scan(); };
  $("btnWifiConn").onclick = wifiConn;
  $("btnWifiForget").onclick = wifiForget;
  $("btnReboot").onclick = reboot;
  $("btnOta").onclick = otaPush;
  $("btnLogClear").onclick = async () => {
    if (confirm("清空所有潜水记录?")) { await clearDives(); renderDives(); toast("已清空"); }
  };
  $("inpFile").onchange = (e) => {
    const f = e.target.files[0]; S.otaFile = f || null;
    $("fileName").textContent = f ? `${f.name} · ${(f.size/1024).toFixed(0)} KB` : "";
    $("fileLbl").textContent = f ? "重新选择" : "选择 .bin 文件";
    updateOtaBtn();
  };
  $("inpIp").oninput = updateOtaBtn;
  for (const t of document.querySelectorAll(".tab")) {
    t.onclick = () => switchPage(t.dataset.page);
  }
}

// ============ Init ============
(async () => {
  bind();
  initCharts();
  await loadCap();
  setConn("offline", "未连接");
  // 恢复 WiFi SSID
  const lastSsid = await loadLocal("wifi-ssid");
  if (lastSsid) $("inpSsid").value = lastSsid;
  await renderDives();
  trackAutoSave();
  log("DiveWatch App ready");
})();
