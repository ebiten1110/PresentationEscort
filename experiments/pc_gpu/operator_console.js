"use strict";
(() => {
  let token = "";
  let connection = false;
  let currentMode = "SAFE";
  let pendingAuto = false;
  let manualTimer = null;
  let holding = null;
  let lastLockState = "UNLOCKED";
  let currentCandidatesKey = "";
  const byId = id => document.getElementById(id);
  const controls = ["auto", "manual", "stop", "left", "right", "estop", "reset"];
  const fmt = val => (val === null || val === undefined) ? "—" : String(val);

  async function call(path, obj) {
    const result = await fetch(path, {
      method: obj ? "POST" : "GET",
      cache: "no-store",
      credentials: "omit",
      headers: {
        "Authorization": "Bearer " + token,
        ...(obj ? {"Content-Type": "application/json"} : {})
      },
      ...(obj ? {body: JSON.stringify(obj)} : {})
    });
    const json = await result.json();
    if (result.status === 401) {
      connection = false;
      controls.forEach(id => byId(id).disabled = true);
      throw Error("トークンが無効です");
    }
    if (!result.ok) throw Error(json.error || result.status);
    return json;
  }

  async function act(action) {
    try {
      const res = await call("/api/action", {action});
      pendingAuto = action === "auto" && !!res.pending;
      byId("feedback").textContent = res.pending
        ? action.toUpperCase() + "：ESP32の応答待ち"
        : res.ok ? action.toUpperCase() + " 実行" : res.error;
      await refresh();
    } catch (err) {
      byId("feedback").textContent = "操作失敗: " + err.message;
    }
  }

  async function targetAction(payload) {
    try {
      const data = await call("/api/target", payload);
      byId("lock-message").textContent = data.ok
        ? "候補を選択しました。次の映像で再確認します。"
        : data.error;
      await refresh();
    } catch (err) {
      byId("lock-message").textContent = "選択できません: " + err.message;
    }
  }

  function updateCandidates(status) {
    const area = byId("candidates");
    const candidates = Array.isArray(status.candidates) ? status.candidates : [];
    const lockName = status.lock_state || "UNAVAILABLE";
    const key = JSON.stringify([
      status.frame_seq, lockName, status.mode, candidates,
    ]);
    if (key === currentCandidatesKey) return;
    currentCandidatesKey = key;
    area.replaceChildren();
    byId("lock-state").textContent = lockName;
    byId("release-target").disabled =
      !status.presenter_selected || status.mode !== "SAFE";
    if (!candidates.length) {
      const empty = document.createElement("span");
      empty.textContent = "人物候補なし／PCが旧版の可能性があります";
      area.appendChild(empty);
      return;
    }
    for (const candidate of candidates) {
      const button = document.createElement("button");
      button.textContent =
        "候補 " + (candidate.index + 1) +
        "（左右 " + candidate.x_percent + "% / 上下 " +
        candidate.y_percent + "% / 信頼度 " +
        Math.round(candidate.confidence * 100) + "%）";
      button.disabled = status.mode !== "SAFE" ||
        !!status.presenter_selected || lockName === "NO_FRESH_FRAME";
      button.addEventListener("click", () => {
        targetAction({index: candidate.index, frame_seq: status.frame_seq});
      });
      area.appendChild(button);
    }
  }

  async function refresh() {
    if (!connection) return;
    try {
      const status = await call("/api/status");
      currentMode = status.mode;
      byId("mode").textContent = status.mode;
      byId("pc").textContent = status.pc_connected ? "接続中" : "未接続";
      byId("person").textContent = status.target_found ? "検出あり" : "検出なし";
      byId("rtt").textContent = status.rtt_ms == null ? "—" : status.rtt_ms + " ms";
      byId("fw").textContent = status.fw_safety;
      const simulated = status.simulation !== false;
      const headOnly = !simulated && status.head_only_test === true;
      const modeLabel = simulated ? "仮想ESP32 / 実モーター出力なし"
        : headOnly ? "実ESP32 / 首Yawのみ最大±3°"
        : "実ESP32 / 監視のみ・駆動禁止";
      byId("motor").textContent = status.motor_status || modeLabel;
      byId("runtime-subtitle").textContent = simulated
        ? "シミュレーション中｜実モーター出力なし"
        : "実ESP32に接続中｜サーボ電源は実測不可（申告：" +
          (status.servo_power_declared || "不明") + "）｜STOPでは電源遮断不可";
      byId("reason").textContent = fmt(status.reason);
      byId("last").textContent = fmt(status.last_command);
      byId("event").textContent = fmt(status.message);
      byId("ready").textContent = status.auto_ready ? "可能" : "未準備（対象または通信未成立）";
      byId("lock-reason").textContent = fmt(status.lock_reason);
      const fault = status.uart_poll_error
        ? "UART_POLL_" + status.uart_poll_error
        : status.fault_cause || status.arm_error || null;
      byId("fault-cause").textContent = fault || "なし";
      byId("fw-arm-status").textContent = status.fw_arm_pending
        ? "ARM受付応答を待機中"
        : status.mode === "AUTO" ? "ARM成立／AUTO"
        : status.arm_error || (status.mode === "ESTOP" ? "ESTOP／ARM不可" : "待機／未ARM");
      if (pendingAuto) {
        if (status.mode === "AUTO") {
          byId("feedback").textContent = "AUTO：ESP32 ARM成立・追従中";
          pendingAuto = false;
        } else if (status.mode === "ESTOP") {
          byId("feedback").textContent = "AUTO中断：" + (fault || "緊急停止");
          pendingAuto = false;
        } else if (status.arm_error || !status.fw_arm_pending) {
          byId("feedback").textContent = "AUTO未成立：" + (status.arm_error || status.reason);
          pendingAuto = false;
        }
      }
      byId("auto").disabled = !status.auto_ready || status.mode === "ESTOP";
      byId("manual").disabled = status.mode === "ESTOP" || !simulated;
      byId("stop").disabled = false;
      byId("estop").disabled = false;
      byId("reset").disabled = status.mode !== "ESTOP";
      ["left", "right"].forEach(id => byId(id).disabled = status.mode !== "MANUAL" || !simulated);
      updateCandidates(status);
      byId("auth-message").textContent = simulated ? "認証済み｜DRY RUN" : "認証済み｜実ESP32";
    } catch (err) {
      byId("auth-message").textContent = "更新に失敗: " + err.message;
    }
  }

  async function sendHold(direction) {
    if (!connection || currentMode !== "MANUAL") return;
    try { await call("/api/manual", {direction}); }
    catch (err) { byId("feedback").textContent = "操作中断: " + err.message; }
  }

  function releaseHold() {
    if (manualTimer !== null) {
      clearInterval(manualTimer);
      manualTimer = null;
    }
    if (holding) {
      holding.classList.remove("pressed");
      holding = null;
      // Safety STOP is a guaranteed best-effort when hold is released.
      if (connection) act("stop");
    }
  }

  ["left","right"].forEach(direction => {
    const btn = byId(direction);
    btn.addEventListener("pointerdown", e => {
      if (btn.disabled || holding) return;
      holding = btn;
      btn.classList.add("pressed");
      btn.setPointerCapture(e.pointerId);
      sendHold(direction);
      manualTimer = setInterval(() => sendHold(direction), 120);
    });
    btn.addEventListener("pointerup", releaseHold);
    btn.addEventListener("pointercancel", releaseHold);
    btn.addEventListener("lostpointercapture", releaseHold);
  });
  window.addEventListener("blur", releaseHold);
  document.addEventListener("visibilitychange", () => {
    if (document.hidden) releaseHold();
  });
  ["auto","manual","stop","estop","reset"].forEach(name => {
    byId(name).addEventListener("click", () => { releaseHold(); act(name); });
  });
  byId("release-target").addEventListener("click", () => targetAction({release: true}));
  byId("connect").addEventListener("click", async () => {
    const supplied = byId("token").value.trim();
    if (!supplied) return;
    token = supplied;
    try {
      await call("/api/status");
      connection = true;
      byId("token").value = "";
      await refresh();
    } catch (err) {
      connection = false;
      byId("auth-message").textContent = err.message;
    }
  });
  window.setInterval(refresh, 450);
})();
