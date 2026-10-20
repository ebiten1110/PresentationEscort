"use strict";
(() => {
  let token = "";
  let connection = false;
  let currentMode = "SAFE";
  let manualTimer = null;
  let holding = null;
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
      byId("feedback").textContent = res.ok ? action.toUpperCase() + " 実行" : res.error;
      await refresh();
    } catch (err) {
      byId("feedback").textContent = "操作失敗: " + err.message;
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
      byId("reason").textContent = fmt(status.reason);
      byId("last").textContent = fmt(status.last_command);
      byId("event").textContent = fmt(status.message);
      byId("ready").textContent = status.auto_ready ? "可能" : "未準備";
      byId("auto").disabled = !status.auto_ready || status.mode === "ESTOP";
      byId("manual").disabled = status.mode === "ESTOP";
      byId("stop").disabled = false;
      byId("estop").disabled = false;
      byId("reset").disabled = status.mode !== "ESTOP";
      ["left", "right"].forEach(id => byId(id).disabled = status.mode !== "MANUAL");
      byId("auth-message").textContent = "認証済み｜DRY RUN";
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
