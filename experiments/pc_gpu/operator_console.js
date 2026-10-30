"use strict";
(() => {
  let token = "";
  let connection = false;
  let currentMode = "SAFE";
  let pendingAuto = false;
  let autoSession = false;
  let manualTimer = null;
  let holding = null;
  let lastLockState = "UNLOCKED";
  let currentCandidatesKey = "";
  let lastStatus = null;
  let previewBusy = false;
  let previewObjectUrl = null;
  const byId = id => document.getElementById(id);
  const controls = ["auto", "manual", "stop", "left", "right", "estop", "reset", "gesture-on", "gesture-off", "largest-on", "largest-off"];
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
      document.body.classList.remove("authorized");
      byId("video-frame").classList.remove("has-preview");
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
      if (action === "auto" && res.ok) autoSession = true;
      if (action === "stop" || action === "estop" || action === "reset" || action === "gesture-off" || action === "largest-off" || action === "largest-on") {
        pendingAuto = false;
        autoSession = false;
      }
      byId("feedback").textContent = res.already_active
        ? "AUTO：すでに追従中（ESP32には再ARMしません）"
        : res.pending ? action.toUpperCase() + "：ESP32の応答待ち"
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

  async function updatePreview() {
    if (!connection || previewBusy) return;
    previewBusy = true;
    try {
      const response = await fetch("/api/preview.jpg", {
        cache: "no-store", credentials: "omit",
        headers: {"Authorization": "Bearer " + token}
      });
      if (response.ok && response.headers.get("Content-Type")?.includes("image/jpeg")) {
        const blob = await response.blob();
        const nextUrl = URL.createObjectURL(blob);
        const previous = previewObjectUrl;
        previewObjectUrl = nextUrl;
        const img = byId("camera-preview");
        img.src = nextUrl;
        byId("video-frame").classList.add("has-preview");
        // Image loading is asynchronous. Revoking the old blob immediately
        // can make a pending image decode look like an empty camera feed.
        if (previous) window.setTimeout(() => URL.revokeObjectURL(previous), 1200);
      } else if (response.status === 404) {
        byId("video-frame").classList.remove("has-preview");
        byId("video-placeholder").textContent = "映像データを待機中（Piのカメラ・描画を確認）";
      } else {
        byId("video-frame").classList.remove("has-preview");
        byId("video-placeholder").textContent = "映像取得エラー：HTTP " + response.status;
      }
    } catch (err) {
      byId("video-frame").classList.remove("has-preview");
      byId("video-placeholder").textContent = "映像通信エラー：" + err.message;
    } finally {
      previewBusy = false;
    }
  }

  function updateCandidates(status) {
    const area = byId("candidates");
    const candidates = Array.isArray(status.candidates) ? status.candidates : [];
    const lockName = status.lock_state || "UNAVAILABLE";
    const key = JSON.stringify([
      status.frame_seq, lockName, status.mode, status.largest_candidate_index, candidates,
    ]);
    if (key === currentCandidatesKey) return;
    currentCandidatesKey = key;
    area.replaceChildren();
    byId("lock-state").textContent =
      lockName === "TRACKING" ? "追従対象を選択済み" :
      lockName === "TEMPORARILY_LOST" ? "一時見失い・停止中" :
      lockName === "LOST" ? "見失い・再選択待ち" :
      lockName === "UNLOCKED" ? "候補を探しています" : lockName;
    byId("release-target").disabled =
      !status.presenter_selected || status.mode !== "SAFE";
    if (!candidates.length) {
      const empty = document.createElement("span");
      empty.textContent = status.preview_available === false
        ? "カメラプレビュー未取得。接続・診断を確認してください。"
        : "人物候補なし（映像に人物がいるか確認してください）";
      area.appendChild(empty);
      return;
    }
    for (const candidate of candidates) {
      const button = document.createElement("button");
      button.textContent =
        (candidate.index === status.selected_index ? "● 追従対象 " :
        candidate.gesturing ? "🙌 ジェスチャー " :
        candidate.index === status.largest_candidate_index ? "★ 最大候補 " : "候補 ") +
        (candidate.index + 1) +
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
      lastStatus = status;
      const cameraTiming = status.camera_stage_timing_ms;
      const captureMode = status.camera_capture_mode;
      byId("camera-resolution").textContent = (status.camera_resolution || "映像待機") +
        (status.camera_effective_fps == null ? "" : " · 実測 " + status.camera_effective_fps + " FPS") +
        (status.inference_ms == null ? "" : " · GPU推論 " + status.inference_ms + " ms") +
        (captureMode && captureMode.local_inference ? " · USB直結" :
          captureMode && captureMode.fourcc ? " · カメラ形式 " + captureMode.fourcc : "") +
        (cameraTiming && cameraTiming.capture_ms != null
          ? (captureMode && captureMode.local_inference
              ? " · 撮影 " + cameraTiming.capture_ms + " ms / GPUローカル推論 " +
                fmt(cameraTiming.local_infer_ms) + " ms"
              : " · 撮影 " + cameraTiming.capture_ms + " ms / 転送+推論 " +
                fmt(cameraTiming.remote_infer_roundtrip_ms) + " ms") : "");
      currentMode = status.mode;
      byId("mode").textContent = status.mode;
      const led = status.led_indicated_color || (
        status.mode === "AUTO" ? "WHITE" :
        status.gesture_control_enabled ? "YELLOW" : "RED"
      );
      byId("led-dot").className = "led-dot " + led.toLowerCase();
      byId("led-label").textContent =
        led === "WHITE" ? "白・追従中" :
        led === "YELLOW" ? "黄・ジェスチャー待機中" : "赤・停止";

      byId("pc").textContent = status.pc_connected ? "接続中" : "未接続";
      byId("person").textContent = status.target_found ? "検出あり" : "検出なし";
      byId("rtt").textContent = status.rtt_ms == null ? "—" : status.rtt_ms + " ms";
      byId("download-diagnostics").disabled = status.diagnostic_capture_enabled !== true;
      byId("diagnostic-export-note").textContent = status.diagnostic_capture_enabled
        ? "最近の画像と判定のみ。人の顔が映るためZIPは非公開で共有してください。"
        : "起動時に --capture-diagnostics を指定すると保存できます。";
      byId("fw").textContent = status.fw_safety;
      const simulated = status.simulation !== false;
      const headOnly = !simulated && status.head_only_test === true;
      const modeLabel = simulated ? "仮想ESP32 / 実モーター出力なし"
        : headOnly ? "実ESP32 / 首Yaw最大±3°（Pitchは校正済みの場合のみ）"
        : "実ESP32 / 監視のみ・駆動禁止";
      byId("motor").textContent = status.motor_status || modeLabel;
      byId("runtime-subtitle").textContent = simulated
        ? "シミュレーション中｜実モーター出力なし"
        : "実ESP32に接続中｜サーボ電源は実測不可（申告：" +
          (status.servo_power_declared || "不明") + "）｜STOPでは電源遮断不可";
      byId("reason").textContent = fmt(status.reason);
      byId("last").textContent = fmt(status.last_command);
      byId("event").textContent = fmt(status.message);
      byId("ready").textContent = status.auto_ready ? "可能"
        : status.head_feedback_error === "PITCH_OUTSIDE_CALIBRATED_RANGE"
          ? "未準備（上下角度が校正範囲外）"
          : status.head_feedback_error === "YAW_OUTSIDE_APPROVED_RANGE"
            ? "未準備（左右角度が許可範囲外）"
            : "未準備（対象または通信未成立）";
      byId("lock-reason").textContent = fmt(status.lock_reason);
      const head = status.esp32_head_feedback;
      const walking = status.esp32_walking_feedback;
      const follow = status.follow_debug;
      const pitchBounds = status.head_pitch_approved_range;
      const headWarning = status.head_feedback_error === "PITCH_OUTSIDE_CALIBRATED_RANGE" &&
        Array.isArray(pitchBounds) && pitchBounds.length === 2
          ? " / 注意：上下角度の許可範囲は " + fmt(pitchBounds[0]) + "～" +
            fmt(pitchBounds[1]) + "°（診断専用なら駆動禁止のまま確認）"
          : status.head_feedback_error === "YAW_OUTSIDE_APPROVED_RANGE"
            ? " / 注意：左右角度が±3°の許可範囲外"
            : "";
      byId("esp32-head-feedback").textContent = (head
        ? (head.fresh ? "Yaw " + fmt(head.yaw_deg) + "° / Pitch " +
           fmt(head.pitch_deg) + "°（ESP32内部値）" : "テレメトリ未受信・期限切れ")
        : "未対応") + headWarning;
      byId("preview-status").textContent = status.preview_available === false
        ? "Piから未生成" : status.preview_age_ms == null
          ? "取得状態不明" : "画像生成から " + status.preview_age_ms + " ms";
      const pwm = status.uart_head_pwm;
      byId("esp32-servo-diagnostic").textContent = pwm && (pwm.yaw_ch2 || pwm.pitch_ch3)
        ? "左右CH2: " + fmt(pwm.yaw_ch2) + " / 上下CH3: " + fmt(pwm.pitch_ch3) +
          (pwm.pitch_ch3 == null && status.pitch_tracking_enabled
            ? " / 上下のPWM指令を未確認（配線・制御条件を確認）" : "")
        : status.uart_last_servo_diagnostic ||
          "未報告（ESP32診断ファーム未導入／首PWM未出力の可能性）";
      byId("esp32-walking-feedback").textContent = walking
        ? fmt(walking.state) + " / Busy " + fmt(walking.busy) +
          " / IDLE確定 " + fmt(walking.idle_confirmed)
        : "未対応";
      byId("follow-debug").textContent = follow
        ? fmt(follow.reason) + " / 首送信 " + fmt(follow.head_sent) +
          " / 本体送信 " + fmt(follow.body_sent) +
          " / 方位 " + fmt(follow.bearing_deg) + "°" +
          (status.distance_adaptive_head
            ? " / 距離感度 " + fmt(status.apparent_distance_gain) + "倍（画像サイズ推定）"
            : "") +
          (status.responsive_follow ? " / 低遅延追従ON" : "") +
          (status.auto_recalibrate_brief_loss
            ? " / 短時間消失は自動再認識（STOP後の自動起動なし）" : "") +
          (status.pitch_tracking_enabled
            ? " / 上下目標 " + fmt(follow.pitch_target_deg) + "°" +
              "（実測報告 " + fmt(head && head.pitch_deg) + "°、速度上限 " +
              fmt(status.pitch_speed_limit_deg_s) + "°/秒）" +
              (status.relative_pitch_reference
                ? " / 上下は初回ロック時の顔高さ基準（物理原点の校正ではない）"
                : " / 上下は画面中央基準") +
              (follow.pitch_at_limit ? " / 上下可動域の端" : "") +
              (!follow.pitch_aim_valid ? " / 顔位置の信頼性不足で上下保持" : "")
            : " / 上下追従OFF")
        : "人物ロック後のデータ待ち";
      byId("motion-diagnosis").textContent =
        status.simulation !== false ? "仮想ESP32" :
        status.mode !== "AUTO" ? "AUTOではありません" :
        !head || !head.fresh ? "ESP32の首角度応答を取得できていません" :
        status.pitch_tracking_enabled && follow && follow.pitch_at_limit
          ? "上下角度が許可範囲の端です。映像中央に届かない場合、実機での可動範囲校正が必要です" :
        status.pitch_tracking_enabled && pwm && pwm.pitch_ch3 &&
          head.pitch_deg != null && Math.abs(head.pitch_deg) >= 0.5
          ? "上下CH3へPWM命令あり。実際に動かない場合は上下サーボの電源・接続・機構を確認" :
        head.yaw_deg != null && Math.abs(head.yaw_deg) >= 0.5
          ? "ESP32内部角度は更新済み。実サーボの電源・配線・PCA9685出力を確認"
          : follow && !follow.head_sent
            ? "追従判断で首命令が未送信。理由欄を確認"
            : "首の目標値／ESP32内部角度の更新待ち";
      const fault = status.uart_poll_error
        ? "UART_POLL_" + status.uart_poll_error
        : status.fault_cause || status.arm_error || null;
      byId("fault-cause").textContent = fault || "なし";
      byId("fw-arm-status").textContent = status.fw_arm_pending
        ? "ARM受付応答を待機中"
        : status.mode === "AUTO" ? "ARM成立／AUTO"
        : status.arm_error || (status.mode === "ESTOP" ? "ESTOP／ARM不可" : "待機／未ARM");
      if (status.mode === "AUTO" && autoSession) {
        byId("feedback").textContent = "AUTO：ESP32 ARM成立・追従中";
        pendingAuto = false;
      } else if (status.mode === "ESTOP" && (pendingAuto || autoSession)) {
        byId("feedback").textContent = "AUTO中断：" + (fault || status.stop_reason || "緊急停止");
        pendingAuto = false;
        autoSession = false;
      } else if (status.mode === "SAFE" && autoSession && !status.fw_arm_pending) {
        byId("feedback").textContent = "AUTO停止：" + (status.arm_error || status.reason);
        pendingAuto = false;
        autoSession = false;
      } else if (pendingAuto && status.arm_error) {
        byId("feedback").textContent = "AUTO未成立：" + status.arm_error;
        pendingAuto = false;
        autoSession = false;
      }
      byId("auto").disabled = !status.auto_ready ||
        status.mode !== "SAFE" || !!status.fw_arm_pending || pendingAuto;
      const gestureEnabled = status.gesture_control_enabled === true;
      const largestEnabled = status.largest_control_enabled === true;
      byId("gesture-on").disabled = simulated || status.mode !== "SAFE" || gestureEnabled;
      byId("gesture-off").disabled = simulated || !gestureEnabled;
      byId("largest-on").disabled = simulated || status.mode !== "SAFE" || largestEnabled;
      byId("largest-off").disabled = simulated || status.mode !== "SAFE" || !largestEnabled;
      byId("largest-on").textContent = largestEnabled
        ? "最大人物を選択中（手動AUTOが必要）"
        : "最大の人物を自動ロック（別モード）";
      byId("gesture-on").textContent = gestureEnabled ? "ジェスチャー待機中" : "ジェスチャー追従を待機";
      byId("largest-help").textContent = largestEnabled
        ? "一番大きい人物を候補として連続確認中。緑でロックされたら、操作者が対象と周囲の安全を確認してから手動AUTOを押してください。自動ARMはしません。"
        : "最大人物モードはジェスチャー待機との切替式。前景の顔が一意で8フレーム安定した人だけ選びます。選択後も手動AUTOが必要です。";
      byId("gesture-help").textContent = gestureEnabled
        ? "待機中：両手を頭より上に挙げて保持。認識後、対象が緑になり、ESP32へのAUTO切替を試みます。"
        : "待機を有効にし、両手を頭より上に挙げて保持。見失ったら停止し、再度ジェスチャーで開始。";
      byId("manual").disabled = status.mode === "ESTOP" || !simulated;
      byId("stop").disabled = false;
      byId("estop").disabled = false;
      byId("reset").disabled = status.mode !== "ESTOP";
      ["left", "right"].forEach(id => byId(id).disabled = status.mode !== "MANUAL" || !simulated);
      updateCandidates(status);
      byId("auth-message").textContent = simulated ? "接続中（仮想）" : "接続中（実機）";
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
  byId("gesture-on").addEventListener("click", () => act("gesture_on"));
  byId("gesture-off").addEventListener("click", () => act("gesture_off"));
  byId("largest-on").addEventListener("click", () => act("largest_on"));
  byId("largest-off").addEventListener("click", () => act("largest_off"));
  byId("release-target").addEventListener("click", () => targetAction({release: true}));
  byId("download-diagnostics").addEventListener("click", async () => {
    const button = byId("download-diagnostics");
    button.disabled = true;
    byId("diagnostic-export-note").textContent = "診断ZIPを準備しています…";
    try {
      const response = await fetch("/api/diagnostics.zip", {
        cache: "no-store", credentials: "omit",
        headers: {"Authorization": "Bearer " + token}
      });
      if (!response.ok) throw Error("HTTP " + response.status);
      const archive = await response.blob();
      const url = URL.createObjectURL(archive);
      const link = document.createElement("a");
      link.href = url;
      link.download = "PresentationEscort-diagnostics.zip";
      document.body.appendChild(link);
      link.click();
      link.remove();
      window.setTimeout(() => URL.revokeObjectURL(url), 10000);
      byId("diagnostic-export-note").textContent =
        "保存しました。画像が含まれるので、確認相手にだけ送ってください。";
    } catch (err) {
      byId("diagnostic-export-note").textContent = "保存失敗: " + err.message;
    } finally {
      button.disabled = !lastStatus?.diagnostic_capture_enabled;
    }
  });
  byId("connect").addEventListener("click", async () => {
    const supplied = byId("token").value.trim();
    if (!supplied) return;
    token = supplied;
    try {
      await call("/api/status");
      connection = true;
      document.body.classList.add("authorized");
      byId("token").value = "";
      updatePreview();
      await refresh();
    } catch (err) {
      connection = false;
      byId("auth-message").textContent = err.message;
    }
  });
  byId("camera-preview").addEventListener("click", event => {
    if (!lastStatus || lastStatus.mode !== "SAFE" || lastStatus.presenter_selected) return;
    const img = byId("camera-preview");
    const rect = img.getBoundingClientRect();
    if (!img.naturalWidth || !img.naturalHeight) return;
    const scale = Math.min(rect.width / img.naturalWidth, rect.height / img.naturalHeight);
    const displayWidth = img.naturalWidth * scale;
    const displayHeight = img.naturalHeight * scale;
    const x = (event.clientX - rect.left - (rect.width - displayWidth) / 2) / displayWidth * 100;
    const y = (event.clientY - rect.top - (rect.height - displayHeight) / 2) / displayHeight * 100;
    if (!(x >= 0 && x <= 100 && y >= 0 && y <= 100)) return;
    const matches = (lastStatus.candidates || []).filter(box =>
      x >= box.x1_percent && x <= box.x2_percent &&
      y >= box.y1_percent && y <= box.y2_percent);
    if (matches.length === 1) {
      targetAction({index: matches[0].index, frame_seq: lastStatus.frame_seq});
    } else if (matches.length > 1) {
      byId("lock-message").textContent = "重なった人物を判別できません。候補一覧から選択してください。";
    }
  });
  window.setInterval(refresh, 450);
  window.setInterval(updatePreview, 300);
})();
