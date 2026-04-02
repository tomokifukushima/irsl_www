// ============================================================================
// Camera Mirror System - Client Logic
// Meta Quest 3 カメラミラーリングシステム クライアント
// ============================================================================

(async () => {

const elem = {
  senderVideo: document.getElementById('senderVideo'),
  receiverCanvas: document.getElementById('receiverCanvas'),
  cameraSelect: document.getElementById('cameraSelect'),
  startSendBtn: document.getElementById('startSendBtn'),
  stopSendBtn: document.getElementById('stopSendBtn'),
  clearCanvasBtn: document.getElementById('clearCanvasBtn'),
  downloadImageBtn: document.getElementById('downloadImageBtn'),
  
  senderStatus: document.getElementById('senderStatus'),
  senderFrameCount: document.getElementById('senderFrameCount'),
  senderResolution: document.getElementById('senderResolution'),
  senderBytes: document.getElementById('senderBytes'),
  
  receiverStatus: document.getElementById('receiverStatus'),
  receiverFrameCount: document.getElementById('receiverFrameCount'),
  lastUpdateTime: document.getElementById('lastUpdateTime'),
  latencyMs: document.getElementById('latencyMs'),
  
  wsStatus: document.getElementById('wsStatus'),
  fpsInfo: document.getElementById('fpsInfo'),

  controllerCount: document.getElementById('controllerCount'),
  leftStick: document.getElementById('leftStick'),
  rightStick: document.getElementById('rightStick'),
  posePos: document.getElementById('posePos'),
  poseOri: document.getElementById('poseOri'),
  buttonList: document.getElementById('buttonList'),
  startXRBtn: document.getElementById('startXRBtn'),
  xrStatus: document.getElementById('xrStatus'),
};

const state = {
  ws: null,
  stream: null,
  ctx: null,
  cameras: [],
  currentCameraId: null,
  ros: null,
  rosImagePub: null,
  
  senderFrameCount: 0,
  receiverFrameCount: 0,
  isStreaming: false,
  frameStartTime: 0,
  fps: 0,
  lastSendTime: 0,
  
  // フレーム品質向上用
  maxFramesPerSecond: 30,  // 送信フレームレート制限を30FPSに
  lastFrameTime: 0,
  
  // 受信側の品質向上
  pendingFrame: null,
  isProcessing: false,
  expectedFrameNum: 0,
  droppedFrames: 0,

  // Controller
  controllerPollId: null,
  // XR
  xrSession: null,
  xrRefSpace: null,
  xrFrameHandle: null,
};

// ========== WebSocket 接続 ==========
function connectWebSocket() {
  const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
  const wsUrl = `${protocol}//${window.location.host}`;
  
  state.ws = new WebSocket(wsUrl);
  
  state.ws.onopen = () => {
    elem.wsStatus.textContent = '✓ WS接続';
    elem.wsStatus.className = 'info status-ok';
    console.log('[Mirror] WebSocket接続成功');
  };
  
  state.ws.onmessage = (event) => {
    try {
      const msg = JSON.parse(event.data);
      
      if (msg.type === 'camera_stream') {
        handleRemoteFrame(msg.data);
      }
    } catch(e) {
      console.error('[Mirror] メッセージ解析エラー:', e);
    }
  };
  
  state.ws.onerror = (error) => {
    elem.wsStatus.textContent = '❌ WS失敗';
    elem.wsStatus.className = 'info status-error';
    console.error('[Mirror] WebSocket エラー:', error);
  };
  
  state.ws.onclose = () => {
    elem.wsStatus.textContent = '✗ WS切断';
    elem.wsStatus.className = 'info status-warn';
    console.log('[Mirror] WebSocket切断');
    setTimeout(connectWebSocket, 3000);
  };
}

// ========== ROS への接続 (rostest 形式のクエリパラメータ対応) ==========
function connectRosFromQuery() {
  if (typeof ROSLIB === 'undefined') {
    console.warn('[Mirror] roslib.js が読み込まれていません');
    return;
  }
  const params = new URLSearchParams(window.location.search);
  const wsport = params.get('wsport');
  const wsaddr = params.get('wsaddr');
  const ssl = params.get('ssl');
  if (!wsport || !wsaddr) {
    console.log('[Mirror] rosbridge接続パラメータなし');
    return;
  }
  const rosUrl = `${ssl ? 'wss' : 'ws'}://${wsaddr}:${wsport}`;
  console.log('[Mirror] rosbridge接続:', rosUrl);
  const ros = new ROSLIB.Ros({ url: rosUrl });
  ros.on('connection', () => console.log('[Mirror] rosbridge connected'));
  ros.on('error', (e) => console.error('[Mirror] rosbridge error', e));
  ros.on('close', () => console.warn('[Mirror] rosbridge closed'));

  const imagePub = new ROSLIB.Topic({
    ros,
    name: '/hmd/camera/compressed',
    messageType: 'sensor_msgs/CompressedImage'
  });

  state.ros = ros;
  state.rosImagePub = imagePub;
}

// ========== カメラ一覧取得 ==========
async function enumerateCameras() {
  try {
    const devices = await navigator.mediaDevices.enumerateDevices();
    state.cameras = devices.filter(d => d.kind === 'videoinput');
    
    if (state.cameras.length === 0) {
      console.error('[Mirror] カメラが見つかりません');
      return;
    }
    
    elem.cameraSelect.innerHTML = '<option value="">カメラを選択...</option>';
    state.cameras.forEach((device, index) => {
      const option = document.createElement('option');
      option.value = device.deviceId;
      option.textContent = device.label || `カメラ ${index + 1}`;
      elem.cameraSelect.appendChild(option);
    });
    
    elem.cameraSelect.disabled = false;
    
    // 最初のカメラを自動選択
    if (state.cameras.length > 0) {
      elem.cameraSelect.value = state.cameras[0].deviceId;
      state.currentCameraId = state.cameras[0].deviceId;
    }
    
  } catch(e) {
    console.error('[Mirror] カメラ列挙エラー:', e);
  }
}

// ========== カメラ選択 ==========
elem.cameraSelect.addEventListener('change', (e) => {
  state.currentCameraId = e.target.value;
});

// ========== 送信開始 ==========
elem.startSendBtn.addEventListener('click', async () => {
  if (!state.currentCameraId) {
    alert('カメラを選択してください');
    return;
  }
  
  try {
    const stream = await navigator.mediaDevices.getUserMedia({
      video: { deviceId: { exact: state.currentCameraId } },
      audio: false
    });
    
    state.stream = stream;
    elem.senderVideo.srcObject = stream;
    
    elem.senderStatus.textContent = '再生中';
    elem.startSendBtn.disabled = true;
    elem.stopSendBtn.disabled = false;
    elem.cameraSelect.disabled = true;
    
    // ビデオメタデータ取得
    elem.senderVideo.onloadedmetadata = () => {
      elem.senderResolution.textContent = `${elem.senderVideo.videoWidth}x${elem.senderVideo.videoHeight}`;
    };
    
    // フレーム送信ループ開始
    state.isStreaming = true;
    state.frameStartTime = performance.now();
    sendFrames();
    
  } catch(e) {
    console.error('[Mirror] getUserMedia エラー:', e);
    elem.senderStatus.textContent = `エラー: ${e.name}`;
  }
});

// ========== 送信停止 ==========
elem.stopSendBtn.addEventListener('click', () => {
  state.isStreaming = false;
  
  if (state.stream) {
    state.stream.getTracks().forEach(t => t.stop());
    state.stream = null;
  }
  
  elem.senderVideo.srcObject = null;
  elem.senderStatus.textContent = '停止';
  elem.startSendBtn.disabled = false;
  elem.stopSendBtn.disabled = true;
  elem.cameraSelect.disabled = false;
});

// ========== フレーム送信 ==========
function sendFrames() {
  if (!state.isStreaming) return;
  
  const now = performance.now();
  const frameInterval = 1000 / state.maxFramesPerSecond;
  
  // フレームレート制限（15FPS以下に限定）
  if (now - state.lastFrameTime < frameInterval) {
    requestAnimationFrame(sendFrames);
    return;
  }
  
  state.lastFrameTime = now;
  
  // キャンバスに描画
  const canvas = document.createElement('canvas');
  canvas.width = elem.senderVideo.videoWidth;
  canvas.height = elem.senderVideo.videoHeight;
  const ctx = canvas.getContext('2d');
  
  ctx.drawImage(elem.senderVideo, 0, 0);
  
  // JPEG品質を下げて圧縮（0.6でさらに軽く）
  canvas.toBlob((blob) => {
    if (!blob || !state.ws || state.ws.readyState !== 1) {
      requestAnimationFrame(sendFrames);
      return;
    }
    
    blob.arrayBuffer().then((buffer) => {
      const timestamp = Date.now();
      const frameNum = state.senderFrameCount;
      
      // メッセージの構造：
      // [4 bytes: frameNum] [4 bytes: timestamp] [4 bytes: width] [4 bytes: height] [rest: JPEG data]
      const header = new ArrayBuffer(16);
      const view = new DataView(header);
      view.setUint32(0, frameNum, true);        // フレーム番号
      view.setUint32(4, timestamp, true);       // タイムスタンプ
      view.setUint32(8, canvas.width, true);    // 幅
      view.setUint32(12, canvas.height, true);  // 高さ
      
      const combined = new Uint8Array(header.byteLength + buffer.byteLength);
      combined.set(new Uint8Array(header), 0);
      combined.set(new Uint8Array(buffer), header.byteLength);
      
      // WebSocketバッファサイズをチェック（バッファオーバーフロー防止）
      if (state.ws.bufferedAmount > 500000) {
        console.warn('[Mirror] WSバッファがいっぱい、フレームスキップ');
        requestAnimationFrame(sendFrames);
        return;
      }
      
      state.ws.send(JSON.stringify({
        type: 'camera_stream',
        frameNum: frameNum,
        timestamp: timestamp,
        width: canvas.width,
        height: canvas.height,
        data: btoa(String.fromCharCode(...combined))
      }));

      // ROS BridgeへCompressedImageとして送信（クエリでwsaddr/wsport指定時）
      if (state.rosImagePub) {
        const jpegBytes = new Uint8Array(buffer);
        const msg = new ROSLIB.Message({
          format: 'jpeg',
          data: Array.from(jpegBytes)
        });
        try {
          state.rosImagePub.publish(msg);
        } catch (err) {
          console.warn('[Mirror] ROS publish失敗', err);
        }
      }
      
      state.senderFrameCount++;
      elem.senderFrameCount.textContent = state.senderFrameCount;
      
      const bytes = blob.size / 1024;
      elem.senderBytes.textContent = bytes < 1 ? `${(bytes * 1024).toFixed(0)} B` : `${bytes.toFixed(1)} KB`;
      
      // FPS計算
      const elapsed = (performance.now() - state.frameStartTime) / 1000;
      state.fps = state.senderFrameCount / elapsed;
      elem.fpsInfo.textContent = `FPS: ${state.fps.toFixed(1)}`;
      
      requestAnimationFrame(sendFrames);
    });
    
  }, 'image/jpeg', 0.6);
}

// ========== リモートフレーム処理 ==========
function handleRemoteFrame(frameData) {
  try {
    // 処理中なら古いフレームは無視（最新フレームを優先）
    if (state.isProcessing) {
      state.droppedFrames++;
      document.getElementById('droppedFrames').textContent = state.droppedFrames;
      return;
    }
    
    state.isProcessing = true;
    
    const canvas = elem.receiverCanvas;
    
    if (!state.ctx) {
      state.ctx = canvas.getContext('2d');
    }
    
    // バイナリデータをデコード
    const binaryStr = atob(frameData.data);
    const bytes = new Uint8Array(binaryStr.length);
    for (let i = 0; i < binaryStr.length; i++) {
      bytes[i] = binaryStr.charCodeAt(i);
    }
    
    // ヘッダーを解析（16バイト）
    const header = new DataView(bytes.buffer, 0, 16);
    const frameNum = header.getUint32(0, true);
    const timestamp = header.getUint32(4, true);
    const width = header.getUint32(8, true);
    const height = header.getUint32(12, true);
    
    // フレーム抜けをチェック
    if (frameNum > state.expectedFrameNum + 1) {
      const skip = frameNum - state.expectedFrameNum - 1;
      state.droppedFrames += skip;
      console.warn(`[Mirror] ${skip}フレーム抜け（${state.expectedFrameNum} → ${frameNum}）`);
    }
    state.expectedFrameNum = frameNum;
    
    // キャンバスサイズを変更する必要がある場合のみ変更（黒くなるのを防ぐ）
    if (canvas.width !== width || canvas.height !== height) {
      canvas.width = width;
      canvas.height = height;
    }
    
    // JPEGをイメージとして描画
    const jpegData = bytes.slice(16);
    const blob = new Blob([jpegData], { type: 'image/jpeg' });
    const url = URL.createObjectURL(blob);
    
    const img = new Image();
    img.onload = () => {
      // 最後の描画からの経過時間をチェック（古いフレームなら捨てる）
      if (frameNum < state.expectedFrameNum - 2) {
        console.warn('[Mirror] 古いフレーム、スキップ');
        URL.revokeObjectURL(url);
        state.isProcessing = false;
        state.droppedFrames++;
        return;
      }
      
      state.ctx.drawImage(img, 0, 0);
      URL.revokeObjectURL(url);
      
      state.receiverFrameCount++;
      elem.receiverFrameCount.textContent = state.receiverFrameCount;
      elem.receiverStatus.textContent = '受信中';
      elem.lastUpdateTime.textContent = new Date().toLocaleTimeString('ja-JP');
      
      // 遅延計算
      const latency = Date.now() - timestamp;
      elem.latencyMs.textContent = `${latency} ms`;
      elem.downloadImageBtn.disabled = false;
      
      document.getElementById('droppedFrames').textContent = state.droppedFrames;
      
      state.isProcessing = false;
    };
    
    img.onerror = () => {
      console.error('[Mirror] イメージデコード失敗 (frame#' + frameNum + ')');
      URL.revokeObjectURL(url);
      state.isProcessing = false;
      state.droppedFrames++;
    };
    
    img.onabort = () => {
      console.warn('[Mirror] イメージロード中止');
      URL.revokeObjectURL(url);
      state.isProcessing = false;
    };
    
    img.src = url;
    
  } catch(e) {
    console.error('[Mirror] フレーム処理エラー:', e);
    state.isProcessing = false;
  }
}

// ========== キャンバスクリア ==========
elem.clearCanvasBtn.addEventListener('click', () => {
  if (state.ctx) {
    state.ctx.fillStyle = '#000';
    state.ctx.fillRect(0, 0, elem.receiverCanvas.width, elem.receiverCanvas.height);
  }
});

// ========== 画像保存 ==========
elem.downloadImageBtn.addEventListener('click', () => {
  const link = document.createElement('a');
  link.href = elem.receiverCanvas.toDataURL('image/png');
  link.download = `mirror_${Date.now()}.png`;
  link.click();
});

// ========== 初期化 ==========
// キャンバスの初期サイズを設定（1280x720）
elem.receiverCanvas.width = 1280;
elem.receiverCanvas.height = 720;
const initCtx = elem.receiverCanvas.getContext('2d');
initCtx.fillStyle = '#000';
initCtx.fillRect(0, 0, 1280, 720);

connectWebSocket();
connectRosFromQuery();
// 事前にカメラ権限を明示要求しておく（初回アクセスでポップアップを確実に出すため）
async function ensureCameraPermission() {
  try {
    const tmp = await navigator.mediaDevices.getUserMedia({ video: true, audio: false });
    tmp.getTracks().forEach(t => t.stop());
    return true;
  } catch (err) {
    console.warn('[Mirror] カメラ権限取得に失敗:', err);
    alert('カメラの使用を許可してください');
    return false;
  }
}

const permOk = await ensureCameraPermission();
if (permOk) {
  await enumerateCameras();
} else {
  // 権限が取れないと列挙できないので、状態を表示しておく
  elem.senderStatus.textContent = '権限なし';
  elem.cameraSelect.disabled = true;
}

// ========== コントローラポーリング ==========
function formatVal(v) {
  if (v === null || v === undefined || Number.isNaN(v)) return '0.000';
  return Number(v).toFixed(3);
}

function updateControllerUI(gamepad) {
  // スティック（axes: 0,1 左 / 2,3 右 想定）
  const ax = gamepad.axes || [];
  elem.leftStick.textContent = `${formatVal(ax[0])} / ${formatVal(ax[1])}`;
  elem.rightStick.textContent = `${formatVal(ax[2])} / ${formatVal(ax[3])}`;
  
  // 位置・姿勢（対応デバイスのみ）
  const pose = gamepad.pose || gamepad.poseStatus || null;
  const pos = pose?.position;
  const ori = pose?.orientation;
  if (pos && pos.length === 3) {
    elem.posePos.textContent = `${formatVal(pos[0])}, ${formatVal(pos[1])}, ${formatVal(pos[2])}`;
  } else {
    elem.posePos.textContent = '-';
  }
  if (ori && ori.length === 4) {
    elem.poseOri.textContent = `${formatVal(ori[0])}, ${formatVal(ori[1])}, ${formatVal(ori[2])}, ${formatVal(ori[3])}`;
  } else {
    elem.poseOri.textContent = '-';
  }
  
  // ボタン一覧
  const buttons = gamepad.buttons || [];
  if (buttons.length === 0) {
    elem.buttonList.textContent = '(ボタンなし)';
  } else {
    const lines = buttons.map((b, idx) => {
      const val = typeof b === 'object' ? b.value : b;
      const pressed = typeof b === 'object' ? b.pressed : (b > 0.5);
      return `#${idx}: ${pressed ? 'ON ' : 'off'} (${formatVal(val)})`;
    });
    elem.buttonList.textContent = lines.join('\n');
  }
}

function pollControllers() {
  const pads = navigator.getGamepads ? navigator.getGamepads() : [];
  const active = Array.from(pads).filter(Boolean);
  elem.controllerCount.textContent = active.length;
  if (active.length === 0) {
    elem.leftStick.textContent = '0.000 / 0.000';
    elem.rightStick.textContent = '0.000 / 0.000';
    elem.posePos.textContent = '-';
    elem.poseOri.textContent = '-';
    elem.buttonList.textContent = '(未検出)';
  } else {
    // 先頭のコントローラを表示（Questなら右手が0になることが多い）
    updateControllerUI(active[0]);
  }
  state.controllerPollId = requestAnimationFrame(pollControllers);
}

pollControllers();

// ========== WebXR セッション（コントローラ情報用） ==========

function setXrStatus(text, cls = '') {
  elem.xrStatus.textContent = text;
  elem.xrStatus.className = `stat-value ${cls}`;
}

async function startXRSession() {
  if (!navigator.xr) {
    alert('このブラウザは WebXR に対応していません');
    return;
  }
  try {
    const supported = await navigator.xr.isSessionSupported('immersive-ar');
    if (!supported) {
      alert('immersive-ar がサポートされていません');
      return;
    }
    const session = await navigator.xr.requestSession('immersive-ar', {
      requiredFeatures: ['local-floor'],
      optionalFeatures: ['bounded-floor', 'hand-tracking']
    });
    state.xrSession = session;
    state.xrRefSpace = await session.requestReferenceSpace('local-floor');
    setXrStatus('XR: 接続中', 'status-ok');
    session.addEventListener('end', () => {
      setXrStatus('XR: 終了', 'status-warn');
      if (state.xrFrameHandle) {
        state.xrSession.cancelAnimationFrame(state.xrFrameHandle);
        state.xrFrameHandle = null;
      }
      state.xrSession = null;
      state.xrRefSpace = null;
    });
    state.xrFrameHandle = session.requestAnimationFrame(onXRFrame);
  } catch (err) {
    console.error('XR セッション開始失敗', err);
    alert('XR開始に失敗しました: ' + err.message);
    setXrStatus('XR: エラー', 'status-error');
  }
}

function onXRFrame(time, frame) {
  const session = frame.session;
  state.xrFrameHandle = session.requestAnimationFrame(onXRFrame);
  const sources = session.inputSources || [];
  elem.controllerCount.textContent = sources.length;
  if (!sources.length) {
    elem.leftStick.textContent = '0.000 / 0.000';
    elem.rightStick.textContent = '0.000 / 0.000';
    elem.posePos.textContent = '-';
    elem.poseOri.textContent = '-';
    elem.buttonList.textContent = '(未検出)';
    return;
  }
  // 先頭の inputSource を利用
  const src = sources[0];
  const gp = src.gamepad;
  if (gp) updateControllerUI(gp);
  // 位置・姿勢
  const space = src.gripSpace || src.targetRaySpace;
  if (space && state.xrRefSpace) {
    const pose = frame.getPose(space, state.xrRefSpace);
    if (pose) {
      const p = pose.transform.position;
      const o = pose.transform.orientation;
      elem.posePos.textContent = `${formatVal(p.x)}, ${formatVal(p.y)}, ${formatVal(p.z)}`;
      elem.poseOri.textContent = `${formatVal(o.x)}, ${formatVal(o.y)}, ${formatVal(o.z)}, ${formatVal(o.w)}`;
    }
  }
}

elem.startXRBtn.addEventListener('click', startXRSession);

})();
