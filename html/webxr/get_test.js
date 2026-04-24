async function main_func() {
    // 初期化の入口。UI要素取得、ROS接続、WebXR開始準備をここでまとめて行う。
    const startButton = document.getElementById('startButton');//スタートボタンのidを取得
    const endButton = document.getElementById('endButton');//エンドボタンのidを取得
    const status = document.getElementById('status');//ステータス表示のidを取得
    const overlay = document.getElementById('xrOverlay');
    const controllerInfo = document.getElementById('controllerInfo');
    const cameraSelect = document.getElementById('cameraSelect');
    const startCameraButton = document.getElementById('startCameraButton');
    const stopCameraButton = document.getElementById('stopCameraButton');
    const cameraPreview = document.getElementById('cameraPreview');
    const cameraStatus = document.getElementById('cameraStatus');
    const cameraMetrics = document.getElementById('cameraMetrics');
    const canvas = document.getElementById('xrCanvas');//キャンバスの要素idを取得

    let xrSession = null;
    let xrRefSpace = null;
    let gl = null;
    let sessionMode = null;
    let lastStatusMessage = '';
    let domOverlayType = null;
    let ros = null;
    let xrGlBinding = null;
    let rightPositionTopic = null;
    let leftPositionTopic = null;
    let rightButtonTopic = null;
    let leftButtonTopic = null;
    let cameraImageTopic = null;
    let cameraSourceTopic = null;
    let lastPublishTime = 0;
    let cameraStream = null;
    let cameraFrameRequestId = null;
    let selectedCameraId = '';
    let lastCameraPublishTime = 0;
    let sentCameraFrames = 0;
    let rawCameraAccessRequested = false;
    let rawCameraAccessActive = false;
    let rawCameraAnnounced = false;
    let rawCameraReadback = null;
    let rawCameraDetectStartTime = 0;
    let rawCameraFallbackTriggered = false;
    let rawCameraLastSuccessTime = 0;
    let lastCameraStatusMessage = '';

    ///////////// ROS関連の基本設定 ////////////
    const publishIntervalMs = 200;//ROSにコントローラ情報を送る間隔（ミリ秒）
    const rightPositionTopicName = '/webxr/controller_state/right_con/position';//右コントローラ位置
    const leftPositionTopicName = '/webxr/controller_state/left_con/position';//左コントローラ位置
    const rightButtonTopicName = '/webxr/controller_state/right_con/button_a';//右Aボタン
    const leftButtonTopicName = '/webxr/controller_state/left_con/button_x';//左Xボタン
    const cameraImageTopicName = '/hmd/camera/compressed';//HMD外カメラ画像
    const cameraSourceTopicName = '/hmd/camera/source';//カメラ入力ソース種別
    const cameraFrameId = 'hmd_camera';
    const cameraPublishIntervalMs = 100;//カメラ送信間隔（10FPS）
    const cameraJpegQuality = 0.6;
    const rawCameraFallbackTimeoutMs = 3000;
    const rawCameraStallTimeoutMs = 1500;
    ///////////////////////////////////////

    function buildRosStamp() {
        const nowMs = Date.now();
        const secs = Math.floor(nowMs / 1000);
        const nsecs = Math.floor((nowMs % 1000) * 1000000);
        return { secs, nsecs };
    }

    function buildCameraHeader() {
        return {
            stamp: buildRosStamp(),
            frame_id: cameraFrameId
        };
    }

    function publishCameraSource(sourceName) {
        if (!cameraSourceTopic) {
            return;
        }
        cameraSourceTopic.publish(new ROSLIB.Message({
            data: sourceName
        }));
    }

    function supportsRawCameraAccess() {
        return Boolean(
            sessionMode === 'immersive-ar'
            && typeof XRWebGLBinding !== 'undefined'
            && typeof gl?.readPixels === 'function'
        );
    }

    function initializeRawCameraReadback() {
        if (!gl || rawCameraReadback) {
            return rawCameraReadback;
        }

        const vertexSource = `
            attribute vec2 a_position;
            attribute vec2 a_texCoord;
            varying vec2 v_texCoord;
            void main() {
                gl_Position = vec4(a_position, 0.0, 1.0);
                v_texCoord = a_texCoord;
            }
        `;
        const fragmentSource = `
            precision mediump float;
            varying vec2 v_texCoord;
            uniform sampler2D u_texture;
            void main() {
                gl_FragColor = texture2D(u_texture, vec2(v_texCoord.x, 1.0 - v_texCoord.y));
            }
        `;

        function compileShader(type, source) {
            const shader = gl.createShader(type);
            gl.shaderSource(shader, source);
            gl.compileShader(shader);
            if (!gl.getShaderParameter(shader, gl.COMPILE_STATUS)) {
                const info = gl.getShaderInfoLog(shader);
                gl.deleteShader(shader);
                throw new Error(`shader compile failed: ${info}`);
            }
            return shader;
        }

        const vertexShader = compileShader(gl.VERTEX_SHADER, vertexSource);
        const fragmentShader = compileShader(gl.FRAGMENT_SHADER, fragmentSource);
        const program = gl.createProgram();
        gl.attachShader(program, vertexShader);
        gl.attachShader(program, fragmentShader);
        gl.linkProgram(program);
        if (!gl.getProgramParameter(program, gl.LINK_STATUS)) {
            const info = gl.getProgramInfoLog(program);
            gl.deleteProgram(program);
            throw new Error(`program link failed: ${info}`);
        }
        gl.deleteShader(vertexShader);
        gl.deleteShader(fragmentShader);

        const buffer = gl.createBuffer();
        gl.bindBuffer(gl.ARRAY_BUFFER, buffer);
        gl.bufferData(gl.ARRAY_BUFFER, new Float32Array([
            -1, -1, 0, 0,
             1, -1, 1, 0,
            -1,  1, 0, 1,
             1,  1, 1, 1
        ]), gl.STATIC_DRAW);
        gl.bindBuffer(gl.ARRAY_BUFFER, null);

        const framebuffer = gl.createFramebuffer();
        const outputTexture = gl.createTexture();
        gl.bindTexture(gl.TEXTURE_2D, outputTexture);
        gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, gl.LINEAR);
        gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, gl.LINEAR);
        gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_S, gl.CLAMP_TO_EDGE);
        gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_T, gl.CLAMP_TO_EDGE);
        gl.bindTexture(gl.TEXTURE_2D, null);

        rawCameraReadback = {
            program,
            buffer,
            framebuffer,
            outputTexture,
            positionLocation: gl.getAttribLocation(program, 'a_position'),
            texCoordLocation: gl.getAttribLocation(program, 'a_texCoord'),
            textureLocation: gl.getUniformLocation(program, 'u_texture'),
            width: 0,
            height: 0,
            pixels: null,
            canvas: document.createElement('canvas'),
            context2d: null,
            imageData: null
        };
        rawCameraReadback.canvas.width = 1;
        rawCameraReadback.canvas.height = 1;
        rawCameraReadback.context2d = rawCameraReadback.canvas.getContext('2d', { willReadFrequently: true });
        return rawCameraReadback;
    }

    async function publishRawCameraTexture(view) {
        if (!cameraImageTopic || !xrGlBinding || !view?.camera) {
            return false;
        }

        const now = performance.now();
        if (now - lastCameraPublishTime < cameraPublishIntervalMs) {
            return true;
        }

        const readback = initializeRawCameraReadback();
        const cameraTexture = xrGlBinding.getCameraImage(view.camera);
        if (!cameraTexture) {
            return false;
        }

        const width = view.camera.width;
        const height = view.camera.height;
        if (!width || !height) {
            return false;
        }

        if (readback.width !== width || readback.height !== height) {
            readback.width = width;
            readback.height = height;
            readback.pixels = new Uint8Array(width * height * 4);
            readback.canvas.width = width;
            readback.canvas.height = height;
            readback.imageData = readback.context2d.createImageData(width, height);

            gl.bindTexture(gl.TEXTURE_2D, readback.outputTexture);
            gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGBA, width, height, 0, gl.RGBA, gl.UNSIGNED_BYTE, null);
            gl.bindTexture(gl.TEXTURE_2D, null);
        }

        const previousFramebuffer = gl.getParameter(gl.FRAMEBUFFER_BINDING);
        const previousProgram = gl.getParameter(gl.CURRENT_PROGRAM);
        const previousArrayBuffer = gl.getParameter(gl.ARRAY_BUFFER_BINDING);
        const previousActiveTexture = gl.getParameter(gl.ACTIVE_TEXTURE);
        const previousTexture = gl.getParameter(gl.TEXTURE_BINDING_2D);
        const previousViewport = gl.getParameter(gl.VIEWPORT);

        gl.bindFramebuffer(gl.FRAMEBUFFER, readback.framebuffer);
        gl.framebufferTexture2D(gl.FRAMEBUFFER, gl.COLOR_ATTACHMENT0, gl.TEXTURE_2D, readback.outputTexture, 0);
        if (gl.checkFramebufferStatus(gl.FRAMEBUFFER) !== gl.FRAMEBUFFER_COMPLETE) {
            gl.bindFramebuffer(gl.FRAMEBUFFER, previousFramebuffer);
            return false;
        }

        gl.viewport(0, 0, width, height);
        gl.useProgram(readback.program);
        gl.bindBuffer(gl.ARRAY_BUFFER, readback.buffer);
        gl.enableVertexAttribArray(readback.positionLocation);
        gl.vertexAttribPointer(readback.positionLocation, 2, gl.FLOAT, false, 16, 0);
        gl.enableVertexAttribArray(readback.texCoordLocation);
        gl.vertexAttribPointer(readback.texCoordLocation, 2, gl.FLOAT, false, 16, 8);
        gl.activeTexture(gl.TEXTURE0);
        gl.bindTexture(gl.TEXTURE_2D, cameraTexture);
        gl.uniform1i(readback.textureLocation, 0);
        gl.drawArrays(gl.TRIANGLE_STRIP, 0, 4);
        gl.readPixels(0, 0, width, height, gl.RGBA, gl.UNSIGNED_BYTE, readback.pixels);

        gl.disableVertexAttribArray(readback.positionLocation);
        gl.disableVertexAttribArray(readback.texCoordLocation);
        gl.bindTexture(gl.TEXTURE_2D, previousTexture);
        gl.activeTexture(previousActiveTexture);
        gl.bindBuffer(gl.ARRAY_BUFFER, previousArrayBuffer);
        gl.useProgram(previousProgram);
        gl.viewport(previousViewport[0], previousViewport[1], previousViewport[2], previousViewport[3]);
        gl.bindFramebuffer(gl.FRAMEBUFFER, previousFramebuffer);

        // readPixels は左下原点なので、2D canvas に上下反転して詰め直す。
        const rowStride = width * 4;
        for (let y = 0; y < height; y += 1) {
            const sourceOffset = (height - 1 - y) * rowStride;
            const targetOffset = y * rowStride;
            readback.imageData.data.set(readback.pixels.subarray(sourceOffset, sourceOffset + rowStride), targetOffset);
        }
        readback.context2d.putImageData(readback.imageData, 0, 0);

        const blob = await new Promise((resolve) => {
            readback.canvas.toBlob(resolve, 'image/jpeg', cameraJpegQuality);
        });
        if (!blob) {
            return false;
        }

        const buffer = await blob.arrayBuffer();
        cameraImageTopic.publish(new ROSLIB.Message({
            header: buildCameraHeader(),
            format: 'jpeg',
            data: Array.from(new Uint8Array(buffer))
        }));
        publishCameraSource('raw-camera-access');

        lastCameraPublishTime = now;
        rawCameraLastSuccessTime = now;
        sentCameraFrames += 1;
        rawCameraAccessActive = true;
        setCameraStatus('XR camera-access 送信中');
        setCameraMetrics(`送信先: ${cameraImageTopicName}\n入力: WebXR camera-access\n解像度: ${width}x${height}\n送信FPS: ${(1000 / cameraPublishIntervalMs).toFixed(1)}\n送信フレーム: ${sentCameraFrames}`);
        return true;
    }

    function setStatus(message) {//メッセージ表示用関数
        if (message === lastStatusMessage) {
            return;
        }
        lastStatusMessage = message;
        status.textContent = message;
        console.log(`[get_test] ${message}`);
    }

    function setControllerInfo(message) {
        controllerInfo.textContent = message;
    }

    function setCameraStatus(message) {
        if (message === lastCameraStatusMessage) {
            return;
        }
        lastCameraStatusMessage = message;
        cameraStatus.textContent = message;
    }

    function setCameraMetrics(message) {
        cameraMetrics.textContent = message;
    }

    function formatValue(value) {
        if (value === null || value === undefined || Number.isNaN(value)) {
            return '0.000';
        }
        return Number(value).toFixed(3);
    }

    function formatButton(button, index) {
        if (!button || typeof button !== 'object') {
            return `b${index}: off (0.000)`;
        }
        return `b${index}: ${button.pressed ? 'ON ' : 'off'} (${formatValue(button.value)})`;
    }

    function getControllerButtonState(buttons, handedness) {
        const targetIndex = handedness === 'right' ? 4 : 4;
        const targetLabel = handedness === 'right' ? 'A' : 'X';
        const button = Array.isArray(buttons) ? buttons[targetIndex] : null;
        return {
            name: targetLabel,
            pressed: Boolean(button?.pressed),
            touched: Boolean(button?.touched),
            value: Number(formatValue(button?.value))
        };
    }

    async function ensureCameraPermission() {
        try {
            const tempStream = await navigator.mediaDevices.getUserMedia({ video: true, audio: false });
            tempStream.getTracks().forEach((track) => track.stop());
            return true;
        } catch (error) {
            console.warn('[get_test] camera permission failed', error);
            setCameraStatus(`カメラ権限エラー: ${error.name}`);
            return false;
        }
    }

    async function enumerateCameras() {
        try {
            const devices = await navigator.mediaDevices.enumerateDevices();
            const cameras = devices.filter((device) => device.kind === 'videoinput');
            cameraSelect.innerHTML = '<option value="">カメラを選択...</option>';
            cameras.forEach((device, index) => {
                const option = document.createElement('option');
                option.value = device.deviceId;
                option.textContent = device.label || `カメラ ${index + 1}`;
                cameraSelect.appendChild(option);
            });

            if (!cameras.length) {
                setCameraStatus('利用できるカメラが見つかりません');
                return;
            }

            selectedCameraId = cameras[0].deviceId;
            cameraSelect.value = selectedCameraId;
            cameraSelect.disabled = false;
            startCameraButton.disabled = false;
            setCameraStatus(`カメラ準備完了: ${cameras.length} 台`);
        } catch (error) {
            console.error('[get_test] enumerate cameras failed', error);
            setCameraStatus(`カメラ列挙失敗: ${error.message}`);
        }
    }

    function stopCameraStream() {
        if (cameraFrameRequestId) {
            cancelAnimationFrame(cameraFrameRequestId);
            cameraFrameRequestId = null;
        }
        if (cameraStream) {
            cameraStream.getTracks().forEach((track) => track.stop());
            cameraStream = null;
        }
        cameraPreview.srcObject = null;
        rawCameraAccessActive = false;
        startCameraButton.disabled = rawCameraAnnounced || !selectedCameraId;
        stopCameraButton.disabled = true;
        cameraSelect.disabled = rawCameraAnnounced;
        setCameraStatus('カメラ停止');
    }

    function publishCameraFrame() {
        if (!cameraStream || !cameraImageTopic || !cameraPreview.videoWidth || !cameraPreview.videoHeight) {
            cameraFrameRequestId = requestAnimationFrame(publishCameraFrame);
            return;
        }

        const now = performance.now();
        if (now - lastCameraPublishTime < cameraPublishIntervalMs) {
            cameraFrameRequestId = requestAnimationFrame(publishCameraFrame);
            return;
        }
        lastCameraPublishTime = now;

        const frameCanvas = document.createElement('canvas');
        frameCanvas.width = cameraPreview.videoWidth;
        frameCanvas.height = cameraPreview.videoHeight;
        const frameContext = frameCanvas.getContext('2d');
        frameContext.drawImage(cameraPreview, 0, 0);

        frameCanvas.toBlob(async (blob) => {
            if (!blob || !cameraImageTopic || !cameraStream) {
                return;
            }
            const buffer = await blob.arrayBuffer();
            const msg = new ROSLIB.Message({
                header: buildCameraHeader(),
                format: 'jpeg',
                data: Array.from(new Uint8Array(buffer))
            });
            cameraImageTopic.publish(msg);
            publishCameraSource('getusermedia');

            sentCameraFrames += 1;
            setCameraMetrics(`送信先: ${cameraImageTopicName}\n解像度: ${frameCanvas.width}x${frameCanvas.height}\n送信FPS: ${(1000 / cameraPublishIntervalMs).toFixed(1)}\n送信フレーム: ${sentCameraFrames}`);
        }, 'image/jpeg', cameraJpegQuality);

        cameraFrameRequestId = requestAnimationFrame(publishCameraFrame);
    }

    async function startCameraStream() {
        if (!selectedCameraId) {
            setCameraStatus('カメラを選択してください');
            return;
        }
        if (!cameraImageTopic) {
            setCameraStatus('ROS 未接続: 画像 topic を作成できません');
            return;
        }
        try {
            stopCameraStream();
            setCameraStatus('カメラ開始中...');
            rawCameraAccessActive = false;
            cameraStream = await navigator.mediaDevices.getUserMedia({
                video: {
                    deviceId: { exact: selectedCameraId }
                },
                audio: false
            });
            cameraPreview.srcObject = cameraStream;
            cameraSelect.disabled = true;
            startCameraButton.disabled = true;
            stopCameraButton.disabled = false;
            sentCameraFrames = 0;
            setCameraStatus('カメラ送信中');
            publishCameraFrame();
        } catch (error) {
            console.error('[get_test] start camera failed', error);
            setCameraStatus(`カメラ開始失敗: ${error.name}`);
            startCameraButton.disabled = !selectedCameraId;
            stopCameraButton.disabled = true;
        }
    }

    // WebXRの生データを、画面表示とROS送信の両方で使う共通JSONにまとめる。
    function buildControllerPayload(frame, session) {
        const sources = Array.from(session.inputSources || []);
        return {
            sessionMode,
            overlay: domOverlayType,
            sourceCount: sources.length,
            timestamp: new Date().toISOString(),
            controllers: sources.map((source, index) => {
                const gamepad = source.gamepad;
                const space = source.gripSpace || source.targetRaySpace;
                const pose = space && xrRefSpace ? frame.getPose(space, xrRefSpace) : null;
                const position = pose ? pose.transform.position : null;
                const orientation = pose ? pose.transform.orientation : null;

                return {
                    index,
                    handedness: source.handedness || 'unknown',
                    targetRayMode: source.targetRayMode,
                    profiles: Array.from(source.profiles || []),
                    axes: Array.from(gamepad?.axes || []).map((value) => Number(formatValue(value))),
                    buttons: Array.from(gamepad?.buttons || []).map((button, buttonIndex) => ({
                        index: buttonIndex,
                        pressed: Boolean(button?.pressed),
                        touched: Boolean(button?.touched),
                        value: Number(formatValue(button?.value))
                    })),
                    primaryButton: getControllerButtonState(gamepad?.buttons, source.handedness || 'unknown'),
                    position: position ? {
                        x: Number(formatValue(position.x)),
                        y: Number(formatValue(position.y)),
                        z: Number(formatValue(position.z))
                    } : null,
                    orientation: orientation ? {
                        x: Number(formatValue(orientation.x)),
                        y: Number(formatValue(orientation.y)),
                        z: Number(formatValue(orientation.z)),
                        w: Number(formatValue(orientation.w))
                    } : null
                };
            })
        };
    }

    // 取得したコントローラ情報を人が読みやすい文字列に整形して、画面表示に使う。
    function buildControllerText(frame, session) {
        const payload = buildControllerPayload(frame, session);
        if (!payload.controllers.length) {
            return 'コントローラ未検出';
        }

        return payload.controllers.map((controller) => {
            const lines = [];
            const profiles = controller.profiles.length ? controller.profiles.join(', ') : '(none)';
            lines.push(`source ${controller.index}`);
            lines.push(`handedness: ${controller.handedness}`);
            lines.push(`targetRayMode: ${controller.targetRayMode}`);
            lines.push(`profiles: ${profiles}`);

            if (controller.axes.length || controller.buttons.length) {
                lines.push(`axes: ${controller.axes.length ? controller.axes.map((value) => formatValue(value)).join(', ') : '(none)'}`);
                const buttons = controller.buttons.map((button) => formatButton(button, button.index));
                lines.push(`buttons: ${buttons.length ? buttons.join(' | ') : '(none)'}`);
            } else {
                lines.push('gamepad: unavailable');
            }

            if (controller.position && controller.orientation) {
                lines.push(`pos: ${formatValue(controller.position.x)}, ${formatValue(controller.position.y)}, ${formatValue(controller.position.z)}`);
                lines.push(`rot: ${formatValue(controller.orientation.x)}, ${formatValue(controller.orientation.y)}, ${formatValue(controller.orientation.z)}, ${formatValue(controller.orientation.w)}`);
            } else {
                lines.push('pose: unavailable');
            }

            lines.push(`${controller.primaryButton.name}: ${controller.primaryButton.pressed ? 'pressed' : 'released'}`);

            return lines.join('\n');
        }).join('\n\n');
    }

    // ROS送信用。publish間隔を制限しつつ、JSON文字列をstd_msgs/Stringへ詰めて送る。
    function publishControllerState(frame, session) {
        if (!rightPositionTopic && !leftPositionTopic && !rightButtonTopic && !leftButtonTopic) {
            return;
        }

        const now = performance.now();
        if (now - lastPublishTime < publishIntervalMs) {
            return;
        }
        lastPublishTime = now;

        const payload = buildControllerPayload(frame, session);
        const rightController = payload.controllers.find((controller) => controller.handedness === 'right');
        const leftController = payload.controllers.find((controller) => controller.handedness === 'left');

        if (rightController) {
            if (rightController.position && rightPositionTopic) {
                rightPositionTopic.publish(new ROSLIB.Message({
                    x: rightController.position.x,
                    y: rightController.position.y,
                    z: rightController.position.z
                }));
            }
            if (rightButtonTopic) {
                rightButtonTopic.publish(new ROSLIB.Message({
                    data: rightController.primaryButton.pressed
                }));
            }
        }

        if (leftController) {
            if (leftController.position && leftPositionTopic) {
                leftPositionTopic.publish(new ROSLIB.Message({
                    x: leftController.position.x,
                    y: leftController.position.y,
                    z: leftController.position.z
                }));
            }
            if (leftButtonTopic) {
                leftButtonTopic.publish(new ROSLIB.Message({
                    data: leftController.primaryButton.pressed
                }));
            }
        }
    }

    async function detectSessionMode() {//このブラウザがVRに対応しているかどうかを調べる関数
        if (!navigator.xr) {
        return null;
        }
        if (await navigator.xr.isSessionSupported('immersive-ar')) {
        return 'immersive-ar';
        }
        if (await navigator.xr.isSessionSupported('immersive-vr')) {
        return 'immersive-vr';
        }
        return null;
    }

    function onSessionEnded() {//セッション終了時の処理
        xrSession = null;
        xrRefSpace = null;
        xrGlBinding = null;
        domOverlayType = null;
        rawCameraAccessRequested = false;
        rawCameraAccessActive = false;
        rawCameraAnnounced = false;
        rawCameraDetectStartTime = 0;
        rawCameraFallbackTriggered = false;
        rawCameraLastSuccessTime = 0;
        overlay.hidden = true;
        startButton.disabled = false;//スタートボタンを有効にする
        endButton.disabled = true;//エンドボタンを無効にする
        startCameraButton.disabled = !selectedCameraId;
        stopCameraButton.disabled = !cameraStream;
        cameraSelect.disabled = Boolean(cameraStream);
        setControllerInfo('コントローラ待機中...');
        setStatus('セッション終了');
    }

    // XRセッション中のメインループ。描画更新、表示更新、ROS publish を毎フレーム回す。
    function onXRFrame(time, frame) {
        const session = frame.session;
        session.requestAnimationFrame(onXRFrame);

        const baseLayer = session.renderState.baseLayer;
        gl.bindFramebuffer(gl.FRAMEBUFFER, baseLayer.framebuffer);

        const t = time * 0.001;
        gl.clearColor(0.08, 0.12 + 0.08 * Math.sin(t), 0.2 + 0.08 * Math.cos(t), 1.0);
        gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);

        const pose = xrRefSpace ? frame.getViewerPose(xrRefSpace) : null;
        if (pose) {
            setStatus(`セッション中: ${sessionMode} / view=${pose.views.length} / overlay=${domOverlayType || 'none'}`);
            if (rawCameraAccessRequested && pose.views.length) {
                const rawCameraView = pose.views.find((view) => view.camera);
                if (rawCameraView) {
                    if (rawCameraFallbackTriggered) {
                        setCameraStatus('getUserMedia フォールバック中（raw camera-access は使用しません）');
                    } else {
                    if (!rawCameraAnnounced) {
                        if (cameraStream) {
                            stopCameraStream();
                        }
                        rawCameraAnnounced = true;
                        rawCameraLastSuccessTime = performance.now();
                        startCameraButton.disabled = true;
                        stopCameraButton.disabled = true;
                        cameraSelect.disabled = true;
                        setCameraStatus('XR camera-access を検出しました');
                    }
                    publishRawCameraTexture(rawCameraView).catch((error) => {
                        console.error('[get_test] raw camera publish failed', error);
                        setCameraStatus(`XR camera-access 失敗: ${error.message}`);
                    });
                    const rawStalledMs = performance.now() - rawCameraLastSuccessTime;
                    if (!cameraStream && rawCameraAnnounced && rawStalledMs >= rawCameraStallTimeoutMs) {
                        rawCameraFallbackTriggered = true;
                        setCameraStatus('XR raw camera が停止したため getUserMedia に自動フォールバックします');
                        startCameraStream();
                    }
                    }
                } else if (rawCameraAccessRequested && !rawCameraAnnounced) {
                    const elapsed = performance.now() - rawCameraDetectStartTime;
                    if (!rawCameraFallbackTriggered && elapsed >= rawCameraFallbackTimeoutMs) {
                        rawCameraFallbackTriggered = true;
                        setCameraStatus('XR camera-access 未取得のため getUserMedia に自動フォールバックします');
                        startCameraStream();
                    } else if (!rawCameraFallbackTriggered) {
                        setCameraStatus('XR camera-access 要求中: view.camera 待機');
                    }
                }
            }
        }
        setControllerInfo(buildControllerText(frame, session));//コントローラ情報を画面表示用テキストに変換して表示
        publishControllerState(frame, session);//ROSにコントローラ情報を送る
    }

    async function startSession() {
        try {
        setStatus('セッション開始中...');
        // ここで immersive-ar / immersive-vr セッションを開始し、overlayも要求する。
        const sessionInit = {
            optionalFeatures: ['local-floor', 'bounded-floor', 'hand-tracking', 'dom-overlay']
        };
        rawCameraAccessRequested = sessionMode === 'immersive-ar';
        rawCameraAccessActive = false;
        rawCameraAnnounced = false;
        rawCameraDetectStartTime = performance.now();
        rawCameraFallbackTriggered = false;
        rawCameraLastSuccessTime = 0;
        if (rawCameraAccessRequested) {
            sessionInit.optionalFeatures.push('camera-access');
        }
        if (overlay) {
            sessionInit.domOverlay = { root: overlay };
        }
        xrSession = await navigator.xr.requestSession(sessionMode, sessionInit);

        gl = canvas.getContext('webgl', { xrCompatible: true, alpha: false });
        if (!gl) {
            throw new Error('WebGL コンテキストを作成できません');
        }
        if (gl.makeXRCompatible) {
            await gl.makeXRCompatible();
        }
        xrGlBinding = supportsRawCameraAccess() ? new XRWebGLBinding(xrSession, gl) : null;

        xrSession.updateRenderState({
            baseLayer: new XRWebGLLayer(xrSession, gl)
        });

        xrRefSpace = await xrSession.requestReferenceSpace('local-floor').catch(() => xrSession.requestReferenceSpace('local'));
        xrSession.addEventListener('end', onSessionEnded);
        xrSession.addEventListener('inputsourceschange', () => {
            setControllerInfo('コントローラ情報を更新中...');
        });

        domOverlayType = xrSession.domOverlayState?.type || null;
        overlay.hidden = false;
        setControllerInfo('コントローラ情報を取得中...');

        startButton.disabled = true;
        endButton.disabled = false;
        if (rawCameraAccessRequested) {
            setCameraStatus('XR camera-access を試行します。未対応なら通常カメラ操作に戻ります');
        }
        setStatus(`セッション開始: ${sessionMode} / overlay=${domOverlayType || 'none'}`);
        xrSession.requestAnimationFrame(onXRFrame);
        } catch (error) {
        console.error('[get_test] session start failed', error);
        overlay.hidden = true;
        setStatus(`開始失敗: ${error.message}`);
        startButton.disabled = false;
        endButton.disabled = true;
        }
    }

    startButton.addEventListener('click', startSession);
    endButton.addEventListener('click', () => xrSession?.end());
    cameraSelect.addEventListener('change', (event) => {
        selectedCameraId = event.target.value;
        startCameraButton.disabled = !selectedCameraId;
    });
    startCameraButton.addEventListener('click', startCameraStream);
    stopCameraButton.addEventListener('click', stopCameraStream);

    // URLパラメータから rosbridge に接続し、publish先 topic をここで作る。
    if (typeof ros_start === 'function' && typeof ROSLIB !== 'undefined') {
        ros = ros_start();
        if (ros) {
            rightPositionTopic = new ROSLIB.Topic({
                ros,
                name: rightPositionTopicName,
                messageType: 'geometry_msgs/Point'
            });
            leftPositionTopic = new ROSLIB.Topic({
                ros,
                name: leftPositionTopicName,
                messageType: 'geometry_msgs/Point'
            });
            rightButtonTopic = new ROSLIB.Topic({
                ros,
                name: rightButtonTopicName,
                messageType: 'std_msgs/Bool'
            });
            leftButtonTopic = new ROSLIB.Topic({
                ros,
                name: leftButtonTopicName,
                messageType: 'std_msgs/Bool'
            });
            cameraImageTopic = new ROSLIB.Topic({
                ros,
                name: cameraImageTopicName,
                messageType: 'sensor_msgs/CompressedImage'
            });
            cameraSourceTopic = new ROSLIB.Topic({
                ros,
                name: cameraSourceTopicName,
                messageType: 'std_msgs/String'
            });
            setStatus(`準備中: ${rightPositionTopicName}, ${leftPositionTopicName}, ${rightButtonTopicName}, ${leftButtonTopicName}`);
            setCameraMetrics(`送信先: ${cameraImageTopicName}\nsource送信先: ${cameraSourceTopicName}`);
        }
    }

    // このブラウザがAR/VRセッションを開始できるかを最初に判定する。
    if (navigator.mediaDevices?.getUserMedia) {
        const cameraPermissionOk = await ensureCameraPermission();
        if (cameraPermissionOk) {
            await enumerateCameras();
        }
    } else {
        setCameraStatus('MediaDevices API 非対応です');
    }

    sessionMode = await detectSessionMode();
    overlay.addEventListener('beforexrselect', (event) => event.preventDefault());
    if (!sessionMode) {
        startButton.textContent = '未対応';
        startButton.disabled = true;
        setStatus('WebXR 非対応です。Meta Quest Browser + HTTPS で開いてください。');
        return;
    }

    startButton.textContent = 'セッション開始';
    startButton.disabled = false;
    if (sessionMode === 'immersive-ar' && typeof XRWebGLBinding !== 'undefined') {
        setCameraStatus('camera-access 対応候補あり。XR開始時に raw camera を試します');
    }
    setStatus(`準備完了: ${sessionMode}`);
}


window.addEventListener('DOMContentLoaded', main_func);//このページを読み込み終わったらこの関数を実行