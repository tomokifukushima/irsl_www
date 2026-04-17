async function main_func() {
    // 初期化の入口。UI要素取得、ROS接続、WebXR開始準備をここでまとめて行う。
    const startButton = document.getElementById('startButton');//スタートボタンのidを取得
    const endButton = document.getElementById('endButton');//エンドボタンのidを取得
    const status = document.getElementById('status');//ステータス表示のidを取得
    const overlay = document.getElementById('xrOverlay');
    const controllerInfo = document.getElementById('controllerInfo');
    const canvas = document.getElementById('xrCanvas');//キャンバスの要素idを取得

    let xrSession = null;
    let xrRefSpace = null;
    let gl = null;
    let sessionMode = null;
    let lastStatusMessage = '';
    let domOverlayType = null;
    let ros = null;
    let rightPositionTopic = null;
    let leftPositionTopic = null;
    let rightButtonTopic = null;
    let leftButtonTopic = null;
    let lastPublishTime = 0;

    ///////////// ROS関連の基本設定 ////////////
    const publishIntervalMs = 200;//ROSにコントローラ情報を送る間隔（ミリ秒）
    const rightPositionTopicName = '/webxr/controller_state/right_con/position';//右コントローラ位置
    const leftPositionTopicName = '/webxr/controller_state/left_con/position';//左コントローラ位置
    const rightButtonTopicName = '/webxr/controller_state/right_con/button_a';//右Aボタン
    const leftButtonTopicName = '/webxr/controller_state/left_con/button_x';//左Xボタン
    ///////////////////////////////////////

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
        domOverlayType = null;
        overlay.hidden = true;
        startButton.disabled = false;//スタートボタンを有効にする
        endButton.disabled = true;//エンドボタンを無効にする
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
            setStatus(`準備中: ${rightPositionTopicName}, ${leftPositionTopicName}, ${rightButtonTopicName}, ${leftButtonTopicName}`);
        }
    }

    // このブラウザがAR/VRセッションを開始できるかを最初に判定する。
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
    setStatus(`準備完了: ${sessionMode}`);
}


window.addEventListener('DOMContentLoaded', main_func);//このページを読み込み終わったらこの関数を実行