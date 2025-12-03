// img.js

// === 画像用の ROS トピック定義 ===
const base_img_listener = new ROSLIB.Topic({
  ros: ros,
  name: '/base_camera/image_raw/compressed',
  messageType: 'sensor_msgs/CompressedImage' // ROS1
});

// 今どの購読が有効かを管理（今はほぼ base 用のフラグ）
var current_unsubscribe = undefined;

// ---------- base カメラ ----------

function ros_base_img_subscribe() {
  // 以前の購読があれば解除
  if (current_unsubscribe) {
    current_unsubscribe();
  }
  current_unsubscribe = ros_base_img_unsubscribe;

  console.log('subscribe: base');

  base_img_listener.subscribe(msg => {
    // msg.data は JPEG の base64
    let content = document.getElementById('rosimg');
    if (!content) {
      console.error('img#rosimg が見つかりません');
      return;
    }
    content.src = 'data:image/jpeg;base64,' + msg.data;
  });
}

function ros_base_img_unsubscribe() {
  console.log('unsubscribe: base');
  base_img_listener.unsubscribe();
}

// ---------- arm / hand（今は base を流用 or 何もしない） ----------

// まだ /arm_camera/... トピックが無いので、
// いったん base をそのまま表示するようにしておくパターン
function ros_arm_img_subscribe() {
  console.log('arm カメラは未定義なので base を表示します');
  ros_base_img_subscribe();
}
function ros_arm_img_unsubscribe() {
  ros_base_img_unsubscribe();
}

function ros_hand_img_subscribe() {
  console.log('hand カメラは未定義なので base を表示します');
  ros_base_img_subscribe();
}
function ros_hand_img_unsubscribe() {
  ros_base_img_unsubscribe();
}

// ページ読み込み時にデフォルトで base を表示
window.addEventListener('load', () => {
  ros_base_img_subscribe();
});