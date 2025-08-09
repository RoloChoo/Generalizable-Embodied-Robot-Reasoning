// 필요한 헤더 파일들
#include <stdio.h>
#include <string.h>
#include <pthread.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdbool.h>

#include "Walk.hpp"
#include <webots/LED.hpp>
#include <webots/Accelerometer.hpp>
#include <webots/Gyro.hpp>
#include <webots/Motor.hpp>
#include <DARwInOPMotionManager.hpp>
#include <DARwInOPGaitManager.hpp>

#include <cstdlib>
#include <cmath>
#include <iostream>
#include <fstream>
#include <string>

using namespace webots;
using namespace managers;
using namespace std;

// ------------------------ 공용 유틸 ------------------------
static inline double clamp(double v, double mn, double mx) {
  if (mn > mx) return v;
  return v < mn ? mn : (v > mx ? mx : v);
}

static const int NECK_INDEX = 18;

// ------------------------ 전역 상태 ------------------------
// (기존) 로봇 상태
int gain = 128;
bool isWalking = false;
bool shouldStartWalk = false;
bool shouldStopWalk = false;
double xAmplitude = 0.0;
double yAmplitude = 0.0;
double aAmplitude = 0.0;

// (기존) LED 상태
bool eyeLedOn = true;
bool headLedOn = true;
bool shouldToggleEyeLed = false;
bool shouldToggleHeadLed = false;
int eyeLedColor = 0x00FF00;    // 기본 초록
int headLedColor = 0xFF0000;   // 기본 빨강

// (기존) LED 깜빡임
bool blinkMode = false;
bool shouldToggleBlink = false;
double lastBlinkTime = 0.0;
bool blinkState = true;

// (추가) Yaw 제어(“Soccer” 스타일)
double webYawDeg = -1.0;            // 웹에서 받은 Minecraft-like yaw [0..360], <0면 비활성
string turnDirection = "NONE";       // "LEFT" / "RIGHT" / "NONE"
double targetYawRad = 0.0;           // 목표 목 각(라디안)
double filteredYawRad = 0.0;         // 저속 누적 필터 결과
double prevTargetYawRad = 0.0;       // 이전 목표 라디안
const double YAW_FILTER_ALPHA = 0.015;

// (모터 한계값)
static double minMotorPositions[NMOTORS];
static double maxMotorPositions[NMOTORS];

pthread_mutex_t stateMutex = PTHREAD_MUTEX_INITIALIZER;

static const char *motorNames[NMOTORS] = {
  "ShoulderR","ShoulderL","ArmUpperR","ArmUpperL",
  "ArmLowerR","ArmLowerL","PelvYR","PelvYL",
  "PelvR","PelvL","LegUpperR","LegUpperL",
  "LegLowerR","LegLowerL","AnkleR","AnkleL",
  "FootR","FootL","Neck","Head"
};

// ------------------------ HTML 응답 ------------------------
char* load_html() {
  FILE* f = fopen("walk.html", "r");
  if (!f) {
    // 기본(내장) HTML: Yaw 슬라이더 + 회전 NONE + 기존 컨트롤 포함
    static const char html[] =
      "HTTP/1.1 200 OK\r\n"
      "Content-Type: text/html\r\n"
      "Access-Control-Allow-Origin: *\r\n"
      "Connection: close\r\n\r\n"
      "<!DOCTYPE html>"
      "<html><head><meta charset='utf-8'><title>Robot Control</title>"
      "<style>"
      "body { font-family: Arial, sans-serif; margin: 20px; background: #f5f5f5; }"
      "h1 { color: #333; text-align: center; }"
      "h3 { color: #555; margin-top: 20px; }"
      ".control-group { background: white; padding: 15px; margin: 10px 0; border-radius: 8px; box-shadow: 0 2px 4px rgba(0,0,0,0.1); }"
      "button { padding: 12px 16px; margin: 5px; border: none; border-radius: 4px; cursor: pointer; font-size: 14px; font-weight: bold; }"
      ".walk-btn { background: #4CAF50; color: white; }"
      ".stop-btn { background: #f44336; color: white; }"
      ".move-btn { background: #2196F3; color: white; }"
      ".led-btn { background: #FF9800; color: white; }"
      ".eye-led-btn { background: #9C27B0; color: white; }"
      ".blink-btn { background: #607D8B; color: white; }"
      ".yaw-btn { background: #795548; color: white; }"
      "button:hover { opacity: 0.9; transform: translateY(-1px); }"
      "#status { background: #e8f5e8; border: 1px solid #4CAF50; color: #2e7d32; }"
      ".row { display: flex; align-items: center; gap: 12px; flex-wrap: wrap; }"
      "input[type=range]{ width: 280px; }"
      ".mono { font-family: ui-monospace, SFMono-Regular, Menlo, Consolas, monospace; }"
      "</style>"
      "</head><body>"
      "<h1>🤖 DARwIn-OP Robot Control</h1>"

      "<div class='control-group'>"
      "<h3>🚶 Walking Control</h3>"
      "<button class='walk-btn' onclick=\"sendCommand('walk_start')\">▶️ Start Walking</button>"
      "<button class='stop-btn' onclick=\"sendCommand('walk_stop')\">⏹️ Stop Walking</button>"
      "</div>"

      "<div class='control-group'>"
      "<h3>🧭 Direction Control (Manual)</h3>"
      "<button class='move-btn' onclick=\"sendCommand('move_forward')\">⬆️ Forward</button>"
      "<button class='move-btn' onclick=\"sendCommand('move_backward')\">⬇️ Backward</button><br>"
      "<button class='move-btn' onclick=\"sendCommand('turn_left')\">⬅️ Turn Left</button>"
      "<button class='move-btn' onclick=\"sendCommand('turn_right')\">➡️ Turn Right</button>"
      "</div>"

      "<div class='control-group'>"
      "<h3>🧠 Head / Yaw Control (Original-Style Filter)</h3>"
      "<div class='row'>"
      "<label>Yaw: <span id='yawVal' class='mono'>-</span>°</label>"
      "<input id='yawRange' type='range' min='0' max='360' value='180' oninput='setYaw(this.value)'/>"
      "<button class='yaw-btn' onclick=\"setYaw(180)\">🎯 Center Head</button>"
      "<button class='yaw-btn' onclick=\"sendCommand('turn_none')\">🚫 Turn NONE</button>"
      "</div>"
      "<small>Tip: 슬라이더를 움직이면 Yaw 기반 제어가 활성화되고, 로봇 목(Neck)에 저속 누적 필터(α=0.015)가 적용됩니다.</small>"
      "</div>"

      "<div class='control-group'>"
      "<h3>💡 LED Control</h3>"
      "<button class='eye-led-btn' onclick=\"sendCommand('eye_led_toggle')\">👁️ Toggle Eye LED</button>"
      "<button class='led-btn' onclick=\"sendCommand('head_led_toggle')\">💡 Toggle Head LED</button><br>"
      "<button class='blink-btn' onclick=\"sendCommand('led_blink_toggle')\">✨ Toggle Blink Mode</button>"
      "</div>"

      "<div id='status' class='control-group'>🟢 Ready - Robot control interface loaded</div>"

      "<script>"
      "function sendCommand(cmd) {"
      "  const statusDiv = document.getElementById('status');"
      "  statusDiv.innerHTML = '🔄 Sending: ' + cmd;"
      "  statusDiv.style.background = '#fff3e0';"
      "  statusDiv.style.borderColor = '#FF9800';"
      "  statusDiv.style.color = '#e65100';"
      "  fetch('/?command=' + cmd)"
      "    .then(r => r.text())"
      "    .then(_ => {"
      "      statusDiv.innerHTML = '✅ Command sent: ' + cmd;"
      "      statusDiv.style.background = '#e8f5e8';"
      "      statusDiv.style.borderColor = '#4CAF50';"
      "      statusDiv.style.color = '#2e7d32';"
      "    })"
      "    .catch(err => {"
      "      statusDiv.innerHTML = '❌ Error: ' + err;"
      "      statusDiv.style.background = '#ffebee';"
      "      statusDiv.style.borderColor = '#f44336';"
      "      statusDiv.style.color = '#c62828';"
      "    });"
      "}"
      "function setYaw(val) {"
      "  document.getElementById('yawVal').textContent = val;"
      "  sendCommand('set_yaw&yaw=' + val);"
      "}"
      "</script>"
      "</body></html>";

    char* result = (char*)malloc(strlen(html) + 1);
    strcpy(result, html);
    return result;
  }

  fseek(f, 0, SEEK_END);
  long size = ftell(f);
  rewind(f);

  const char* header =
    "HTTP/1.1 200 OK\r\n"
    "Content-Type: text/html\r\n"
    "Access-Control-Allow-Origin: *\r\n"
    "Connection: close\r\n\r\n";
  char* html = (char*)malloc(strlen(header) + size + 1);
  strcpy(html, header);
  fread(html + strlen(header), 1, size, f);
  html[strlen(header) + size] = '\0';
  fclose(f);
  return html;
}

// ------------------------ 간단 HTTP 서버 ------------------------
void* server(void* arg) {
  (void)arg;

  printf("Server thread starting...\n");
  int s = socket(AF_INET, SOCK_STREAM, 0);
  if (s < 0) { printf("Socket creation failed!\n"); return NULL; }

  struct sockaddr_in addr;
  addr.sin_family = AF_INET;
  addr.sin_port = htons(8080);
  addr.sin_addr.s_addr = INADDR_ANY;

  int opt = 1;
  setsockopt(s, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

  if (bind(s, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
    printf("Bind failed! Port 8080 might be in use.\n");
    close(s);
    return NULL;
  }
  if (listen(s, 5) < 0) {
    printf("Listen failed!\n");
    close(s);
    return NULL;
  }
  printf("Server running on http://0.0.0.0:8080\n");

  while (1) {
    struct sockaddr_in client_addr;
    socklen_t client_len = sizeof(client_addr);
    int client = accept(s, (struct sockaddr*)&client_addr, &client_len);
    if (client < 0) { printf("Accept failed!\n"); continue; }

    printf("Client connected from %s\n", inet_ntoa(client_addr.sin_addr));

    char buf[1024];
    ssize_t bytes_read = recv(client, buf, sizeof(buf) - 1, 0);
    if (bytes_read <= 0) { close(client); continue; }

    buf[bytes_read] = '\0';
    char *get_line = strtok(buf, "\r\n");
    if (!get_line) { close(client); continue; }

    printf("Request: %s\n", get_line);

    pthread_mutex_lock(&stateMutex);

    // ---- 걷기 제어 ----
    if (strstr(get_line, "command=walk_start")) {
      shouldStartWalk = true;
      printf("Walk start\n");
    }
    else if (strstr(get_line, "command=walk_stop")) {
      shouldStopWalk = true;
      printf("Walk stop\n");
    }
    // ---- 수동 방향/속도 ----
    else if (strstr(get_line, "command=move_forward")) {
      xAmplitude = (xAmplitude == 1.0) ? 0.0 : 1.0;
      printf("Forward: x=%.1f\n", xAmplitude);
    }
    else if (strstr(get_line, "command=move_backward")) {
      xAmplitude = (xAmplitude == -1.0) ? 0.0 : -1.0;
      printf("Backward: x=%.1f\n", xAmplitude);
    }
    else if (strstr(get_line, "command=turn_left")) {
      // 수동 토글도 유지, 동시에 yaw 기반 turnDirection도 반영
      aAmplitude = (aAmplitude == 0.5) ? 0.0 : 0.5;
      turnDirection = "LEFT";
      printf("Turn LEFT (manual a=%.1f)\n", aAmplitude);
    }
    else if (strstr(get_line, "command=turn_right")) {
      aAmplitude = (aAmplitude == -0.5) ? 0.0 : -0.5;
      turnDirection = "RIGHT";
      printf("Turn RIGHT (manual a=%.1f)\n", aAmplitude);
    }
    else if (strstr(get_line, "command=turn_none")) {
      turnDirection = "NONE";
      aAmplitude = 0.0; // 수동 기준도 0으로
      printf("Turn NONE\n");
    }
    // ---- Yaw 설정 ----
    else if (strstr(get_line, "command=set_yaw")) {
      char *yawPos = strstr(get_line, "yaw=");
      if (yawPos) {
        webYawDeg = atof(yawPos + 4);
        if (webYawDeg < 0.0) webYawDeg = 0.0;
        if (webYawDeg > 360.0) webYawDeg = 360.0;
        printf("Set yaw: %.2f deg\n", webYawDeg);
      }
    }
    // ---- LED ----
    else if (strstr(get_line, "command=eye_led_toggle")) {
      shouldToggleEyeLed = true;
      printf("Eye LED toggle\n");
    }
    else if (strstr(get_line, "command=head_led_toggle")) {
      shouldToggleHeadLed = true;
      printf("Head LED toggle\n");
    }
    else if (strstr(get_line, "command=led_blink_toggle")) {
      shouldToggleBlink = true;
      printf("LED blink toggle\n");
    }

    pthread_mutex_unlock(&stateMutex);

    char* response = load_html();
    send(client, response, strlen(response), 0);
    free(response);
    close(client);
  }

  close(s);
  return NULL;
}

// ------------------------ Walk 클래스 ------------------------
Walk::Walk(): Robot() {
  mTimeStep = getBasicTimeStep();

  // LED
  mEyeLED  = getLED("EyeLed");
  mHeadLED = getLED("HeadLed");
  mEyeLED->set(eyeLedColor);
  mHeadLED->set(headLedColor);

  // 센서
  mAccelerometer = getAccelerometer("Accelerometer");
  mAccelerometer->enable(mTimeStep);
  getGyro("Gyro")->enable(mTimeStep);

  // 모터 준비 + 한계값 획득
  for (int i = 0; i < NMOTORS; i++) {
    mMotors[i] = getMotor(motorNames[i]);
    mMotors[i]->enablePosition(mTimeStep);
    // 위치 한계 (Neck clamp용)
    minMotorPositions[i] = mMotors[i]->getMinPosition();
    maxMotorPositions[i] = mMotors[i]->getMaxPosition();
  }

  keyboardEnable(mTimeStep);

  mMotionManager = new DARwInOPMotionManager(this);
  mGaitManager   = new DARwInOPGaitManager(this, "config.ini");
}

Walk::~Walk() {
  delete mMotionManager;
  delete mGaitManager;
}

void Walk::myStep() {
  if (step(mTimeStep) == -1) exit(EXIT_SUCCESS);
}

void Walk::wait(int ms) {
  double t0 = getTime(), dur = ms / 1000.0;
  while (getTime() < t0 + dur) myStep();
}

// ----- LED 상태 처리 -----
void Walk::updateLEDs() {
  if (shouldToggleEyeLed) {
    eyeLedOn = !eyeLedOn;
    shouldToggleEyeLed = false;
    printf("Eye LED %s\n", eyeLedOn ? "ON" : "OFF");
  }
  if (shouldToggleHeadLed) {
    headLedOn = !headLedOn;
    shouldToggleHeadLed = false;
    printf("Head LED %s\n", headLedOn ? "ON" : "OFF");
  }
  if (shouldToggleBlink) {
    blinkMode = !blinkMode;
    shouldToggleBlink = false;
    lastBlinkTime = getTime();
    printf("Blink %s\n", blinkMode ? "ON" : "OFF");
  }

  if (blinkMode) {
    double now = getTime();
    if (now - lastBlinkTime > 0.5) {
      blinkState = !blinkState;
      lastBlinkTime = now;
    }
    if (blinkState) {
      if (eyeLedOn)  mEyeLED->set(eyeLedColor);
      if (headLedOn) mHeadLED->set(headLedColor);
    } else {
      mEyeLED->set(0x000000);
      mHeadLED->set(0x000000);
    }
  } else {
    if (eyeLedOn)  mEyeLED->set(eyeLedColor);
    else           mEyeLED->set(0x000000);
    if (headLedOn) mHeadLED->set(headLedColor);
    else           mHeadLED->set(0x000000);
  }
}

// ----- 넘어짐 감지 -----
void Walk::checkIfFallen() {
  static int fup = 0;
  static int fdown = 0;
  static const double acc_tolerance = 80.0;
  static const double acc_step = 100;

  const double *acc = mAccelerometer->getValues();
  if (acc[1] < 512.0 - acc_tolerance) fup++; else fup = 0;
  if (acc[1] > 512.0 + acc_tolerance) fdown++; else fdown = 0;

  if (fup > acc_step) {
    mMotionManager->playPage(10); // f_up
    mMotionManager->playPage(9);  // init
    fup = 0;
  } else if (fdown > acc_step) {
    mMotionManager->playPage(11); // b_up
    mMotionManager->playPage(9);  // init
    fdown = 0;
  }
}

// ----- Yaw 변환 (Minecraft 0~360 -> Neck 라디안) -----
static inline double convertYawToNeckAngle(double yawDeg) {
  // 로봇 기준 남쪽(180도)을 정면으로 가정
  double robotBaseYaw = 180.0;
  double rel = yawDeg - robotBaseYaw;
  while (rel > 180.0)  rel -= 360.0;
  while (rel < -180.0) rel += 360.0;
  // 좌표계 보정: 부호 반전 후 라디안
  return -rel * M_PI / 180.0;
}

// ------------------------ 메인 루프 ------------------------
void Walk::run() {
  cout << "-------Walk example of DARwIn-OP with HTML + Yaw Filter-------" << endl;
  cout << "Web control:  http://localhost:8080" << endl;

  // 웹 서버 스레드 시작
  pthread_t serverThread;
  if (pthread_create(&serverThread, NULL, server, NULL) != 0) {
    cout << "Failed to create server thread!" << endl;
    return;
  }

  myStep();
  mMotionManager->playPage(9); // init/walkready
  wait(200);

  bool gaitStarted = false;

  while (true) {
    checkIfFallen();

    pthread_mutex_lock(&stateMutex);

    // 1) LED(토글/깜빡임) 기본 처리
    updateLEDs();

    // 2) Walk start/stop
    if (shouldStartWalk && !isWalking) {
      cout << "Starting gait..." << endl;
      mGaitManager->start();
      mGaitManager->step(mTimeStep);
      isWalking = true;
      gaitStarted = true;
      shouldStartWalk = false;
      cout << "Gait started." << endl;
    }
    if (shouldStopWalk && isWalking) {
      cout << "Stopping gait..." << endl;
      mGaitManager->stop();
      isWalking = false;
      gaitStarted = false;
      shouldStopWalk = false;
      xAmplitude = yAmplitude = aAmplitude = 0.0;
      cout << "Gait stopped." << endl;
    }

    // 3) 보행/회전 제어
    if (isWalking && gaitStarted) {
      bool yawMode = (webYawDeg >= 0.0);  // 슬라이더 한 번이라도 움직이면 활성화

      if (yawMode) {
        // --- (A) Yaw 기반: "원본 스타일" ---
        // 3-1. 목표/필터 Yaw 계산 + Neck 모터 적용
        double target = convertYawToNeckAngle(webYawDeg);
        targetYawRad = target;
        filteredYawRad = YAW_FILTER_ALPHA * targetYawRad + (1.0 - YAW_FILTER_ALPHA) * filteredYawRad;
        filteredYawRad = clamp(filteredYawRad, minMotorPositions[NECK_INDEX], maxMotorPositions[NECK_INDEX]);
        mMotors[NECK_INDEX]->setPosition(filteredYawRad);

        // 3-2. 회전 방향에 따른 보행 파라미터
        if (turnDirection == "LEFT") {
          if (!blinkMode && eyeLedOn) mEyeLED->set(0xFF9900); // 주황
          mGaitManager->setXAmplitude(0.5);          // 천천히 전진
          mGaitManager->setAAmplitude(filteredYawRad);// 목 각도만큼 회전
        } else if (turnDirection == "RIGHT") {
          if (!blinkMode && eyeLedOn) mEyeLED->set(0x9900FF); // 보라
          mGaitManager->setXAmplitude(0.5);
          mGaitManager->setAAmplitude(filteredYawRad);
        } else { // NONE
          if (!blinkMode && eyeLedOn) mEyeLED->set(0x00FF00); // 초록
          double yawDiff = fabs(targetYawRad - prevTargetYawRad);
          if (yawDiff < 0.1) {
            // 미세 변화: 빠르게 전진, 회전은 절반
            mGaitManager->setXAmplitude(1.0);
            mGaitManager->setAAmplitude(filteredYawRad * 0.5);
          } else {
            // 큰 변화: 천천히 전진, 회전 충분히
            mGaitManager->setXAmplitude(0.5);
            mGaitManager->setAAmplitude(filteredYawRad);
          }
        }

        mGaitManager->step(mTimeStep);
        prevTargetYawRad = targetYawRad;
      } else {
        // --- (B) 수동(기존 버튼) 모드 ---
        mGaitManager->setXAmplitude(xAmplitude);
        mGaitManager->setYAmplitude(yAmplitude);
        mGaitManager->setAAmplitude(aAmplitude);
        mGaitManager->step(mTimeStep);
      }
    }

    pthread_mutex_unlock(&stateMutex);
    myStep();
  }
}
