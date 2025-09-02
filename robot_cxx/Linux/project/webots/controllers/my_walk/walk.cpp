// 필요한 헤더 파일들
#include <stdio.h>
#include <string.h>
#include <pthread.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdbool.h>
#include <signal.h>
#include <libgen.h>
#include <limits.h>

// C++ 헤더들은 extern "C" 밖에 위치
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

// DARwIn-OP Linux 카메라 및 MJPEG 스트리밍을 위한 C 헤더만 포함
extern "C" {
    // C 함수들만 여기에 선언
    int darwin_camera_init(const char* ini_path);
    int darwin_camera_capture_and_stream(void);
    void darwin_camera_cleanup(void);
}

using namespace webots;
using namespace managers;
using namespace std;

// ------------------------ 공용 유틸 ------------------------
static inline double clamp(double v, double mn, double mx) {
  if (mn > mx) return v;
  return v < mn ? mn : (v > mx ? mx : v);
}

static const int NECK_INDEX = 18;
#define INI_FILE_PATH "../../../Data/config.ini"

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

// (추가) Yaw 제어("Soccer" 스타일)
double webYawDeg = -1.0;            // 웹에서 받은 Minecraft-like yaw [0..360], <0면 비활성
string turnDirection = "NONE";       // "LEFT" / "RIGHT" / "NONE"
double targetYawRad = 0.0;           // 목표 목 각(라디안)
double filteredYawRad = 0.0;         // 저속 누적 필터 결과
double prevTargetYawRad = 0.0;       // 이전 목표 라디안
const double YAW_FILTER_ALPHA = 0.015;

// (카메라 스트리밍 상태)
bool cameraEnabled = false;
bool shouldToggleCamera = false;
void* streamer = NULL;  // mjpg_streamer 포인터를 void*로 처리
void* rgb_output = NULL;  // Image 포인터를 void*로 처리  
void* ini = NULL;  // minIni 포인터를 void*로 처리

// DARwIn-OP Camera 크기 상수 정의
static const int DARWIN_CAMERA_WIDTH = 320;
static const int DARWIN_CAMERA_HEIGHT = 240;

// (모터 한계값)
static double minMotorPositions[NMOTORS];
static double maxMotorPositions[NMOTORS];

pthread_mutex_t stateMutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t cameraMutex = PTHREAD_MUTEX_INITIALIZER;
pthread_t cameraThread;

static const char *motorNames[NMOTORS] = {
  "ShoulderR","ShoulderL","ArmUpperR","ArmUpperL",
  "ArmLowerR","ArmLowerL","PelvYR","PelvYL",
  "PelvR","PelvL","LegUpperR","LegUpperL",
  "LegLowerR","LegLowerL","AnkleR","AnkleL",
  "FootR","FootL","Neck","Head"
};

// ------------------------ 유틸리티 함수 ------------------------
void change_current_dir()
{
    char exepath[1024] = {0};
    if(readlink("/proc/self/exe", exepath, sizeof(exepath)) != -1)
    {
        if(chdir(dirname(exepath)))
            fprintf(stderr, "chdir error!! \n");
    }
}

void sighandler(int sig)
{
    printf("Signal %d received, cleaning up...\n", sig);
    // 포인터들을 안전하게 해제
    if (streamer) {
        streamer = NULL;
    }
    if (rgb_output) {
        rgb_output = NULL;
    }
    if (ini) {
        ini = NULL;
    }
    exit(0);
}

// ------------------------ 카메라 관련 구현 ------------------------
// 구현부를 extern "C" 밖으로 이동
static void* camera_ini = NULL;
static void* camera_image = NULL;
static void* camera_streamer = NULL;

// C 함수들의 실제 구현
int darwin_camera_init(const char* ini_path) {
    // 구현은 카메라 스레드에서 직접 수행
    return 0;
}

int darwin_camera_capture_and_stream(void) {
    // 구현은 카메라 스레드에서 직접 수행  
    return 0;
}

void darwin_camera_cleanup(void) {
    // 정리 작업
    camera_ini = NULL;
    camera_image = NULL;
    camera_streamer = NULL;
}

// ------------------------ MJPEG 카메라 스트리밍 스레드 ------------------------
void* camera_streaming_thread(void* arg) {
    (void)arg;
    
    printf("Camera streaming thread starting...\n");
    
    // 간단한 더미 구현 (실제 DARwIn-OP 카메라 코드는 별도 구현 필요)
    while(1) {
        pthread_mutex_lock(&cameraMutex);
        bool enabled = cameraEnabled;
        pthread_mutex_unlock(&cameraMutex);
        
        if (enabled) {
            printf("Camera streaming active...\n");
            // 실제 카메라 캡처 및 스트리밍 코드는 여기에 구현
            // TODO: DARwIn-OP LinuxCamera 및 mjpg_streamer 연동
        }
        
        // CPU 사용량 제한
        usleep(100000); // 100ms 대기
    }
    
    return NULL;
}

// ------------------------ MJPEG 스트림 응답 함수 ------------------------
void send_mjpeg_stream(int client) {
    // MJPEG 스트리밍 헤더 전송
    const char* mjpeg_header = 
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: multipart/x-mixed-replace; boundary=--myboundary\r\n"
        "Cache-Control: no-cache\r\n"
        "Pragma: no-cache\r\n"
        "Connection: close\r\n"
        "\r\n";
    
    send(client, mjpeg_header, strlen(mjpeg_header), 0);
    
    // 더미 JPEG 이미지 데이터 (실제로는 카메라에서 캡처한 데이터 사용)
    // 320x240 크기의 간단한 테스트 패턴 JPEG
    const unsigned char dummy_jpeg[] = {
        0xFF, 0xD8, 0xFF, 0xE0, 0x00, 0x10, 0x4A, 0x46, 0x49, 0x46, 0x00, 0x01,
        0x01, 0x01, 0x00, 0x48, 0x00, 0x48, 0x00, 0x00, 0xFF, 0xDB, 0x00, 0x43,
        // ... (실제 JPEG 데이터는 카메라에서 가져와야 함)
        0xFF, 0xD9  // JPEG 끝
    };
    
    int frame_count = 0;
    while(cameraEnabled && frame_count < 100) {  // 최대 100프레임 또는 카메라 비활성화까지
        // MJPEG 프레임 경계
        const char* frame_header = 
            "--myboundary\r\n"
            "Content-Type: image/jpeg\r\n"
            "Content-Length: %d\r\n"
            "\r\n";
        
        char header_buf[256];
        snprintf(header_buf, sizeof(header_buf), frame_header, (int)sizeof(dummy_jpeg));
        
        if (send(client, header_buf, strlen(header_buf), 0) < 0) break;
        if (send(client, dummy_jpeg, sizeof(dummy_jpeg), 0) < 0) break;
        if (send(client, "\r\n", 2, 0) < 0) break;
        
        frame_count++;
        usleep(33333); // ~30 FPS (33.33ms)
        
        // 카메라 상태 재확인
        pthread_mutex_lock(&cameraMutex);
        bool still_enabled = cameraEnabled;
        pthread_mutex_unlock(&cameraMutex);
        
        if (!still_enabled) break;
    }
    
    printf("MJPEG stream ended (frames sent: %d)\n", frame_count);
}

// ------------------------ HTML 응답 (8080 포트 통합) ------------------------
char* load_html() {
  FILE* f = fopen("walk.html", "r");
  if (!f) {
    // 8080 포트로 통합된 HTML
    static const char html[] =
      "HTTP/1.1 200 OK\r\n"
      "Content-Type: text/html\r\n"
      "Access-Control-Allow-Origin: *\r\n"
      "Connection: close\r\n\r\n"
      "<!DOCTYPE html>"
      "<html><head><meta charset='utf-8'><title>DARwIn-OP Robot Control with MJPEG Camera</title>"
      "<style>"
      "body { font-family: Arial, sans-serif; margin: 20px; background: #f5f5f5; }"
      "h1 { color: #333; text-align: center; }"
      "h3 { color: #555; margin-top: 20px; }"
      ".control-group { background: white; padding: 15px; margin: 10px 0; border-radius: 8px; box-shadow: 0 2px 4px rgba(0,0,0,0.1); }"
      ".camera-group { background: white; padding: 15px; margin: 10px 0; border-radius: 8px; box-shadow: 0 2px 4px rgba(0,0,0,0.1); text-align: center; }"
      "button { padding: 12px 16px; margin: 5px; border: none; border-radius: 4px; cursor: pointer; font-size: 14px; font-weight: bold; }"
      ".walk-btn { background: #4CAF50; color: white; }"
      ".stop-btn { background: #f44336; color: white; }"
      ".move-btn { background: #2196F3; color: white; }"
      ".led-btn { background: #FF9800; color: white; }"
      ".eye-led-btn { background: #9C27B0; color: white; }"
      ".blink-btn { background: #607D8B; color: white; }"
      ".yaw-btn { background: #795548; color: white; }"
      ".camera-btn { background: #E91E63; color: white; }"
      "button:hover { opacity: 0.9; transform: translateY(-1px); }"
      "#status { background: #e8f5e8; border: 1px solid #4CAF50; color: #2e7d32; }"
      ".row { display: flex; align-items: center; gap: 12px; flex-wrap: wrap; }"
      "input[type=range]{ width: 280px; }"
      ".mono { font-family: ui-monospace, SFMono-Regular, Menlo, Consolas, monospace; }"
      "#cameraFeed { max-width: 100%; height: auto; border: 2px solid #ddd; border-radius: 8px; background: #f0f0f0; }"
      ".camera-controls { margin: 10px 0; }"
      ".camera-status { margin: 10px 0; padding: 8px; background: #e3f2fd; border-radius: 4px; }"
      "</style>"
      "</head><body>"
      "<h1>🤖 DARwIn-OP Robot Control with MJPEG Camera</h1>"

      "<div class='camera-group'>"
      "<h3>📹 Live Camera Feed (MJPEG Stream)</h3>"
      "<div class='camera-controls'>"
      "<button class='camera-btn' id='cameraBtn' onclick=\"toggleCamera()\">📹 Enable Camera</button>"
      "</div>"
      "<div class='camera-status' id='cameraStatus'>📍 Camera stream on same port (8080)</div>"
      "<img id='cameraFeed' src='data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAYAAAAfFcSJAAAADUlEQVR42mNkYPhfDwAChwGA60e6kgAAAABJRU5ErkJggg==' alt='Camera will show here when enabled' />"
      "<div><small>📡 MJPEG stream integrated into port 8080</small></div>"
      "</div>"

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
      "<h3>🧠 Head / Yaw Control (Separated)</h3>"
      "<div class='row'>"
      "<label>Yaw: <span id='yawVal' class='mono'>-</span>°</label>"
      "<input id='yawRange' type='range' min='0' max='360' value='180' oninput='setYaw(this.value)'/>"
      "<button class='yaw-btn' onclick=\"setYaw(180)\">🎯 Center Head</button>"
      "</div>"
      "<small>Tip: 슬라이더를 움직이면 목이 독립적으로 움직입니다 (걷기와 무관).</small>"
      "</div>"

      "<div class='control-group'>"
      "<h3>💡 LED Control</h3>"
      "<button class='eye-led-btn' onclick=\"sendCommand('eye_led_toggle')\">👁️ Toggle Eye LED</button>"
      "<button class='led-btn' onclick=\"sendCommand('head_led_toggle')\">💡 Toggle Head LED</button><br>"
      "<button class='blink-btn' onclick=\"sendCommand('led_blink_toggle')\">✨ Toggle Blink Mode</button>"
      "</div>"

      "<div id='status' class='control-group'>🟢 Ready - Robot control interface loaded</div>"

      "<script>"
      "let cameraOn = false;"
      "let mjpegStreamUrl = '';"
      
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
      
      "function toggleCamera() {"
      "  const btn = document.getElementById('cameraBtn');"
      "  const feed = document.getElementById('cameraFeed');"
      "  const status = document.getElementById('cameraStatus');"
      "  "
      "  cameraOn = !cameraOn;"
      "  if (cameraOn) {"
      "    sendCommand('camera_on');"
      "    btn.innerHTML = '📹 Camera ON';"
      "    btn.style.background = '#4CAF50';"
      "    status.innerHTML = '🟢 Camera streaming active on port 8080';"
      "    status.style.background = '#e8f5e8';"
      "    // MJPEG 스트림 연결 (같은 포트 8080)"
      "    const host = window.location.hostname;"
      "    const port = window.location.port || '8080';"
      "    mjpegStreamUrl = 'http://' + host + ':' + port + '/stream';"
      "    feed.src = mjpegStreamUrl;"
      "    feed.style.display = 'block';"
      "  } else {"
      "    sendCommand('camera_off');"
      "    btn.innerHTML = '📹 Camera OFF';"
      "    btn.style.background = '#f44336';"
      "    status.innerHTML = '🔴 Camera streaming disabled';"
      "    status.style.background = '#ffebee';"
      "    feed.src = 'data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAYAAAAfFcSJAAAADUlEQVR42mNkYPhfDwAChwGA60e6kgAAAABJRU5ErkJggg==';"
      "  }"
      "}"
      
      "// 페이지 로드 시 카메라 상태 확인"
      "window.onload = function() {"
      "  const status = document.getElementById('cameraStatus');"
      "  const host = window.location.hostname;"
      "  const port = window.location.port || '8080';"
      "  status.innerHTML = '📍 MJPEG stream available at http://' + host + ':' + port + '/stream';"
      "};"
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

// ------------------------ 통합 HTTP 서버 (8080 포트) ------------------------
void* server(void* arg) {
  (void)arg;

  printf("Unified server thread starting...\n");
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
  printf("Unified server running on http://0.0.0.0:8080\n");
  printf("  - Web interface: http://localhost:8080/\n");
  printf("  - MJPEG stream: http://localhost:8080/stream\n");

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

    // MJPEG 스트림 요청 처리
    if (strstr(get_line, "GET /stream")) {
      printf("MJPEG stream request received\n");
      pthread_mutex_lock(&cameraMutex);
      bool cam_enabled = cameraEnabled;
      pthread_mutex_unlock(&cameraMutex);
      
      if (cam_enabled) {
        send_mjpeg_stream(client);
      } else {
        // 카메라가 비활성화된 경우 에러 응답
        const char* error_response = 
          "HTTP/1.1 503 Service Unavailable\r\n"
          "Content-Type: text/plain\r\n"
          "Connection: close\r\n\r\n"
          "Camera is not enabled. Enable camera first from web interface.";
        send(client, error_response, strlen(error_response), 0);
      }
      close(client);
      continue;
    }

    // 일반 웹 인터페이스 및 명령 처리
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
      aAmplitude = (aAmplitude == 0.5) ? 0.0 : 0.5;
      printf("Turn LEFT (manual a=%.1f)\n", aAmplitude);
    }
    else if (strstr(get_line, "command=turn_right")) {
      aAmplitude = (aAmplitude == -0.5) ? 0.0 : -0.5;
      printf("Turn RIGHT (manual a=%.1f)\n", aAmplitude);
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
    // ---- 카메라 제어 ----
    else if (strstr(get_line, "command=camera_on")) {
      pthread_mutex_lock(&cameraMutex);
      cameraEnabled = true;
      shouldToggleCamera = true;
      pthread_mutex_unlock(&cameraMutex);
      printf("Camera ON\n");
    }
    else if (strstr(get_line, "command=camera_off")) {
      pthread_mutex_lock(&cameraMutex);
      cameraEnabled = false;
      shouldToggleCamera = true;
      pthread_mutex_unlock(&cameraMutex);
      printf("Camera OFF\n");
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

    // HTML 인터페이스 응답
    char* response = load_html();
    send(client, response, strlen(response), 0);
    free(response);
    close(client);
  }

  close(s);
  return NULL;
}

// ------------------------ Walk 클래스 ------------------------
Walk::Walk(): webots::Robot() {
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
  cout << "-------Walk example of DARwIn-OP with MJPEG Camera Streaming (Unified Port)-------" << endl;
  cout << "Web control & Camera: http://localhost:8080" << endl;
  cout << "MJPEG stream: http://localhost:8080/stream" << endl;

  // 시그널 핸들러 설정
  signal(SIGABRT, &sighandler);
  signal(SIGTERM, &sighandler);
  signal(SIGQUIT, &sighandler);
  signal(SIGINT, &sighandler);

  // 통합 웹 서버 스레드 시작
  pthread_t serverThread;
  if (pthread_create(&serverThread, NULL, server, NULL) != 0) {
    cout << "Failed to create server thread!" << endl;
    return;
  }

  // MJPEG 카메라 스트리밍 스레드 시작
  if (pthread_create(&cameraThread, NULL, camera_streaming_thread, NULL) != 0) {
    cout << "Failed to create camera streaming thread!" << endl;
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

    // 3) 목 제어 (걷기와 무관하게 항상 동작)
    bool yawMode = (webYawDeg >= 0.0);
    if (yawMode) {
      double target = convertYawToNeckAngle(webYawDeg);
      targetYawRad = target;
      filteredYawRad = YAW_FILTER_ALPHA * targetYawRad + (1.0 - YAW_FILTER_ALPHA) * filteredYawRad;
      filteredYawRad = clamp(filteredYawRad, minMotorPositions[NECK_INDEX], maxMotorPositions[NECK_INDEX]);
      mMotors[NECK_INDEX]->setPosition(filteredYawRad);
      
      // 목 움직임에 따른 LED 색상 표시
      if (!blinkMode && eyeLedOn) {
        mEyeLED->set(0x00FFFF); // 시안색 (목 제어 활성)
      }
    }

    // 4) 걷기 제어 (걷기 상태일 때만, 순수 버튼 제어)
    if (isWalking && gaitStarted) {
      mGaitManager->setXAmplitude(xAmplitude);
      mGaitManager->setYAmplitude(yAmplitude);
      mGaitManager->setAAmplitude(aAmplitude);
      mGaitManager->step(mTimeStep);
      
      // 걷기 상태에 따른 Head LED 색상
      if (!blinkMode && headLedOn) {
        if (xAmplitude > 0) {
          mHeadLED->set(0x00FF00); // 전진시 초록
        } else if (xAmplitude < 0) {
          mHeadLED->set(0xFF0000); // 후진시 빨강
        } else if (aAmplitude != 0) {
          mHeadLED->set(0x0000FF); // 회전시 파랑
        } else {
          mHeadLED->set(headLedColor); // 기본 색상
        }
      }
    }

    // 5) 카메라 상태에 따른 LED 표시
    pthread_mutex_lock(&cameraMutex);
    bool camEnabled = cameraEnabled;
    pthread_mutex_unlock(&cameraMutex);
    
    if (camEnabled && !blinkMode && eyeLedOn && webYawDeg < 0.0) {
      // 카메라가 켜져있고 목 제어가 비활성일 때 Eye LED를 주황색으로
      mEyeLED->set(0xFFA500); // 주황색 (카메라 활성)
    }

    pthread_mutex_unlock(&stateMutex);
    myStep();
  }
}
