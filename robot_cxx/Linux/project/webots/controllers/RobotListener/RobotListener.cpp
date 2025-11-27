// RobotListener.cpp - 완전 통합 버전
// VisualTracking.cpp (OpenCV 2.4.13) + Walk.cpp (걷기 제어)
//
// 기능:
// - HTTP 서버 (포트 8080)
//   - / : 웹 대시보드 (카메라 + 걷기 + 모션 + 제어)
//   - /camera : 실시간 JPEG 스트림
//   - /info : 카메라 상태 JSON
// - 걷기 제어 (GaitManager)
// - 모션 재생 (MotionManager)
// - 비전 트래킹 (VisionManager)
// - Blink 제어 (TTS 연동용)
// - 넘어짐 복구
//
// 빌드: OpenCV 2.4.13 필요
//   g++ ... -lopencv_core -lopencv_imgproc -lopencv_highgui

#include "RobotListener.hpp"

#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/Camera.hpp>
#include <webots/LED.hpp>
#include <webots/Accelerometer.hpp>
#include <webots/Gyro.hpp>

#include <DARwInOPVisionManager.hpp>
#include <DARwInOPMotionManager.hpp>
#include <DARwInOPGaitManager.hpp>

// ===== OpenCV 2.4.13 =====
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pthread.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <csignal>
#include <cassert>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <cstdio>
#include <vector>
#include <iostream>
#include <queue>

using namespace webots;
using namespace managers;
using namespace std;
using namespace cv;

#define NMOTORS 20

// ------------------------ 유틸리티 ------------------------
static void signal_handler(int sig) {
  (void)sig;
  printf("\n[EXIT]\n");
  exit(0);
}

static inline double clampd(double v, double lo, double hi) {
  if (lo > hi) return v;
  return v < lo ? lo : (v > hi ? hi : v);
}

// ------------------------ 카메라 JPEG 공유 ------------------------
static pthread_mutex_t g_camera_lock = PTHREAD_MUTEX_INITIALIZER;
static vector<uchar> g_latest_jpeg;
static bool g_has_image = false;

static void encodeToJPEG(const unsigned char* image, int width, int height) {
  if (!image) return;
  pthread_mutex_lock(&g_camera_lock);
  try {
    Mat bgra(height, width, CV_8UC4, const_cast<unsigned char*>(image));
    Mat bgr;
    cvtColor(bgra, bgr, CV_BGRA2BGR);
    vector<int> params;
    params.push_back(CV_IMWRITE_JPEG_QUALITY);
    params.push_back(85);
    imencode(".jpg", bgr, g_latest_jpeg, params);
    g_has_image = true;
  } catch (const cv::Exception& e) {
    fprintf(stderr, "❌ JPEG encode: %s\n", e.what());
  }
  pthread_mutex_unlock(&g_camera_lock);
}

// ------------------------ 상태 공유 (글로벌) ------------------------
static pthread_mutex_t g_lock = PTHREAD_MUTEX_INITIALIZER;

// Blink & Track
static bool   g_blinkMode   = false;
static bool   g_trackMode   = true;
static bool   g_blinkState  = true;
static double g_lastBlinkT  = 0.0;

// 걷기 (Walk.cpp에서)
static bool   g_isWalking      = false;
static bool   g_startWalk      = false;
static bool   g_stopWalk       = false;
static double g_xAmplitude     = 0.0;
static double g_yAmplitude     = 0.0;
static double g_aAmplitude     = 0.0;

// 모션
struct MotionCommand {
  int page;
  int motorId;
  double position;
  double velocity;
};
static queue<MotionCommand> g_motionQueue;

static int    g_currentMotion    = 0;
static double g_motionStartTime  = 0.0;
static double g_motionDuration   = 0.0;

// ==================== Java WebotsController 호환 ====================
// 관절 직접 제어 (walk.cpp 참조)
static double g_targetPositions[NMOTORS] = {0};
static bool   g_jointUpdated[NMOTORS] = {false};

// DARwIn-OP 관절 한계값 (walk.cpp에서 참조)
struct JointLimit {
  double min;
  double max;
  const char* name;
};

static const JointLimit DARWIN_JOINT_LIMITS[NMOTORS] = {
  // 팔 (Arms)
  {-1.57, 0.52,  "ShoulderR"},   // 0: Pitch ±90°/30°
  {-1.57, 0.52,  "ShoulderL"},   // 1: Pitch ±90°/30°
  {-0.68, 2.30,  "ArmUpperR"},   // 2: Roll -39°~131°
  {-2.25, 0.77,  "ArmUpperL"},   // 3: Roll -129°~44°
  {-1.57, -0.10, "ArmLowerR"},   // 4: Elbow -90°~-5.7° (항상 음수!)
  {-1.57, -0.10, "ArmLowerL"},   // 5: Elbow -90°~-5.7° (항상 음수!)
  
  // 골반 (Pelvis)
  {-1.047, 1.047, "PelvYR"},     // 6: Yaw ±60°
  {-0.69, 2.50,   "PelvYL"},     // 7: Yaw -39°~143°
  {-1.01, 1.01,   "PelvR"},      // 8: Roll ±58°
  {-0.35, 0.35,   "PelvL"},      // 9: Roll ±20°
  
  // 다리 (Legs)
  {-2.50, 0.87,  "LegUpperR"},   // 10: Hip Pitch -143°~50°
  {-2.50, 0.87,  "LegUpperL"},   // 11: Hip Pitch -143°~50°
  {-0.35, 0.35,  "LegLowerR"},   // 12: Hip Roll ±20°
  {-0.35, 0.35,  "LegLowerL"},   // 13: Hip Roll ±20°
  
  // 발목 (Ankle)
  {-0.87, 0.87,  "AnkleR"},      // 14: Pitch ±50°
  {-1.39, 1.22,  "AnkleL"},      // 15: Pitch -80°~70°
  {-0.87, 0.87,  "FootR"},       // 16: Roll ±50°
  {-0.87, 0.87,  "FootL"},       // 17: Roll ±50°
  
  // 머리 (Head) - 360도 회전 방지 핵심!
  {-1.57, 1.57,  "Neck"},        // 18: Pan ±90°
  {-0.52, 0.52,  "Head"}         // 19: Tilt ±30°
};

// 필터 상수 (부드러운 보간)
static const double FILTER_ALPHA = 0.15;
static double g_filteredPositions[NMOTORS] = {0};

// ------------------------ 모터 범위 ------------------------
static double minMotorPositions[NMOTORS];
static double maxMotorPositions[NMOTORS];
static const char *motorNames[NMOTORS] = {
  "ShoulderR","ShoulderL","ArmUpperR","ArmUpperL",
  "ArmLowerR","ArmLowerL","PelvYR","PelvYL",
  "PelvR","PelvL","LegUpperR","LegUpperL",
  "LegLowerR","LegLowerL","AnkleR","AnkleL",
  "FootR","FootL","Neck","Head"
};

// ------------------------ 모션 길이 테이블 ------------------------
static double getMotionDuration(int page) {
  switch(page) {
    case 1:  return 2.0;
    case 2:  return 2.0;
    case 3:  return 2.0;
    case 4:  return 2.0;
    case 6:  return 3.0;
    case 9:  return 2.5;
    case 10: return 3.0;
    case 11: return 3.0;
    case 12: return 3.0;
    case 13: return 3.0;
    case 15: return 2.0;
    case 16: return 2.0;
    case 23: return 2.5;
    case 24: return 3.0;
    case 27: return 2.5;
    case 29: return 3.0;
    case 31: return 3.0;
    case 38: return 2.5;
    case 54: return 3.0;
    case 57: return 3.0;
    case 70: return 3.0;
    case 71: return 3.0;
    case 90: return 3.0;
    case 91: return 3.0;
    default: return 3.0;
  }
}

// ------------------------ HTML 대시보드 ------------------------
static const char* HTML =
"HTTP/1.1 200 OK\r\nContent-Type:text/html;charset=utf-8\r\nConnection:close\r\n\r\n"
"<!doctype html><html><head><meta charset='utf-8'>"
"<title>🤖 DARwIn-OP Complete Control</title>"
"<style>"
"body{font-family:'Segoe UI',Arial;padding:20px;background:#f5f5f5;margin:0}"
"h1{color:#2196F3;text-align:center;margin:20px 0}"
"h2{color:#333;margin:12px 0 8px;font-size:18px}"
".wrap{max-width:1100px;margin:0 auto}"
".card{background:#fff;border-radius:12px;padding:18px;margin:12px 0;box-shadow:0 3px 10px rgba(0,0,0,.1)}"
".row{display:flex;flex-wrap:wrap;gap:8px;margin-top:8px}"
"button{padding:11px 16px;border:none;border-radius:8px;cursor:pointer;font-size:14px;font-weight:500;transition:all .3s}"
"button:hover{transform:translateY(-2px);box-shadow:0 4px 12px rgba(0,0,0,.2)}"
".g{background:#4CAF50;color:#fff}.b{background:#2196F3;color:#fff}"
".o{background:#FF9800;color:#fff}.r{background:#f44336;color:#fff}"
".cam{max-width:640px;width:100%;border:3px solid #ddd;border-radius:10px;display:block;margin:10px auto}"
"#status{font-family:monospace;background:#e3f2fd;border-left:4px solid #2196F3;border-radius:6px;padding:12px;margin-top:10px}"
".kv{font-family:monospace;background:#eef;border-radius:4px;padding:4px 8px;display:inline-block}"
".grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(300px,1fr));gap:12px}"
"</style>"
"<script>"
"function tick(){var img=document.getElementById('cam');if(img)img.src='/camera?t='+Date.now()}"
"setInterval(tick,200);"
"function cmd(c){fetch('/?command='+c);document.getElementById('status').innerText='✅ '+c}"
"function go(p){fetch('/?motion='+p);document.getElementById('status').innerText='🎬 Motion '+p}"
"</script></head><body><div class='wrap'>"

"<h1>🤖 DARwIn-OP Complete Control Panel</h1>"

"<div class='card'>"
"<h2>📷 Live Camera Feed</h2>"
"<img id='cam' class='cam' src='/camera'>"
"<div style='text-align:center;margin-top:8px'>API: <span class='kv'>GET /camera</span> <span class='kv'>GET /info</span></div>"
"</div>"

"<div class='grid'>"

"<div class='card'>"
"<h2>🚶 Walking Control</h2>"
"<div class='row'>"
"<button class='g' onclick=\"cmd('walk_start')\">▶️ Start Walk</button>"
"<button class='r' onclick=\"cmd('walk_stop')\">⏹️ Stop Walk</button>"
"</div>"
"<div class='row'>"
"<button class='b' onclick=\"cmd('move_forward')\">⬆️ Forward</button>"
"<button class='b' onclick=\"cmd('move_backward')\">⬇️ Backward</button>"
"</div>"
"<div class='row'>"
"<button class='o' onclick=\"cmd('turn_left')\">↪️ Left</button>"
"<button class='o' onclick=\"cmd('turn_right')\">↩️ Right</button>"
"</div>"
"</div>"

"<div class='card'>"
"<h2>⚙️ System Control</h2>"
"<div class='row'>"
"<button class='g' onclick=\"cmd('blink_toggle')\">💡 Toggle Blink</button>"
"<button class='g' onclick=\"cmd('track_toggle')\">🎯 Toggle Track</button>"
"</div>"
"</div>"

"</div>"

"<div class='card'>"
"<h2>🙋 Basic Gestures</h2>"
"<div class='row'>"
"<button class='b' onclick='go(1)'>ini (1)</button>"
"<button class='b' onclick='go(2)'>OK/Nod (2)</button>"
"<button class='b' onclick='go(3)'>no/Shake (3)</button>"
"<button class='b' onclick='go(4)'>hi/Tilt (4)</button>"
"<button class='o' onclick='go(6)'>talk1 (6)</button>"
"<button class='o' onclick='go(29)'>talk2 (29)</button>"
"<button class='o' onclick='go(9)'>walkready (9)</button>"
"</div>"
"</div>"

"<div class='card'>"
"<h2>🎭 Advanced Gestures</h2>"
"<div class='row'>"
"<button class='b' onclick='go(23)'>Arm YES (23)</button>"
"<button class='b' onclick='go(24)'>Applaud (24)</button>"
"<button class='b' onclick='go(27)'>Arm+Head YES (27)</button>"
"<button class='b' onclick='go(31)'>Stretch (31)</button>"
"<button class='b' onclick='go(38)'>Wave (38)</button>"
"</div>"
"</div>"

"<div class='card'>"
"<h2>⚽ Soccer Actions</h2>"
"<div class='row'>"
"<button class='o' onclick='go(12)'>Right Kick (12)</button>"
"<button class='o' onclick='go(13)'>Left Kick (13)</button>"
"<button class='o' onclick='go(70)'>Right Pass (70)</button>"
"<button class='o' onclick='go(71)'>Left Pass (71)</button>"
"</div>"
"</div>"

"<div class='card'>"
"<h2>↕️ Get Up / Sit / Lie</h2>"
"<div class='row'>"
"<button class='r' onclick='go(10)'>f up (10)</button>"
"<button class='r' onclick='go(11)'>b up (11)</button>"
"<button class='r' onclick='go(15)'>sit down (15)</button>"
"<button class='r' onclick='go(16)'>stand up (16)</button>"
"<button class='g' onclick='go(90)'>lie down (90)</button>"
"<button class='g' onclick='go(91)'>lie up (91)</button>"
"</div>"
"</div>"

"<div id='status'>✅ Ready</div>"

"</div></body></html>";

// ------------------------ HTTP 서버 ------------------------
static void* http_server(void* arg) {
  (void)arg;

  int s = socket(AF_INET, SOCK_STREAM, 0);
  if (s < 0) { perror("socket"); return NULL; }

  int opt = 1;
  setsockopt(s, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

  sockaddr_in addr;
  memset(&addr, 0, sizeof(addr));
  addr.sin_family = AF_INET;
  addr.sin_port   = htons(8080);
  addr.sin_addr.s_addr = INADDR_ANY;

  if (bind(s, (sockaddr*)&addr, sizeof(addr)) < 0) {
    perror("bind");
    close(s);
    return NULL;
  }
  if (listen(s, 8) < 0) {
    perror("listen");
    close(s);
    return NULL;
  }

  printf("✅ HTTP Server: http://0.0.0.0:8080\n");

  while (true) {
    sockaddr_in caddr;
    socklen_t clen = sizeof(caddr);
    int c = accept(s, (sockaddr*)&caddr, &clen);
    if (c < 0) {
      perror("accept");
      continue;
    }

    char buf[2048];
    ssize_t n = recv(c, buf, sizeof(buf)-1, 0);
    if (n <= 0) {
      close(c);
      continue;
    }
    buf[n] = '\0';

    char* line = strtok(buf, "\r\n");
    if (!line) {
      close(c);
      continue;
    }

    // ===== /camera 엔드포인트 =====
    if (strstr(line, "GET /camera")) {
      pthread_mutex_lock(&g_camera_lock);
      if (g_has_image && !g_latest_jpeg.empty()) {
        char header[256];
        snprintf(header, sizeof(header),
          "HTTP/1.1 200 OK\r\nContent-Type:image/jpeg\r\n"
          "Content-Length:%zu\r\nAccess-Control-Allow-Origin:*\r\n"
          "Cache-Control:no-cache\r\nConnection:close\r\n\r\n",
          g_latest_jpeg.size());
        send(c, header, strlen(header), 0);
        send(c, &g_latest_jpeg[0], g_latest_jpeg.size(), 0);
      } else {
        const char* err = "HTTP/1.1 503 Unavailable\r\n\r\nNo image";
        send(c, err, strlen(err), 0);
      }
      pthread_mutex_unlock(&g_camera_lock);
      close(c);
      continue;
    }

    // ===== /info 엔드포인트 =====
    if (strstr(line, "GET /info")) {
      char info[256];
      pthread_mutex_lock(&g_camera_lock);
      size_t sz = g_latest_jpeg.size();
      bool has = g_has_image;
      pthread_mutex_unlock(&g_camera_lock);
      snprintf(info, sizeof(info),
        "HTTP/1.1 200 OK\r\nContent-Type:application/json\r\n\r\n"
        "{\"has_image\":%s,\"size\":%zu}", has ? "true" : "false", sz);
      send(c, info, strlen(info), 0);
      close(c);
      continue;
    }

    // ===== 명령 처리 =====
    pthread_mutex_lock(&g_lock);

    if (strstr(line, "command=walk_start")) {
      g_startWalk = true;
      printf("🚶 Walk START\n");
    }
    else if (strstr(line, "command=walk_stop")) {
      g_stopWalk = true;
      printf("🛑 Walk STOP\n");
    }
    else if (strstr(line, "command=move_forward")) {
      g_xAmplitude = (g_xAmplitude == 1.0) ? 0.0 : 1.0;
      printf("⬆️ Forward: %.1f\n", g_xAmplitude);
    }
    else if (strstr(line, "command=move_backward")) {
      g_xAmplitude = (g_xAmplitude == -1.0) ? 0.0 : -1.0;
      printf("⬇️ Backward: %.1f\n", g_xAmplitude);
    }
    else if (strstr(line, "command=turn_left")) {
      g_aAmplitude = (g_aAmplitude == 0.5) ? 0.0 : 0.5;
      printf("↪️ Left: %.1f\n", g_aAmplitude);
    }
    else if (strstr(line, "command=turn_right")) {
      g_aAmplitude = (g_aAmplitude == -0.5) ? 0.0 : -0.5;
      printf("↩️ Right: %.1f\n", g_aAmplitude);
    }
    else if (strstr(line, "command=blink_toggle")) {
      g_blinkMode = !g_blinkMode;
      printf("💡 Blink: %s\n", g_blinkMode ? "ON" : "OFF");
    }
    else if (strstr(line, "command=track_toggle")) {
      g_trackMode = !g_trackMode;
      printf("🎯 Track: %s\n", g_trackMode ? "ON" : "OFF");
    }
    else if (strstr(line, "motion=")) {
      char* p = strstr(line, "motion=");
      if (p) {
        int page = atoi(p + 7);
        MotionCommand cmd = {page, -1, 0.0, 0.0};
        g_motionQueue.push(cmd);
        printf("🎬 Motion %d\n", page);
      }
    }
    // ==================== Java WebotsController 호환 API ====================
    // set_joint: 개별 관절 직접 제어
    // 사용법: /?command=set_joint&index=18&value=0.5
    else if (strstr(line, "command=set_joint")) {
      char* indexPos = strstr(line, "index=");
      char* valuePos = strstr(line, "value=");
      
      if (indexPos && valuePos) {
        int index = atoi(indexPos + 6);
        double value = atof(valuePos + 6);
        
        if (index >= 0 && index < NMOTORS) {
          // 안전 범위로 클램핑
          double safeMin = DARWIN_JOINT_LIMITS[index].min;
          double safeMax = DARWIN_JOINT_LIMITS[index].max;
          double clamped = clampd(value, safeMin, safeMax);
          
          if (value != clamped) {
            printf("⚠️ Joint %d (%s) clamped: %.3f -> %.3f\n", 
                   index, motorNames[index], value, clamped);
          }
          
          g_targetPositions[index] = clamped;
          g_jointUpdated[index] = true;
        }
      }
    }
    // set_walk: WASD 스타일 걷기 제어 (Java WebotsController 호환)
    // 사용법: /?command=set_walk&f=1&b=0&l=0&r=0
    else if (strstr(line, "command=set_walk")) {
      char* fPos = strstr(line, "f=");
      char* bPos = strstr(line, "b=");
      char* lPos = strstr(line, "l=");
      char* rPos = strstr(line, "r=");
      
      bool forward  = fPos && atoi(fPos + 2) == 1;
      bool backward = bPos && atoi(bPos + 2) == 1;
      bool left     = lPos && atoi(lPos + 2) == 1;
      bool right    = rPos && atoi(rPos + 2) == 1;
      
      // 방향 계산
      if (forward && !backward) {
        g_xAmplitude = 1.0;
      } else if (backward && !forward) {
        g_xAmplitude = -1.0;
      } else {
        g_xAmplitude = 0.0;
      }
      
      if (left && !right) {
        g_aAmplitude = 0.5;
      } else if (right && !left) {
        g_aAmplitude = -0.5;
      } else {
        g_aAmplitude = 0.0;
      }
      
      // 걷기 자동 시작/정지
      bool anyMovement = forward || backward || left || right;
      if (anyMovement && !g_isWalking) {
        g_startWalk = true;
      } else if (!anyMovement && g_isWalking) {
        g_stopWalk = true;
      }
    }
    // stop_all: 모든 동작 정지
    else if (strstr(line, "command=stop_all")) {
      g_stopWalk = true;
      g_xAmplitude = 0.0;
      g_yAmplitude = 0.0;
      g_aAmplitude = 0.0;
      printf("🛑 STOP ALL\n");
    }

    pthread_mutex_unlock(&g_lock);
    
    // 간단한 OK 응답 (JSON API용)
    if (strstr(line, "command=set_joint") || 
        strstr(line, "command=set_walk") || 
        strstr(line, "command=stop_all")) {
      const char* ok = "HTTP/1.1 200 OK\r\nContent-Type:text/plain\r\n"
                       "Access-Control-Allow-Origin:*\r\nConnection:close\r\n\r\nOK";
      send(c, ok, strlen(ok), 0);
      close(c);
      continue;
    }

    // 기본: HTML 대시보드
    send(c, HTML, strlen(HTML), 0);
    close(c);
  }

  return NULL;
}

// ------------------------ RobotListener 구현 ------------------------
RobotListener::RobotListener() : Robot() {
  mTimeStep = getBasicTimeStep();
  if (mTimeStep <= 0) mTimeStep = 8;

  // 카메라
  mCamera = getCamera("Camera");
  if (mCamera) mCamera->enable(2 * mTimeStep);

  // LED
  mEyeLED  = getLED("EyeLed");
  mHeadLED = getLED("HeadLed");
  if (mEyeLED)  mEyeLED->set(0x00FF00);
  if (mHeadLED) mHeadLED->set(0xFF0000);

  // 센서
  mAccelerometer = getAccelerometer("Accelerometer");
  if (mAccelerometer) mAccelerometer->enable(mTimeStep);
  
  Gyro* gyro = getGyro("Gyro");
  if (gyro) gyro->enable(mTimeStep);

  // 모터
  for (int i = 0; i < NMOTORS; i++) {
    mMotors[i] = getMotor(motorNames[i]);
    if (mMotors[i]) {
      minMotorPositions[i] = mMotors[i]->getMinPosition();
      maxMotorPositions[i] = mMotors[i]->getMaxPosition();
    } else {
      minMotorPositions[i] = -1e9;
      maxMotorPositions[i] =  1e9;
    }
  }

  keyboardEnable(mTimeStep);

  // 매니저
  mMotionManager = new DARwInOPMotionManager(this);
  mGaitManager   = new DARwInOPGaitManager(this, "config.ini");

  if (mCamera) {
    mVisionManager = new DARwInOPVisionManager(
      mCamera->getWidth(), mCamera->getHeight(),
      355, 15, 60, 15, 0, 30
    );
  } else {
    mVisionManager = NULL;
  }
}

RobotListener::~RobotListener() {
  delete mVisionManager;
  delete mMotionManager;
  delete mGaitManager;
}

void RobotListener::myStep() {
  int ret = step(mTimeStep);
  if (ret == -1) exit(EXIT_SUCCESS);
}

void RobotListener::wait(int ms) {
  double start = getTime();
  double s = ms / 1000.0;
  while (s + start >= getTime()) myStep();
}

void RobotListener::checkIfFallen() {
  if (!mAccelerometer) return;
  
  static int fup = 0, fdown = 0;
  const double *acc = mAccelerometer->getValues();
  
  if (acc[1] < 432.0) fup++; else fup = 0;
  if (acc[1] > 592.0) fdown++; else fdown = 0;
  
  if (fup > 100) {
    printf("🤕 Fallen forward! Getting up...\n");
    mMotionManager->playPage(10);
    mMotionManager->playPage(9);
    fup = 0;
  }
  if (fdown > 100) {
    printf("🤕 Fallen backward! Getting up...\n");
    mMotionManager->playPage(11);
    mMotionManager->playPage(9);
    fdown = 0;
  }
}

void RobotListener::run() {
  signal(SIGINT,  signal_handler);
  signal(SIGTERM, signal_handler);

  printf("========================================\n");
  printf("🤖 RobotListener Complete v1.0\n");
  printf("📷 Camera: %dx%d\n",
    mCamera ? mCamera->getWidth() : 0,
    mCamera ? mCamera->getHeight() : 0);
  printf("========================================\n");

  // HTTP 서버 시작
  pthread_t th;
  if (pthread_create(&th, NULL, http_server, NULL) != 0) {
    printf("❌ Failed to start HTTP server\n");
  }

  // 초기 자세
  myStep();
  mMotionManager->playPage(1, false);
  wait(300);

  // 트래킹 변수
  double horizontal = 0.0;
  double vertical   = 0.0;
  int width  = mCamera ? mCamera->getWidth()  : 1;
  int height = mCamera ? mCamera->getHeight() : 1;

  bool gaitStarted = false;

  printf("✅ Ready\n");

  while (true) {
    double now = getTime();

    // 넘어짐 체크
    checkIfFallen();

    // 카메라 → JPEG 인코딩
    if (mCamera) {
      const unsigned char* img = mCamera->getImage();
      if (img) encodeToJPEG(img, width, height);
    }

    // 상태 스냅샷
    pthread_mutex_lock(&g_lock);
    
    bool blink = g_blinkMode;
    bool track = g_trackMode;
    
    bool startWalk = g_startWalk;
    bool stopWalk  = g_stopWalk;
    g_startWalk = false;
    g_stopWalk  = false;
    
    double xAmp = g_xAmplitude;
    double yAmp = g_yAmplitude;
    double aAmp = g_aAmplitude;
    
    pthread_mutex_unlock(&g_lock);

    // ===== 걷기 제어 =====
    if (startWalk && !g_isWalking) {
      printf("🚶 Starting gait...\n");
      mGaitManager->start();
      mGaitManager->step(mTimeStep);
      g_isWalking = true;
      gaitStarted = true;
    }
    
    if (stopWalk && g_isWalking) {
      printf("🛑 Stopping gait...\n");
      mGaitManager->stop();
      g_isWalking = false;
      gaitStarted = false;
      
      pthread_mutex_lock(&g_lock);
      g_xAmplitude = g_yAmplitude = g_aAmplitude = 0.0;
      pthread_mutex_unlock(&g_lock);
    }
    
    if (g_isWalking && gaitStarted) {
      mGaitManager->setXAmplitude(xAmp);
      mGaitManager->setYAmplitude(yAmp);
      mGaitManager->setAAmplitude(aAmp);
      mGaitManager->step(mTimeStep);
    }

    // ===== 모션 재생 =====
    bool isPlaying = (now - g_motionStartTime) < g_motionDuration;
    
    pthread_mutex_lock(&g_lock);
    if (!g_motionQueue.empty() && !isPlaying && !g_isWalking) {
      MotionCommand cmd = g_motionQueue.front();
      g_motionQueue.pop();
      pthread_mutex_unlock(&g_lock);
      
      if (cmd.motorId == -1) {
        printf("🎬 Motion page %d\n", cmd.page);
        mMotionManager->playPage(cmd.page, false);
        g_motionStartTime = now;
        g_motionDuration  = getMotionDuration(cmd.page);
        g_currentMotion   = cmd.page;
      } else {
        if (mMotors[cmd.motorId]) {
          mMotors[cmd.motorId]->setPosition(cmd.position);
          mMotors[cmd.motorId]->setVelocity(cmd.velocity);
        }
      }
    } else {
      pthread_mutex_unlock(&g_lock);
    }

    // ===== LED 제어 =====
    if (mHeadLED) mHeadLED->set(0xFF0000);
    
    if (blink) {
      if (now - g_lastBlinkT > 0.1) {
        g_blinkState = !g_blinkState;
        g_lastBlinkT = now;
      }
      if (mEyeLED) mEyeLED->set(g_blinkState ? 0x00FF00 : 0x000000);
    } else {
      if (mEyeLED) mEyeLED->set(0x00FF00);
    }

    // ===== 비전 트래킹 =====
    if (track && mVisionManager && mCamera && !g_isWalking) {
      double x = 0.0, y = 0.0;
      bool ok = mVisionManager->getBallCenter(x, y, mCamera->getImage());
      if (ok) {
        double dh = 0.1 * ((x / width ) - 0.5);
        double dv = 0.1 * ((y / height) - 0.5);
        horizontal -= dh;
        vertical   -= dv;
        horizontal = clampd(horizontal, minMotorPositions[18], maxMotorPositions[18]);
        vertical   = clampd(vertical,   minMotorPositions[19], maxMotorPositions[19]);
        if (mMotors[18]) mMotors[18]->setPosition(horizontal);
        if (mMotors[19]) mMotors[19]->setPosition(vertical);
      }
    }
    
    // ===== Java WebotsController: 관절 직접 제어 적용 =====
    pthread_mutex_lock(&g_lock);
    for (int i = 0; i < NMOTORS; i++) {
      if (g_jointUpdated[i]) {
        // 부드러운 보간 적용
        g_filteredPositions[i] = FILTER_ALPHA * g_targetPositions[i] + 
                                 (1.0 - FILTER_ALPHA) * g_filteredPositions[i];
        
        // 안전 범위로 클램핑
        double safePos = clampd(g_filteredPositions[i], 
                                DARWIN_JOINT_LIMITS[i].min, 
                                DARWIN_JOINT_LIMITS[i].max);
        
        // 모터에 적용
        if (mMotors[i]) {
          mMotors[i]->setPosition(safePos);
        }
        
        // 목표에 도달하면 업데이트 플래그 해제
        if (fabs(g_filteredPositions[i] - g_targetPositions[i]) < 0.001) {
          g_jointUpdated[i] = false;
        }
      }
    }
    pthread_mutex_unlock(&g_lock);

    // ===== 모션 종료 처리 =====
    bool wasPlaying = isPlaying;
    isPlaying = (now - g_motionStartTime) < g_motionDuration;
    if (wasPlaying && !isPlaying && g_currentMotion != 0) {
      int finished = g_currentMotion;
      g_currentMotion = 0;
      printf("⏹️ Motion %d complete\n", finished);
      
      // ini로 복귀 (1번 제외)
      if (finished != 1 && !g_isWalking) {
        mMotionManager->playPage(1, false);
        g_motionStartTime = now;
        g_motionDuration  = getMotionDuration(1);
        g_currentMotion   = 1;
        wait(200);
      }
    }

    myStep();
  }
}

// ------------------------ main ------------------------
int main() {
  RobotListener robot;
  robot.run();
  return 0;
}
