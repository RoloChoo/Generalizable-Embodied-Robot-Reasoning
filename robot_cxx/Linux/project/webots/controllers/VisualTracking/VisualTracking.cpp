// VisualTracking + HTTP server → LED blink, tracking toggle & Motion actions (비동기 개선 - 수정됨)

#include "VisualTracking.hpp"
#include <webots/Motor.hpp>
#include <webots/Camera.hpp>
#include <webots/LED.hpp>
#include <DARwInOPVisionManager.hpp>
#include <DARwInOPMotionManager.hpp>

#include <pthread.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <csignal>
#include <cassert>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <iostream>
#include <fstream>

using namespace webots;
using namespace managers;
using namespace std;

#define NMOTORS 20

// ------------------------ 시그널 핸들러 ------------------------
static void signal_handler(int sig) {
  (void)sig;
  printf("\n[EXIT]\n");
  exit(0);
}

// ------------------------ 공용 유틸 ------------------------
static inline double clampd(double v, double lo, double hi) {
  if (lo > hi) { assert(0); return v; }
  return v < lo ? lo : (v > hi ? hi : v);
}

// ------------------------ HTTP/상태 공유 ------------------------
static pthread_mutex_t g_lock = PTHREAD_MUTEX_INITIALIZER;
static bool   g_blinkMode   = false;
static bool   g_trackMode   = true;
static bool   g_blinkState  = true;
static double g_lastBlinkT  = 0.0;

// 모션 상태 관리 (타이머 기반)
static int    g_motionCmd         = 0;
static double g_motionStartTime   = 0.0;
static double g_motionDuration    = 0.0;
static int    g_currentMotion     = 0;

static const char* HTML =
  "HTTP/1.1 200 OK\r\n"
  "Content-Type: text/html\r\n"
  "Connection: close\r\n\r\n"
  "<!doctype html><html><head><meta charset='utf-8'><title>DARwIn-OP Control</title>"
  "<style>"
  "body{font-family:Arial;padding:24px;background:#f5f5f5}"
  "h2{color:#333}"
  "button{padding:12px 20px;font-weight:700;margin:8px;border:none;border-radius:4px;cursor:pointer;font-size:14px}"
  ".green{background:#4CAF50;color:white}"
  ".blue{background:#2196F3;color:white}"
  ".orange{background:#FF9800;color:white}"
  ".red{background:#f44336;color:white}"
  "button:hover{opacity:0.8}"
  ".section{margin:20px 0;padding:15px;background:white;border-radius:8px;box-shadow:0 2px 4px rgba(0,0,0,0.1)}"
  ".status{background:#e3f2fd;padding:10px;border-radius:4px;margin:10px 0;font-family:monospace;font-size:12px}"
  "</style></head>"
  "<body>"
  "<h2>🤖 DARwIn-OP Remote Control (Async Mode)</h2>"
  
  "<div class='status'>"
  "✅ <strong>Async Mode:</strong> LED blink and tracking work during motions!"
  "</div>"
  
  "<div class='section'>"
  "<h3>⚙️ System Control</h3>"
  "<button class='green' onclick=\"fetch('/?blink=toggle').then(()=>location.reload())\">Toggle Eye Blink</button>"
  "<button class='green' onclick=\"fetch('/?track=toggle').then(()=>location.reload())\">Toggle Tracking</button>"
  "</div>"
  
  "<div class='section'>"
  "<h3>👋 Greetings</h3>"
  "<button class='blue' onclick=\"fetch('/?motion=24').then(()=>alert('Applauding!'))\">Applaud (24)</button>"
  "<button class='blue' onclick=\"fetch('/?motion=38').then(()=>alert('Waving!'))\">Wave Hand (38)</button>"
  "<button class='blue' onclick=\"fetch('/?motion=4').then(()=>alert('Hi!'))\">Tilt Hi (4)</button>"
  "<button class='blue' onclick=\"fetch('/?motion=6').then(()=>alert('Talking!'))\">Talk1 (6)</button>"
  "<button class='blue' onclick=\"fetch('/?motion=29').then(()=>alert('Talking!'))\">Talk2 (29)</button>"
  "</div>"
  
  "<div class='section'>"
  "<h3>⚽ Soccer Moves</h3>"
  "<button class='orange' onclick=\"fetch('/?motion=12').then(()=>alert('Right Kick!'))\">Right Kick (12)</button>"
  "<button class='orange' onclick=\"fetch('/?motion=13').then(()=>alert('Left Kick!'))\">Left Kick (13)</button>"
  "<button class='orange' onclick=\"fetch('/?motion=70').then(()=>alert('Right Pass!'))\">Right Pass (70)</button>"
  "<button class='orange' onclick=\"fetch('/?motion=71').then(()=>alert('Left Pass!'))\">Left Pass (71)</button>"
  "</div>"
  
  "<div class='section'>"
  "<h3>🎭 Expressions</h3>"
  "<button class='red' onclick=\"fetch('/?motion=2').then(()=>alert('Nodding!'))\">Nod Yes (2)</button>"
  "<button class='red' onclick=\"fetch('/?motion=3').then(()=>alert('Shaking head!'))\">Shake No (3)</button>"
  "<button class='red' onclick=\"fetch('/?motion=23').then(()=>alert('Arm Yes!'))\">Arm Yes (23)</button>"
  "<button class='red' onclick=\"fetch('/?motion=27').then(()=>alert('Arm+Head Yes!'))\">Arm+Head Yes (27)</button>"
  "<button class='red' onclick=\"fetch('/?motion=31').then(()=>alert('Stretching!'))\">Stretch (31)</button>"
  "<button class='red' onclick=\"fetch('/?motion=237').then(()=>alert('Jumping!'))\">Jump (237)</button>"
  "<button class='red' onclick=\"fetch('/?motion=239').then(()=>alert('Quick Jump!'))\">Quick Jump (239)</button>"
  "</div>"
  
  "<p style='color:#666;margin-top:20px'>💡 Now you can blink eyes while performing actions!</p>"
  "</body></html>";

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
    perror("bind"); close(s); return NULL;
  }
  if (listen(s, 8) < 0) {
    perror("listen"); close(s); return NULL;
  }
  printf("[HTTP] listening on http://0.0.0.0:8080\n");

  while (true) {
    sockaddr_in caddr;
    memset(&caddr, 0, sizeof(caddr));
    socklen_t clen = sizeof(caddr);
    int c = accept(s, (sockaddr*)&caddr, &clen);
    if (c < 0) { perror("accept"); continue; }

    char buf[1024];
    ssize_t n = recv(c, buf, sizeof(buf)-1, 0);
    if (n <= 0) { close(c); continue; }
    buf[n] = '\0';

    char* line = strtok(buf, "\r\n");
    if (line) {
      if (strstr(line, "GET /?blink=toggle")) {
        pthread_mutex_lock(&g_lock);
        g_blinkMode = !g_blinkMode;
        pthread_mutex_unlock(&g_lock);
        printf("[HTTP] eye blink: %s\n", g_blinkMode ? "ON" : "OFF");
      } 
      else if (strstr(line, "GET /?track=toggle")) {
        pthread_mutex_lock(&g_lock);
        g_trackMode = !g_trackMode;
        pthread_mutex_unlock(&g_lock);
        printf("[HTTP] track: %s\n", g_trackMode ? "ON" : "OFF");
      }
      else if (strstr(line, "GET /?motion=")) {
        char* p = strstr(line, "motion=");
        if (p) {
          int page = atoi(p + 7);
          pthread_mutex_lock(&g_lock);
          g_motionCmd = page;
          pthread_mutex_unlock(&g_lock);
          printf("[HTTP] motion request: page %d\n", page);
        }
      }
    }

    send(c, HTML, strlen(HTML), 0);
    close(c);
  }
  return NULL;
}

// ------------------------ 모터/이름/범위 ------------------------
static double minMotorPositions[NMOTORS];
static double maxMotorPositions[NMOTORS];
static const char *motorNames[NMOTORS] = {
  "ShoulderR","ShoulderL","ArmUpperR","ArmUpperL",
  "ArmLowerR","ArmLowerL","PelvYR","PelvYL",
  "PelvR","PelvL","LegUpperR","LegUpperL",
  "LegLowerR","LegLowerL","AnkleR","AnkleL",
  "FootR","FootL","Neck","Head"
};

// 모션별 대략적인 재생 시간 (초) - 실제 모션 파일에 맞게 조정 필요
static double getMotionDuration(int page) {
  switch(page) {
    case 1:   return 2.0;  // Standing up
    case 2:   return 2.0;  // Nod
    case 3:   return 2.0;  // Shake
    case 4:   return 2.0;  // Tilt Hi
    case 6:   return 3.0;  // Talk1
    case 12:  return 3.0;  // Right Kick
    case 13:  return 3.0;  // Left Kick
    case 23:  return 2.5;  // Arm Yes
    case 24:  return 3.0;  // Applaud
    case 27:  return 2.5;  // Arm+Head Yes
    case 29:  return 3.0;  // Talk2
    case 31:  return 3.0;  // Stretch
    case 38:  return 2.5;  // Wave
    case 70:  return 3.0;  // Right Pass
    case 71:  return 3.0;  // Left Pass
    case 237: return 2.0;  // Jump
    case 239: return 1.5;  // Quick Jump
    default:  return 3.0;  // 기본값
  }
}

// ------------------------ VisualTracking 구현 ------------------------
VisualTracking::VisualTracking() : Robot() {
  mTimeStep = getBasicTimeStep();

  mCamera = getCamera("Camera");
  if (mCamera) mCamera->enable(2*mTimeStep);

  for (int i=0; i<NMOTORS; i++) {
    mMotors[i] = getMotor(motorNames[i]);
    if (mMotors[i]) {
      minMotorPositions[i] = mMotors[i]->getMinPosition();
      maxMotorPositions[i] = mMotors[i]->getMaxPosition();
    } else {
      minMotorPositions[i] = -1e9;
      maxMotorPositions[i] =  1e9;
    }
  }

  mEyeLED  = getLED("EyeLed");
  mHeadLED = getLED("HeadLed");
  if (mEyeLED)  mEyeLED->set(0x00FF00);
  if (mHeadLED) mHeadLED->set(0xFF0000);

  if (mCamera)
    mVisionManager = new DARwInOPVisionManager(mCamera->getWidth(), mCamera->getHeight(),
                                               355, 15, 60, 15, 0, 30);
  else
    mVisionManager = NULL;
    
  mMotionManager = new DARwInOPMotionManager(this);
}

VisualTracking::~VisualTracking() {
  delete mVisionManager;
  delete mMotionManager;
}

void VisualTracking::myStep() {
  // ✅ step() 메소드 호출 제거 - 존재하지 않음
  // Robot::step()만으로 모션이 자동으로 진행됨
  int ret = step(mTimeStep);
  if (ret == -1) exit(EXIT_SUCCESS);
}

void VisualTracking::wait(int ms) {
  double startTime = getTime();
  double s = (double) ms / 1000.0;
  while (s + startTime >= getTime())
    myStep();
}

void VisualTracking::run() {
  signal(SIGINT,  signal_handler);
  signal(SIGTERM, signal_handler);

  pthread_t th;
  if (pthread_create(&th, NULL, http_server, NULL) != 0)
    printf("Failed to start HTTP server thread\n");

  double horizontal = 0.0;
  double vertical   = 0.0;
  int width  = mCamera ? mCamera->getWidth()  : 1;
  int height = mCamera ? mCamera->getHeight() : 1;

  cout << "---------------Visual Tracking + HTTP + Async Motions---------------" << endl;
  cout << "Open browser: http://<robot-ip>:8080" << endl;
  cout << "✨ NEW: LED blink and tracking work during motions!" << endl;

  // 초기 자세 - Standing up 상태로 시작 (비동기 재생)
  mMotionManager->playPage(1, false);
  g_motionStartTime = getTime();
  g_motionDuration = getMotionDuration(1);
  g_currentMotion = 1;
  wait(500);

  myStep();

  while (true) {
    double currentTime = getTime();

    // === 1. 상태 및 명령 읽기 ===
    pthread_mutex_lock(&g_lock);
    bool blink = g_blinkMode;
    bool track = g_trackMode;
    int motionPage = g_motionCmd;
    g_motionCmd = 0;
    pthread_mutex_unlock(&g_lock);

    // === 2. 모션 재생 상태 체크 (타이머 기반) ===
    bool isPlaying = (currentTime - g_motionStartTime) < g_motionDuration;

    // === 3. 모션 시작 (새 명령이 있고 현재 재생 중이 아닐 때) ===
    if (motionPage > 0 && !isPlaying) {
      cout << "▶️ Playing motion page: " << motionPage << " (async)" << endl;
      mMotionManager->playPage(motionPage, false);
      g_motionStartTime = currentTime;
      g_motionDuration = getMotionDuration(motionPage);
      g_currentMotion = motionPage;
    }

    // === 4. 머리 LED 항상 켜짐 ===
    if (mHeadLED) mHeadLED->set(0xFF0000);

    // === 5. 눈 LED 블링크 (모션 중에도 계속 작동) ===
    if (blink) {
      if (currentTime - g_lastBlinkT > 0.1) {
        g_blinkState = !g_blinkState;
        g_lastBlinkT = currentTime;
      }
      if (mEyeLED) mEyeLED->set(g_blinkState ? 0x00FF00 : 0x000000);
    } else {
      if (mEyeLED) mEyeLED->set(0x00FF00);
    }

    // === 6. 트래킹 (모션 중에도 계속 작동) ===
    if (track && mVisionManager && mCamera) {
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

    // === 7. 시뮬레이터 틱 ===
    myStep();

    // === 8. 모션 종료 체크 및 Standing 복귀 ===
    bool wasPlaying = isPlaying;
    isPlaying = (currentTime - g_motionStartTime) < g_motionDuration;

    if (wasPlaying && !isPlaying && g_currentMotion != 0) {
      int finished = g_currentMotion;
      g_currentMotion = 0;
      
      cout << "⏹️ Motion " << finished << " completed" << endl;

      // standing 자세로 복귀 (1번 제외)
      if (finished != 1) {
        mMotionManager->playPage(1, false);
        g_motionStartTime = currentTime;
        g_motionDuration = getMotionDuration(1);
        g_currentMotion = 1;
        wait(200);
      }
    }
  }
}

int main() {
  VisualTracking c;
  c.run();
  return 0;
}
