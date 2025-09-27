// VisualTracking + Ultra-minimal HTTP server → LED blink & tracking toggle

#include "VisualTracking.hpp"
#include <webots/Motor.hpp>
#include <webots/Camera.hpp>
#include <webots/LED.hpp>
#include <DARwInOPVisionManager.hpp>

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

// ------------------------ 공용 유틸 ------------------------
static inline double clampd(double v, double lo, double hi) {
  if (lo > hi) { assert(0); return v; }
  return v < lo ? lo : (v > hi ? hi : v);
}

// ------------------------ HTTP/상태 공유 ------------------------
static pthread_mutex_t g_lock = PTHREAD_MUTEX_INITIALIZER;
static bool   g_blinkMode   = false;   // /?blink=toggle
static bool   g_trackMode   = true;    // /?track=toggle  (기본: 추적 ON)
static bool   g_blinkState  = true;
static double g_lastBlinkT  = 0.0;

static const char* HTML =
  "HTTP/1.1 200 OK\r\n"
  "Content-Type: text/html\r\n"
  "Connection: close\r\n\r\n"
  "<!doctype html><html><head><meta charset='utf-8'><title>DARwIn-OP Control</title></head>"
  "<body style='font-family:Arial;padding:24px'>"
  "<h2>DARwIn-OP Control (HTTP)</h2>"
  "<button style='padding:10px 14px;font-weight:700;margin-right:8px' "
  "onclick=\"fetch('/?blink=toggle').then(()=>location.reload())\">Toggle Eye Blink</button>"
  "<button style='padding:10px 14px;font-weight:700' "
  "onclick=\"fetch('/?track=toggle').then(()=>location.reload())\">Toggle Tracking</button>"
  "<p>Refresh after clicking to see current state.</p>"
  "</body></html>";

static void* http_server(void*) {
  int s = socket(AF_INET, SOCK_STREAM, 0);
  if (s < 0) { perror("socket"); return NULL; }

  int opt = 1;
  setsockopt(s, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

  sockaddr_in addr{};
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
    sockaddr_in caddr{}; socklen_t clen = sizeof(caddr);
    int c = accept(s, (sockaddr*)&caddr, &clen);
    if (c < 0) { perror("accept"); continue; }

    char buf[1024];
    ssize_t n = recv(c, buf, sizeof(buf)-1, 0);
    if (n <= 0) { close(c); continue; }
    buf[n] = '\0';

    // 첫 줄만 간단 파싱
    char* line = strtok(buf, "\r\n");
    if (line) {
      if (strstr(line, "GET /?blink=toggle")) {
        pthread_mutex_lock(&g_lock);
        g_blinkMode = !g_blinkMode;
        pthread_mutex_unlock(&g_lock);
        printf("[HTTP] eye blink: %s\n", g_blinkMode ? "ON" : "OFF");
      } else if (strstr(line, "GET /?track=toggle")) {
        pthread_mutex_lock(&g_lock);
        g_trackMode = !g_trackMode;
        pthread_mutex_unlock(&g_lock);
        printf("[HTTP] track: %s\n", g_trackMode ? "ON" : "OFF");
      }
    }

    send(c, HTML, strlen(HTML), 0);
    close(c);
  }
  return NULL;
}

// ------------------------ 모터/이름/범위 ------------------------
#define NMOTORS 20
static double minMotorPositions[NMOTORS];
static double maxMotorPositions[NMOTORS];
static const char *motorNames[NMOTORS] = {
  "ShoulderR","ShoulderL","ArmUpperR","ArmUpperL",
  "ArmLowerR","ArmLowerL","PelvYR","PelvYL",
  "PelvR","PelvL","LegUpperR","LegUpperL",
  "LegLowerR","LegLowerL","AnkleR","AnkleL",
  "FootR","FootL","Neck","Head"
};

// ------------------------ VisualTracking 구현 ------------------------
VisualTracking::VisualTracking() : Robot() {
  mTimeStep = getBasicTimeStep();

  // 장치
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

  // LED
  mEye  = getLED("EyeLed");
  mHead = getLED("HeadLed");
  if (mEye)  mEye->set(0x00FF00); // green
  if (mHead) mHead->set(0xFF0000); // red - 항상 켜진 상태로 유지

  // 비전
  if (mCamera)
    mVisionManager = new DARwInOPVisionManager(mCamera->getWidth(), mCamera->getHeight(),
                                               355, 15, 60, 15, 0, 30);
  else
    mVisionManager = nullptr;
}

VisualTracking::~VisualTracking() {
  delete mVisionManager;
}

void VisualTracking::myStep() {
  int ret = step(mTimeStep);
  if (ret == -1) exit(EXIT_SUCCESS);
}

void VisualTracking::run() {
  // HTTP 서버 스레드 시작
  pthread_t th;
  if (pthread_create(&th, NULL, http_server, NULL) != 0)
    printf("Failed to start HTTP server thread\n");

  signal(SIGINT,  [](int){ printf("\n[EXIT]\n"); exit(0); });
  signal(SIGTERM, [](int){ printf("\n[EXIT]\n"); exit(0); });

  double horizontal = 0.0;
  double vertical   = 0.0;
  int width  = mCamera ? mCamera->getWidth()  : 1;
  int height = mCamera ? mCamera->getHeight() : 1;

  cout << "---------------Visual Tracking + HTTP---------------" << endl;
  cout << "GET /?blink=toggle → Eye LED blink, GET /?track=toggle → tracking on/off" << endl;

  myStep(); // 첫 센서 업데이트

  while (true) {
    // 현재 모드 읽기
    pthread_mutex_lock(&g_lock);
    bool blink = g_blinkMode;
    bool track = g_trackMode;
    pthread_mutex_unlock(&g_lock);

    // 1) 머리 LED는 항상 켜진 상태 유지
    if (mHead) mHead->set(0xFF0000); // 빨간색으로 항상 켜짐
    
    // 2) 눈 LED만 블링크 갱신 (0.1s)
    if (blink) {
      double now = getTime();
      if (now - g_lastBlinkT > 0.1) {
        g_blinkState = !g_blinkState;
        g_lastBlinkT = now;
      }
      if (mEye) mEye->set(g_blinkState ? 0x00FF00 : 0x000000); // 초록/꺼짐
    } else {
      if (mEye) mEye->set(0x00FF00); // 깜빡임 모드가 아니면 항상 초록색
    }

    // 3) 트래킹이 켜져 있으면 머리 추적
    if (track && mVisionManager && mCamera) {
      double x=0, y=0;
      bool ok = mVisionManager->getBallCenter(x, y, mCamera->getImage());
      if (ok) {
        double dh = 0.1 * ((x / width ) - 0.5);
        double dv = 0.1 * ((y / height) - 0.5);
        horizontal -= dh;
        vertical   -= dv;

        horizontal = clampd(horizontal, minMotorPositions[18], maxMotorPositions[18]); // Neck
        vertical   = clampd(vertical,   minMotorPositions[19], maxMotorPositions[19]); // Head

        if (mMotors[18]) mMotors[18]->setPosition(horizontal);
        if (mMotors[19]) mMotors[19]->setPosition(vertical);
      }
    }

    myStep();
  }
}

// ------------------------ main ------------------------
int main() {
  VisualTracking c;
  c.run();
  return 0;
}
