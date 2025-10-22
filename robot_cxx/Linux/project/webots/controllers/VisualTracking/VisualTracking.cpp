// VisualTracking.cpp - OpenCV 2.4.13 + DARwIn-OP (실기기/Webots 겸용)
// - /camera : 최신 프레임 JPEG 제공
// - /info   : {"has_image":bool, "size":bytes}
// - 루트( / ) : 카메라 미리보기 + Controls + 모션 버튼 대시보드
//
// 빌드 시 OpenCV 2.4 링크 필요:
//   ... -lopencv_core -lopencv_imgproc -lopencv_highgui
//
// 주의: 모션 페이지(ID)는 장치에 탑재된 모션과 일치해야 합니다.

#include "VisualTracking.hpp"

#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/Camera.hpp>
#include <webots/LED.hpp>

#include <DARwInOPVisionManager.hpp>
#include <DARwInOPMotionManager.hpp>

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

using namespace webots;
using namespace managers;
using namespace std;
using namespace cv;

#define NMOTORS 20

// ------------------------ 공용/시스템 ------------------------
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
    vector<int> params; params.push_back(CV_IMWRITE_JPEG_QUALITY); params.push_back(80);
    imencode(".jpg", bgr, g_latest_jpeg, params);
    g_has_image = true;
  } catch (const cv::Exception& e) {
    fprintf(stderr, "JPEG encode error: %s\n", e.what());
  }
  pthread_mutex_unlock(&g_camera_lock);
}

// ------------------------ HTTP/상태 공유 ------------------------
static pthread_mutex_t g_lock = PTHREAD_MUTEX_INITIALIZER;
static bool   g_blinkMode   = false;
static bool   g_trackMode   = true;
static bool   g_blinkState  = true;
static double g_lastBlinkT  = 0.0;

static int    g_motionCmd         = 0;     // 새로 요청된 모션
static double g_motionStartTime   = 0.0;   // 현재 재생 시작 시각
static double g_motionDuration    = 0.0;   // 모션 예상 지속 시간
static int    g_currentMotion     = 0;     // 현재 재생중인 모션

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

// ------------------------ 모션 길이 테이블 ------------------------
// (장치에 로드된 페이지와 일치해야 함. 필요시 값 조정)
static double getMotionDuration(int page) {
  switch(page) {
    case 1:  return 2.0;   // ini / stand
    case 2:  return 2.0;   // OK / nod yes
    case 3:  return 2.0;   // no / shake head
    case 4:  return 2.0;   // hi / tilt
    case 6:  return 3.0;   // talk1
    case 9:  return 2.5;   // walkready
    case 10: return 3.0;   // f up (lying face → up)
    case 11: return 3.0;   // b up (lying back → up)
    case 12: return 3.0;   // rk
    case 13: return 3.0;   // lk
    case 15: return 2.0;   // sit down
    case 16: return 2.0;   // stand up
    case 23: return 2.5;   // d1 / arm yes
    case 24: return 3.0;   // d2 / applaud
    case 27: return 2.5;   // d3 / arm+head yes
    case 29: return 3.0;   // talk2
    case 31: return 3.0;   // d4 / stretch
    case 38: return 2.5;   // wave hand
    case 54: return 3.0;   // int (applaud louder?) - 환경에 맞게
    case 57: return 3.0;   // int (applaud?)        - 환경에 맞게
    case 70: return 3.0;   // rPASS
    case 71: return 3.0;   // lPASS
    case 90: return 3.0;   // lie down (front)
    case 91: return 3.0;   // lie up (back)
    default: return 3.0;
  }
}

// ------------------------ HTML (카메라 + 컨트롤 + 표 기반 버튼) ------------------------
static const char* HTML =
"HTTP/1.1 200 OK\r\nContent-Type:text/html\r\nConnection:close\r\n\r\n"
"<!doctype html><html><head><meta charset='utf-8'>"
"<title>DARwIn-OP Camera & Controls</title>"
"<style>"
"body{font-family:Arial;padding:20px;background:#f5f5f5}"
"h2{color:#333;margin:0 0 10px}"
".wrap{max-width:980px;margin:0 auto}"
".card{background:#fff;border-radius:10px;padding:16px;margin:14px 0;box-shadow:0 2px 6px rgba(0,0,0,.08)}"
".row{display:flex;flex-wrap:wrap;gap:8px}"
"button{padding:10px 14px;border:none;border-radius:6px;cursor:pointer;font-size:13px}"
".g{background:#4CAF50;color:#fff}.b{background:#2196F3;color:#fff}"
".o{background:#FF9800;color:#fff}.r{background:#f44336;color:#fff}"
".cam{max-width:640px;width:100%;border:2px solid #333;border-radius:8px;display:block;margin:0 auto}"
".kv{font-family:monospace;background:#eef;border-radius:6px;padding:8px;display:inline-block}"
"</style>"
"<script>"
"function tick(){var img=document.getElementById('cam'); if(img){img.src='/camera?t='+Date.now();}}"
"setInterval(tick,150);"
"function go(page){fetch('/?motion='+page)}"
"</script></head><body><div class='wrap'>"

"<div class='card'>"
"<h2>📷 Camera</h2>"
"<img id='cam' class='cam' src='/camera'>"
"<div style='margin-top:8px'>Info API: <span class='kv'>GET /info</span></div>"
"</div>"

"<div class='card'>"
"<h2>⚙️ Controls</h2>"
"<div class='row'>"
"<button class='g' onclick=\"fetch('/?blink=toggle')\">Toggle Blink</button>"
"<button class='g' onclick=\"fetch('/?track=toggle')\">Toggle Tracking</button>"
"</div>"
"</div>"

"<div class='card'>"
"<h2>🙋 Basics</h2>"
"<div class='row'>"
"<button class='b' onclick='go(1)'>ini (1)</button>"
"<button class='b' onclick='go(2)'>OK / Nod (2)</button>"
"<button class='b' onclick='go(3)'>no / Shake (3)</button>"
"<button class='b' onclick='go(4)'>hi / Tilt (4)</button>"
"</div>"
"</div>"

"<div class='card'>"
"<h2>🗣️ Talk / Prep</h2>"
"<div class='row'>"
"<button class='o' onclick='go(6)'>talk1 (6)</button>"
"<button class='o' onclick='go(29)'>talk2 (29)</button>"
"<button class='o' onclick='go(9)'>walkready (9)</button>"
"</div>"
"</div>"

"<div class='card'>"
"<h2>↕️ Get up / Sit / Stand</h2>"
"<div class='row'>"
"<button class='r' onclick='go(10)'>f up (10)</button>"
"<button class='r' onclick='go(11)'>b up (11)</button>"
"<button class='r' onclick='go(15)'>sit down (15)</button>"
"<button class='r' onclick='go(16)'>stand up (16)</button>"
"</div>"
"</div>"

"<div class='card'>"
"<h2>🎭 Gestures</h2>"
"<div class='row'>"
"<button class='b' onclick='go(23)'>d1 / Arm YES (23)</button>"
"<button class='b' onclick='go(24)'>d2 / Applaud (24)</button>"
"<button class='b' onclick='go(27)'>d3 / Arm+Head YES (27)</button>"
"<button class='b' onclick='go(31)'>d4 / Stretch (31)</button>"
"<button class='b' onclick='go(38)'>Wave hand (38)</button>"
"</div>"
"</div>"

"<div class='card'>"
"<h2>⚽ Soccer</h2>"
"<div class='row'>"
"<button class='o' onclick='go(12)'>rk / Right kick (12)</button>"
"<button class='o' onclick='go(13)'>lk / Left kick (13)</button>"
"<button class='o' onclick='go(70)'>rPASS (70)</button>"
"<button class='o' onclick='go(71)'>lPASS (71)</button>"
"</div>"
"</div>"

"<div class='card'>"
"<h2>🛌 Lie</h2>"
"<div class='row'>"
"<button class='g' onclick='go(90)'>lie down (front) (90)</button>"
"<button class='g' onclick='go(91)'>lie up (back) (91)</button>"
"</div>"
"</div>"

"</div></body></html>";

// ------------------------ HTTP 서버 ------------------------
static void* http_server(void* arg) {
  (void)arg;

  int s = socket(AF_INET, SOCK_STREAM, 0);
  if (s < 0) { perror("socket"); return NULL; }

  int opt = 1;
  setsockopt(s, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

  sockaddr_in addr; memset(&addr, 0, sizeof(addr));
  addr.sin_family = AF_INET;
  addr.sin_port   = htons(8080);
  addr.sin_addr.s_addr = INADDR_ANY;

  if (bind(s, (sockaddr*)&addr, sizeof(addr)) < 0) { perror("bind"); close(s); return NULL; }
  if (listen(s, 8) < 0) { perror("listen"); close(s); return NULL; }
  printf("[HTTP] listening on http://0.0.0.0:8080\n");

  while (true) {
    sockaddr_in caddr; socklen_t clen = sizeof(caddr);
    int c = accept(s, (sockaddr*)&caddr, &clen);
    if (c < 0) { perror("accept"); continue; }

    char buf[1024]; ssize_t n = recv(c, buf, sizeof(buf)-1, 0);
    if (n <= 0) { close(c); continue; }
    buf[n] = '\0';

    char* line = strtok(buf, "\r\n");
    if (!line) { close(c); continue; }

    // ---- 라우팅 ----
    if (strstr(line, "GET /camera")) {
      pthread_mutex_lock(&g_camera_lock);
      if (g_has_image && !g_latest_jpeg.empty()) {
        char header[256];
        snprintf(header, sizeof(header),
          "HTTP/1.1 200 OK\r\nContent-Type:image/jpeg\r\n"
          "Content-Length:%zu\r\nAccess-Control-Allow-Origin:*\r\n\r\n",
          g_latest_jpeg.size());
        send(c, header, strlen(header), 0);
        send(c, &g_latest_jpeg[0], g_latest_jpeg.size(), 0);
      } else {
        const char* err = "HTTP/1.1 503 Service Unavailable\r\n\r\nNo image";
        send(c, err, strlen(err), 0);
      }
      pthread_mutex_unlock(&g_camera_lock);
      close(c);
      continue;
    }
    else if (strstr(line, "GET /info")) {
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
    else if (strstr(line, "GET /?blink=toggle")) {
      pthread_mutex_lock(&g_lock); g_blinkMode = !g_blinkMode; pthread_mutex_unlock(&g_lock);
      printf("[HTTP] eye blink: %s\n", g_blinkMode ? "ON" : "OFF");
    }
    else if (strstr(line, "GET /?track=toggle")) {
      pthread_mutex_lock(&g_lock); g_trackMode = !g_trackMode; pthread_mutex_unlock(&g_lock);
      printf("[HTTP] track: %s\n", g_trackMode ? "ON" : "OFF");
    }
    else if (strstr(line, "GET /?motion=")) {
      char* p = strstr(line, "motion=");
      if (p) {
        int page = atoi(p + 7);
        pthread_mutex_lock(&g_lock); g_motionCmd = page; pthread_mutex_unlock(&g_lock);
        printf("[HTTP] motion request: page %d\n", page);
      }
    }

    // 기본: 대시보드 HTML
    send(c, HTML, strlen(HTML), 0);
    close(c);
  }

  return NULL;
}

// ------------------------ VisualTracking 구현 ------------------------
VisualTracking::VisualTracking() : Robot() {
  mTimeStep = getBasicTimeStep();
  if (mTimeStep <= 0) mTimeStep = 8; // 실기기 0 보정

  mCamera = getCamera("Camera");
  if (mCamera) mCamera->enable(2 * mTimeStep);

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
  int ret = step(mTimeStep);
  if (ret == -1) exit(EXIT_SUCCESS);
}

void VisualTracking::wait(int ms) {
  double start = getTime();
  double s = ms / 1000.0;
  while (s + start >= getTime()) myStep();
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

  cout << "--------------- Visual Tracking + HTTP + Async Motions ---------------\n";
  cout << "Open: http://<robot-ip>:8080\n";

  // 초기 자세 (ini / stand)
  mMotionManager->playPage(1, false);
  g_motionStartTime = getTime();
  g_motionDuration  = getMotionDuration(1);
  g_currentMotion   = 1;
  wait(300);

  myStep();

  while (true) {
    double now = getTime();

    // 0) 카메라 프레임 → JPEG 업데이트
    if (mCamera) {
      const unsigned char* img = mCamera->getImage();
      if (img) encodeToJPEG(img, width, height);
    }

    // 1) 상태/명령 스냅샷
    pthread_mutex_lock(&g_lock);
    bool blink = g_blinkMode;
    bool track = g_trackMode;
    int  req   = g_motionCmd;
    g_motionCmd = 0;
    pthread_mutex_unlock(&g_lock);

    // 2) 모션 재생 상태
    bool isPlaying = (now - g_motionStartTime) < g_motionDuration;

    // 3) 새 모션 시작
    if (req > 0 && !isPlaying) {
      cout << "▶️ Motion page: " << req << " (async)\n";
      mMotionManager->playPage(req, false);
      g_motionStartTime = now;
      g_motionDuration  = getMotionDuration(req);
      g_currentMotion   = req;
    }

    // 4) 머리 LED 고정
    if (mHeadLED) mHeadLED->set(0xFF0000);

    // 5) 눈 LED 블링크
    if (blink) {
      if (now - g_lastBlinkT > 0.1) { g_blinkState = !g_blinkState; g_lastBlinkT = now; }
      if (mEyeLED) mEyeLED->set(g_blinkState ? 0x00FF00 : 0x000000);
    } else {
      if (mEyeLED) mEyeLED->set(0x00FF00);
    }

    // 6) 트래킹 (모션 중에도 동작)
    if (track && mVisionManager && mCamera) {
      double x=0.0, y=0.0;
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

    // 7) 시뮬레이터 틱
    myStep();

    // 8) 모션 종료 → ini 복귀
    bool wasPlaying = isPlaying;
    isPlaying = (now - g_motionStartTime) < g_motionDuration;
    if (wasPlaying && !isPlaying && g_currentMotion != 0) {
      int finished = g_currentMotion; g_currentMotion = 0;
      cout << "⏹️ Motion " << finished << " completed\n";
      if (finished != 1) {
        mMotionManager->playPage(1, false);
        g_motionStartTime = now;
        g_motionDuration  = getMotionDuration(1);
        g_currentMotion   = 1;
        wait(200);
      }
    }
  }
}

// ------------------------ main ------------------------
int main() {
  VisualTracking c;
  c.run();
  return 0;
}
