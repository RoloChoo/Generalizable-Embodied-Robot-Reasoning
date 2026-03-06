// walk.cpp

#include <stdio.h>
#include <string.h>
#include <pthread.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdbool.h>
#include <errno.h>
#include <signal.h>

#include "Walk.hpp"
#include <webots/LED.hpp>
#include <webots/Accelerometer.hpp>
#include <webots/Gyro.hpp>
#include <webots/Motor.hpp>
#include <webots/Camera.hpp>

#include <DARwInOPMotionManager.hpp>
#include <DARwInOPGaitManager.hpp>

// OpenCV 2.4.13
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <vector>

#include <cmath>
#include <iostream>
#include <fstream>
#include <string>
#include <queue>
#include <limits>

using namespace webots;
using namespace managers;
using namespace std;
using namespace cv;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static inline bool isFiniteD(double v) {
  return (v == v) &&
         (v !=  std::numeric_limits<double>::infinity()) &&
         (v != -std::numeric_limits<double>::infinity());
}

static inline double clampd(double v, double mn, double mx) {
  if (!isFiniteD(v)) return 0.0;
  if (mn > mx) return v;
  return (v < mn) ? mn : ((v > mx) ? mx : v);
}

static inline double deadZone(double v, double dz) {
  return fabs(v) < dz ? 0.0 : v;
}

static inline bool parseFiniteDoubleToken(const char* s, double* out) {
  if (!s || !out) return false;
  errno = 0;
  char* endp = NULL;
  double v = strtod(s, &endp);
  if (endp == s || errno == ERANGE || !isFiniteD(v)) return false;
  *out = v;
  return true;
}

static inline bool readQueryValue(const char* line, const char* key, char* out, size_t outSize) {
  if (!line || !key || !out || outSize == 0) return false;
  const char* p = strstr(line, key);
  if (!p) return false;
  p += strlen(key);
  size_t i = 0;
  while (p[i] && p[i] != '&' && p[i] != ' ' && i + 1 < outSize) {
    out[i] = p[i];
    ++i;
  }
  out[i] = '\0';
  return i > 0;
}

static inline bool readQueryDouble(const char* line, const char* key, double* out) {
  char buf[64];
  if (!readQueryValue(line, key, buf, sizeof(buf))) return false;
  return parseFiniteDoubleToken(buf, out);
}

static inline bool readQueryInt(const char* line, const char* key, int* out) {
  char buf[64];
  if (!readQueryValue(line, key, buf, sizeof(buf))) return false;
  *out = atoi(buf);
  return true;
}

static inline double convertYawToNeckAngle(double yawDeg) {
  double robotBaseYaw = 180.0;
  double rel = yawDeg - robotBaseYaw;
  while (rel > 180.0) rel -= 360.0;
  while (rel < -180.0) rel += 360.0;
  return -rel * M_PI / 180.0;
}

static const int NECK_INDEX = 18;
static const int HEAD_INDEX = 19;
static const double MAX_X_AMP = 1.0;
static const double MAX_Y_AMP = 0.8;
static const double MAX_A_AMP = 0.7;
static const double YAW_FILTER_ALPHA = 0.05;
static const double JOINT_FILTER_ALPHA = 0.15;

int gain = 128;
bool isWalking = false;
bool shouldStartWalk = false;
bool shouldStopWalk = false;
double xAmplitude = 0.0;
double yAmplitude = 0.0;
double aAmplitude = 0.0;

bool eyeLedOn = true;
bool headLedOn = true;
bool shouldToggleEyeLed = false;
bool shouldToggleHeadLed = false;
int eyeLedColor = 0x00FF00;
int headLedColor = 0xFF0000;

bool blinkMode = false;
bool shouldToggleBlink = false;
double lastBlinkTime = 0.0;
bool blinkState = true;

double webYawDeg = -1.0;
double targetYawRad = 0.0;
double filteredYawRad = 0.0;
double prevTargetYawRad = 0.0;

// ------------------------ 카메라 JPEG 공유 ------------------------
static pthread_mutex_t g_camera_lock = PTHREAD_MUTEX_INITIALIZER;
static std::vector<uchar> g_latest_jpeg;
static bool g_has_image = false;
static int g_cameraWidth = 0;
static int g_cameraHeight = 0;

static void encodeToJPEG(const unsigned char* image, int width, int height) {
  if (!image || width <= 0 || height <= 0)
    return;

  pthread_mutex_lock(&g_camera_lock);
  try {
    Mat bgra(height, width, CV_8UC4, const_cast<unsigned char*>(image));
    Mat bgr;
    cvtColor(bgra, bgr, CV_BGRA2BGR);

    std::vector<int> params;
    params.push_back(CV_IMWRITE_JPEG_QUALITY);
    params.push_back(80);

    imencode(".jpg", bgr, g_latest_jpeg, params);
    g_has_image = true;
  } catch (const cv::Exception& e) {
    fprintf(stderr, "❌ JPEG encode error: %s\n", e.what());
  }
  pthread_mutex_unlock(&g_camera_lock);
}

static double minMotorPositions[NMOTORS];
static double maxMotorPositions[NMOTORS];
static double targetPositions[NMOTORS] = {0};
static double filteredPositions[NMOTORS] = {0};
static bool jointUpdated[NMOTORS] = {false};

struct MotionCommand {
  int page;
  int returnPage;   // 0 = 마지막 자세 유지
};

static queue<MotionCommand> motionQueue;
static int currentMotionPage = 0;
static int currentMotionReturnPage = 0;
static double motionStartTime = 0.0;
static double motionDuration = 0.0;

pthread_mutex_t stateMutex = PTHREAD_MUTEX_INITIALIZER;

static const char *motorNames[NMOTORS] = {
  "ShoulderR","ShoulderL","ArmUpperR","ArmUpperL",
  "ArmLowerR","ArmLowerL","PelvYR","PelvYL",
  "PelvR","PelvL","LegUpperR","LegUpperL",
  "LegLowerR","LegLowerL","AnkleR","AnkleL",
  "FootR","FootL","Neck","Head"
};

static double getMotionDuration(int page) {
  switch(page) {
    case 1:  return 2.0;
    case 2:  return 2.0;
    case 3:  return 2.0;
    case 4:  return 2.2;
    case 6:  return 3.0;
    case 9:  return 2.5;
    case 10: return 3.0;
    case 11: return 3.0;
    case 12: return 2.6; // right kick
    case 13: return 2.6; // left kick
    case 15: return 2.2; // sit down
    case 16: return 2.0;
    case 23: return 2.0; // yes/go
    case 24: return 3.0; // wow
    case 27: return 2.4; // oops
    case 29: return 3.0;
    case 31: return 3.0;
    case 38: return 4.0; // bye-bye start page
    case 41: return 8.0; // intro start page
    case 54: return 6.0; // clap please start page
    case 57: return 3.0;
    case 70: return 3.0;
    case 71: return 3.0;
    case 90: return 3.0;
    case 91: return 3.0;
    default: return 3.0;
  }
}

static int getMotionReturnPage(int page) {
  switch (page) {
    case 1:   // stand/init
    case 9:   // walkready
    case 15:  // sit
      return 0;   // 자세 유지
    case 12:  // kick right
    case 13:  // kick left
      return 9;   // 킥 후 walkready
    default:
      return 1;   // 나머지는 기본 standing(init)으로 복귀
  }
}

static void queueMotionPageEx(int page, int returnPage = 1) {
  MotionCommand cmd;
  cmd.page = page;
  cmd.returnPage = returnPage;
  motionQueue.push(cmd);
}

// ✅ send()가 끊겨도 SIGPIPE로 프로세스 죽는 거 막고,
// ✅ 부분전송도 안전하게 끝까지 보내기
static bool sendAll(int sock, const void* data, size_t len) {
  const char* p = (const char*)data;
  while (len > 0) {
    ssize_t n = send(sock, p, len, 0);
    if (n < 0) {
      if (errno == EINTR) continue;
      return false;
    }
    if (n == 0) return false;
    p += n;
    len -= (size_t)n;
  }
  return true;
}

char* load_html() {
  static const char html[] =
    "HTTP/1.1 200 OK\r\n"
    "Content-Type: text/html\r\n"
    "Access-Control-Allow-Origin: *\r\n"
    "Connection: close\r\n\r\n"
    "<!DOCTYPE html><html><head><meta charset='utf-8'><title>DARwIn-OP Full Control</title>"
    "<style>body{font-family:Arial,sans-serif;background:#f4f4f4;margin:20px}"
    ".card{background:#fff;padding:16px;border-radius:10px;margin:12px 0;box-shadow:0 2px 6px rgba(0,0,0,.08)}"
    "button{padding:10px 14px;margin:4px;border:none;border-radius:6px;cursor:pointer;font-weight:bold}"
    ".g{background:#4CAF50;color:white}.r{background:#f44336;color:white}.b{background:#2196F3;color:white}.p{background:#9C27B0;color:white}.o{background:#FF9800;color:white}"
    "input[type=range]{width:220px}.row{display:flex;gap:8px;flex-wrap:wrap;align-items:center}</style>"
    "</head><body><h1>🤖 DARwIn-OP Full Controller</h1>"

    "<div class='card'><h3>📷 Camera Preview</h3>"
    "<img id='cam' src='/camera' style='max-width:640px;width:100%;border:2px solid #333;border-radius:8px;'>"
    "<div style='margin-top:8px'>"
    "<button class='b' onclick='refreshCam()'>🔄 Refresh Camera</button>"
    "</div></div>"

    "<div class='card'><h3>Walk</h3>"
    "<div class='row'>"
    "<button class='g' onclick=\"cmd('walk_start')\">Start</button>"
    "<button class='r' onclick=\"cmd('walk_stop')\">Stop</button>"
    "<button class='b' onclick=\"cmd('move_forward')\">Forward</button>"
    "<button class='b' onclick=\"cmd('move_backward')\">Backward</button>"
    "<button class='b' onclick=\"cmd('strafe_left')\">Strafe Left</button>"
    "<button class='b' onclick=\"cmd('strafe_right')\">Strafe Right</button>"
    "<button class='o' onclick=\"cmd('turn_left')\">Turn Left</button>"
    "<button class='o' onclick=\"cmd('turn_right')\">Turn Right</button>"
    "<button class='r' onclick=\"cmd('move_stop')\">Vector Stop</button>"
    "</div></div>"

    "<div class='card'><h3>Vector Control</h3>"
    "<div class='row'>"
    "<label>X <input id='vx' type='range' min='-100' max='100' value='0' oninput='show()'></label>"
    "<label>Y <input id='vy' type='range' min='-80' max='80' value='0' oninput='show()'></label>"
    "<label>A <input id='va' type='range' min='-70' max='70' value='0' oninput='show()'></label>"
    "<button class='g' onclick='sendVector()'>Apply Vector</button>"
    "</div><div id='vec'>x=0 y=0 a=0</div></div>"

    "<div class='card'><h3>Yaw / Head</h3>"
    "<div class='row'>"
    "<label>Yaw <input id='yaw' type='range' min='0' max='360' value='180' oninput='setYaw(this.value)'></label>"
    "<button class='b' onclick='setYaw(180)'>Center</button>"
    "<button class='r' onclick=\"cmd('yaw_off')\">Yaw Off</button>"
    "</div></div>"

    "<div class='card'><h3>Motions</h3>"
    "<div class='row'>"
    "<button class='p' onclick='motion(4)'>Greet</button>"
    "<button class='p' onclick='motion(12)'>Right Kick</button>"
    "<button class='p' onclick='motion(13)'>Left Kick</button>"
    "<button class='p' onclick='motion(15)'>Sit</button>"
    "<button class='p' onclick='motion(1)'>Stand</button>"
    "<button class='p' onclick='motion(23)'>Yes / Go</button>"
    "<button class='p' onclick='motion(24)'>Wow</button>"
    "<button class='p' onclick='motion(27)'>Oops</button>"
    "<button class='p' onclick='motion(38)'>Bye Bye</button>"
    "<button class='p' onclick='motion(41)'>Intro</button>"
    "<button class='p' onclick='motion(54)'>Clap</button>"
    "<button class='o' onclick=\"cmd('motion_showcase')\">Showcase</button>"
    "</div></div>"

    "<div class='card'><h3>LED</h3>"
    "<div class='row'>"
    "<button class='o' onclick=\"cmd('eye_led_toggle')\">Eye LED</button>"
    "<button class='o' onclick=\"cmd('head_led_toggle')\">Head LED</button>"
    "<button class='o' onclick=\"cmd('blink_toggle')\">Blink</button>"
    "</div></div>"

    "<script>"
    "function cmd(c){fetch('/?command='+c).catch(()=>{})}"
    "function motion(p){fetch('/?motion='+p).catch(()=>{})}"
    "function show(){document.getElementById('vec').innerText='x='+(document.getElementById('vx').value/100).toFixed(2)+' y='+(document.getElementById('vy').value/100).toFixed(2)+' a='+(document.getElementById('va').value/100).toFixed(2)}"
    "function sendVector(){var x=(document.getElementById('vx').value/100).toFixed(2);var y=(document.getElementById('vy').value/100).toFixed(2);var a=(document.getElementById('va').value/100).toFixed(2);fetch('/?command=set_walk&x='+x+'&y='+y+'&a='+a).catch(()=>{})}"
    "function setYaw(v){fetch('/?command=set_yaw&yaw='+v).catch(()=>{})}"
    "function refreshCam(){var img=document.getElementById('cam');if(img) img.src='/camera?t='+Date.now();}"
    "setInterval(refreshCam, 250);"
    "show();"
    "</script></body></html>";

  char* result = (char*)malloc(strlen(html) + 1);
  strcpy(result, html);
  return result;
}

static void sendText(int client, const char* contentType, const char* body) {
  if (!body) body = "";
  size_t blen = strlen(body);

  char header[256];
  snprintf(header, sizeof(header),
           "HTTP/1.1 200 OK\r\n"
           "Content-Type: %s\r\n"
           "Content-Length: %zu\r\n"
           "Access-Control-Allow-Origin: *\r\n"
           "Connection: close\r\n\r\n",
           contentType, blen);

  sendAll(client, header, strlen(header));
  if (blen) sendAll(client, body, blen);
}

static void sendStatusJson(int client) {
  char body[512];
  pthread_mutex_lock(&stateMutex);
  snprintf(body, sizeof(body),
           "{\"ok\":true,\"walking\":%s,\"x\":%.3f,\"y\":%.3f,\"a\":%.3f,\"blink\":%s,\"yawDeg\":%.3f,\"motion\":%d}",
           isWalking ? "true" : "false",
           xAmplitude, yAmplitude, aAmplitude,
           blinkMode ? "true" : "false",
           webYawDeg,
           currentMotionPage);
  pthread_mutex_unlock(&stateMutex);
  sendText(client, "application/json", body);
}

void* server(void* arg) {
  (void)arg;

  signal(SIGPIPE, SIG_IGN);

  int s = socket(AF_INET, SOCK_STREAM, 0);
  if (s < 0) {
    printf("Socket creation failed!\n");
    return NULL;
  }

  struct sockaddr_in addr;
  memset(&addr, 0, sizeof(addr));
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
  if (listen(s, 8) < 0) {
    printf("Listen failed!\n");
    close(s);
    return NULL;
  }

  printf("Server running on http://0.0.0.0:8080\n");

  while (1) {
    struct sockaddr_in client_addr;
    socklen_t client_len = sizeof(client_addr);
    int client = accept(s, (struct sockaddr*)&client_addr, &client_len);
    if (client < 0) continue;

    char buf[2048];
    ssize_t bytes_read = recv(client, buf, sizeof(buf) - 1, 0);
    if (bytes_read <= 0) {
      close(client);
      continue;
    }

    buf[bytes_read] = '\0';
    char *get_line = strtok(buf, "\r\n");
    if (!get_line) {
      close(client);
      continue;
    }

    if (strstr(get_line, "command=health")) {
      sendText(client, "text/plain", "OK");
      close(client);
      continue;
    }

    if (strstr(get_line, "command=status")) {
      sendStatusJson(client);
      close(client);
      continue;
    }

    // ---- 카메라 JPEG ----
    if (strstr(get_line, "GET /camera")) {
      pthread_mutex_lock(&g_camera_lock);
      if (g_has_image && !g_latest_jpeg.empty()) {
        char header[256];
        snprintf(header, sizeof(header),
          "HTTP/1.1 200 OK\r\n"
          "Content-Type: image/jpeg\r\n"
          "Content-Length: %zu\r\n"
          "Access-Control-Allow-Origin: *\r\n"
          "Connection: close\r\n\r\n",
          g_latest_jpeg.size());

        sendAll(client, header, strlen(header));
        sendAll(client, &g_latest_jpeg[0], g_latest_jpeg.size());
      } else {
        const char* err =
          "HTTP/1.1 503 Service Unavailable\r\n"
          "Content-Type: text/plain\r\n"
          "Access-Control-Allow-Origin: *\r\n"
          "Connection: close\r\n\r\n"
          "No image";
        sendAll(client, err, strlen(err));
      }
      pthread_mutex_unlock(&g_camera_lock);

      close(client);
      continue;
    }

    // ---- 카메라 상태 JSON ----
    if (strstr(get_line, "GET /info")) {
      char out[256];

      pthread_mutex_lock(&g_camera_lock);
      const bool has = g_has_image;
      const size_t sz = g_latest_jpeg.size();
      const int w = g_cameraWidth;
      const int h = g_cameraHeight;
      pthread_mutex_unlock(&g_camera_lock);

      snprintf(out, sizeof(out),
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: application/json\r\n"
        "Access-Control-Allow-Origin: *\r\n"
        "Connection: close\r\n\r\n"
        "{\"has_image\":%s,\"size\":%zu,\"width\":%d,\"height\":%d}",
        has ? "true" : "false", sz, w, h);

      sendAll(client, out, strlen(out));
      close(client);
      continue;
    }

    pthread_mutex_lock(&stateMutex);

    if (strstr(get_line, "command=walk_start")) {
      shouldStartWalk = true;
      if (fabs(xAmplitude) < 0.001 && fabs(yAmplitude) < 0.001 && fabs(aAmplitude) < 0.001) {
        xAmplitude = 0.35;
      }
      printf("Walk start\n");
    }
    else if (strstr(get_line, "command=walk_stop")) {
      shouldStopWalk = true;
      printf("Walk stop\n");
    }
    else if (strstr(get_line, "command=set_walk")) {
      double x = 0.0, y = 0.0, a = 0.0;
      bool hasAny = false;
      if (readQueryDouble(get_line, "x=", &x)) { x = clampd(x, -MAX_X_AMP, MAX_X_AMP); hasAny = true; }
      if (readQueryDouble(get_line, "y=", &y)) { y = clampd(y, -MAX_Y_AMP, MAX_Y_AMP); hasAny = true; }
      if (readQueryDouble(get_line, "a=", &a)) { a = clampd(a, -MAX_A_AMP, MAX_A_AMP); hasAny = true; }

      if (hasAny) {
        xAmplitude = deadZone(x, 0.03);
        yAmplitude = deadZone(y, 0.03);
        aAmplitude = deadZone(a, 0.03);
        webYawDeg = -1.0;

        bool moving = fabs(xAmplitude) > 0.001 || fabs(yAmplitude) > 0.001 || fabs(aAmplitude) > 0.001;
        if (moving && !isWalking) shouldStartWalk = true;
        if (!moving && isWalking) shouldStopWalk = true;

        printf("Set walk vector: x=%.3f y=%.3f a=%.3f\n", xAmplitude, yAmplitude, aAmplitude);
      }
    }
    else if (strstr(get_line, "command=move_forward")) {
      xAmplitude = 0.5; yAmplitude = 0.0; aAmplitude = 0.0; webYawDeg = -1.0;
      if (!isWalking) shouldStartWalk = true;
      printf("Forward\n");
    }
    else if (strstr(get_line, "command=move_backward")) {
      xAmplitude = -0.4; yAmplitude = 0.0; aAmplitude = 0.0; webYawDeg = -1.0;
      if (!isWalking) shouldStartWalk = true;
      printf("Backward\n");
    }
    else if (strstr(get_line, "command=strafe_left")) {
      xAmplitude = 0.0; yAmplitude = 0.35; aAmplitude = 0.0; webYawDeg = -1.0;
      if (!isWalking) shouldStartWalk = true;
      printf("Strafe left\n");
    }
    else if (strstr(get_line, "command=strafe_right")) {
      xAmplitude = 0.0; yAmplitude = -0.35; aAmplitude = 0.0; webYawDeg = -1.0;
      if (!isWalking) shouldStartWalk = true;
      printf("Strafe right\n");
    }
    else if (strstr(get_line, "command=turn_left")) {
      xAmplitude = 0.0; yAmplitude = 0.0; aAmplitude = 0.35; webYawDeg = -1.0;
      if (!isWalking) shouldStartWalk = true;
      printf("Turn left\n");
    }
    else if (strstr(get_line, "command=turn_right")) {
      xAmplitude = 0.0; yAmplitude = 0.0; aAmplitude = -0.35; webYawDeg = -1.0;
      if (!isWalking) shouldStartWalk = true;
      printf("Turn right\n");
    }
    else if (strstr(get_line, "command=move_stop")) {
      xAmplitude = yAmplitude = aAmplitude = 0.0;
      webYawDeg = -1.0;
      if (isWalking) shouldStopWalk = true;
      printf("Move stop\n");
    }
    else if (strstr(get_line, "command=set_yaw")) {
      double yawDeg = 180.0;
      if (readQueryDouble(get_line, "yaw=", &yawDeg)) {
        webYawDeg = clampd(yawDeg, 0.0, 360.0);
        printf("Set yaw: %.2f deg\n", webYawDeg);
      }
    }
    else if (strstr(get_line, "command=yaw_off")) {
      webYawDeg = -1.0;
      printf("Yaw off\n");
    }
    else if (strstr(get_line, "command=eye_led_toggle")) {
      shouldToggleEyeLed = true;
    }
    else if (strstr(get_line, "command=head_led_toggle")) {
      shouldToggleHeadLed = true;
    }
    else if (strstr(get_line, "command=blink_toggle") || strstr(get_line, "command=led_blink_toggle")) {
      shouldToggleBlink = true;
    }
    else if (strstr(get_line, "command=motion_showcase")) {
      if (isWalking) shouldStopWalk = true;

      queueMotionPageEx(4, 0);   // greet
      queueMotionPageEx(23, 0);  // yes/go
      queueMotionPageEx(24, 0);  // wow
      queueMotionPageEx(38, 0);  // bye-bye
      queueMotionPageEx(54, 1);  // clap -> 끝나면 standing으로 복귀

      printf("Queued motion showcase\n");
    }
    else if (strstr(get_line, "motion=")) {
      int page = 0;
      if (readQueryInt(get_line, "motion=", &page) && page > 0) {
        if (isWalking) shouldStopWalk = true;
        queueMotionPageEx(page, getMotionReturnPage(page));
        printf("Queued motion page: %d (return=%d)\n", page, getMotionReturnPage(page));
      }
    }
    else if (strstr(get_line, "command=set_joint")) {
      int idx = -1;
      double value = 0.0;
      if (readQueryInt(get_line, "index=", &idx) && readQueryDouble(get_line, "value=", &value)) {
        if (idx >= 0 && idx < NMOTORS) {
          targetPositions[idx] = clampd(value, minMotorPositions[idx], maxMotorPositions[idx]);
          jointUpdated[idx] = true;
          printf("Set joint %d -> %.3f\n", idx, targetPositions[idx]);
        }
      }
    }
    else if (strstr(get_line, "command=set_joints")) {
      char valuesBuf[1024];
      if (readQueryValue(get_line, "v=", valuesBuf, sizeof(valuesBuf))) {
        char* tok = strtok(valuesBuf, ",");
        int idx = 0;
        while (tok && idx < NMOTORS) {
          double v = 0.0;
          if (parseFiniteDoubleToken(tok, &v)) {
            targetPositions[idx] = clampd(v, minMotorPositions[idx], maxMotorPositions[idx]);
            jointUpdated[idx] = true;
          }
          tok = strtok(NULL, ",");
          ++idx;
        }
      }
    }

    pthread_mutex_unlock(&stateMutex);

    if (strstr(get_line, "GET /?")) {
      sendText(client, "text/plain", "OK");
      close(client);
      continue;
    }

    char* response = load_html();
    sendAll(client, response, strlen(response));
    free(response);
    close(client);
  }

  close(s);
  return NULL;
}

Walk::Walk(): Robot() {
  mTimeStep = getBasicTimeStep();
  std::cerr << "[INFO] Controller started. TimeStep=" << mTimeStep << "\n";

  mCamera = getCamera("Camera");
  if (mCamera) {
    mCamera->enable(6 * mTimeStep);
    g_cameraWidth = mCamera->getWidth();
    g_cameraHeight = mCamera->getHeight();
    std::cerr << "[INFO] Camera enabled: "
              << g_cameraWidth << "x" << g_cameraHeight << "\n";
  } else {
    std::cerr << "[WARN] Camera not found.\n";
  }

  mEyeLED  = getLED("EyeLed");
  if (mEyeLED) mEyeLED->set(eyeLedColor);

  mHeadLED = getLED("HeadLed");
  if (mHeadLED) mHeadLED->set(headLedColor);

  mAccelerometer = getAccelerometer("Accelerometer");
  if (mAccelerometer == NULL) {
    std::cerr << "[FATAL] Accelerometer not found.\n";
    exit(EXIT_FAILURE);
  }
  mAccelerometer->enable(mTimeStep);

  webots::Gyro* g = getGyro("Gyro");
  if (g == NULL) {
    std::cerr << "[FATAL] Gyro not found.\n";
    exit(EXIT_FAILURE);
  }
  g->enable(mTimeStep);

  int missingMotors = 0;
  for (int i = 0; i < NMOTORS; ++i) {
    mMotors[i] = getMotor(motorNames[i]);
    if (mMotors[i] == NULL) {
      std::cerr << "[ERROR] Motor not found: " << motorNames[i] << "\n";
      missingMotors++;
      continue;
    }
    minMotorPositions[i] = mMotors[i]->getMinPosition();
    maxMotorPositions[i] = mMotors[i]->getMaxPosition();
    targetPositions[i] = 0.0;
    filteredPositions[i] = 0.0;
  }
  if (missingMotors) {
    std::cerr << "[FATAL] Missing motors: " << missingMotors << "\n";
    exit(EXIT_FAILURE);
  }

#ifndef CROSSCOMPILATION
  keyboardEnable(mTimeStep);
#endif

  std::ifstream cfg("config.ini");
  if (!cfg.good()) {
    std::cerr << "[FATAL] config.ini not found in current dir.\n";
    exit(EXIT_FAILURE);
  }

  mMotionManager = new DARwInOPMotionManager(this);
  mGaitManager   = new DARwInOPGaitManager(this, "config.ini");
  std::cerr << "[INFO] Managers initialized.\n";
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

void Walk::updateLEDs() {
  if (shouldToggleEyeLed) {
    eyeLedOn = !eyeLedOn;
    shouldToggleEyeLed = false;
  }
  if (shouldToggleHeadLed) {
    headLedOn = !headLedOn;
    shouldToggleHeadLed = false;
  }
  if (shouldToggleBlink) {
    blinkMode = !blinkMode;
    shouldToggleBlink = false;
    lastBlinkTime = getTime();
  }

  if (blinkMode) {
    double now = getTime();
    if (now - lastBlinkTime > 0.35) {
      blinkState = !blinkState;
      lastBlinkTime = now;
    }
    if (blinkState) {
      if (eyeLedOn && mEyeLED)  mEyeLED->set(eyeLedColor);
      if (headLedOn && mHeadLED) mHeadLED->set(headLedColor);
    } else {
      if (mEyeLED)  mEyeLED->set(0x000000);
      if (mHeadLED) mHeadLED->set(0x000000);
    }
  } else {
    if (mEyeLED)  mEyeLED->set(eyeLedOn ? eyeLedColor : 0x000000);
    if (mHeadLED) mHeadLED->set(headLedOn ? headLedColor : 0x000000);
  }
}

void Walk::checkIfFallen() {
  static int fup = 0;
  static int fdown = 0;
  static const double acc_tolerance = 80.0;
  static const double acc_step = 100;

  const double *acc = mAccelerometer->getValues();
  if (acc[1] < 512.0 - acc_tolerance) fup++; else fup = 0;
  if (acc[1] > 512.0 + acc_tolerance) fdown++; else fdown = 0;

  if (fup > acc_step) {
    mMotionManager->playPage(10);
    mMotionManager->playPage(9);
    fup = 0;
  } else if (fdown > acc_step) {
    mMotionManager->playPage(11);
    mMotionManager->playPage(9);
    fdown = 0;
  }
}

void Walk::run() {
  cout << "-------Walk example of DARwIn-OP Full Controller-------" << endl;
  cout << "Web control:  http://localhost:8080" << endl;

  pthread_t serverThread;
  if (pthread_create(&serverThread, NULL, server, NULL) != 0) {
    cout << "Failed to create server thread!" << endl;
    return;
  }

  myStep();
  mMotionManager->playPage(9);
  wait(200);

  bool gaitStarted = false;

  int camWarmup = 10;
  double lastCamEnc = 0.0;

  while (true) {
    checkIfFallen();

    if (mCamera) {
      double t = getTime();
      if (camWarmup > 0) {
        camWarmup--;
      } else if ((t - lastCamEnc) > 0.12) {
        const unsigned char* img = mCamera->getImage();
        if (img) encodeToJPEG(img, g_cameraWidth, g_cameraHeight);
        lastCamEnc = t;
      }
    }

    pthread_mutex_lock(&stateMutex);

    updateLEDs();

    if (shouldStartWalk && !isWalking) {
      mGaitManager->start();
      mGaitManager->step(mTimeStep);
      isWalking = true;
      gaitStarted = true;
      shouldStartWalk = false;
      shouldStopWalk = false;
    }

    if (shouldStopWalk && isWalking) {
      mGaitManager->stop();
      isWalking = false;
      gaitStarted = false;
      shouldStopWalk = false;
      shouldStartWalk = false;
      xAmplitude = yAmplitude = aAmplitude = 0.0;
    }

    double now = getTime();
    bool motionPlaying = (currentMotionPage != 0) && ((now - motionStartTime) < motionDuration);

    if (!motionQueue.empty() && !isWalking && !motionPlaying) {
      MotionCommand cmd = motionQueue.front();
      motionQueue.pop();

      mMotionManager->playPage(cmd.page, false);
      currentMotionPage = cmd.page;
      currentMotionReturnPage = cmd.returnPage;
      motionStartTime = now;
      motionDuration = getMotionDuration(cmd.page);
      motionPlaying = true;
    }

    if (!motionPlaying && currentMotionPage != 0) {
      if (currentMotionReturnPage > 0 &&
          currentMotionPage != currentMotionReturnPage &&
          !isWalking) {
        mMotionManager->playPage(currentMotionReturnPage, false);
        currentMotionPage = currentMotionReturnPage;
        motionStartTime = now;
        motionDuration = getMotionDuration(currentMotionReturnPage);
        currentMotionReturnPage = 0;
      } else {
        currentMotionPage = 0;
        currentMotionReturnPage = 0;
      }
    }

    if (isWalking && gaitStarted) {
      bool yawMode = (webYawDeg >= 0.0);
      if (yawMode) {
        targetYawRad = convertYawToNeckAngle(webYawDeg);
        filteredYawRad = YAW_FILTER_ALPHA * targetYawRad + (1.0 - YAW_FILTER_ALPHA) * filteredYawRad;
        filteredYawRad = clampd(filteredYawRad, minMotorPositions[NECK_INDEX], maxMotorPositions[NECK_INDEX]);
        if (mMotors[NECK_INDEX]) {
          mMotors[NECK_INDEX]->setPosition(filteredYawRad);
        }

        double yawDiff = fabs(targetYawRad - prevTargetYawRad);
        if (fabs(xAmplitude) < 0.001 && fabs(yAmplitude) < 0.001 && fabs(aAmplitude) < 0.001) {
          if (yawDiff < 0.1) {
            mGaitManager->setXAmplitude(0.8);
            mGaitManager->setYAmplitude(0.0);
            mGaitManager->setAAmplitude(filteredYawRad * 0.5);
          } else {
            mGaitManager->setXAmplitude(0.4);
            mGaitManager->setYAmplitude(0.0);
            mGaitManager->setAAmplitude(filteredYawRad);
          }
        } else {
          mGaitManager->setXAmplitude(xAmplitude);
          mGaitManager->setYAmplitude(yAmplitude);
          mGaitManager->setAAmplitude(aAmplitude);
        }
        mGaitManager->step(mTimeStep);
        prevTargetYawRad = targetYawRad;
      } else {
        mGaitManager->setXAmplitude(xAmplitude);
        mGaitManager->setYAmplitude(yAmplitude);
        mGaitManager->setAAmplitude(aAmplitude);
        mGaitManager->step(mTimeStep);
      }
    }

    for (int i = 0; i < NMOTORS; ++i) {
      if (!jointUpdated[i]) continue;
      double alpha = (i == NECK_INDEX || i == HEAD_INDEX) ? 0.4 : JOINT_FILTER_ALPHA;
      filteredPositions[i] = alpha * targetPositions[i] + (1.0 - alpha) * filteredPositions[i];
      double safe = clampd(filteredPositions[i], minMotorPositions[i], maxMotorPositions[i]);
      if (mMotors[i]) mMotors[i]->setPosition(safe);
      if (fabs(filteredPositions[i] - targetPositions[i]) < 0.001) {
        jointUpdated[i] = false;
      }
    }

    pthread_mutex_unlock(&stateMutex);
    myStep();
  }
}
