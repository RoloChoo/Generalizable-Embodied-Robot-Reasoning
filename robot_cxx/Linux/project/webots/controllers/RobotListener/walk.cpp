//walk.cpp

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
#include <queue>
#include <limits>
#include <cmath>
#include <iostream>
#include <fstream>
#include <string>

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
  if (!isFiniteD(v))
    return 0.0;
  if (mn > mx)
    return v;
  return (v < mn) ? mn : ((v > mx) ? mx : v);
}

static inline double deadZone(double v, double dz) {
  return fabs(v) < dz ? 0.0 : v;
}

static inline bool parseFiniteDoubleToken(const char* s, double* out) {
  if (!s || !out)
    return false;
  errno = 0;
  char* endp = NULL;
  double v = strtod(s, &endp);
  if (endp == s || errno == ERANGE || !isFiniteD(v))
    return false;
  *out = v;
  return true;
}

static inline bool readQueryValue(const char* line, const char* key, char* out, size_t outSize) {
  if (!line || !key || !out || outSize == 0)
    return false;

  const char* p = strstr(line, key);
  if (!p)
    return false;

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
  if (!readQueryValue(line, key, buf, sizeof(buf)))
    return false;
  return parseFiniteDoubleToken(buf, out);
}

static inline bool readQueryInt(const char* line, const char* key, int* out) {
  char buf[64];
  if (!readQueryValue(line, key, buf, sizeof(buf)))
    return false;
  *out = atoi(buf);
  return true;
}

static inline double convertYawToNeckAngle(double yawDeg) {
  double robotBaseYaw = 180.0;
  double rel = yawDeg - robotBaseYaw;
  while (rel > 180.0)
    rel -= 360.0;
  while (rel < -180.0)
    rel += 360.0;
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

// ------------------------ camera JPEG 공유 ------------------------
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
    fprintf(stderr, "JPEG encode error: %s\n", e.what());
  }
  pthread_mutex_unlock(&g_camera_lock);
}

static double minMotorPositions[NMOTORS];
static double maxMotorPositions[NMOTORS];
static double targetPositions[NMOTORS] = {0};
static double filteredPositions[NMOTORS] = {0};
static bool jointUpdated[NMOTORS] = {false};

enum PoseState {
  POSE_UNKNOWN = 0,
  POSE_STANDING,
  POSE_WALKREADY,
  POSE_SEATED,
  POSE_LYING_FRONT,
  POSE_LYING_BACK
};

static const char* poseName(PoseState pose) {
  switch (pose) {
    case POSE_STANDING:    return "standing";
    case POSE_WALKREADY:   return "walkready";
    case POSE_SEATED:      return "seated";
    case POSE_LYING_FRONT: return "lying_front";
    case POSE_LYING_BACK:  return "lying_back";
    default:               return "unknown";
  }
}

static inline bool isStandingLikePose(PoseState pose) {
  return pose == POSE_STANDING || pose == POSE_WALKREADY || pose == POSE_UNKNOWN;
}

struct MotionCommand {
  int page;
  int returnPage;   // 0 = 마지막 자세 유지
  PoseState finalPose;
};

static queue<MotionCommand> motionQueue;
static int currentMotionPage = 0;
static int currentMotionReturnPage = 0;
static PoseState currentPose = POSE_WALKREADY;
static PoseState plannedPose = POSE_WALKREADY;
static PoseState currentMotionFinalPose = POSE_WALKREADY;

pthread_mutex_t stateMutex = PTHREAD_MUTEX_INITIALIZER;

static const char *motorNames[NMOTORS] = {
  "ShoulderR", "ShoulderL", "ArmUpperR", "ArmUpperL",
  "ArmLowerR", "ArmLowerL", "PelvYR", "PelvYL",
  "PelvR", "PelvL", "LegUpperR", "LegUpperL",
  "LegLowerR", "LegLowerL", "AnkleR", "AnkleL",
  "FootR", "FootL", "Neck", "Head"
};

static int getMotionReturnPage(int page) {
  switch (page) {
    case 1:
    case 9:
    case 15:
    case 16:
    case 90:
    case 91:
      return 0;   // final pose 유지

    case 10:
    case 11:
    case 12:
    case 13:
    case 70:
    case 71:
      return 9;   // walkready로 정리

    case 17:
      return 1;   // headstand는 안전하게 standing 복귀

    default:
      return 1;   // 기본 standing(init)으로 복귀
  }
}

static PoseState getRequiredPoseForMotion(int page) {
  switch (page) {
    case 10: return POSE_LYING_FRONT;
    case 11: return POSE_LYING_BACK;
    case 16: return POSE_SEATED;

    case 1:
    case 2:
    case 3:
    case 4:
    case 6:
    case 9:
    case 12:
    case 13:
    case 15:
    case 17:
    case 23:
    case 24:
    case 27:
    case 29:
    case 31:
    case 38:
    case 41:
    case 54:
    case 57:
    case 70:
    case 71:
    case 90:
    case 91:
    case 237:
    case 239:
      return POSE_STANDING;

    default:
      return POSE_UNKNOWN;
  }
}

static PoseState getPoseForPage(int page) {
  switch (page) {
    case 1:  return POSE_STANDING;
    case 9:  return POSE_WALKREADY;
    case 15: return POSE_SEATED;
    case 16: return POSE_STANDING;
    case 90: return POSE_LYING_FRONT;
    case 91: return POSE_LYING_BACK;

    case 10:
    case 11:
    case 12:
    case 13:
    case 70:
    case 71:
      return POSE_WALKREADY;

    default:
      return POSE_STANDING;
  }
}

static PoseState getFinalPoseForMotion(int page, int returnPage) {
  if (returnPage > 0 && returnPage != page)
    return getPoseForPage(returnPage);
  return getPoseForPage(page);
}

static bool canStartMotionFromPose(int page, PoseState pose) {
  PoseState required = getRequiredPoseForMotion(page);
  if (required == POSE_UNKNOWN || pose == POSE_UNKNOWN)
    return true;

  if (required == POSE_STANDING)
    return isStandingLikePose(pose);

  return pose == required;
}

static bool queueMotionPageEx(int page, int returnPage = -1) {
  if (page <= 0)
    return false;

  if (returnPage < 0)
    returnPage = getMotionReturnPage(page);

  if (!canStartMotionFromPose(page, plannedPose)) {
    printf("Rejected motion %d: plannedPose=%s required=%s\n",
           page, poseName(plannedPose), poseName(getRequiredPoseForMotion(page)));
    return false;
  }

  MotionCommand cmd;
  cmd.page = page;
  cmd.returnPage = returnPage;
  cmd.finalPose = getFinalPoseForMotion(page, returnPage);

  motionQueue.push(cmd);
  plannedPose = cmd.finalPose;
  return true;
}

static bool queueRecoverToStandingLike() {
  if (isStandingLikePose(plannedPose))
    return true;

  if (plannedPose == POSE_SEATED)
    return queueMotionPageEx(16, 0);

  if (plannedPose == POSE_LYING_FRONT)
    return queueMotionPageEx(10, 9);

  if (plannedPose == POSE_LYING_BACK)
    return queueMotionPageEx(11, 9);

  return false;
}

static bool queueSitStandSequence() {
  if (plannedPose == POSE_SEATED)
    return queueMotionPageEx(16, 0);

  if (isStandingLikePose(plannedPose)) {
    return queueMotionPageEx(15, 0) && queueMotionPageEx(16, 0);
  }

  return false;
}

static bool queueFrontRecoverSequence() {
  if (plannedPose == POSE_LYING_FRONT)
    return queueMotionPageEx(10, 9);

  if (isStandingLikePose(plannedPose)) {
    return queueMotionPageEx(90, 0) && queueMotionPageEx(10, 9);
  }

  return false;
}

static bool queueBackRecoverSequence() {
  if (plannedPose == POSE_LYING_BACK)
    return queueMotionPageEx(11, 9);

  if (isStandingLikePose(plannedPose)) {
    return queueMotionPageEx(91, 0) && queueMotionPageEx(11, 9);
  }

  return false;
}

// send()가 끊겨도 SIGPIPE로 프로세스 죽는 거 방지 +
// 부분 전송도 끝까지 보내기
static bool sendAll(int sock, const void* data, size_t len) {
  const char* p = (const char*)data;
  while (len > 0) {
    ssize_t n = send(sock, p, len, 0);
    if (n < 0) {
      if (errno == EINTR)
        continue;
      return false;
    }
    if (n == 0)
      return false;
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
    "<style>"
    "body{font-family:Arial,sans-serif;background:#f4f4f4;margin:20px}"
    ".card{background:#fff;padding:16px;border-radius:10px;margin:12px 0;box-shadow:0 2px 6px rgba(0,0,0,.08)}"
    "button{padding:10px 14px;margin:4px;border:none;border-radius:6px;cursor:pointer;font-weight:bold}"
    ".g{background:#4CAF50;color:white}.r{background:#f44336;color:white}.b{background:#2196F3;color:white}.p{background:#9C27B0;color:white}.o{background:#FF9800;color:white}"
    "input[type=range]{width:220px}.row{display:flex;gap:8px;flex-wrap:wrap;align-items:center}"
    ".mono{font-family:monospace;background:#f8f8f8;padding:8px;border-radius:6px}"
    ".note{color:#555;font-size:14px}"
    "</style>"
    "</head><body><h1>DARwIn-OP Full Controller</h1>"

    "<div class='card'><h3>Camera Preview</h3>"
    "<img id='cam' src='/camera' style='max-width:640px;width:100%;border:2px solid #333;border-radius:8px;'>"
    "<div style='margin-top:8px'>"
    "<button class='b' onclick='refreshCam()'>Refresh Camera</button>"
    "</div></div>"

    "<div class='card'><h3>Status</h3>"
    "<div id='status' class='mono'>loading...</div>"
    "<div class='note'>Direct pages 10/11/16 need the correct initial pose. Use recovery buttons when needed.</div>"
    "</div>"

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
    "<button class='p' onclick='motion(1)'>Init</button>"
    "<button class='p' onclick='motion(9)'>Walk Ready</button>"
    "<button class='p' onclick='motion(2)'>Nod</button>"
    "<button class='p' onclick='motion(3)'>Shake</button>"
    "<button class='p' onclick='motion(4)'>Bow</button>"
    "<button class='p' onclick='motion(6)'>Talk1</button>"
    "<button class='p' onclick='motion(23)'>Yes / Go</button>"
    "<button class='p' onclick='motion(24)'>Applaud</button>"
    "<button class='p' onclick='motion(27)'>Yes+Head</button>"
    "<button class='p' onclick='motion(29)'>Talk2</button>"
    "<button class='p' onclick='motion(31)'>Stretch</button>"
    "<button class='p' onclick='motion(38)'>Wave</button>"
    "<button class='p' onclick='motion(41)'>Intro</button>"
    "<button class='p' onclick='motion(54)'>Clap Loud</button>"
    "<button class='p' onclick='motion(57)'>Clap</button>"
    "</div>"

    "<div class='row'>"
    "<button class='p' onclick='motion(12)'>Right Kick</button>"
    "<button class='p' onclick='motion(13)'>Left Kick</button>"
    "<button class='p' onclick='motion(70)'>Right Pass</button>"
    "<button class='p' onclick='motion(71)'>Left Pass</button>"
    "<button class='p' onclick='motion(17)'>Headstand</button>"
    "<button class='p' onclick='motion(237)'>Jump</button>"
    "<button class='p' onclick='motion(239)'>Jump Fast</button>"
    "</div>"

    "<div class='row'>"
    "<button class='p' onclick='motion(15)'>Sit</button>"
    "<button class='p' onclick='motion(16)'>Stand Up</button>"
    "<button class='p' onclick=\"cmd('sit_stand')\">Sit -> Stand</button>"
    "<button class='p' onclick='motion(90)'>Lie Front</button>"
    "<button class='p' onclick='motion(10)'>Get Up Front</button>"
    "<button class='p' onclick=\"cmd('front_recover')\">Front Recover</button>"
    "<button class='p' onclick='motion(91)'>Lie Back</button>"
    "<button class='p' onclick='motion(11)'>Get Up Back</button>"
    "<button class='p' onclick=\"cmd('back_recover')\">Back Recover</button>"
    "<button class='o' onclick=\"cmd('motion_showcase')\">Showcase</button>"
    "</div></div>"

    "<div class='card'><h3>LED</h3>"
    "<div class='row'>"
    "<button class='o' onclick=\"cmd('eye_led_toggle')\">Eye LED</button>"
    "<button class='o' onclick=\"cmd('head_led_toggle')\">Head LED</button>"
    "<button class='o' onclick=\"cmd('blink_toggle')\">Blink</button>"
    "</div></div>"

    "<script>"
    "function cmd(c){fetch('/?command='+c).catch(function(){})}"
    "function motion(p){fetch('/?motion='+p).catch(function(){})}"
    "function show(){"
      "document.getElementById('vec').innerText="
      "'x='+(document.getElementById('vx').value/100).toFixed(2)+"
      "' y='+(document.getElementById('vy').value/100).toFixed(2)+"
      "' a='+(document.getElementById('va').value/100).toFixed(2)"
    "}"
    "function sendVector(){"
      "var x=(document.getElementById('vx').value/100).toFixed(2);"
      "var y=(document.getElementById('vy').value/100).toFixed(2);"
      "var a=(document.getElementById('va').value/100).toFixed(2);"
      "fetch('/?command=set_walk&x='+x+'&y='+y+'&a='+a).catch(function(){})"
    "}"
    "function setYaw(v){fetch('/?command=set_yaw&yaw='+v).catch(function(){})}"
    "function refreshCam(){"
      "var img=document.getElementById('cam');"
      "if(img) img.src='/camera?t='+Date.now();"
    "}"
    "function refreshStatus(){"
      "fetch('/?command=status').then(function(r){return r.json();}).then(function(s){"
        "var text='pose='+s.pose+' planned='+s.plannedPose+' walking='+s.walking+' motion='+s.motion+' queue='+s.queue;"
        "text+=' | x='+s.x.toFixed(2)+' y='+s.y.toFixed(2)+' a='+s.a.toFixed(2);"
        "document.getElementById('status').innerText=text;"
      "}).catch(function(){})"
    "}"
    "setInterval(refreshCam, 250);"
    "setInterval(refreshStatus, 350);"
    "show();"
    "refreshStatus();"
    "</script></body></html>";

  char* result = (char*)malloc(strlen(html) + 1);
  strcpy(result, html);
  return result;
}

static void sendText(int client, const char* contentType, const char* body) {
  if (!body)
    body = "";

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
  if (blen)
    sendAll(client, body, blen);
}

static void sendStatusJson(int client) {
  char body[768];
  pthread_mutex_lock(&stateMutex);
  const size_t qsz = motionQueue.size();
  snprintf(body, sizeof(body),
           "{\"ok\":true,\"walking\":%s,\"x\":%.3f,\"y\":%.3f,\"a\":%.3f,"
           "\"blink\":%s,\"yawDeg\":%.3f,\"motion\":%d,"
           "\"pose\":\"%s\",\"plannedPose\":\"%s\",\"queue\":%zu}",
           isWalking ? "true" : "false",
           xAmplitude, yAmplitude, aAmplitude,
           blinkMode ? "true" : "false",
           webYawDeg,
           currentMotionPage,
           poseName(currentPose),
           poseName(plannedPose),
           qsz);
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
    if (client < 0)
      continue;

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

    // ---- camera JPEG ----
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

    // ---- camera state JSON ----
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
      if (fabs(xAmplitude) < 0.001 && fabs(yAmplitude) < 0.001 && fabs(aAmplitude) < 0.001)
        xAmplitude = 0.35;
      printf("Walk start requested\n");
    }
    else if (strstr(get_line, "command=walk_stop")) {
      shouldStopWalk = true;
      printf("Walk stop requested\n");
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
        if (moving && !isWalking)
          shouldStartWalk = true;
        if (!moving && isWalking)
          shouldStopWalk = true;

        printf("Set walk vector: x=%.3f y=%.3f a=%.3f\n", xAmplitude, yAmplitude, aAmplitude);
      }
    }
    else if (strstr(get_line, "command=move_forward")) {
      xAmplitude = 0.50;
      yAmplitude = 0.0;
      aAmplitude = 0.0;
      webYawDeg = -1.0;
      if (!isWalking)
        shouldStartWalk = true;
      printf("Forward\n");
    }
    else if (strstr(get_line, "command=move_backward")) {
      xAmplitude = -0.40;
      yAmplitude = 0.0;
      aAmplitude = 0.0;
      webYawDeg = -1.0;
      if (!isWalking)
        shouldStartWalk = true;
      printf("Backward\n");
    }
    else if (strstr(get_line, "command=strafe_left")) {
      xAmplitude = 0.0;
      yAmplitude = 0.35;
      aAmplitude = 0.0;
      webYawDeg = -1.0;
      if (!isWalking)
        shouldStartWalk = true;
      printf("Strafe left\n");
    }
    else if (strstr(get_line, "command=strafe_right")) {
      xAmplitude = 0.0;
      yAmplitude = -0.35;
      aAmplitude = 0.0;
      webYawDeg = -1.0;
      if (!isWalking)
        shouldStartWalk = true;
      printf("Strafe right\n");
    }
    else if (strstr(get_line, "command=turn_left")) {
      xAmplitude = 0.0;
      yAmplitude = 0.0;
      aAmplitude = 0.35;
      webYawDeg = -1.0;
      if (!isWalking)
        shouldStartWalk = true;
      printf("Turn left\n");
    }
    else if (strstr(get_line, "command=turn_right")) {
      xAmplitude = 0.0;
      yAmplitude = 0.0;
      aAmplitude = -0.35;
      webYawDeg = -1.0;
      if (!isWalking)
        shouldStartWalk = true;
      printf("Turn right\n");
    }
    else if (strstr(get_line, "command=move_stop")) {
      xAmplitude = 0.0;
      yAmplitude = 0.0;
      aAmplitude = 0.0;
      webYawDeg = -1.0;
      if (isWalking)
        shouldStopWalk = true;
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
    else if (strstr(get_line, "command=sit_stand")) {
      if (isWalking)
        shouldStopWalk = true;

      if (queueSitStandSequence())
        printf("Queued sit -> stand sequence\n");
      else
        printf("Cannot queue sit -> stand from pose=%s\n", poseName(plannedPose));
    }
    else if (strstr(get_line, "command=front_recover")) {
      if (isWalking)
        shouldStopWalk = true;

      if (queueFrontRecoverSequence())
        printf("Queued front recover sequence\n");
      else
        printf("Cannot queue front recover from pose=%s\n", poseName(plannedPose));
    }
    else if (strstr(get_line, "command=back_recover")) {
      if (isWalking)
        shouldStopWalk = true;

      if (queueBackRecoverSequence())
        printf("Queued back recover sequence\n");
      else
        printf("Cannot queue back recover from pose=%s\n", poseName(plannedPose));
    }
    else if (strstr(get_line, "command=motion_showcase")) {
      if (isWalking)
        shouldStopWalk = true;

      if (queueRecoverToStandingLike()) {
        queueMotionPageEx(2);
        queueMotionPageEx(3);
        queueMotionPageEx(4);
        queueMotionPageEx(6);
        queueMotionPageEx(23);
        queueMotionPageEx(29);
        queueMotionPageEx(31);
        queueMotionPageEx(38);
        queueMotionPageEx(57);
        printf("Queued motion showcase\n");
      } else {
        printf("Cannot queue showcase from pose=%s\n", poseName(plannedPose));
      }
    }
    else if (strstr(get_line, "motion=")) {
      int page = 0;
      if (readQueryInt(get_line, "motion=", &page) && page > 0) {
        if (isWalking)
          shouldStopWalk = true;

        if (queueMotionPageEx(page))
          printf("Queued motion page: %d\n", page);
        else
          printf("Motion page %d rejected from pose=%s\n", page, poseName(plannedPose));
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
        printf("Queued batch joint update\n");
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

Walk::Walk() : Robot() {
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

  mEyeLED = getLED("EyeLed");
  if (mEyeLED)
    mEyeLED->set(eyeLedColor);

  mHeadLED = getLED("HeadLed");
  if (mHeadLED)
    mHeadLED->set(headLedColor);

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
      ++missingMotors;
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
  mGaitManager = new DARwInOPGaitManager(this, "config.ini");
  std::cerr << "[INFO] Managers initialized.\n";
}

Walk::~Walk() {
  delete mMotionManager;
  delete mGaitManager;
}

void Walk::myStep() {
  if (step(mTimeStep) == -1)
    exit(EXIT_SUCCESS);
}

void Walk::wait(int ms) {
  double t0 = getTime();
  double dur = ms / 1000.0;
  while (getTime() < t0 + dur)
    myStep();
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
      if (eyeLedOn && mEyeLED)
        mEyeLED->set(eyeLedColor);
      if (headLedOn && mHeadLED)
        mHeadLED->set(headLedColor);
    } else {
      if (mEyeLED)
        mEyeLED->set(0x000000);
      if (mHeadLED)
        mHeadLED->set(0x000000);
    }
  } else {
    if (mEyeLED)
      mEyeLED->set(eyeLedOn ? eyeLedColor : 0x000000);
    if (mHeadLED)
      mHeadLED->set(headLedOn ? headLedColor : 0x000000);
  }
}

void Walk::checkIfFallen() {
  static int fup = 0;
  static int fdown = 0;
  static const double acc_tolerance = 80.0;
  static const double acc_step = 100.0;

  pthread_mutex_lock(&stateMutex);
  PoseState pose = currentPose;
  int motionPage = currentMotionPage;
  pthread_mutex_unlock(&stateMutex);

  // 의도적으로 눕거나 앉아 있는 상태 / 모션 진행 중에는 자동 기상 비활성화
  if (mMotionManager->isMotionPlaying() ||
      motionPage != 0 ||
      pose == POSE_SEATED ||
      pose == POSE_LYING_FRONT ||
      pose == POSE_LYING_BACK) {
    fup = 0;
    fdown = 0;
    return;
  }

  const double *acc = mAccelerometer->getValues();
  if (acc[1] < 512.0 - acc_tolerance)
    ++fup;
  else
    fup = 0;

  if (acc[1] > 512.0 + acc_tolerance)
    ++fdown;
  else
    fdown = 0;

  auto runMotionBlocking = [&](int page) {
    mMotionManager->playPage(page, false);
    while (mMotionManager->isMotionPlaying()) {
      mMotionManager->step(mTimeStep);
      myStep();
    }
  };

  if (fup > acc_step) {
    pthread_mutex_lock(&stateMutex);
    if (isWalking) {
      mGaitManager->stop();
      isWalking = false;
    }
    shouldStartWalk = false;
    shouldStopWalk = false;
    xAmplitude = 0.0;
    yAmplitude = 0.0;
    aAmplitude = 0.0;
    currentMotionPage = 10;
    currentMotionReturnPage = 9;
    currentMotionFinalPose = POSE_WALKREADY;
    pthread_mutex_unlock(&stateMutex);

    runMotionBlocking(10);

    pthread_mutex_lock(&stateMutex);
    currentMotionPage = 9;
    currentMotionReturnPage = 0;
    pthread_mutex_unlock(&stateMutex);

    runMotionBlocking(9);

    pthread_mutex_lock(&stateMutex);
    currentMotionPage = 0;
    currentMotionReturnPage = 0;
    currentMotionFinalPose = POSE_WALKREADY;
    currentPose = POSE_WALKREADY;
    if (motionQueue.empty())
      plannedPose = currentPose;
    pthread_mutex_unlock(&stateMutex);

    fup = 0;
    fdown = 0;
    printf("Auto recovery from front fall\n");
  } else if (fdown > acc_step) {
    pthread_mutex_lock(&stateMutex);
    if (isWalking) {
      mGaitManager->stop();
      isWalking = false;
    }
    shouldStartWalk = false;
    shouldStopWalk = false;
    xAmplitude = 0.0;
    yAmplitude = 0.0;
    aAmplitude = 0.0;
    currentMotionPage = 11;
    currentMotionReturnPage = 9;
    currentMotionFinalPose = POSE_WALKREADY;
    pthread_mutex_unlock(&stateMutex);

    runMotionBlocking(11);

    pthread_mutex_lock(&stateMutex);
    currentMotionPage = 9;
    currentMotionReturnPage = 0;
    pthread_mutex_unlock(&stateMutex);

    runMotionBlocking(9);

    pthread_mutex_lock(&stateMutex);
    currentMotionPage = 0;
    currentMotionReturnPage = 0;
    currentMotionFinalPose = POSE_WALKREADY;
    currentPose = POSE_WALKREADY;
    if (motionQueue.empty())
      plannedPose = currentPose;
    pthread_mutex_unlock(&stateMutex);

    fup = 0;
    fdown = 0;
    printf("Auto recovery from back fall\n");
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

  pthread_mutex_lock(&stateMutex);
  currentPose = POSE_WALKREADY;
  plannedPose = currentPose;
  currentMotionPage = 9;
  currentMotionReturnPage = 0;
  currentMotionFinalPose = POSE_WALKREADY;
  pthread_mutex_unlock(&stateMutex);

  mMotionManager->playPage(9, false);
  while (mMotionManager->isMotionPlaying()) {
    mMotionManager->step(mTimeStep);
    myStep();
  }

  pthread_mutex_lock(&stateMutex);
  currentMotionPage = 0;
  currentMotionReturnPage = 0;
  currentMotionFinalPose = POSE_WALKREADY;
  currentPose = POSE_WALKREADY;
  plannedPose = currentPose;
  pthread_mutex_unlock(&stateMutex);

  bool gaitStarted = false;
  int camWarmup = 10;
  double lastCamEnc = 0.0;

  while (true) {
    checkIfFallen();

    if (mCamera) {
      double t = getTime();
      if (camWarmup > 0) {
        --camWarmup;
      } else if ((t - lastCamEnc) > 0.12) {
        const unsigned char* img = mCamera->getImage();
        if (img)
          encodeToJPEG(img, g_cameraWidth, g_cameraHeight);
        lastCamEnc = t;
      }
    }

    pthread_mutex_lock(&stateMutex);

    updateLEDs();

    if (shouldStartWalk && !isWalking) {
      if (currentMotionPage != 0 || !motionQueue.empty()) {
        shouldStartWalk = false;
      } else if (currentPose == POSE_SEATED ||
                 currentPose == POSE_LYING_FRONT ||
                 currentPose == POSE_LYING_BACK) {
        printf("Cannot start walk from pose=%s\n", poseName(currentPose));
        shouldStartWalk = false;
      } else {
        mGaitManager->start();
        mGaitManager->step(mTimeStep);
        isWalking = true;
        gaitStarted = true;
        shouldStartWalk = false;
        shouldStopWalk = false;
        currentPose = POSE_WALKREADY;
        plannedPose = currentPose;
      }
    }

    if (shouldStopWalk && isWalking) {
      mGaitManager->stop();
      isWalking = false;
      gaitStarted = false;
      shouldStopWalk = false;
      shouldStartWalk = false;
      xAmplitude = 0.0;
      yAmplitude = 0.0;
      aAmplitude = 0.0;
      currentPose = POSE_WALKREADY;
      if (motionQueue.empty())
        plannedPose = currentPose;
    }

    bool motionPlaying = mMotionManager->isMotionPlaying();

    if (!motionQueue.empty() && !isWalking && !motionPlaying && currentMotionPage == 0) {
      MotionCommand cmd = motionQueue.front();
      motionQueue.pop();

      mMotionManager->playPage(cmd.page, false);
      currentMotionPage = cmd.page;
      currentMotionReturnPage = cmd.returnPage;
      currentMotionFinalPose = cmd.finalPose;
      motionPlaying = true;
    }

    if (currentMotionPage != 0 || motionPlaying)
      mMotionManager->step(mTimeStep);

    motionPlaying = mMotionManager->isMotionPlaying();

    if (!motionPlaying && currentMotionPage != 0) {
      if (currentMotionReturnPage > 0 &&
          currentMotionPage != currentMotionReturnPage &&
          !isWalking) {
        mMotionManager->playPage(currentMotionReturnPage, false);
        currentMotionPage = currentMotionReturnPage;
        currentMotionReturnPage = 0;
      } else {
        currentPose = currentMotionFinalPose;
        currentMotionPage = 0;
        currentMotionReturnPage = 0;
        currentMotionFinalPose = currentPose;
        if (motionQueue.empty())
          plannedPose = currentPose;
      }
    }

    if (isWalking && gaitStarted) {
      bool yawMode = (webYawDeg >= 0.0);
      if (yawMode) {
        targetYawRad = convertYawToNeckAngle(webYawDeg);
        filteredYawRad = YAW_FILTER_ALPHA * targetYawRad + (1.0 - YAW_FILTER_ALPHA) * filteredYawRad;
        filteredYawRad = clampd(filteredYawRad, minMotorPositions[NECK_INDEX], maxMotorPositions[NECK_INDEX]);

        if (mMotors[NECK_INDEX])
          mMotors[NECK_INDEX]->setPosition(filteredYawRad);

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

    bool manualJointsAllowed = !isWalking && currentMotionPage == 0 && !mMotionManager->isMotionPlaying();
    if (manualJointsAllowed) {
      for (int i = 0; i < NMOTORS; ++i) {
        if (!jointUpdated[i])
          continue;

        double alpha = (i == NECK_INDEX || i == HEAD_INDEX) ? 0.4 : JOINT_FILTER_ALPHA;
        filteredPositions[i] = alpha * targetPositions[i] + (1.0 - alpha) * filteredPositions[i];
        double safe = clampd(filteredPositions[i], minMotorPositions[i], maxMotorPositions[i]);

        if (mMotors[i])
          mMotors[i]->setPosition(safe);

        if (fabs(filteredPositions[i] - targetPositions[i]) < 0.001)
          jointUpdated[i] = false;
      }
    }

    pthread_mutex_unlock(&stateMutex);
    myStep();
  }
}
