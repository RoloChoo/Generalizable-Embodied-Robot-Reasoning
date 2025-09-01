/*
 * 통합 로봇 제어 시스템
 * - Webots DARwIn-OP 보행 제어 (Walk.cpp 기반)
 * - MJPEG 카메라 스트리밍 (httpd.cpp 기반)
 * - 단일 HTTP 서버에서 웹 컨트롤과 카메라 스트림 모두 제공
 */

// Walk.cpp의 헤더들
#include <stdio.h>
#include <string.h>
#include <pthread.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdbool.h>
#include <cstdlib>
#include <cmath>
#include <iostream>
#include <fstream>
#include <string>

#include "Walk.hpp"
#include <webots/LED.hpp>
#include <webots/Accelerometer.hpp>
#include <webots/Gyro.hpp>
#include <webots/Motor.hpp>
#include <DARwInOPMotionManager.hpp>
#include <DARwInOPGaitManager.hpp>

// httpd.cpp의 추가 헤더들
#include <sys/time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <syslog.h>
#include "LinuxCamera.h"  // 카메라 스트리밍용

using namespace webots;
using namespace managers;
using namespace std;

// ------------------------ 공용 유틸 및 상수 ------------------------
static inline double clamp(double v, double mn, double mx) {
  if (mn > mx) return v;
  return v < mn ? mn : (v > mx ? mx : v);
}

static const int NECK_INDEX = 18;
static const int SHOULDER_R_INDEX = 0;
static const int ARM_UPPER_R_INDEX = 2;
static const int ARM_LOWER_R_INDEX = 4;

// HTTP 관련 상수 (httpd.cpp에서)
#define BUFFER_SIZE 4096
#define STD_HEADER "Cache-Control: no-store, no-cache, must-revalidate, pre-check=0, post-check=0, max-age=0\r\nPragma: no-cache\r\nExpires: Mon, 3 Jan 2000 12:34:56 GMT\r\n"
#define BOUNDARY "boundarydonotcross"
#define TEN_K (10*1024)
#define IO_BUFFER 256
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#define MAX(a, b) (((a) > (b)) ? (a) : (b))
#define DBG(format, ...) do { fprintf(stderr, "[DBG] " format, ##__VA_ARGS__); } while(0)

// ------------------------ 전역 상태 (Walk.cpp 기반) ------------------------
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
string turnDirection = "NONE";
double targetYawRad = 0.0;
double filteredYawRad = 0.0;
double prevTargetYawRad = 0.0;
const double YAW_FILTER_ALPHA = 0.015;

double webRightArmDeg = -1.0;
double targetArmRad = 0.0;
double filteredArmRad = 0.0;
const double ARM_FILTER_ALPHA = 0.020;

static double minMotorPositions[NMOTORS];
static double maxMotorPositions[NMOTORS];

// ------------------------ 카메라 스트리밍 전역 상태 (httpd.cpp 기반) ------------------------
typedef struct {
  unsigned char buffer[IO_BUFFER];
  int level;
} iobuffer;

typedef struct {
  unsigned char* buf;
  int size;
  pthread_mutex_t db;
  pthread_cond_t db_update;
} globals;

typedef struct {
  int fd;
  void* pc;
} cfd;

static globals camera_global;
static bool ClientRequest = false;

// ------------------------ HTTP 요청 처리 구조체 ------------------------
typedef enum {
  A_UNKNOWN,
  A_SNAPSHOT,
  A_STREAM,
  A_COMMAND,
  A_FILE
} answer_t;

typedef struct {
  answer_t type;
  char* parameter;
  char* client;
  char* credentials;
} request;

pthread_mutex_t stateMutex = PTHREAD_MUTEX_INITIALIZER;

static const char *motorNames[NMOTORS] = {
  "ShoulderR","ShoulderL","ArmUpperR","ArmUpperL",
  "ArmLowerR","ArmLowerL","PelvYR","PelvYL",
  "PelvR","PelvL","LegUpperR","LegUpperL",
  "LegLowerR","LegLowerL","AnkleR","AnkleL",
  "FootR","FootL","Neck","Head"
};

// ------------------------ HTTP 유틸리티 함수들 (httpd.cpp 기반) ------------------------
void init_iobuffer(iobuffer *iobuf) {
  memset(iobuf->buffer, 0, sizeof(iobuf->buffer));
  iobuf->level = 0;
}

void init_request(request *req) {
  req->type = A_UNKNOWN;
  req->parameter = NULL;
  req->client = NULL;
  req->credentials = NULL;
}

void free_request(request *req) {
  if (req->parameter != NULL) free(req->parameter);
  if (req->client != NULL) free(req->client);
  if (req->credentials != NULL) free(req->credentials);
}

int _read(int fd, iobuffer *iobuf, void *buffer, size_t len, int timeout) {
  int copied=0, rc, i;
  fd_set fds;
  struct timeval tv;

  memset(buffer, 0, len);

  while ((copied < (int)len)) {
    i = MIN(iobuf->level, (int)len-copied);
    memcpy(buffer+copied, iobuf->buffer+IO_BUFFER-iobuf->level, i);

    iobuf->level -= i;
    copied += i;
    if (copied >= len) return copied;

    tv.tv_sec = timeout;
    tv.tv_usec = 0;
    FD_ZERO(&fds);
    FD_SET(fd, &fds);
    if ((rc = select(fd+1, &fds, NULL, NULL, &tv)) <= 0) {
      if (rc < 0) exit(EXIT_FAILURE);
      return copied;
    }

    init_iobuffer(iobuf);

    if ((iobuf->level = read(fd, &iobuf->buffer, IO_BUFFER)) <= 0) {
      return -1;
    }

    memmove(iobuf->buffer+(IO_BUFFER-iobuf->level), iobuf->buffer, iobuf->level);
  }

  return 0;
}

int _readline(int fd, iobuffer *iobuf, void *buffer, size_t len, int timeout) {
  char c='\0', *out=(char*)buffer;
  int i;

  memset(buffer, 0, len);

  for (i=0; i<len && c != '\n'; i++) {
    if (_read(fd, iobuf, &c, 1, timeout) <= 0) {
      return -1;
    }
    *out++ = c;
  }

  return i;
}

void send_error(int fd, int which, char *message) {
  char buffer[BUFFER_SIZE] = {0};

  if (which == 404) {
    sprintf(buffer, "HTTP/1.0 404 Not Found\r\n"
                    "Content-type: text/plain\r\n"
                    STD_HEADER
                    "\r\n"
                    "404: Not Found!\r\n%s", message);
  } else if (which == 500) {
    sprintf(buffer, "HTTP/1.0 500 Internal Server Error\r\n"
                    "Content-type: text/plain\r\n"
                    STD_HEADER
                    "\r\n"
                    "500: Internal Server Error!\r\n%s", message);
  } else {
    sprintf(buffer, "HTTP/1.0 501 Not Implemented\r\n"
                    "Content-type: text/plain\r\n"
                    STD_HEADER
                    "\r\n"
                    "501: Not Implemented!\r\n%s", message);
  }

  write(fd, buffer, strlen(buffer));
}

// ------------------------ 카메라 스트리밍 함수들 ------------------------
void send_snapshot(int fd) {
  unsigned char *frame=NULL;
  int frame_size=0;
  char buffer[BUFFER_SIZE] = {0};

  pthread_cond_wait(&camera_global.db_update, &camera_global.db);

  frame_size = camera_global.size;

  if ((frame = (unsigned char*)malloc(frame_size+1)) == NULL) {
    free(frame);
    pthread_mutex_unlock(&camera_global.db);
    send_error(fd, 500, "not enough memory");
    return;
  }

  memcpy(frame, camera_global.buf, frame_size);
  DBG("got frame (size: %d kB)\n", frame_size/1024);

  pthread_mutex_unlock(&camera_global.db);

  sprintf(buffer, "HTTP/1.0 200 OK\r\n"
                  STD_HEADER
                  "Content-type: image/jpeg\r\n"
                  "\r\n");

  if(write(fd, buffer, strlen(buffer)) < 0) {
    free(frame);
    return;
  }
  write(fd, frame, frame_size);

  free(frame);
}

void send_stream(int fd) {
  unsigned char *frame=NULL, *tmp=NULL;
  int frame_size=0, max_frame_size=0;
  char buffer[BUFFER_SIZE] = {0};

  DBG("preparing header\n");

  sprintf(buffer, "HTTP/1.0 200 OK\r\n"
                  STD_HEADER
                  "Content-Type: multipart/x-mixed-replace;boundary=" BOUNDARY "\r\n"
                  "\r\n"
                  "--" BOUNDARY "\r\n");

  if (write(fd, buffer, strlen(buffer)) < 0) {
    free(frame);
    return;
  }

  DBG("Headers send, sending stream now\n");

  while (1) {
    pthread_cond_wait(&camera_global.db_update, &camera_global.db);

    frame_size = camera_global.size;

    if (frame_size > max_frame_size) {
      DBG("increasing buffer size to %d\n", frame_size);

      max_frame_size = frame_size+TEN_K;
      if ((tmp = (unsigned char*)realloc(frame, max_frame_size)) == NULL) {
        free(frame);
        pthread_mutex_unlock(&camera_global.db);
        send_error(fd, 500, "not enough memory");
        return;
      }
      frame = tmp;
    }

    memcpy(frame, camera_global.buf, frame_size);
    DBG("got frame (size: %d kB)\n", frame_size/1024);

    pthread_mutex_unlock(&camera_global.db);

    sprintf(buffer, "Content-Type: image/jpeg\r\n"
                    "Content-Length: %d\r\n"
                    "\r\n", frame_size);
    DBG("sending intermediate header\n");
    if (write(fd, buffer, strlen(buffer)) < 0) break;

    DBG("sending frame\n");
    if(write(fd, frame, frame_size) < 0) break;

    DBG("sending boundary\n");
    sprintf(buffer, "\r\n--" BOUNDARY "\r\n");
    if (write(fd, buffer, strlen(buffer)) < 0) break;
  }

  free(frame);
}

// ------------------------ HTML 응답 (Walk.cpp 기반, 카메라 추가) ------------------------
char* load_html() {
  FILE* f = fopen("walk.html", "r");
  if (!f) {
    static const char html[] =
      "HTTP/1.1 200 OK\r\n"
      "Content-Type: text/html\r\n"
      "Access-Control-Allow-Origin: *\r\n"
      "Connection: close\r\n\r\n"
      "<!DOCTYPE html>"
      "<html><head><meta charset='utf-8'><title>Robot Control with Camera</title>"
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
      ".camera-btn { background: #E91E63; color: white; }"
      "button:hover { opacity: 0.9; transform: translateY(-1px); }"
      "#status { background: #e8f5e8; border: 1px solid #4CAF50; color: #2e7d32; }"
      ".row { display: flex; align-items: center; gap: 12px; flex-wrap: wrap; }"
      "input[type=range]{ width: 280px; }"
      ".mono { font-family: ui-monospace, SFMono-Regular, Menlo, Consolas, monospace; }"
      "#camera-view { max-width: 100%; border: 2px solid #ddd; border-radius: 8px; }"
      "</style>"
      "</head><body>"
      "<h1>🤖 DARwIn-OP Robot Control with Camera</h1>"

      "<div class='control-group'>"
      "<h3>📹 Camera Stream</h3>"
      "<img id='camera-view' src='/?action=stream' alt='Robot Camera Stream'/><br>"
      "<button class='camera-btn' onclick=\"document.getElementById('camera-view').src='/?action=snapshot&t='+Date.now()\">📸 Take Snapshot</button>"
      "<button class='camera-btn' onclick=\"document.getElementById('camera-view').src='/?action=stream&t='+Date.now()\">🎥 Start Stream</button>"
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
      "</div>"

      "<div class='control-group'>"
      "<h3>🦾 Right Arm Control</h3>"
      "<div class='row'>"
      "<label>Right Arm: <span id='armVal' class='mono'>-</span>°</label>"
      "<input id='armRange' type='range' min='0' max='180' value='90' oninput='setRightArm(this.value)'/>"
      "<button class='yaw-btn' onclick=\"setRightArm(90)\">🎯 Reset Arm</button>"
      "</div>"
      "</div>"

      "<div class='control-group'>"
      "<h3>💡 LED Control</h3>"
      "<button class='eye-led-btn' onclick=\"sendCommand('eye_led_toggle')\">👁️ Toggle Eye LED</button>"
      "<button class='led-btn' onclick=\"sendCommand('head_led_toggle')\">💡 Toggle Head LED</button><br>"
      "<button class='blink-btn' onclick=\"sendCommand('led_blink_toggle')\">✨ Toggle Blink Mode</button>"
      "</div>"

      "<div id='status' class='control-group'>🟢 Ready - Robot control interface with camera loaded</div>"

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
      "function setRightArm(val) {"
      "  document.getElementById('armVal').textContent = val;"
      "  sendCommand('set_right_arm&arm=' + val);"
      "}"
      "</script>"
      "</body></html>";

    char* result = (char*)malloc(strlen(html) + 1);
    strcpy(result, html);
    return result;
  }

  // 파일에서 HTML 로드 로직 (기존 Walk.cpp와 동일)
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

// ------------------------ 통합 HTTP 서버 ------------------------
void* client_thread(void *arg) {
  int cnt;
  char buffer[BUFFER_SIZE]={0}, *pb=buffer;
  iobuffer iobuf;
  request req;
  cfd lcfd;

  if (arg != NULL) {
    memcpy(&lcfd, arg, sizeof(cfd));
    free(arg);
  } else {
    return NULL;
  }

  init_iobuffer(&iobuf);
  init_request(&req);

  memset(buffer, 0, sizeof(buffer));
  if ((cnt = _readline(lcfd.fd, &iobuf, buffer, sizeof(buffer)-1, 5)) == -1) {
    close(lcfd.fd);
    return NULL;
  }

  // 요청 타입 판별
  if (strstr(buffer, "GET /?action=snapshot") != NULL) {
    req.type = A_SNAPSHOT;
  }
  else if (strstr(buffer, "GET /?action=stream") != NULL) {
    req.type = A_STREAM;
  }
  else if (strstr(buffer, "GET /?command=") != NULL) {
    req.type = A_COMMAND;
    
    // 명령 파라미터 추출
    if ((pb = strstr(buffer, "GET /?command=")) != NULL) {
      pb += strlen("GET /?command=");
      int len = MIN(MAX(strspn(pb, "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ_-=&1234567890."), 0), 100);
      req.parameter = (char*)malloc(len+1);
      if (req.parameter != NULL) {
        memset(req.parameter, 0, len+1);
        strncpy(req.parameter, pb, len);
      }
    }
  }

  // HTTP 헤더 파싱 완료까지 읽기
  do {
    memset(buffer, 0, sizeof(buffer));
    if ((cnt = _readline(lcfd.fd, &iobuf, buffer, sizeof(buffer)-1, 5)) == -1) {
      free_request(&req);
      close(lcfd.fd);
      return NULL;
    }
  } while(cnt > 2 && !(buffer[0] == '\r' && buffer[1] == '\n'));

  ClientRequest = true;

  // 요청 처리
  switch (req.type) {
    case A_SNAPSHOT:
      DBG("Request for snapshot\n");
      send_snapshot(lcfd.fd);
      break;
      
    case A_STREAM:
      DBG("Request for stream\n");
      send_stream(lcfd.fd);
      break;
      
    case A_COMMAND:
      DBG("Command: %s\n", req.parameter ? req.parameter : "NULL");
      
      // Walk.cpp의 명령 처리 로직
      pthread_mutex_lock(&stateMutex);
      
      if (req.parameter) {
        if (strstr(req.parameter, "walk_start")) {
          shouldStartWalk = true;
          printf("Walk start\n");
        }
        else if (strstr(req.parameter, "walk_stop")) {
          shouldStopWalk = true;
          printf("Walk stop\n");
        }
        else if (strstr(req.parameter, "move_forward")) {
          xAmplitude = (xAmplitude == 1.0) ? 0.0 : 1.0;
          printf("Forward: x=%.1f\n", xAmplitude);
        }
        else if (strstr(req.parameter, "move_backward")) {
          xAmplitude = (xAmplitude == -1.0) ? 0.0 : -1.0;
          printf("Backward: x=%.1f\n", xAmplitude);
        }
        else if (strstr(req.parameter, "turn_left")) {
          aAmplitude = (aAmplitude == 0.5) ? 0.0 : 0.5;
          printf("Turn LEFT: a=%.1f\n", aAmplitude);
        }
        else if (strstr(req.parameter, "turn_right")) {
          aAmplitude = (aAmplitude == -0.5) ? 0.0 : -0.5;
          printf("Turn RIGHT: a=%.1f\n", aAmplitude);
        }
        else if (strstr(req.parameter, "set_yaw")) {
          char *yawPos = strstr(req.parameter, "yaw=");
          if (yawPos) {
            webYawDeg = atof(yawPos + 4);
            if (webYawDeg < 0.0) webYawDeg = 0.0;
            if (webYawDeg > 360.0) webYawDeg = 360.0;
            printf("Set yaw: %.2f deg\n", webYawDeg);
          }
        }
        else if (strstr(req.parameter, "set_right_arm")) {
          char *armPos = strstr(req.parameter, "arm=");
          if (armPos) {
            webRightArmDeg = atof(armPos + 4);
            if (webRightArmDeg < 0.0) webRightArmDeg = 0.0;
            if (webRightArmDeg > 180.0) webRightArmDeg = 180.0;
            printf("Set right arm: %.2f deg\n", webRightArmDeg);
          }
        }
        else if (strstr(req.parameter, "eye_led_toggle")) {
          shouldToggleEyeLed = true;
          printf("Eye LED toggle\n");
        }
        else if (strstr(req.parameter, "head_led_toggle")) {
          shouldToggleHeadLed = true;
          printf("Head LED toggle\n");
        }
        else if (strstr(req.parameter, "led_blink_toggle")) {
          shouldToggleBlink = true;
          printf("LED blink toggle\n");
        }
      }
      
      pthread_mutex_unlock(&stateMutex);
      
      // 간단한 OK 응답
      char* response = load_html();
      send(lcfd.fd, response, strlen(response), 0);
      free(response);
      break;
      
    default:
      // 기본 페이지 (웹 컨트롤 인터페이스)
      char* response = load_html();
      send(lcfd.fd, response, strlen(response), 0);
      free(response);
      break;
  }

  close(lcfd.fd);
  free_request(&req);
  return NULL;
}

void* server(void* arg) {
  (void)arg;

  printf("Integrated server thread starting...\n");
  int s = socket(AF_INET, SOCK_STREAM, 0);
  if (s < 0) {
    printf("Socket creation failed!\n");
    return NULL;
  }

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
  if (listen(s, 10) < 0) {
    printf("Listen failed!\n");
    close(s);
    return NULL;
  }
  printf("Integrated server running on http://0.0.0.0:8080\n");

  while (1) {
    struct sockaddr_in client_addr;
    socklen_t client_len = sizeof(client_addr);
    int client = accept(s, (struct sockaddr*)&client_addr, &client_len);
    if (client < 0) {
      printf("Accept failed!\n");
      continue;
    }

    printf("Client connected from %s\n", inet_ntoa(client_addr.sin_addr));

    cfd *pcfd = (cfd*)malloc(sizeof(cfd));
    if (pcfd == NULL) {
      fprintf(stderr, "failed to allocate memory for client\n");
      close(client);
      continue;
    }

    pcfd->fd = client;
    pcfd->pc = NULL; // 간단화

    pthread_t client_thread_id;
    if (pthread_create(&client_thread_id, NULL, &client_thread, pcfd) != 0) {
      printf("Could not launch client thread\n");
      close(client);
      free(pcfd);
      continue;
    }
    pthread_detach(client_thread_id);
  }

  close
