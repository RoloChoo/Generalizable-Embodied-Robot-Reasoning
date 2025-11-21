// 필요한 헤더 파일들
#include <stdio.h>
#include <string.h>
#include <pthread.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <stdlib.h>
#include <cmath>
#include <iostream>

#include "Walk.hpp"
#include <webots/Motor.hpp>
#include <webots/Accelerometer.hpp>

using namespace webots;
using namespace std;

// ------------------------ 공용 유틸 ------------------------
static inline double clamp(double v, double mn, double mx) {
  if (mn > mx) return v;
  return v < mn ? mn : (v > mx ? mx : v);
}

// ------------------------ 전역 상태 ------------------------
// 각 관절의 목표 위치 (라디안)
double targetPositions[NMOTORS];

// 모터 한계값
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

// 과행동 방지를 위한 필터 상수
const double FILTER_ALPHA = 0.015;  // 느린 움직임을 위한 작은 값
double filteredPositions[NMOTORS];  // 필터링된 현재 위치

// ------------------------ HTML 응답 ------------------------
char* load_html() {
  static const char html[] =
    "HTTP/1.1 200 OK\r\n"
    "Content-Type: text/html\r\n"
    "Access-Control-Allow-Origin: *\r\n"
    "Connection: close\r\n\r\n"
    "<!DOCTYPE html>"
    "<html><head><meta charset='utf-8'><title>DARwIn-OP Joint Control</title>"
    "<style>"
    "body { font-family: Arial, sans-serif; margin: 20px; background: #f5f5f5; }"
    "h1 { color: #333; text-align: center; }"
    "h3 { color: #555; margin-top: 20px; }"
    ".control-group { background: white; padding: 15px; margin: 10px 0; border-radius: 8px; box-shadow: 0 2px 4px rgba(0,0,0,0.1); }"
    ".joint-control { display: flex; align-items: center; gap: 12px; margin: 8px 0; }"
    ".joint-label { min-width: 120px; font-weight: bold; }"
    "input[type=range] { width: 300px; }"
    ".joint-value { min-width: 60px; font-family: monospace; }"
    "button { padding: 10px 16px; margin: 5px; border: none; border-radius: 4px; cursor: pointer; font-size: 14px; background: #4CAF50; color: white; }"
    "button:hover { opacity: 0.9; }"
    "#status { background: #e8f5e8; border: 1px solid #4CAF50; color: #2e7d32; padding: 10px; border-radius: 4px; }"
    "</style>"
    "</head><body>"
    "<h1>🤖 DARwIn-OP Joint Control</h1>"

    "<div class='control-group'>"
    "<h3>🦾 Joint Control (과행동 방지 적용)</h3>"
    "<button onclick='resetAll()'>🔄 Reset All Joints</button>"
    "<div id='joints'></div>"
    "</div>"

    "<div id='status' class='control-group'>🟢 Ready - Joint control interface loaded</div>"

    "<script>"
    "const motorNames = ["
    "  'ShoulderR','ShoulderL','ArmUpperR','ArmUpperL',"
    "  'ArmLowerR','ArmLowerL','PelvYR','PelvYL',"
    "  'PelvR','PelvL','LegUpperR','LegUpperL',"
    "  'LegLowerR','LegLowerL','AnkleR','AnkleL',"
    "  'FootR','FootL','Neck','Head'"
    "];"
    
    "// 각 관절의 대략적인 범위 (라디안, -PI ~ PI)"
    "const minPos = -3.14;"
    "const maxPos = 3.14;"
    "const steps = 628; // 0.01 rad 단위"
    
    "function createJointControls() {"
    "  const container = document.getElementById('joints');"
    "  motorNames.forEach((name, idx) => {"
    "    const div = document.createElement('div');"
    "    div.className = 'joint-control';"
    "    div.innerHTML = `"
    "      <span class='joint-label'>${name}:</span>"
    "      <input type='range' id='joint${idx}' "
    "        min='${minPos}' max='${maxPos}' step='0.01' value='0' "
    "        oninput='updateJoint(${idx}, this.value)'/>"
    "      <span class='joint-value' id='value${idx}'>0.00</span>"
    "    `;"
    "    container.appendChild(div);"
    "  });"
    "}"
    
    "function updateJoint(index, value) {"
    "  document.getElementById('value' + index).textContent = parseFloat(value).toFixed(2);"
    "  sendCommand('set_joint&index=' + index + '&value=' + value);"
    "}"
    
    "function resetAll() {"
    "  motorNames.forEach((name, idx) => {"
    "    document.getElementById('joint' + idx).value = 0;"
    "    document.getElementById('value' + idx).textContent = '0.00';"
    "    sendCommand('set_joint&index=' + idx + '&value=0');"
    "  });"
    "}"
    
    "function sendCommand(cmd) {"
    "  const statusDiv = document.getElementById('status');"
    "  fetch('/?command=' + cmd)"
    "    .then(r => r.text())"
    "    .then(_ => {"
    "      statusDiv.innerHTML = '✅ Joint updated';"
    "      statusDiv.style.background = '#e8f5e8';"
    "    })"
    "    .catch(err => {"
    "      statusDiv.innerHTML = '❌ Error: ' + err;"
    "      statusDiv.style.background = '#ffebee';"
    "    });"
    "}"
    
    "window.onload = createJointControls;"
    "</script>"
    "</body></html>";

  char* result = (char*)malloc(strlen(html) + 1);
  strcpy(result, html);
  return result;
}

// ------------------------ HTTP 서버 ------------------------
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

    char buf[1024];
    ssize_t bytes_read = recv(client, buf, sizeof(buf) - 1, 0);
    if (bytes_read <= 0) { close(client); continue; }

    buf[bytes_read] = '\0';
    char *get_line = strtok(buf, "\r\n");
    if (!get_line) { close(client); continue; }

    printf("Request: %s\n", get_line);

    pthread_mutex_lock(&stateMutex);

    // 관절 제어 명령 파싱
    if (strstr(get_line, "command=set_joint")) {
      char *indexPos = strstr(get_line, "index=");
      char *valuePos = strstr(get_line, "value=");
      
      if (indexPos && valuePos) {
        int index = atoi(indexPos + 6);
        double value = atof(valuePos + 6);
        
        if (index >= 0 && index < NMOTORS) {
          // 값을 제한 범위 내로 클램핑
          value = clamp(value, minMotorPositions[index], maxMotorPositions[index]);
          targetPositions[index] = value;
          printf("Set joint %d (%s) to %.2f rad\n", index, motorNames[index], value);
        }
      }
    }

    pthread_mutex_unlock(&stateMutex);

    // HTML 응답
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

  // 센서
  mAccelerometer = getAccelerometer("Accelerometer");
  mAccelerometer->enable(mTimeStep);

  // 모터 준비 + 한계값 획득
  for (int i = 0; i < NMOTORS; i++) {
    mMotors[i] = getMotor(motorNames[i]);
    mMotors[i]->enablePosition(mTimeStep);
    
    // 위치 한계값 저장
    minMotorPositions[i] = mMotors[i]->getMinPosition();
    maxMotorPositions[i] = mMotors[i]->getMaxPosition();
    
    // 초기 위치 설정 (현재 위치)
    targetPositions[i] = 0.0;
    filteredPositions[i] = 0.0;
    
    printf("Motor %d (%s): min=%.2f, max=%.2f\n", 
           i, motorNames[i], minMotorPositions[i], maxMotorPositions[i]);
  }
}

Walk::~Walk() {
}

void Walk::myStep() {
  if (step(mTimeStep) == -1) exit(EXIT_SUCCESS);
}

void Walk::wait(int ms) {
  double t0 = getTime(), dur = ms / 1000.0;
  while (getTime() < t0 + dur) myStep();
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
    printf("Robot fell forward! Attempting recovery...\n");
    // 복구 동작 (필요시 구현)
    fup = 0;
  } else if (fdown > acc_step) {
    printf("Robot fell backward! Attempting recovery...\n");
    // 복구 동작 (필요시 구현)
    fdown = 0;
  }
}

// ------------------------ 메인 루프 ------------------------
void Walk::run() {
  cout << "-------Joint Control Example of DARwIn-OP-------" << endl;
  cout << "Web control: http://localhost:8080" << endl;

  // 웹 서버 스레드 시작
  pthread_t serverThread;
  if (pthread_create(&serverThread, NULL, server, NULL) != 0) {
    cout << "Failed to create server thread!" << endl;
    return;
  }

  myStep();
  wait(200);

  while (true) {
    checkIfFallen();

    pthread_mutex_lock(&stateMutex);

    // 과행동 방지를 위한 저속 필터 적용
    for (int i = 0; i < NMOTORS; i++) {
      // 지수 이동 평균 필터 적용
      filteredPositions[i] = FILTER_ALPHA * targetPositions[i] + 
                            (1.0 - FILTER_ALPHA) * filteredPositions[i];
      
      // 안전 범위 내로 클램핑
      double safePos = clamp(filteredPositions[i], 
                            minMotorPositions[i], 
                            maxMotorPositions[i]);
      
      // 모터에 위치 설정
      mMotors[i]->setPosition(safePos);
    }

    pthread_mutex_unlock(&stateMutex);
    myStep();
  }
}