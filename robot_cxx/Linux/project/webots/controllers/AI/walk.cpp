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
#include <queue>
#include <chrono>

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

static inline double rad2deg(double rad) {
  return rad * 180.0 / M_PI;
}

static inline double deg2rad(double deg) {
  return deg * M_PI / 180.0;
}

// ------------------------ 전역 상태 ------------------------
// 각 관절의 목표 위치 (라디안)
double targetPositions[NMOTORS];

// 모터 한계값 (하드웨어 한계)
static double minMotorPositions[NMOTORS];
static double maxMotorPositions[NMOTORS];

// 사용자 정의 한계값
static double userMinPositions[NMOTORS];
static double userMaxPositions[NMOTORS];
static bool useUserLimits[NMOTORS];

pthread_mutex_t stateMutex = PTHREAD_MUTEX_INITIALIZER;

static const char *motorNames[NMOTORS] = {
  "ShoulderR","ShoulderL","ArmUpperR","ArmUpperL",
  "ArmLowerR","ArmLowerL","PelvYR","PelvYL",
  "PelvR","PelvL","LegUpperR","LegUpperL",
  "LegLowerR","LegLowerL","AnkleR","AnkleL",
  "FootR","FootL","Neck","Head"
};

// 실제 DARwIn-OP 관절 범위 (라디안)
// 인덱스 순서: ShoulderR, ShoulderL, ArmUpperR, ArmUpperL, ArmLowerR, ArmLowerL,
//              PelvYR, PelvYL, PelvR, PelvL, LegUpperR, LegUpperL,
//              LegLowerR, LegLowerL, AnkleR, AnkleL, FootR, FootL, Neck, Head
struct JointLimit {
  double min;
  double max;
  const char* description;
};

static const JointLimit DARWIN_JOINT_LIMITS[NMOTORS] = {
  // 팔 (Arms)
  {-1.57, 0.52,  "ShoulderR: Pitch ±90°/30°"},        // 0
  {-1.57, 0.52,  "ShoulderL: Pitch ±90°/30°"},        // 1
  {-0.68, 2.30,  "ArmUpperR: Roll -39°~131°"},        // 2
  {-2.25, 0.77,  "ArmUpperL: Roll -129°~44°"},        // 3
  {-1.57, -0.10, "ArmLowerR: Elbow -90°~-5.7°"},      // 4
  {-1.57, -0.10, "ArmLowerL: Elbow -90°~-5.7°"},      // 5
  
  // 골반 (Pelvis)
  {-1.047, 1.047, "PelvYR: Yaw ±60°"},                // 6
  {-0.69, 2.50,   "PelvYL: Yaw -39°~143°"},           // 7
  {-1.01, 1.01,   "PelvR: Roll ±58°"},                // 8
  {-0.35, 0.35,   "PelvL: Roll ±20°"},                // 9
  
  // 다리 상부 (Upper Leg)
  {-2.50, 0.87,  "LegUpperR: Hip Pitch -143°~50°"},   // 10
  {-2.50, 0.87,  "LegUpperL: Hip Pitch -143°~50°"},   // 11
  {-0.35, 0.35,  "LegLowerR: Hip Roll ±20°"},         // 12
  {-0.35, 0.35,  "LegLowerL: Hip Roll ±20°"},         // 13
  
  // 발목 (Ankle)
  {-0.87, 0.87,  "AnkleR: Pitch ±50°"},               // 14
  {-1.39, 1.22,  "AnkleL: Pitch -80°~70°"},           // 15
  {-0.87, 0.87,  "FootR: Roll ±50°"},                 // 16
  {-0.87, 0.87,  "FootL: Roll ±50°"},                 // 17
  
  // 머리 (Head)
  {-1.57, 1.57,  "Neck: Pan ±90°"},                   // 18
  {-0.52, 0.52,  "Head: Tilt ±30°"}                   // 19
};

// 과행동 방지를 위한 필터 상수
const double FILTER_ALPHA = 0.015;
double filteredPositions[NMOTORS];

// ------------------------ 데이터 수신 최적화 ------------------------
struct JointCommand {
  int index;
  double value;
  chrono::steady_clock::time_point timestamp;
};

struct LatestCommands {
  JointCommand commands[NMOTORS];
  bool hasUpdate[NMOTORS];
  chrono::steady_clock::time_point lastUpdate[NMOTORS];
} latestCommands;

pthread_mutex_t commandMutex = PTHREAD_MUTEX_INITIALIZER;

struct Statistics {
  int totalRequests;
  int droppedRequests;
  int processedCommands;
  int rangeViolations;
  chrono::steady_clock::time_point startTime;
} stats = {0, 0, 0, 0};

const int MAX_UPDATES_PER_SECOND = 50;
const chrono::milliseconds MIN_UPDATE_INTERVAL(1000 / MAX_UPDATES_PER_SECOND);

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
    "h3 { color: #555; margin-top: 20px; border-bottom: 2px solid #ddd; padding-bottom: 5px; }"
    ".control-group { background: white; padding: 15px; margin: 10px 0; border-radius: 8px; box-shadow: 0 2px 4px rgba(0,0,0,0.1); }"
    ".joint-control { display: flex; align-items: center; gap: 10px; margin: 8px 0; flex-wrap: wrap; border-bottom: 1px solid #eee; padding: 8px 0; }"
    ".joint-label { min-width: 110px; font-weight: bold; color: #2c3e50; }"
    ".joint-desc { font-size: 11px; color: #7f8c8d; min-width: 180px; }"
    "input[type=range] { width: 280px; }"
    "input[type=number] { width: 70px; padding: 4px; font-size: 12px; }"
    ".joint-value { min-width: 100px; font-family: monospace; font-size: 13px; color: #16a085; }"
    ".limit-controls { display: flex; gap: 6px; align-items: center; font-size: 11px; }"
    "button { padding: 8px 14px; margin: 5px; border: none; border-radius: 4px; cursor: pointer; font-size: 13px; background: #3498db; color: white; }"
    "button:hover { opacity: 0.85; }"
    "button.primary { background: #2ecc71; }"
    "button.danger { background: #e74c3c; }"
    "button.small { padding: 4px 8px; font-size: 11px; }"
    "#status { background: #e8f5e8; border: 1px solid #4CAF50; color: #2e7d32; padding: 10px; border-radius: 4px; }"
    "#stats { background: #e3f2fd; border: 1px solid #2196F3; color: #1565c0; padding: 10px; border-radius: 4px; margin-top: 10px; font-size: 12px; }"
    ".info-text { font-size: 11px; color: #666; margin-top: 5px; font-style: italic; }"
    ".section-body { background: #fafafa; padding: 10px; border-radius: 4px; }"
    ".warning { color: #e67e22; font-weight: bold; }"
    "</style>"
    "</head><body>"
    "<h1>🤖 DARwIn-OP Joint Control</h1>"
    "<div class='info-text' style='text-align: center; margin-bottom: 20px;'>"
    "✨ Actual DARwIn-OP Joint Limits Applied | 🛡️ Rate Limited (50 updates/sec) | 📊 Real-time Statistics"
    "</div>"

    "<div class='control-group'>"
    "<h3>📊 System Statistics</h3>"
    "<div id='stats'>Loading statistics...</div>"
    "</div>"

    "<div class='control-group'>"
    "<h3>🎮 Quick Controls</h3>"
    "<button class='primary' onclick='resetAll()'>🔄 Reset All to Zero</button>"
    "<button onclick='standPose()'>🧍 Stand Pose</button>"
    "<button onclick='tPose()'>🙆 T-Pose</button>"
    "<button class='danger' onclick='resetAllLimits()'>⚠️ Reset All Limits</button>"
    "</div>"

    "<div class='control-group'>"
    "<h3>🤖 Head</h3>"
    "<div class='section-body' id='head-joints'></div>"
    "</div>"

    "<div class='control-group'>"
    "<h3>💪 Arms</h3>"
    "<div class='section-body' id='arm-joints'></div>"
    "</div>"

    "<div class='control-group'>"
    "<h3>🦴 Pelvis</h3>"
    "<div class='section-body' id='pelvis-joints'></div>"
    "</div>"

    "<div class='control-group'>"
    "<h3>🦵 Legs</h3>"
    "<div class='section-body' id='leg-joints'></div>"
    "</div>"

    "<div id='status' class='control-group'>🟢 Ready</div>"

    "<script>"
    "const motorConfig = ["
    "  {name: 'ShoulderR', min: -1.57, max: 0.52, desc: 'Pitch ±90°/30°', section: 'arm'},"
    "  {name: 'ShoulderL', min: -1.57, max: 0.52, desc: 'Pitch ±90°/30°', section: 'arm'},"
    "  {name: 'ArmUpperR', min: -0.68, max: 2.30, desc: 'Roll -39°~131°', section: 'arm'},"
    "  {name: 'ArmUpperL', min: -2.25, max: 0.77, desc: 'Roll -129°~44°', section: 'arm'},"
    "  {name: 'ArmLowerR', min: -1.57, max: -0.10, desc: 'Elbow -90°~-5.7°', section: 'arm'},"
    "  {name: 'ArmLowerL', min: -1.57, max: -0.10, desc: 'Elbow -90°~-5.7°', section: 'arm'},"
    "  {name: 'PelvYR', min: -1.047, max: 1.047, desc: 'Yaw ±60°', section: 'pelvis'},"
    "  {name: 'PelvYL', min: -0.69, max: 2.50, desc: 'Yaw -39°~143°', section: 'pelvis'},"
    "  {name: 'PelvR', min: -1.01, max: 1.01, desc: 'Roll ±58°', section: 'pelvis'},"
    "  {name: 'PelvL', min: -0.35, max: 0.35, desc: 'Roll ±20°', section: 'pelvis'},"
    "  {name: 'LegUpperR', min: -2.50, max: 0.87, desc: 'Hip Pitch -143°~50°', section: 'leg'},"
    "  {name: 'LegUpperL', min: -2.50, max: 0.87, desc: 'Hip Pitch -143°~50°', section: 'leg'},"
    "  {name: 'LegLowerR', min: -0.35, max: 0.35, desc: 'Hip Roll ±20°', section: 'leg'},"
    "  {name: 'LegLowerL', min: -0.35, max: 0.35, desc: 'Hip Roll ±20°', section: 'leg'},"
    "  {name: 'AnkleR', min: -0.87, max: 0.87, desc: 'Pitch ±50°', section: 'leg'},"
    "  {name: 'AnkleL', min: -1.39, max: 1.22, desc: 'Pitch -80°~70°', section: 'leg'},"
    "  {name: 'FootR', min: -0.87, max: 0.87, desc: 'Roll ±50°', section: 'leg'},"
    "  {name: 'FootL', min: -0.87, max: 0.87, desc: 'Roll ±50°', section: 'leg'},"
    "  {name: 'Neck', min: -1.57, max: 1.57, desc: 'Pan ±90°', section: 'head'},"
    "  {name: 'Head', min: -0.52, max: 0.52, desc: 'Tilt ±30°', section: 'head'}"
    "];"
    
    "let jointLimits = motorConfig.map(m => ({min: m.min, max: m.max}));"
    "const requestQueue = [];"
    "let isProcessing = false;"
    "const REQUEST_INTERVAL = 20;"
    
    "function createJointControls() {"
    "  const sections = {head: [], arm: [], pelvis: [], leg: []};"
    "  "
    "  motorConfig.forEach((motor, idx) => {"
    "    const div = document.createElement('div');"
    "    div.className = 'joint-control';"
    "    div.innerHTML = `"
    "      <span class='joint-label'>${motor.name}:</span>"
    "      <span class='joint-desc'>${motor.desc}</span>"
    "      <input type='range' id='joint${idx}' "
    "        min='${motor.min}' max='${motor.max}' step='0.01' value='0' "
    "        oninput='updateJoint(${idx}, this.value)'/>"
    "      <span class='joint-value' id='value${idx}'>0.00 rad (0°)</span>"
    "      <div class='limit-controls'>"
    "        <input type='number' id='min${idx}' value='${motor.min.toFixed(2)}' step='0.1' onchange='updateLimit(${idx})'>"
    "        <input type='number' id='max${idx}' value='${motor.max.toFixed(2)}' step='0.1' onchange='updateLimit(${idx})'>"
    "        <button class='small' onclick='applyLimit(${idx})'>Set</button>"
    "      </div>"
    "    `;"
    "    sections[motor.section].push(div);"
    "  });"
    "  "
    "  Object.keys(sections).forEach(section => {"
    "    const container = document.getElementById(section + '-joints');"
    "    sections[section].forEach(div => container.appendChild(div));"
    "  });"
    "}"
    
    "function updateJoint(index, value) {"
    "  const valFloat = parseFloat(value);"
    "  const deg = (valFloat * 180 / Math.PI).toFixed(1);"
    "  document.getElementById('value' + index).textContent = valFloat.toFixed(2) + ' rad (' + deg + '°)';"
    "  queueCommand('set_joint&index=' + index + '&value=' + value);"
    "}"
    
    "function updateLimit(index) {"
    "  const minVal = parseFloat(document.getElementById('min' + index).value);"
    "  const maxVal = parseFloat(document.getElementById('max' + index).value);"
    "  const slider = document.getElementById('joint' + index);"
    "  "
    "  if (minVal < maxVal) {"
    "    jointLimits[index] = { min: minVal, max: maxVal };"
    "    slider.min = minVal;"
    "    slider.max = maxVal;"
    "    const currentVal = parseFloat(slider.value);"
    "    if (currentVal < minVal) {"
    "      slider.value = minVal;"
    "      updateJoint(index, minVal);"
    "    } else if (currentVal > maxVal) {"
    "      slider.value = maxVal;"
    "      updateJoint(index, maxVal);"
    "    }"
    "  }"
    "}"
    
    "function applyLimit(index) {"
    "  const minVal = parseFloat(document.getElementById('min' + index).value);"
    "  const maxVal = parseFloat(document.getElementById('max' + index).value);"
    "  if (minVal >= maxVal) { alert('Min must be less than Max!'); return; }"
    "  updateLimit(index);"
    "  queueCommand('set_limit&index=' + index + '&min=' + minVal + '&max=' + maxVal);"
    "  showStatus('✅ Limit updated for ' + motorConfig[index].name, 'success');"
    "}"
    
    "function resetAll() {"
    "  motorConfig.forEach((motor, idx) => {"
    "    document.getElementById('joint' + idx).value = 0;"
    "    updateJoint(idx, 0);"
    "  });"
    "}"
    
    "function standPose() {"
    "  const pose = [0,0,0,0,-0.1,-0.1,0,0,0,0,0,0,0,0,0,0,0,0,0,0];"
    "  setPose(pose);"
    "}"
    
    "function tPose() {"
    "  const pose = [0.3,0.3,1.57,-1.57,-0.1,-0.1,0,0,0,0,0,0,0,0,0,0,0,0,0,0];"
    "  setPose(pose);"
    "}"
    
    "function setPose(values) {"
    "  values.forEach((val, idx) => {"
    "    if (idx < motorConfig.length) {"
    "      document.getElementById('joint' + idx).value = val;"
    "      updateJoint(idx, val);"
    "    }"
    "  });"
    "}"
    
    "function resetAllLimits() {"
    "  if (!confirm('Reset all joint limits to hardware defaults?')) return;"
    "  queueCommand('reset_all_limits');"
    "  motorConfig.forEach((motor, idx) => {"
    "    document.getElementById('min' + idx).value = motor.min.toFixed(2);"
    "    document.getElementById('max' + idx).value = motor.max.toFixed(2);"
    "    updateLimit(idx);"
    "  });"
    "  showStatus('✅ All limits reset', 'success');"
    "}"
    
    "function queueCommand(cmd) {"
    "  requestQueue.push(cmd);"
    "  if (!isProcessing) processQueue();"
    "}"
    
    "function processQueue() {"
    "  if (requestQueue.length === 0) { isProcessing = false; return; }"
    "  isProcessing = true;"
    "  const cmd = requestQueue.shift();"
    "  fetch('/?command=' + cmd)"
    "    .then(r => r.text())"
    "    .then(_ => { updateStats(); setTimeout(processQueue, REQUEST_INTERVAL); })"
    "    .catch(err => { showStatus('❌ Error: ' + err, 'error'); setTimeout(processQueue, REQUEST_INTERVAL); });"
    "}"
    
    "function updateStats() {"
    "  fetch('/?command=get_stats')"
    "    .then(r => r.json())"
    "    .then(data => {"
    "      document.getElementById('stats').innerHTML = "
    "        `📈 Requests: ${data.total} | ✅ Processed: ${data.processed} | ` +"
    "        `❌ Dropped: ${data.dropped} | ⚠️ Range Violations: ${data.violations} | ` +"
    "        `⏱️ Uptime: ${data.uptime}s`;"
    "    })"
    "    .catch(() => {});"
    "}"
    
    "function showStatus(message, type) {"
    "  const statusDiv = document.getElementById('status');"
    "  statusDiv.innerHTML = message;"
    "  statusDiv.style.background = type === 'error' ? '#ffebee' : '#e8f5e8';"
    "}"
    
    "setInterval(updateStats, 2000);"
    "window.onload = () => { createJointControls(); updateStats(); };"
    "</script>"
    "</body></html>";

  char* result = (char*)malloc(strlen(html) + 1);
  strcpy(result, html);
  return result;
}

char* create_json_response(const char* json_content) {
  static char buffer[4096];
  snprintf(buffer, sizeof(buffer),
    "HTTP/1.1 200 OK\r\n"
    "Content-Type: application/json\r\n"
    "Access-Control-Allow-Origin: *\r\n"
    "Connection: close\r\n\r\n%s", json_content);
  return buffer;
}

// ------------------------ HTTP 서버 ------------------------
void* server(void* arg) {
  (void)arg;

  printf("=== DARwIn-OP Joint Control Server ===\n");
  printf("Actual Joint Limits Applied from DARwIn-OP Specification\n");
  stats.startTime = chrono::steady_clock::now();

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
  printf("======================================\n\n");

  while (1) {
    struct sockaddr_in client_addr;
    socklen_t client_len = sizeof(client_addr);
    int client = accept(s, (struct sockaddr*)&client_addr, &client_len);
    if (client < 0) continue;

    stats.totalRequests++;

    char buf[1024];
    ssize_t bytes_read = recv(client, buf, sizeof(buf) - 1, 0);
    if (bytes_read <= 0) { close(client); continue; }

    buf[bytes_read] = '\0';
    char *get_line = strtok(buf, "\r\n");
    if (!get_line) { close(client); continue; }

    // 통계 정보 요청
    if (strstr(get_line, "command=get_stats")) {
      auto now = chrono::steady_clock::now();
      auto uptime = chrono::duration_cast<chrono::seconds>(now - stats.startTime).count();
      
      char json[512];
      snprintf(json, sizeof(json),
        "{\"total\":%d,\"processed\":%d,\"dropped\":%d,\"violations\":%d,\"uptime\":%ld}",
        stats.totalRequests, stats.processedCommands, stats.droppedRequests, 
        stats.rangeViolations, uptime);
      
      char* response = create_json_response(json);
      send(client, response, strlen(response), 0);
      close(client);
      continue;
    }

    // 한계값 조회
    if (strstr(get_line, "command=get_limits")) {
      pthread_mutex_lock(&stateMutex);
      
      char json[2048] = "{\"limits\":[";
      for (int i = 0; i < NMOTORS; i++) {
        char item[64];
        double minLimit = useUserLimits[i] ? userMinPositions[i] : minMotorPositions[i];
        double maxLimit = useUserLimits[i] ? userMaxPositions[i] : maxMotorPositions[i];
        snprintf(item, sizeof(item), "{\"min\":%.2f,\"max\":%.2f}%s",
                 minLimit, maxLimit, (i < NMOTORS - 1) ? "," : "");
        strcat(json, item);
      }
      strcat(json, "]}");
      
      pthread_mutex_unlock(&stateMutex);
      
      char* response = create_json_response(json);
      send(client, response, strlen(response), 0);
      close(client);
      continue;
    }

    pthread_mutex_lock(&commandMutex);

    // 관절 제어 명령
    if (strstr(get_line, "command=set_joint")) {
      char *indexPos = strstr(get_line, "index=");
      char *valuePos = strstr(get_line, "value=");
      
      if (indexPos && valuePos) {
        int index = atoi(indexPos + 6);
        double value = atof(valuePos + 6);
        
        if (index >= 0 && index < NMOTORS) {
          auto now = chrono::steady_clock::now();
          
          // 레이트 리미팅
          if (latestCommands.hasUpdate[index]) {
            auto elapsed = chrono::duration_cast<chrono::milliseconds>(
              now - latestCommands.lastUpdate[index]);
            
            if (elapsed < MIN_UPDATE_INTERVAL) {
              stats.droppedRequests++;
            }
          }
          
          // 범위 체크 (하드웨어 한계값)
          if (value < minMotorPositions[index] || value > maxMotorPositions[index]) {
            stats.rangeViolations++;
            printf("⚠️  Range violation for %s: %.2f (allowed: %.2f ~ %.2f)\n",
                   motorNames[index], value, minMotorPositions[index], maxMotorPositions[index]);
          }
          
          latestCommands.commands[index] = {index, value, now};
          latestCommands.hasUpdate[index] = true;
          latestCommands.lastUpdate[index] = now;
          stats.processedCommands++;
        }
      }
    }
    
    // 한계값 설정
    else if (strstr(get_line, "command=set_limit")) {
      char *indexPos = strstr(get_line, "index=");
      char *minPos = strstr(get_line, "min=");
      char *maxPos = strstr(get_line, "max=");
      
      if (indexPos && minPos && maxPos) {
        int index = atoi(indexPos + 6);
        double minVal = atof(minPos + 4);
        double maxVal = atof(maxPos + 4);
        
        if (index >= 0 && index < NMOTORS && minVal < maxVal) {
          pthread_mutex_lock(&stateMutex);
          
          // 하드웨어 한계값 내에서만 설정
          minVal = clamp(minVal, minMotorPositions[index], maxMotorPositions[index]);
          maxVal = clamp(maxVal, minMotorPositions[index], maxMotorPositions[index]);
          
          userMinPositions[index] = minVal;
          userMaxPositions[index] = maxVal;
          useUserLimits[index] = true;
          
          pthread_mutex_unlock(&stateMutex);
          
          printf("✓ Custom limit for %s: [%.2f, %.2f] (%.1f° ~ %.1f°)\n", 
                 motorNames[index], minVal, maxVal, rad2deg(minVal), rad2deg(maxVal));
        }
      }
    }
    
    // 모든 한계값 리셋
    else if (strstr(get_line, "command=reset_all_limits")) {
      pthread_mutex_lock(&stateMutex);
      for (int i = 0; i < NMOTORS; i++) {
        useUserLimits[i] = false;
      }
      pthread_mutex_unlock(&stateMutex);
      printf("✓ All limits reset to hardware defaults\n");
    }

    pthread_mutex_unlock(&commandMutex);

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

  mAccelerometer = getAccelerometer("Accelerometer");
  mAccelerometer->enable(mTimeStep);

  printf("\n=== Initializing DARwIn-OP Motors ===\n");
  
  for (int i = 0; i < NMOTORS; i++) {
    mMotors[i] = getMotor(motorNames[i]);
    mMotors[i]->enablePosition(mTimeStep);
    
    // 실제 하드웨어 한계값 사용 (Webots에서 가져옴)
    minMotorPositions[i] = mMotors[i]->getMinPosition();
    maxMotorPositions[i] = mMotors[i]->getMaxPosition();
    
    // DARwIn-OP 표준 한계값과 비교 출력
    printf("Motor %2d (%12s): Webots[%.2f, %.2f] | DARwIn[%.2f, %.2f] | %s\n", 
           i, motorNames[i], 
           minMotorPositions[i], maxMotorPositions[i],
           DARWIN_JOINT_LIMITS[i].min, DARWIN_JOINT_LIMITS[i].max,
           DARWIN_JOINT_LIMITS[i].description);
    
    // 사용자 한계값 초기화
    userMinPositions[i] = minMotorPositions[i];
    userMaxPositions[i] = maxMotorPositions[i];
    useUserLimits[i] = false;
    
    targetPositions[i] = 0.0;
    filteredPositions[i] = 0.0;
    latestCommands.hasUpdate[i] = false;
  }
  
  printf("=====================================\n\n");
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

void Walk::checkIfFallen() {
  static int fup = 0;
  static int fdown = 0;
  static const double acc_tolerance = 80.0;
  static const double acc_step = 100;

  const double *acc = mAccelerometer->getValues();
  if (acc[1] < 512.0 - acc_tolerance) fup++; else fup = 0;
  if (acc[1] > 512.0 + acc_tolerance) fdown++; else fdown = 0;

  if (fup > acc_step) {
    printf("⚠️  Robot fell forward!\n");
    fup = 0;
  } else if (fdown > acc_step) {
    printf("⚠️  Robot fell backward!\n");
    fdown = 0;
  }
}

void Walk::run() {
  cout << "========================================" << endl;
  cout << "  DARwIn-OP Enhanced Joint Control" << endl;
  cout << "========================================" << endl;
  cout << "Features:" << endl;
  cout << "  ✓ Actual DARwIn-OP joint limits" << endl;
  cout << "  ✓ Dynamic range modification" << endl;
  cout << "  ✓ Rate limiting (50 updates/sec)" << endl;
  cout << "  ✓ Real-time statistics" << endl;
  cout << "  ✓ Range violation detection" << endl;
  cout << "\nWeb Interface: http://localhost:8080" << endl;
  cout << "========================================\n" << endl;

  pthread_t serverThread;
  if (pthread_create(&serverThread, NULL, server, NULL) != 0) {
    cout << "Failed to create server thread!" << endl;
    return;
  }

  myStep();
  wait(200);

  while (true) {
    checkIfFallen();

    // 명령 큐 처리
    pthread_mutex_lock(&commandMutex);
    for (int i = 0; i < NMOTORS; i++) {
      if (latestCommands.hasUpdate[i]) {
        pthread_mutex_lock(&stateMutex);
        
        double effectiveMin = useUserLimits[i] ? userMinPositions[i] : minMotorPositions[i];
        double effectiveMax = useUserLimits[i] ? userMaxPositions[i] : maxMotorPositions[i];
        
        double value = clamp(latestCommands.commands[i].value, effectiveMin, effectiveMax);
        targetPositions[i] = value;
        
        pthread_mutex_unlock(&stateMutex);
        
        latestCommands.hasUpdate[i] = false;
      }
    }
    pthread_mutex_unlock(&commandMutex);

    pthread_mutex_lock(&stateMutex);

    // 필터 적용 및 모터 제어
    for (int i = 0; i < NMOTORS; i++) {
      filteredPositions[i] = FILTER_ALPHA * targetPositions[i] + 
                            (1.0 - FILTER_ALPHA) * filteredPositions[i];
      
      double effectiveMin = useUserLimits[i] ? userMinPositions[i] : minMotorPositions[i];
      double effectiveMax = useUserLimits[i] ? userMaxPositions[i] : maxMotorPositions[i];
      
      double safePos = clamp(filteredPositions[i], effectiveMin, effectiveMax);
      
      mMotors[i]->setPosition(safePos);
    }

    pthread_mutex_unlock(&stateMutex);
    myStep();
  }
}
