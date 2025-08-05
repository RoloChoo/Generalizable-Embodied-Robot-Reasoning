// 필요한 헤더 파일들
#include <stdio.h>
#include <string.h>
#include <pthread.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdbool.h>

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

using namespace webots;
using namespace managers;
using namespace std;

// 로봇 상태 전역변수
int gain = 128;
bool isWalking = false;
bool shouldStartWalk = false;
bool shouldStopWalk = false;
double xAmplitude = 0.0;
double yAmplitude = 0.0;
double aAmplitude = 0.0;

// LED 제어 전역변수
bool eyeLedOn = true;
bool headLedOn = true;
bool shouldToggleEyeLed = false;
bool shouldToggleHeadLed = false;
int eyeLedColor = 0x00FF00;    // 기본 초록색
int headLedColor = 0xFF0000;   // 기본 빨간색

// LED 깜빡임 효과용 변수
bool blinkMode = false;
bool shouldToggleBlink = false;
double lastBlinkTime = 0.0;
bool blinkState = true;

pthread_mutex_t stateMutex = PTHREAD_MUTEX_INITIALIZER;

static const char *motorNames[NMOTORS] = {
  "ShoulderR", "ShoulderL", "ArmUpperR", "ArmUpperL",
  "ArmLowerR", "ArmLowerL", "PelvYR", "PelvYL",
  "PelvR", "PelvL", "LegUpperR", "LegUpperL",
  "LegLowerR", "LegLowerL", "AnkleR", "AnkleL",
  "FootR", "FootL", "Neck", "Head"
};

char* load_html() {
    FILE* f = fopen("walk.html", "r");
    if (!f) {
        static const char html[] = 
            "HTTP/1.1 200 OK\r\n"
            "Content-Type: text/html\r\n"
            "Access-Control-Allow-Origin: *\r\n"
            "Connection: close\r\n\r\n"
            "<!DOCTYPE html>"
            "<html><head><title>Robot Control</title>"
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
            "button:hover { opacity: 0.8; transform: translateY(-1px); }"
            "#status { background: #e8f5e8; border: 1px solid #4CAF50; color: #2e7d32; }"
            "</style>"
            "</head><body>"
            "<h1>🤖 DARwIn-OP Robot Control</h1>"
            
            "<div class='control-group'>"
            "<h3>🚶 Walking Control</h3>"
            "<button class='walk-btn' onclick=\"sendCommand('walk_start')\">▶️ Start Walking</button>"
            "<button class='stop-btn' onclick=\"sendCommand('walk_stop')\">⏹️ Stop Walking</button>"
            "</div>"
            
            "<div class='control-group'>"
            "<h3>🧭 Direction Control</h3>"
            "<button class='move-btn' onclick=\"sendCommand('move_forward')\">⬆️ Forward</button>"
            "<button class='move-btn' onclick=\"sendCommand('move_backward')\">⬇️ Backward</button><br>"
            "<button class='move-btn' onclick=\"sendCommand('turn_left')\">⬅️ Turn Left</button>"
            "<button class='move-btn' onclick=\"sendCommand('turn_right')\">➡️ Turn Right</button>"
            "</div>"
            
            "<div class='control-group'>"
            "<h3>💡 LED Control</h3>"
            "<button class='eye-led-btn' onclick=\"sendCommand('eye_led_toggle')\">👁️ Toggle Eye LED</button>"
            "<button class='led-btn' onclick=\"sendCommand('head_led_toggle')\">💡 Toggle Head LED</button><br>"
            "<button class='blink-btn' onclick=\"sendCommand('led_blink_toggle')\">✨ Toggle Blink Mode</button>"
            "</div>"
            
            "<div id='status' class='control-group'>🟢 Ready - Robot control interface loaded</div>"
            
            "<script>"
            "function sendCommand(cmd) {"
            "  const statusDiv = document.getElementById('status');"
            "  statusDiv.innerHTML = '🔄 Sending: ' + cmd;"
            "  statusDiv.style.background = '#fff3e0';"
            "  statusDiv.style.borderColor = '#FF9800';"
            "  statusDiv.style.color = '#e65100';"
            "  "
            "  fetch('/?command=' + cmd)"
            "    .then(response => response.text())"
            "    .then(data => {"
            "      statusDiv.innerHTML = '✅ Command sent: ' + cmd;"
            "      statusDiv.style.background = '#e8f5e8';"
            "      statusDiv.style.borderColor = '#4CAF50';"
            "      statusDiv.style.color = '#2e7d32';"
            "    })"
            "    .catch(error => {"
            "      statusDiv.innerHTML = '❌ Error: ' + error;"
            "      statusDiv.style.background = '#ffebee';"
            "      statusDiv.style.borderColor = '#f44336';"
            "      statusDiv.style.color = '#c62828';"
            "    });"
            "}"
            "</script>"
            "</body></html>";
        
        char* result = (char*)malloc(strlen(html) + 1);
        strcpy(result, html);
        return result;
    }
    
    fseek(f, 0, SEEK_END);
    long size = ftell(f);
    rewind(f);
    
    const char* header = "HTTP/1.1 200 OK\r\nContent-Type: text/html\r\nConnection: close\r\n\r\n";
    char* html = (char*)malloc(strlen(header) + size + 1);
    strcpy(html, header);
    
    fread(html + strlen(header), 1, size, f);
    html[strlen(header) + size] = '\0';
    fclose(f);
    return html;
}

void* server(void* arg) {
    (void)arg;
    
    printf("Server thread starting...\n");
    
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
    
    if (listen(s, 5) < 0) {
        printf("Listen failed!\n");
        close(s);
        return NULL;
    }
    
    printf("Server running on http://0.0.0.0:8080\n");
    
    while (1) {
        printf("Waiting for connection...\n");
        
        struct sockaddr_in client_addr;
        socklen_t client_len = sizeof(client_addr);
        int client = accept(s, (struct sockaddr*)&client_addr, &client_len);
        
        if (client < 0) {
            printf("Accept failed!\n");
            continue;
        }
        
        printf("Client connected from %s\n", inet_ntoa(client_addr.sin_addr));
        
        char buf[1024];
        ssize_t bytes_read = recv(client, buf, sizeof(buf) - 1, 0);
        
        if (bytes_read <= 0) {
            close(client);
            continue;
        }
        
        buf[bytes_read] = '\0';
        printf("Received request:\n%s\n", buf);
        
        char *get_line = strtok(buf, "\r\n");
        if (!get_line) {
            close(client);
            continue;
        }
        
        printf("Processing: %s\n", get_line);
        
        pthread_mutex_lock(&stateMutex);
        
        // 걷기 제어 명령
        if (strstr(get_line, "command=walk_start")) {
            shouldStartWalk = true;
            printf("Walk start command received\n");
        }
        else if (strstr(get_line, "command=walk_stop")) {
            shouldStopWalk = true;
            printf("Walk stop command received\n");
        }
        // 방향 제어 명령
        else if (strstr(get_line, "command=move_forward")) {
            xAmplitude = (xAmplitude == 1.0) ? 0.0 : 1.0;
            printf("Forward command: amplitude = %.1f\n", xAmplitude);
        }
        else if (strstr(get_line, "command=move_backward")) {
            xAmplitude = (xAmplitude == -1.0) ? 0.0 : -1.0;
            printf("Backward command: amplitude = %.1f\n", xAmplitude);
        }
        else if (strstr(get_line, "command=turn_left")) {
            aAmplitude = (aAmplitude == 0.5) ? 0.0 : 0.5;
            printf("Left turn command: amplitude = %.1f\n", aAmplitude);
        }
        else if (strstr(get_line, "command=turn_right")) {
            aAmplitude = (aAmplitude == -0.5) ? 0.0 : -0.5;
            printf("Right turn command: amplitude = %.1f\n", aAmplitude);
        }
        // LED 제어 명령
        else if (strstr(get_line, "command=eye_led_toggle")) {
            shouldToggleEyeLed = true;
            printf("Eye LED toggle command received\n");
        }
        else if (strstr(get_line, "command=head_led_toggle")) {
            shouldToggleHeadLed = true;
            printf("Head LED toggle command received\n");
        }
        else if (strstr(get_line, "command=led_blink_toggle")) {
            shouldToggleBlink = true;
            printf("LED blink toggle command received\n");
        }
        
        pthread_mutex_unlock(&stateMutex);
        
        char* response = load_html();
        send(client, response, strlen(response), 0);
        free(response);
        
        close(client);
        printf("Response sent, connection closed\n");
    }
    
    close(s);
    return NULL;
}

Walk::Walk(): Robot() {
  mTimeStep = getBasicTimeStep();
  
  // LED 초기화 (VisualTracking 방식 참고)
  mEyeLED = getLED("EyeLed");
  mHeadLED = getLED("HeadLed");
  
  // 초기 LED 상태 설정
  mEyeLED->set(eyeLedColor);
  mHeadLED->set(headLedColor);
  
  mAccelerometer = getAccelerometer("Accelerometer");
  mAccelerometer->enable(mTimeStep);
  
  getGyro("Gyro")->enable(mTimeStep);
  
  for (int i=0; i<NMOTORS; i++) {
    mMotors[i] = getMotor(motorNames[i]);
    mMotors[i]->enablePosition(mTimeStep);
  }
  
  keyboardEnable(mTimeStep);
  
  mMotionManager = new DARwInOPMotionManager(this);
  mGaitManager = new DARwInOPGaitManager(this, "config.ini");
}

Walk::~Walk() {
  delete mMotionManager;
  delete mGaitManager;
}

void Walk::myStep() {
  int ret = step(mTimeStep);
  if (ret == -1)
    exit(EXIT_SUCCESS);
}

void Walk::wait(int ms) {
  double startTime = getTime();
  double s = (double) ms / 1000.0;
  while (s + startTime >= getTime())
    myStep();
}

void Walk::updateLEDs() {
    // LED 토글 명령 처리
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
        printf("Blink mode %s\n", blinkMode ? "ON" : "OFF");
    }
    
    // 깜빡임 모드 처리
    if (blinkMode) {
        double currentTime = getTime();
        if (currentTime - lastBlinkTime > 0.5) {  // 0.5초마다 깜빡임
            blinkState = !blinkState;
            lastBlinkTime = currentTime;
        }
        
        // 깜빡임 상태에 따라 LED 설정
        if (blinkState) {
            if (eyeLedOn) mEyeLED->set(eyeLedColor);
            if (headLedOn) mHeadLED->set(headLedColor);
        } else {
            mEyeLED->set(0x000000);  // OFF
            mHeadLED->set(0x000000); // OFF
        }
    } else {
        // 일반 모드에서 LED 상태 적용
        if (eyeLedOn) {
            mEyeLED->set(eyeLedColor);
        } else {
            mEyeLED->set(0x000000);  // OFF
        }
        
        if (headLedOn) {
            mHeadLED->set(headLedColor);
        } else {
            mHeadLED->set(0x000000);  // OFF
        }
    }
}

void Walk::run() {
  cout << "-------Walk example of DARwIn-OP with LED Control-------" << endl;
  cout << "Web control enabled on http://localhost:8080" << endl;
  cout << "Features: Walking control + LED on/off + Blink mode" << endl;
  
  // 웹 서버 스레드 시작
  pthread_t serverThread;
  if (pthread_create(&serverThread, NULL, server, NULL) != 0) {
    cout << "Failed to create server thread!" << endl;
    return;
  }
  
  myStep();
  mMotionManager->playPage(9); // init position
  wait(200);
  
  bool gaitStarted = false;
  
  while (true) {
    checkIfFallen();
    
    pthread_mutex_lock(&stateMutex);
    
    // LED 상태 업데이트
    updateLEDs();
    
    // Walk start/stop 처리
    if (shouldStartWalk && !isWalking) {
      cout << "Starting gait manager..." << endl;
      mGaitManager->start();
      mGaitManager->step(mTimeStep);
      isWalking = true;
      gaitStarted = true;
      shouldStartWalk = false;
      cout << "Gait manager started!" << endl;
    }
    
    if (shouldStopWalk && isWalking) {
      cout << "Stopping gait manager..." << endl;
      mGaitManager->stop();
      isWalking = false;
      gaitStarted = false;
      shouldStopWalk = false;
      xAmplitude = yAmplitude = aAmplitude = 0.0;
      cout << "Gait manager stopped!" << endl;
    }
    
    // Walking이 활성화된 경우에만 amplitude 설정
    if (isWalking && gaitStarted) {
      mGaitManager->setXAmplitude(xAmplitude);
      mGaitManager->setYAmplitude(yAmplitude);
      mGaitManager->setAAmplitude(aAmplitude);
      mGaitManager->step(mTimeStep);
    }
    
    pthread_mutex_unlock(&stateMutex);
    
    myStep();
  }
}

void Walk::checkIfFallen() {
  static int fup = 0;
  static int fdown = 0;
  static const double acc_tolerance = 80.0;
  static const double acc_step = 100;
  
  const double *acc = mAccelerometer->getValues();
  if (acc[1] < 512.0 - acc_tolerance)
    fup++;
  else
    fup = 0;
  
  if (acc[1] > 512.0 + acc_tolerance)
    fdown++;
  else
    fdown = 0;
  
  if (fup > acc_step) {
    mMotionManager->playPage(10); // f_up
    mMotionManager->playPage(9); // init position    
    fup = 0;
  }
  else if (fdown > acc_step) {
    mMotionManager->playPage(11); // b_up
    mMotionManager->playPage(9); // init position
    fdown = 0;
  }
}
