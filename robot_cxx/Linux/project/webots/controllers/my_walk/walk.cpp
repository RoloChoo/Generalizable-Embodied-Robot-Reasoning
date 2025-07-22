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
            "<html><head><title>Robot Control</title></head><body>"
            "<h1>DARwIn-OP Robot Control</h1>"
            "<div style='margin: 10px;'>"
            "<button onclick=\"sendCommand('walk_start')\" style='padding: 10px; margin: 5px; background: green; color: white;'>Start Walking</button>"
            "<button onclick=\"sendCommand('walk_stop')\" style='padding: 10px; margin: 5px; background: red; color: white;'>Stop Walking</button>"
            "</div>"
            "<div style='margin: 10px;'>"
            "<button onclick=\"sendCommand('move_forward')\" style='padding: 10px; margin: 5px;'>Forward</button>"
            "<button onclick=\"sendCommand('move_backward')\" style='padding: 10px; margin: 5px;'>Backward</button>"
            "</div>"
            "<div style='margin: 10px;'>"
            "<button onclick=\"sendCommand('turn_left')\" style='padding: 10px; margin: 5px;'>Turn Left</button>"
            "<button onclick=\"sendCommand('turn_right')\" style='padding: 10px; margin: 5px;'>Turn Right</button>"
            "</div>"
            "<div id='status' style='margin: 10px; padding: 10px; background: #f0f0f0;'>Ready</div>"
            "<script>"
            "function sendCommand(cmd) {"
            "  document.getElementById('status').innerText = 'Sending: ' + cmd;"
            "  fetch('/?command=' + cmd)"
            "    .then(response => response.text())"
            "    .then(data => {"
            "      document.getElementById('status').innerText = 'Command sent: ' + cmd;"
            "    })"
            "    .catch(error => {"
            "      document.getElementById('status').innerText = 'Error: ' + error;"
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
    
    // HTTP 헤더 추가
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
        
        // GET 라인 파싱
        char *get_line = strtok(buf, "\r\n");
        if (!get_line) {
            close(client);
            continue;
        }
        
        printf("Processing: %s\n", get_line);
        
        pthread_mutex_lock(&stateMutex);
        
        if (strstr(get_line, "command=walk_start")) {
            shouldStartWalk = true;
            printf("Walk start command received\n");
        }
        else if (strstr(get_line, "command=walk_stop")) {
            shouldStopWalk = true;
            printf("Walk stop command received\n");
        }
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
        
        pthread_mutex_unlock(&stateMutex);
        
        // HTTP 응답 전송
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
  
  getLED("HeadLed")->set(0xFF0000);
  getLED("EyeLed")->set(0x00FF00);
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

void Walk::run() {
  cout << "-------Walk example of DARwIn-OP-------" << endl;
  cout << "Web control enabled on http://localhost:8080" << endl;
  
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
