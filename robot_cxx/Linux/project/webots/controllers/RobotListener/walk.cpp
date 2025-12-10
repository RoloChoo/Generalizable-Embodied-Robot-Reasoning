// p - 목/머리 관절 제어 개선 (비대칭 한계 반영)

#include "walk.hpp"

#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/Camera.hpp>
#include <webots/LED.hpp>
#include <webots/Accelerometer.hpp>
#include <webots/Gyro.hpp>

#include <DARwInOPVisionManager.hpp>
#include <DARwInOPMotionManager.hpp>
#include <DARwInOPGaitManager.hpp>

// ===== OpenCV 2.4.13 =====
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pthread.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <cstdio>
#include <vector>
#include <queue>

using namespace webots;
using namespace managers;
using namespace std;
using namespace cv;

#define NMOTORS 20

// ======================== 목/머리 관절 한계값 정의 ========================
#define NECK_MOTOR_INDEX 18
#define HEAD_MOTOR_INDEX 19
// 목(Neck Pan)은 Java 컨트롤러의 넓고 대칭적인 한계값을 사용 (좌우 회전)
#define NECK_LIMIT 1.5708  // ±90도 (Java 컨트롤러와 동일)
// 머리(Head Tilt)는 물리적 비대칭 한계값을 사용하기 위해 이 상수를 더 이상 사용하지 않음
#define HEAD_LIMIT 0.5236  // ±30도 (참고용, 코드에서는 사용 안 함)

// ------------------------ 유틸리티 ------------------------
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
        vector<int> params;
        params.push_back(CV_IMWRITE_JPEG_QUALITY);
        params.push_back(85);
        imencode(".jpg", bgr, g_latest_jpeg, params);
        g_has_image = true;
    } catch (const cv::Exception& e) {
        fprintf(stderr, "❌ JPEG encode: %s\n", e.what());
    }
    pthread_mutex_unlock(&g_camera_lock);
}

// ------------------------ 전역 상태 변수 ------------------------
static pthread_mutex_t g_lock = PTHREAD_MUTIZER;

// Blink (눈 깜빡임)
static bool    g_blinkMode   = false;
static bool    g_blinkState  = true;
static double g_lastBlinkT  = 0.0;

// 걷기
static bool    g_isWalking     = false;
static bool    g_startWalk     = false;
static bool    g_stopWalk      = false;
static double g_xAmplitude    = 0.0;
static double g_yAmplitude    = 0.0;
static double g_aAmplitude    = 0.0;

// 모션
struct MotionCommand {
    int page;
    int motorId;
    double position;
    double velocity;
};
static queue<MotionCommand> g_motionQueue;

static int     g_currentMotion   = 0;
static double g_motionStartTime = 0.0;
static double g_motionDuration  = 0.0;

// 관절 제어 (외부 명령용)
static double g_targetPositions[NMOTORS] = {0};
static bool    g_jointUpdated[NMOTORS] = {false};

// 필터 (부드러운 움직임)
static const double FILTER_ALPHA = 0.15;
static double g_filteredPositions[NMOTORS] = {0};

// 모터 정보 (Webots에서 읽어옴)
static double minMotorPositions[NMOTORS];
static double maxMotorPositions[NMOTORS];
static const char *motorNames[NMOTORS] = {
    "ShoulderR","ShoulderL","ArmUpperR","ArmUpperL",
    "ArmLowerR","ArmLowerL","PelvYR","PelvYL",
    "PelvR","PelvL","LegUpperR","LegUpperL",
    "LegLowerR","LegLowerL","AnkleR","AnkleL",
    "FootR","FootL","Neck","Head"
};

// ------------------------ 모션 길이 테이블 (확장됨) ------------------------
static double getMotionDuration(int page) {
    switch(page) {
        case 1:  return 2.0;    // ini / stand
        case 2:  return 2.0;    // OK / nod yes
        case 3:  return 2.0;    // no / shake head
        case 4:  return 2.0;    // hi / tilt
        case 6:  return 3.0;    // talk1
        case 9:  return 2.5;    // walkready
        case 10: return 3.0;    // f up (lying face → up)
        case 11: return 3.0;    // b up (lying back → up)
        case 12: return 3.0;    // rk / right kick
        case 13: return 3.0;    // lk / left kick
        case 15: return 2.0;    // sit down
        case 16: return 2.0;    // stand up
        case 23: return 2.5;    // d1 / arm yes
        case 24: return 3.0;    // d2 / applaud
        case 27: return 2.5;    // d3 / arm+head yes
        case 29: return 3.0;    // talk2
        case 31: return 3.0;    // d4 / stretch
        case 38: return 2.5;    // wave hand
        case 54: return 3.0;    // int (applaud louder)
        case 57: return 3.0;    // int (applaud)
        case 70: return 3.0;    // rPASS
        case 71: return 3.0;    // lPASS
        case 90: return 3.0;    // lie down (front)
        case 91: return 3.0;    // lie up (back)
        default: return 3.0;
    }
}

// ------------------------ HTML 대시보드 (확장된 모션 + 눈 깜빡임) ------------------------
static const char* HTML =
"HTTP/1.1 200 OK\r\nContent-Type:text/html;charset=utf-8\r\nConnection:close\r\n\r\n"
"<!doctype html><html><head><meta charset='utf-8'>"
"<title>🤖 DARwIn-OP Enhanced Control</title>"
"<style>"
"body{font-family:'Segoe UI',Arial;padding:20px;background:#f5f5f5;margin:0}"
"h1{color:#2196F3;text-align:center;margin:20px 0}"
"h2{color:#333;margin:12px 0 8px;font-size:18px}"
".card{background:#fff;border-radius:12px;padding:18px;margin:12px 0;box-shadow:0 3px 10px rgba(0,0,0,.1)}"
".row{display:flex;flex-wrap:wrap;gap:8px;margin-top:8px}"
"button{padding:11px 16px;border:none;border-radius:8px;cursor:pointer;font-size:14px;font-weight:500;transition:all .3s}"
".g{background:#4CAF50;color:#fff}.b{background:#2196F3;color:#fff}"
".o{background:#FF9800;color:#fff}.r{background:#f44336;color:#fff}"
".p{background:#9C27B0;color:#fff}.y{background:#FFC107;color:#333}"
".cam{max-width:640px;width:100%;border:3px solid #ddd;border-radius:10px;display:block;margin:10px auto}"
"button:hover{transform:translateY(-2px);box-shadow:0 4px 12px rgba(0,0,0,.2)}"
"</style>"
"<script>"
"function tick(){var img=document.getElementById('cam');if(img)img.src='/camera?t='+Date.now()}"
"setInterval(tick,200);"
"function cmd(c){fetch('/?command='+c)}"
"function go(p){fetch('/?motion='+p)}"
"</script></head><body>"
"<h1>🤖 DARwIn-OP Enhanced Controller</h1>"

"<div class='card'><img id='cam' class='cam' src='/camera'></div>"

"<div class='card'><h2>⚙️ Controls</h2>"
"<div class='row'>"
"<button class='g' onclick=\"cmd('walk_start')\">▶ Walk</button>"
"<button class='r' onclick=\"cmd('walk_stop')\">⏹ Stop</button>"
"<button class='y' onclick=\"cmd('blink_toggle')\">👁️ Toggle Blink</button>"
"</div></div>"

"<div class='card'><h2>🙋 Basic Motions</h2>"
"<div class='row'>"
"<button class='b' onclick='go(1)'>Stand (1)</button>"
"<button class='b' onclick='go(2)'>Nod Yes (2)</button>"
"<button class='b' onclick='go(3)'>Shake No (3)</button>"
"<button class='b' onclick='go(4)'>Hi/Tilt (4)</button>"
"</div></div>"

"<div class='card'><h2>🗣️ Talk & Prep</h2>"
"<div class='row'>"
"<button class='o' onclick='go(6)'>Talk 1 (6)</button>"
"<button class='o' onclick='go(29)'>Talk 2 (29)</button>"
"<button class='o' onclick='go(9)'>Walk Ready (9)</button>"
"</div></div>"

"<div class='card'><h2>↕️ Get Up / Sit / Stand</h2>"
"<div class='row'>"
"<button class='r' onclick='go(10)'>Face Up (10)</button>"
"<button class='r' onclick='go(11)'>Back Up (11)</button>"
"<button class='r' onclick='go(15)'>Sit Down (15)</button>"
"<button class='r' onclick='go(16)'>Stand Up (16)</button>"
"</div></div>"

"<div class='card'><h2>🎭 Gestures</h2>"
"<div class='row'>"
"<button class='p' onclick='go(23)'>Arm Yes (23)</button>"
"<button class='p' onclick='go(24)'>Applaud (24)</button>"
"<button class='p' onclick='go(27)'>Arm+Head Yes (27)</button>"
"<button class='p' onclick='go(31)'>Stretch (31)</button>"
"<button class='p' onclick='go(38)'>Wave Hand (38)</button>"
"<button class='p' onclick='go(54)'>Applaud+ (54)</button>"
"<button class='p' onclick='go(57)'>Applaud++ (57)</button>"
"</div></div>"

"<div class='card'><h2>⚽ Soccer Moves</h2>"
"<div class='row'>"
"<button class='o' onclick='go(12)'>Right Kick (12)</button>"
"<button class='o' onclick='go(13)'>Left Kick (13)</button>"
"<button class='o' onclick='go(70)'>Right Pass (70)</button>"
"<button class='o' onclick='go(71)'>Left Pass (71)</button>"
"</div></div>"

"<div class='card'><h2>🛌 Lie Down</h2>"
"<div class='row'>"
"<button class='g' onclick='go(90)'>Lie Front (90)</button>"
"<button class='g' onclick='go(91)'>Lie Back (91)</button>"
"</div></div>"

"</body></html>";

// ------------------------ HTTP 서버 ------------------------
static void* http_server(void* arg) {
    (void)arg;
    int s = socket(AF_INET, SOCK_STREAM, 0);
    if (s < 0) return NULL;
    
    int opt = 1;
    setsockopt(s, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    
    sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port   = htons(8080);
    addr.sin_addr.s_addr = INADDR_ANY;
    
    if (bind(s, (sockaddr*)&addr, sizeof(addr)) < 0) return NULL;
    listen(s, 8);
    printf("✅ HTTP Server: http://0.0.0.0:8080\n");

    while (true) {
        sockaddr_in caddr;
        socklen_t clen = sizeof(caddr);
        int c = accept(s, (sockaddr*)&caddr, &clen);
        if (c < 0) continue;

        char buf[2048];
        ssize_t n = recv(c, buf, sizeof(buf)-1, 0);
        if (n <= 0) { close(c); continue; }
        buf[n] = '\0';
        
        char* line = strtok(buf, "\r\n");
        if (!line) { close(c); continue; }

        if (strstr(line, "GET /camera")) {
            pthread_mutex_lock(&g_camera_lock);
            if (g_has_image && !g_latest_jpeg.empty()) {
                char h[200]; snprintf(h,sizeof(h),"HTTP/1.1 200 OK\r\nContent-Type:image/jpeg\r\nContent-Length:%zu\r\n\r\n",g_latest_jpeg.size());
                send(c, h, strlen(h), 0);
                send(c, &g_latest_jpeg[0], g_latest_jpeg.size(), 0);
            } else {
                send(c, "HTTP/1.1 503\r\n\r\n", 14, 0);
            }
            pthread_mutex_unlock(&g_camera_lock);
            close(c);
            continue;
        }

        pthread_mutex_lock(&g_lock);
        if (strstr(line, "command=walk_start")) g_startWalk = true;
        else if (strstr(line, "command=walk_stop")) g_stopWalk = true;
        else if (strstr(line, "command=blink_toggle")) {
            g_blinkMode = !g_blinkMode;
            printf("👁️ Blink Mode: %s\n", g_blinkMode ? "ON" : "OFF");
        }
        else if (strstr(line, "motion=")) {
            char* p = strstr(line, "motion=");
            if (p) {
                MotionCommand cmd = {atoi(p+7), -1, 0, 0};
                g_motionQueue.push(cmd);
                printf("🎬 Motion queued: page %d\n", cmd.page);
            }
        }
        // set_joint: 목/머리는 Java 한계값 사용
        else if (strstr(line, "command=set_joint")) {
            char* iPos = strstr(line, "index=");
            char* vPos = strstr(line, "value=");
            if (iPos && vPos) {
                int idx = atoi(iPos+6);
                double val = atof(vPos+6);
                if (idx >= 0 && idx < NMOTORS) {
                    double safe;
                    if (idx == NECK_MOTOR_INDEX) {
                        // Neck Pan (18)는 대칭적인 Java 한계값 사용
                        safe = clampd(val, -NECK_LIMIT, NECK_LIMIT);
                    } else if (idx == HEAD_MOTOR_INDEX) {
                        // 🚨 수정: Head Tilt (19)는 Webots가 감지한 비대칭 물리 한계값 사용
                        safe = clampd(val, minMotorPositions[idx], maxMotorPositions[idx]);
                    } else {
                        // 나머지 관절은 Webots 물리 한계값 사용
                        safe = clampd(val, minMotorPositions[idx], maxMotorPositions[idx]);
                    }
                    g_targetPositions[idx] = safe;
                    g_jointUpdated[idx] = true;
                }
            }
        }
        else if (strstr(line, "command=set_walk")) {
            char* f = strstr(line, "f="); char* b = strstr(line, "b=");
            char* l = strstr(line, "l="); char* r = strstr(line, "r=");
            bool fw = f && atoi(f+2); bool bw = b && atoi(b+2);
            bool lt = l && atoi(l+2); bool rt = r && atoi(r+2);
            
            g_xAmplitude = (fw && !bw) ? 1.0 : ((bw && !fw) ? -1.0 : 0.0);
            g_aAmplitude = (lt && !rt) ? 0.5 : ((rt && !lt) ? -0.5 : 0.0);
            
            if ((fw||bw||lt||rt) && !g_isWalking) g_startWalk = true;
            else if (!(fw||bw||lt||rt) && g_isWalking) g_stopWalk = true;
        }
        pthread_mutex_unlock(&g_lock);

        if (strstr(line, "command=")) send(c, "HTTP/1.1 200 OK\r\n\r\nOK", 19, 0);
        else send(c, HTML, strlen(HTML), 0);
        close(c);
    }
    return NULL;
}

// ------------------------ RobotListener ------------------------
RobotListener::RobotListener() : Robot() {
    mTimeStep = getBasicTimeStep();
    if (mTimeStep <= 0) mTimeStep = 32;

    mCamera = getCamera("Camera");
    if (mCamera) mCamera->enable(2 * mTimeStep);

    mEyeLED = getLED("EyeLed");
    mHeadLED = getLED("HeadLed");

    mAccelerometer = getAccelerometer("Accelerometer");
    if (mAccelerometer) mAccelerometer->enable(mTimeStep);
    
    Gyro* gyro = getGyro("Gyro");
    if (gyro) gyro->enable(mTimeStep);

    // 모터 초기화 및 범위 자동 감지
    for (int i = 0; i < NMOTORS; i++) {
        mMotors[i] = getMotor(motorNames[i]);
        if (mMotors[i]) {
            // Webots에서 모터의 물리적 한계값(비대칭일 수 있음)을 자동으로 읽어옴
            minMotorPositions[i] = mMotors[i]->getMinPosition();
            maxMotorPositions[i] = mMotors[i]->getMaxPosition();
            mMotors[i]->enablePosition(mTimeStep);
        } else {
            minMotorPositions[i] = -100;
            maxMotorPositions[i] = 100;
        }
    }

    mMotionManager = new DARwInOPMotionManager(this);
    mGaitManager    = new DARwInOPGaitManager(this, "config.ini");
}

RobotListener::~RobotListener() {
    delete mMotionManager;
    delete mGaitManager;
}

void RobotListener::myStep() {
    if (step(mTimeStep) == -1) exit(0);
}

void RobotListener::wait(int ms) {
    double start = getTime();
    while (start + ms/1000.0 > getTime()) myStep();
}

void RobotListener::checkIfFallen() {
    if (!mAccelerometer) return;
    static int fup=0, fdown=0;
    const double *acc = mAccelerometer->getValues();
    if (acc[1] < 432.0) fup++; else fup=0;
    if (acc[1] > 592.0) fdown++; else fdown=0;
    if (fup > 100) { mMotionManager->playPage(10); mMotionManager->playPage(9); fup=0; }
    if (fdown > 100) { mMotionManager->playPage(11); mMotionManager->playPage(9); fdown=0; }
}

void RobotListener::run() {
    signal(SIGINT, signal_handler);

    // HTTP 서버 시작
    pthread_t th;
    pthread_create(&th, NULL, http_server, NULL);

    // 초기 상태: 서 있기 (Init Pose)
    myStep();
    printf("🤖 Setting initial pose (Stand)...\n");
    mMotionManager->playPage(1, false);
    wait(500);
    
    int w = mCamera ? mCamera->getWidth() : 1;
    int h = mCamera ? mCamera->getHeight() : 1;

    printf("✅ Ready. Waiting for commands...\n");
    printf("📱 Dashboard: http://0.0.0.0:8080\n");

    while (true) {
        double now = getTime();
        checkIfFallen();

        if (mCamera) encodeToJPEG(mCamera->getImage(), w, h);

        pthread_mutex_lock(&g_lock);
        bool blink = g_blinkMode;
        bool sWalk = g_startWalk; bool eWalk = g_stopWalk;
        g_startWalk = false; g_stopWalk = false;
        double x=g_xAmplitude, y=g_yAmplitude, a=g_aAmplitude;
        pthread_mutex_unlock(&g_lock);

        // 걷기 로직
        if (sWalk && !g_isWalking) { mGaitManager->start(); g_isWalking=true; }
        if (eWalk && g_isWalking)  { mGaitManager->stop(); g_isWalking=false; }
        if (g_isWalking) {
            mGaitManager->setXAmplitude(x);
            mGaitManager->setYAmplitude(y);
            mGaitManager->setAAmplitude(a);
            mGaitManager->step(mTimeStep);
        }

        // 모션 로직
        bool playing = (now - g_motionStartTime) < g_motionDuration;
        pthread_mutex_lock(&g_lock);
        if (!g_motionQueue.empty() && !playing && !g_isWalking) {
            MotionCommand cmd = g_motionQueue.front();
            g_motionQueue.pop();
            pthread_mutex_unlock(&g_lock);
            
            printf("▶️ Playing motion page: %d\n", cmd.page);
            mMotionManager->playPage(cmd.page, false);
            g_motionStartTime = now;
            g_motionDuration = getMotionDuration(cmd.page);
            g_currentMotion = cmd.page;
        } else {
            pthread_mutex_unlock(&g_lock);
        }

        // 관절 직접 제어
        pthread_mutex_lock(&g_lock);
        for(int i=0; i<NMOTORS; i++) {
            if(g_jointUpdated[i]) {
                // 머리(19)와 목(18)은 더 빠르게 반응
                double alpha = (i >= 18) ? 0.5 : FILTER_ALPHA;
                g_filteredPositions[i] = alpha * g_targetPositions[i] + (1.0-alpha)*g_filteredPositions[i];
                
                // 최종 클램핑
                double safe;
                if (i == NECK_MOTOR_INDEX) {
                    // Neck Pan (18): 대칭 한계 (±90도) 사용
                    safe = clampd(g_filteredPositions[i], -NECK_LIMIT, NECK_LIMIT);
                } 
                else if (i == HEAD_MOTOR_INDEX) {
                    // 🚨 수정: Head Tilt (19): Webots의 비대칭 물리 한계값 사용
                    safe = clampd(g_filteredPositions[i], minMotorPositions[i], maxMotorPositions[i]);
                }
                else {
                    // 나머지 관절: Webots의 물리 한계값 사용
                    safe = clampd(g_filteredPositions[i], minMotorPositions[i], maxMotorPositions[i]);
                }
                
                if(mMotors[i]) mMotors[i]->setPosition(safe);
                if(fabs(g_filteredPositions[i]-g_targetPositions[i]) < 0.001) g_jointUpdated[i]=false;
            }
        }
        pthread_mutex_unlock(&g_lock);

        // LED 제어
        if(mHeadLED) mHeadLED->set(0xFF0000); // 머리 LED는 항상 빨간색
        
        // 👁️ 눈 깜빡임 로직
        if(blink) {
            if(now - g_lastBlinkT > 0.15) { 
                g_blinkState = !g_blinkState; 
                g_lastBlinkT = now; 
            }
            if(mEyeLED) mEyeLED->set(g_blinkState ? 0x00FF00 : 0x000000);
        } else {
            if(mEyeLED) mEyeLED->set(0x00FF00); // 항상 초록색
        }

        // 모션 복귀
        if(!playing && g_currentMotion != 0) {
            if(g_currentMotion != 1 && !g_isWalking) {
                printf("⏹️ Motion %d completed, returning to stand\n", g_currentMotion);
                mMotionManager->playPage(1, false);
                g_motionStartTime = now;
                g_motionDuration = 2.0;
                g_currentMotion = 1;
                wait(200);
            } else {
                g_currentMotion = 0;
            }
        }

        myStep();
    }
}

int main() {
    RobotListener robot;
    robot.run();
    return 0;
}
