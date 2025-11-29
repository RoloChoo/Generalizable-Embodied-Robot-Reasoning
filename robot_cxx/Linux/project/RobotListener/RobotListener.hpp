// File:          RobotListener.hpp
// Description:   DARwIn-OP Integrated Controller Header
//                (Walk + Motion + Camera + HTTP Server)

#ifndef ROBOT_LISTENER_HPP
#define ROBOT_LISTENER_HPP

#define NMOTORS 20

#include <webots/Robot.hpp>

// 전방 선언 (Forward Declarations)
namespace managers {
  class DARwInOPMotionManager;
  class DARwInOPGaitManager;
  class DARwInOPVisionManager; // 비전 매니저 추가
}

namespace webots {
  class Motor;
  class LED;
  class Camera;
  class Accelerometer;
  class Gyro;
  // Speaker는 현재 안 쓰므로 제거하거나 남겨둬도 무방
};

class RobotListener : public webots::Robot {
  public:
                                     RobotListener();
    virtual                         ~RobotListener();
    void                             run();

  private:
    int                              mTimeStep;
    
    void                             myStep();
    void                             wait(int ms);
    void                             checkIfFallen(); // private으로 이동 (내부 로직용)

    // Devices
    webots::Motor                   *mMotors[NMOTORS];
    webots::LED                     *mEyeLED;
    webots::LED                     *mHeadLED;
    webots::Camera                  *mCamera;
    webots::Accelerometer           *mAccelerometer;
    
    // Managers
    managers::DARwInOPMotionManager *mMotionManager;
    managers::DARwInOPGaitManager   *mGaitManager;
    managers::DARwInOPVisionManager *mVisionManager; // Vision Manager 포인터 추가
};

#endif
