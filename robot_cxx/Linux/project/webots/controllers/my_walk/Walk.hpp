// File:          Walk.hpp
// Date:          January 2013
// Description:   Example showing how to use the gait manager
//                and keyboard inputs with LED control
// Author:        david.mansolino@epfl.ch

#ifndef WALK_HPP
#define WALK_HPP

#define NMOTORS 20

#include <webots/Robot.hpp>

namespace managers {
  class DARwInOPMotionManager;
  class DARwInOPGaitManager;
}

namespace webots {
  class Motor;
  class LED;
  class Camera;
  class Accelerometer;
  class Gyro;
  class Speaker;
};

class Walk : public webots::Robot {
  public:
                                     Walk();
    virtual                         ~Walk();
    void                             run();
    void                             checkIfFallen();
    void                             updateLEDs();  // LED 제어 함수 추가
    
  private:
    int                              mTimeStep;
    
    void                             myStep();
    void                             wait(int ms);
    
    webots::Motor                   *mMotors[NMOTORS];
    webots::Accelerometer           *mAccelerometer;
    webots::LED                     *mEyeLED;      // LED 포인터 추가
    webots::LED                     *mHeadLED;     // LED 포인터 추가
    
    managers::DARwInOPMotionManager *mMotionManager;
    managers::DARwInOPGaitManager   *mGaitManager;
};

#endif
