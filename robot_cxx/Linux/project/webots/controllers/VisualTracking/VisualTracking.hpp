// File:          VisualTracking.hpp
// Date:          23th of October 2025
// Description:   Simple controller showing how to use vision manager
// Author:        poby7722@gmail.com
#ifndef VISUALTRACKING_HPP
#define VISUALTRACKING_HPP
#define NMOTORS 20
#include <webots/Robot.hpp>

namespace managers {
  class DARwInOPVisionManager;
  class DARwInOPMotionManager;  // ← 추가
}

namespace webots {
  class Motor;
  class LED;
  class Camera;
};

class VisualTracking : public webots::Robot {
  public:
                                     VisualTracking();
    virtual                         ~VisualTracking();
    void                             run();
    void                             wait(int ms);  // ← 추가
    
  private:
    int                              mTimeStep;
    
    void                             myStep();
    
    webots::Motor                   *mMotors[NMOTORS];
    webots::LED                     *mEyeLED;
    webots::LED                     *mHeadLED;
    webots::Camera                  *mCamera;
    
    managers::DARwInOPVisionManager *mVisionManager;
    managers::DARwInOPMotionManager *mMotionManager;  // ← 추가
};
#endif
