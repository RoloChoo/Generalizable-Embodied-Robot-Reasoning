// File:          Walk.hpp
// Date:          January 2013
// Description:   Simple joint control example with web interface
// Author:        david.mansolino@epfl.ch

#ifndef WALK_HPP
#define WALK_HPP

#define NMOTORS 20

#include <webots/Robot.hpp>

namespace webots {
  class Motor;
  class Accelerometer;
}

class Walk : public webots::Robot {
  public:
                                     Walk();
    virtual                         ~Walk();
    void                             run();
    void                             checkIfFallen();
    
  private:
    int                              mTimeStep;
    
    void                             myStep();
    void                             wait(int ms);
    
    webots::Motor                   *mMotors[NMOTORS];
    webots::Accelerometer           *mAccelerometer;
};

#endif