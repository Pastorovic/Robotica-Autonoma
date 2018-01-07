#ifndef CONTROL_H
#define CONTROL_H

#include <Aria.h>

class Control
{
public:
    Control(ArRobot* robot);
    void execute();

protected:
    virtual void init();
    virtual void input();
    virtual void proccess();
    virtual void output();

protected:
    ArRobot* robot;
    ArLaser* laser;
    ArSonarDevice* sonar;

};

#endif // CONTROL_H
