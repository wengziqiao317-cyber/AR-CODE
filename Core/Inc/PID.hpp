#include "main.h"

#include "stdint.h"

struct PID
{
public:
    float kp = 0.0f;
    float ki = 0.0f;
    float kd = 0.0f;

    float alpha = 0.0f;
    float kpmeasure=0.0f;

    float target = 0.0f;
    float current = 0.0f;
    
    float error = 0.0f;
    float last_error = 0.0f;
    float integral = 0.0f;
    float last_current = 0.0f;

    float d_term = 0.0f;
    float i_out = 0.0f;
    float dt = 0.001f;  //pid  I D一个积分一个微分都跟时间有关


    float output = 0.0f;

    float clamp_upper = 0.0f;
    float clamp_lower = 0.0f;
    bool clamp_enabled = false;

    PID(float _kp, float _ki, float _kd , float _alpha,float _kpmeasure) : kp(_kp), ki(_ki), kd(_kd),alpha(_alpha),kpmeasure(_kpmeasure){}
    // void setParameter(float _kp, float _ki, float _kd);
    void setClamp(float _lower, float _upper);
    void disableClamp();
    float computeoutput(float _target, float _current);
    void reset();
    void setIntegralMax();   
};