#include "PID.hpp"

// void PID::setParameter(float _kp, float _ki, float _kd)
// {
//     kp = _kp;
//     ki = _ki;
//     kd = _kd;
//  }

void PID::setClamp(float _lower, float _upper)
{
    clamp_lower = _lower;
    clamp_upper = _upper;
    clamp_enabled = true;
}

void PID::disableClamp()
{
    clamp_enabled = false;
}

float PID::computeoutput(float _target, float _current)
{
    
    
    target = _target;
    current = _current;
    error = target - current;
    integral += error;
    i_out = i_out + ki*error*0.002 - kpmeasure*(current-last_current);
    d_term = (1-alpha)*d_term + alpha*(error-last_error);
    PID::setIntegralMax();
    
    output = kp * (error) + i_out + (kd*d_term)/0.002;
    if (clamp_enabled)
    {
        if (output > clamp_upper)
            output = clamp_upper;
        else if (output < clamp_lower)
            output = clamp_lower;
    }
    last_error = error;
    last_current = current;  
    
    return output;
    
}

void PID::setIntegralMax(){
    // integral = _Integral;
    if(integral > 0.5f ){
        integral = 0.5f;
    }
    if(integral < -0.5f ){
        integral = 0.5f;
    }
}

   
void PID::reset()
{
    error = 0.0f;
    last_error = 0.0f;
    integral = 0.0f;

    output = 0.0f;
}

//输出的值是给pwm那个数值 （speed）  想的是给一个值  然后 3600-这个值