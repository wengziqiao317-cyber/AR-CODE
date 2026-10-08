#include "motor.hpp"

float fuck = 0;
namespace MG370{
    void Mg370::init(TIM_HandleTypeDef* encoderTIM_, TIM_HandleTypeDef* outputTIM_, uint32_t outputChannel1_, uint32_t outputChannel2_)
    {
        encoderTIM = encoderTIM_;
        outputTIM = outputTIM_;
        outputChannel1 = outputChannel1_;
        outputChannel2 = outputChannel2_;

        HAL_TIM_PWM_Start(outputTIM, outputChannel1);
        HAL_TIM_PWM_Start(outputTIM, outputChannel2);

        HAL_TIM_Encoder_Start(encoderTIM, TIM_CHANNEL_ALL);
    }

    void Mg370::setOutput(float output_)
    {
        output = output_;
        if (output > 0.0f)
        {
            __HAL_TIM_SET_COMPARE(outputTIM, outputChannel1, outputTIM->Init.Period + 1);
            __HAL_TIM_SET_COMPARE(outputTIM, outputChannel2, outputTIM->Init.Period + 1 - (output * (float)outputTIM->Init.Period));
        }
        else
        {
            fuck = output * (float)outputTIM->Init.Period;
            __HAL_TIM_SET_COMPARE(outputTIM, outputChannel1, outputTIM->Init.Period + 1 + (output * (float)outputTIM->Init.Period));
            __HAL_TIM_SET_COMPARE(outputTIM, outputChannel2, outputTIM->Init.Period + 1);
        }
    }

    void Mg370::update()
    {
        // Update encoder count
        encodercounter = __HAL_TIM_GetCounter(encoderTIM);

        // Calculate accumulated circles and angles
        if ((last_encodercounter - encodercounter) > (ENCODER_CONSTANT / 2))
        {
            accumulated_circle -= 0.5f;
            motor_real_angle_speed = (ENCODER_CONSTANT - last_encodercounter + encodercounter) / ENCODER_CONSTANT * 3.1415f / 0.001f;
        }
        else if ((last_encodercounter - encodercounter) < -(ENCODER_CONSTANT / 2))
        {
            accumulated_circle += 0.5f;
            motor_real_angle_speed = (ENCODER_CONSTANT - encodercounter + last_encodercounter) / ENCODER_CONSTANT * 3.1415f / 0.001f;
        }
        else
        {
            motor_real_angle_speed = (encodercounter - last_encodercounter) / ENCODER_CONSTANT * 3.1415f / 0.001f;
        }
        

        // Apply filtering to the speed
        motor_real_angle_speed_filter = (1 - filter_alpha) * motor_real_angle_speed_filter + filter_alpha * motor_real_angle_speed;

        // Calculate accumulated angle
        accumulated_angle = (accumulated_circle * ENCODER_CONSTANT + encodercounter) / ENCODER_CONSTANT * 3.1415f;
        diff_angle = accumulated_angle - last_accumulated_angle;
        current_wheel_deg_s = diff_angle / ENCODER_STEP;
        last_accumulated_angle = accumulated_angle;
    }


    Mg370 mg370_left;
    Mg370 mg370_right;
    void motor_init()
    {
        mg370_left.init(&htim3, &htim1, TIM_CHANNEL_3, TIM_CHANNEL_4);
        mg370_right.init(&htim4, &htim1, TIM_CHANNEL_1, TIM_CHANNEL_2); 
    }

   //10.1需要改的地方有我现在是半圈一个  encoderconstant得改   跟angle相关的都得改 然后 一圈大概是68000 
   


// encodercounter =__HAL_TIM_GetCounter(&htim2);
    // if(encodercounter>6000){
    //   encodercounter=0;
    //   __HAL_TIM_SetCounter(&htim2,0);
    // }
    // if(encodercounter>6000){
    //   encodercounter=100;
    //   __HAL_TIM_SetCounter(&htim2,100);
    // }
    // __HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_1,encodercounter);
    

}