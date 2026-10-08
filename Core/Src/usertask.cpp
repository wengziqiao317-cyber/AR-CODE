#include "main.h"

#include "stdint.h"
#include "tim.h"
#include  "motor.hpp"
#include "PID.hpp"
#include "mpu6500.hpp"

#define FABS(x) ((x) > 0 ? (x) : -(x))

// PID Mg370_PID_Speed(800.0f,300.0f,0.0f,0.2f,0.5f);
PID Mg370_PID_Speed(0.0f,0.5f,0.0f,0.2f,0.1f);
PID Mg370_PID_Angle(0.0f,0.0f,0.0f,0.0f,0.0f);
PID Pitch_PID(0.05f,0.0f,0.0f,0.0f,0.0f);
int button=0;
int button_state=0;

static void tickCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2) {
        MG370::mg370_left.update();
        MG370::mg370_right.update();
        MPU6500::MPU6500_Update();
        //首先先判断 按钮
        //button=HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_13);
        // if(button==1 && button_state==0){
        //     button_state = 1;
        //     //向前倾 轮子反转
            if(FABS(MPU6500::mpu6500.pitch) < 0.5f)
            {
                MG370::mg370_right.TARGET_speed=Pitch_PID.computeoutput(
                    0.0f,
                    MPU6500::mpu6500.pitch
                );
                MG370::mg370_left.TARGET_speed=Pitch_PID.computeoutput(
                    0.0f,
                    MPU6500::mpu6500.pitch
                );

                MG370::mg370_right.setOutput(-Mg370_PID_Speed.computeoutput(
                    MG370::mg370_right.TARGET_speed, 
                    MG370::mg370_right.motor_real_angle_speed_filter
                ));
                
                MG370::mg370_left.setOutput(Mg370_PID_Speed.computeoutput(
                    MG370::mg370_left.TARGET_speed, 
                    MG370::mg370_left.motor_real_angle_speed_filter
                ));
            }
            else{
                MG370::mg370_right.setOutput(0.0f);
                MG370::mg370_left.setOutput(0.0f);
            }

        
        // }
        // if(button==1 && button_state==1){
        //     button=0;
        //     MG370::motor_move_left(3599, 3599);
        // }
        // MG370::mg370_left.speed= Mg370_PID_Speed.computeoutput(
        //         MG370::mg370_left.TARGET_speed, 
        //         MG370::mg370_left.motor_real_angle_speed_filter
        //     ) ;
        // MG370::mg370_left.speed1 = 3599 - MG370::mg370_left.speed;
        // MG370::motor_move_left(MG370::mg370_left.speed1 , MG370::mg370_left.speed2);
        // // MG370::motor_move_right(MG370::mg370_right.speed1 , MG370::mg370_right.speed2);


        }
}

void init()
{
    HAL_TIM_RegisterCallback(&htim2, HAL_TIM_PERIOD_ELAPSED_CB_ID, tickCallback);  // 恢复这行
    HAL_TIM_Base_Start_IT(&htim2);
    Mg370_PID_Speed.setClamp(-0.5f, 0.5f); 
    MPU6500::MPU6500_Init();  
    MG370::mg370_left.init(&htim3, &htim1, TIM_CHANNEL_1, TIM_CHANNEL_2);
    MG370::mg370_right.init(&htim4, &htim1, TIM_CHANNEL_3, TIM_CHANNEL_4);


    // M3508_V_PID.setIntegralMax();   
    // m3508.TARGET_speed = 0.0f;
    // m3508.TARGET_angle = 1.0f;
    //感觉可以反过来有点反常识 但是  pid应该也没啥所谓
    //反正就是 DRV8871这个波形 输入和输出是反着来的
  
    
    button=0;
    button_state=0;
    MPU6500::mpu6500.pitch=0;

    
}




static void loop()
{
    while (true)
    {
        // WS2812::blink(0, COLOR_RED);
        // HAL_Delay(500);
        // WS2812::blink(0, COLOR_GREEN);
        // HAL_Delay(500);
        // WS2812::blink(0, COLOR_BLUE);
        // HAL_Delay(500);
    }
}

extern "C"
{
    void systemStart()
    {
        init();
        loop();
    }
}
