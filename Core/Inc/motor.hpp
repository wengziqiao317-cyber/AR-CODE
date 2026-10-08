#include "tim.h"
#include "stdint.h"
#define ENCODER_STEP 0.002f
//得测一下一圈是多少

namespace MG370{
    constexpr float ENCODER_CONSTANT = 34000.0f ; //得测
    struct Mg370{
    public:
        float output = 0.0f;

        //编码器解码  第一个是数字  然后第二个是转了多少圈   第三个转的总角度   第四个是轮子的总路程
        uint16_t encodercounter = 0;
        float accumulated_circle = 0;
        float accumulated_angle = 0 ;
        float diff_angle = 0;
        float last_accumulated_angle = 0 ;
        float wheel_accumulatedPosition = 0;
        float current_wheel_deg_s = 0;
        uint16_t last_encodercounter = 0;
        
        float motor_real_angle_speed = 0;
        float motor_real_angle_speed_filter = 0;
        float filter_alpha = 0.1;

        float wheel_speed = 0;
        float TARGET_speed= 0;
        float  TARGET_angle = 1;

        void init(TIM_HandleTypeDef* encoderTIM_, TIM_HandleTypeDef* outputTIM_, uint32_t outputChannel1_, uint32_t outputChannel2_);
        void setOutput(float output);
        void update();

        void motor_move(uint16_t output1 , uint16_t output2);
        void MG370_encoder(); 
    private:
        TIM_HandleTypeDef* encoderTIM;
        TIM_HandleTypeDef* outputTIM;
        uint32_t outputChannel1;
        uint32_t outputChannel2;


    };
    
    // void motor_init();
    // /**


    extern Mg370 mg370_left;
    extern Mg370 mg370_right;
    

}
//float accumulated_circle = 0;
    // float accumulated_angle = 0 ;
    // int16_t raw_angle ;
    // float rpmFeedback;
    // float accumulatedPosition ;
    // int16_t TARGET_speed;
    // float accumulated_raw_circle;
    // float  TARGET_angle;