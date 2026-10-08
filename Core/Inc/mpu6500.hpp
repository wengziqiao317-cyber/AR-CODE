#include "stm32f1xx.h"       // 芯片系列定义
#include "stm32f1xx_hal.h"   // HAL 库入口
#include "spi.h"
#include "string.h"
#include "gpio.h"


namespace MPU6500{
struct Mpu6500{
    float ax = 0.0f;
    float ay =0.0f;
    float az=0.0f;
    float gx=0.0f;
    float gy=0.0f;
    float gz=0.0f;
    float angle_ax = 0.0f;
    float angle_ay =0.0f;
    float angle_az=0.0f;
    float angle_gx=0.0f;
    float angle_gy=0.0f;
    float angle_gz=0.0f;
    float dt = 0.002;
    float alpha_imu = 0.0f;
    float pitch = 0.00f;
    float yaw = 0.0f;
    float gx_bias = 2.7f;
    uint8_t who = 0.0f;


};
    
    void  MPU6500_Init(void);
    float MPU6500_Calib(void);
    void  MPU6500_Update(void);


    void reg_write(uint8_t reg,uint8_t value);
    uint8_t reg_read(uint8_t reg);



 extern Mpu6500 mpu6500;
// float MPU6500_getax(void);
// float MPU6500_getay(void);
// float MPU6500_getaz(void);

// float MPU6500_getgx(void);
// float MPU6500_getgy(void);
// float MPU6500_getgz(void);
// float MPU6500_gettemp(void);

}


//可以搞一个imu namespace