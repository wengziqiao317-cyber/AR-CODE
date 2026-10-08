#include "mpu6500.hpp"
#include "spi.h"
#include "string.h"
#include "gpio.h"
#include "math.h"

namespace MPU6500{
Mpu6500 mpu6500;
float ax,ay,az;
static float gx,gy,gz;
static float temp;

// static void reg_write(uint8_t reg,uint8_t value);
// static uint8_t reg_read(uint8_t reg);


void MPU6500_Init(void){
    // 设置6500参数
    reg_write(0x6b,0x80);  //复位
    HAL_Delay(100);
    reg_write(0x68,0x07);
    HAL_Delay(100);
    reg_write(0x6b,0x00);
    HAL_Delay(100);
    reg_write(0x1b,0x18);//陀螺仪
    reg_write(0x1c,0x00);//加速度计


}
void reg_write(uint8_t reg,uint8_t value){
    uint8_t tx[2];
    tx[0] = reg & 0x7F; // 最高位=0 表示写
    tx[1] = value;

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); // CS 拉低
    HAL_SPI_Transmit(&hspi1, tx, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);   // CS 拉高

}
uint8_t reg_read(uint8_t reg){
    uint8_t tx[2], rx[2];
    tx[0] = reg | 0x80; // 最高位=1 表示读
    tx[1] = 0x00;       // dummy byte

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); // CS 拉低
    HAL_SPI_TransmitReceive(&hspi1, tx, rx, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);   // CS 拉高

    return rx[1];

}
// float MPU6500_Calib(){
//     float sum = 0;
//     for (int i = 0; i < 1000; i++) {
//         int16_t gx_raw = (uint16_t)((reg_read(0x43) << 8) | reg_read(0x44));
//         int16_t gy_raw = (uint16_t)((reg_read(0x45) << 8) | reg_read(0x46));
//         int16_t gz_raw = (uint16_t)((reg_read(0x47) << 8) | reg_read(0x48));
//         mpu6500.gx = gx_raw * 6.1035e-2f;
//         mpu6500.gy = gy_raw * 6.1035e-2f;
//         mpu6500.gz = gz_raw * 6.1035e-2f;

//         mpu6500.angle_gx += mpu6500.gx*mpu6500.dt;
//         mpu6500.angle_gy += mpu6500.gy*mpu6500.dt;
//         mpu6500.angle_gz += mpu6500.gz*mpu6500.dt;
//         mpu6500.pitch = mpu6500.alpha_imu * mpu6500.angle_gy +(1-mpu6500.alpha_imu)*mpu6500.angle_ay;
//            // 读陀螺仪原始值
//         HAL_Delay(1);                // 约 1 秒
//     }
//     return  mpu6500.pitch / 1000.0f;  
// }


void MPU6500_Update(void){

    int16_t ax_raw = (int16_t)(((uint16_t)reg_read(0x3b) << 8) | (uint16_t)reg_read(0x3c));
    int16_t ay_raw = (int16_t)(((uint16_t)reg_read(0x3d) << 8) | (uint16_t)reg_read(0x3e));
    int16_t az_raw = (int16_t)(((uint16_t)reg_read(0x3f) << 8) | (uint16_t)reg_read(0x40));
    

    mpu6500.ax=ax_raw * 6.1035e-5f;
    mpu6500.ay=ay_raw * 6.1035e-5f;
    mpu6500.az=az_raw * 6.1035e-5f;
    mpu6500.who = reg_read(0x75);

    //差不多100hz?  可以设置dt;
      

    mpu6500.angle_ax =atan2(-(mpu6500.ay), mpu6500.az) ;
    
    
    
    


    int16_t gx_raw = (int16_t)(((uint16_t)reg_read(0x43) << 8) | (uint16_t)reg_read(0x44));
    int16_t gy_raw = (int16_t)(((uint16_t)reg_read(0x45) << 8) | (uint16_t)reg_read(0x46));
    int16_t gz_raw = (int16_t)(((uint16_t)reg_read(0x47) << 8) | (uint16_t)reg_read(0x48));

    // int16_t gx_raw = (uint16_t)((reg_read(0x43) << 8) | reg_read(0x44));
    // int16_t gy_raw = (uint16_t)((reg_read(0x45) << 8) | reg_read(0x46));
    // int16_t gz_raw = (uint16_t)((reg_read(0x47) << 8) | reg_read(0x48));
    mpu6500.gx = gx_raw * 6.1035e-2f;
    mpu6500.gy = gy_raw * 6.1035e-2f;
    mpu6500.gz = gz_raw * 6.1035e-2f;

    mpu6500.angle_gx += mpu6500.gx*mpu6500.dt-mpu6500.gx_bias*mpu6500.dt;;
    mpu6500.angle_gy += mpu6500.gy*mpu6500.dt ;
    mpu6500.angle_gz += mpu6500.gz*mpu6500.dt;

    mpu6500.pitch = mpu6500.alpha_imu * mpu6500.angle_gx +(1-mpu6500.alpha_imu)*mpu6500.angle_ax ;
    mpu6500.yaw = mpu6500.angle_gz;

    // uint16_t mpu6500.temp_raw = (uint16_t)((reg_read(0x41) << 8) + reg_read(0x42));
    // mpu6500.temp = temp_raw/333.87f+21.0f;


    
}





}