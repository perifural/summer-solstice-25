#include "stm32f10x.h"
#include "AllHeader.h"

float* mpu_data;

int main()
{
    bsp_init();

    float battery = 0;
    float angle = 0;
    int left_encoder = 0;
    int right_encoder = 0;

    while (1)
    {
        battery = Get_Battery_Volotage();
        angle = calc_angle(mpu_data);
        left_encoder += Read_Encoder(MOTOR_ID_ML);
        right_encoder += Read_Encoder(MOTOR_ID_MR);

        // PWM 1500~2880
        Set_Pwm(2000, -2000);

        printf("%.1f, %.1f, %d, %d\n", battery, angle, left_encoder, right_encoder);

        delay_ms(500);
    }
}

void EXTI15_10_IRQHandler() {
    if(MPU6050_INT==0)		
    {   
        EXTI->PR=1<<12;
        mpu_data = MPU6050_getDate(1);
    }
}
