#include "MPU6050.h"

#define PRINT_ACCEL     (0x01)
#define PRINT_GYRO      (0x02)
#define PRINT_QUAT      (0x04)
#define ACCEL_ON        (0x01)
#define GYRO_ON         (0x02)
#define MOTION          (0)
#define NO_MOTION       (1)
#define DEFAULT_MPU_HZ  (200)
#define FLASH_SIZE      (512)
#define FLASH_MEM_START ((void*)0x1800)
#define q30  1073741824.0f
short gyro[3], accel[3], sensors;
float Roll,Pitch,Yaw; 
float q0=1.0f,q1=0.0f,q2=0.0f,q3=0.0f;
static signed char gyro_orientation[9] = {-1, 0, 0,
                                           0,-1, 0,
                                           0, 0, 1};

static  unsigned short inv_row_2_scale(const signed char *row)
{
    unsigned short b;

    if (row[0] > 0)
        b = 0;
    else if (row[0] < 0)
        b = 4;
    else if (row[1] > 0)
        b = 1;
    else if (row[1] < 0)
        b = 5;
    else if (row[2] > 0)
        b = 2;
    else if (row[2] < 0)
        b = 6;
    else
        b = 7;      // error
    return b;
}


static  unsigned short inv_orientation_matrix_to_scalar(
    const signed char *mtx)
{
    unsigned short scalar;
    scalar = inv_row_2_scale(mtx);
    scalar |= inv_row_2_scale(mtx + 3) << 3;
    scalar |= inv_row_2_scale(mtx + 6) << 6;


    return scalar;
}

static void run_self_test(void)
{
    int result;
    long gyro[3], accel[3];

    result = mpu_run_self_test(gyro, accel);
    if (result == 0x7) {
        /* Test passed. We can trust the gyro data here, so let's push it down
         * to the DMP.
         */
        float sens;
        unsigned short accel_sens;
        mpu_get_gyro_sens(&sens);
        gyro[0] = (long)(gyro[0] * sens);
        gyro[1] = (long)(gyro[1] * sens);
        gyro[2] = (long)(gyro[2] * sens);
        dmp_set_gyro_bias(gyro);
        mpu_get_accel_sens(&accel_sens);
        accel[0] *= accel_sens;
        accel[1] *= accel_sens;
        accel[2] *= accel_sens;
        dmp_set_accel_bias(accel);
		//printf("setting bias succesfully ......\r\n");
    }
}



uint8_t buffer[14];

int16_t  MPU6050_FIFO[6][11];
int16_t Gx_offset=0,Gy_offset=0,Gz_offset=0;



/**************************************************************************
Function: The new ADC data is updated to FIFO array for filtering
Input   : ax，ay，az：x，y, z-axis acceleration data；gx，gy，gz：x. Y, z-axis angular acceleration data
Output  : none
函数功能：将新的ADC数据更新到 FIFO数组，进行滤波处理
入口参数：ax，ay，az：x，y，z轴加速度数据；gx，gy，gz：x，y，z轴角加速度数据
返回  值：无
**************************************************************************/
void  MPU6050_newValues(int16_t ax,int16_t ay,int16_t az,int16_t gx,int16_t gy,int16_t gz)
{
unsigned char i ;
int32_t sum=0;
for(i=1;i<10;i++){	//FIFO 操作 FIFO Operation
MPU6050_FIFO[0][i-1]=MPU6050_FIFO[0][i];
MPU6050_FIFO[1][i-1]=MPU6050_FIFO[1][i];
MPU6050_FIFO[2][i-1]=MPU6050_FIFO[2][i];
MPU6050_FIFO[3][i-1]=MPU6050_FIFO[3][i];
MPU6050_FIFO[4][i-1]=MPU6050_FIFO[4][i];
MPU6050_FIFO[5][i-1]=MPU6050_FIFO[5][i];
}
MPU6050_FIFO[0][9]=ax;//将新的数据放置到 数据的最后面 Place the new data at the end of the data
MPU6050_FIFO[1][9]=ay;
MPU6050_FIFO[2][9]=az;
MPU6050_FIFO[3][9]=gx;
MPU6050_FIFO[4][9]=gy;
MPU6050_FIFO[5][9]=gz;

sum=0;
for(i=0;i<10;i++){	//求当前数组的合，再取平均值 Find the sum of the current array and then take the average
   sum+=MPU6050_FIFO[0][i];
}
MPU6050_FIFO[0][10]=sum/10;

sum=0;
for(i=0;i<10;i++){
   sum+=MPU6050_FIFO[1][i];
}
MPU6050_FIFO[1][10]=sum/10;

sum=0;
for(i=0;i<10;i++){
   sum+=MPU6050_FIFO[2][i];
}
MPU6050_FIFO[2][10]=sum/10;

sum=0;
for(i=0;i<10;i++){
   sum+=MPU6050_FIFO[3][i];
}
MPU6050_FIFO[3][10]=sum/10;

sum=0;
for(i=0;i<10;i++){
   sum+=MPU6050_FIFO[4][i];
}
MPU6050_FIFO[4][10]=sum/10;

sum=0;
for(i=0;i<10;i++){
   sum+=MPU6050_FIFO[5][i];
}
MPU6050_FIFO[5][10]=sum/10;
}

/**************************************************************************
Function: Setting the clock source of mpu6050
Input   : source：Clock source number
Output  : none
函数功能：设置  MPU6050 的时钟源
入口参数：source：时钟源编号
返回  值：无
 * CLK_SEL | Clock Source
 * --------+--------------------------------------
 * 0       | Internal oscillator
 * 1       | PLL with X Gyro reference
 * 2       | PLL with Y Gyro reference
 * 3       | PLL with Z Gyro reference
 * 4       | PLL with external 32.768kHz reference
 * 5       | PLL with external 19.2MHz reference
 * 6       | Reserved
 * 7       | Stops the clock and keeps the timing generator in reset
**************************************************************************/
void MPU6050_setClockSource(uint8_t source){
    IICwriteBits(devAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH, source);

}

/** Set full-scale gyroscope range.
 * @param range New full-scale gyroscope range value
 * @see getFullScaleRange()
 * @see MPU6050_GYRO_FS_250
 * @see MPU6050_RA_GYRO_CONFIG
 * @see MPU6050_GCONFIG_FS_SEL_BIT
 * @see MPU6050_GCONFIG_FS_SEL_LENGTH
 */
void MPU6050_setFullScaleGyroRange(uint8_t range) {
    IICwriteBits(devAddr, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, range);
}

/**************************************************************************
Function: Setting the maximum range of mpu6050 accelerometer
Input   : range：Acceleration maximum range number
Output  : none
函数功能：设置 MPU6050 加速度计的最大量程
入口参数：range：加速度最大量程编号
返回  值：无
**************************************************************************/
//#define MPU6050_ACCEL_FS_2          0x00  		//===最大量程+-2G  Maximum range +-2G
//#define MPU6050_ACCEL_FS_4          0x01			//===最大量程+-4G  Maximum range +-4G
//#define MPU6050_ACCEL_FS_8          0x02			//===最大量程+-8G  Maximum range +-8G
//#define MPU6050_ACCEL_FS_16         0x03			//===最大量程+-16G Maximum range +-16G
void MPU6050_setFullScaleAccelRange(uint8_t range) {
    IICwriteBits(devAddr, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, range);
}

/**************************************************************************
Function: Set mpu6050 to sleep mode or not
Input   : enable：1，sleep；0，work；
Output  : none
函数功能：设置 MPU6050 是否进入睡眠模式
入口参数：enable：1，睡觉；0，工作；
返回  值：无
**************************************************************************/
void MPU6050_setSleepEnabled(uint8_t enabled) {
    IICwriteBit(devAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, enabled);
}

/**************************************************************************
Function: Read identity
Input   : none
Output  : 0x68
函数功能：读取  MPU6050 WHO_AM_I 标识
入口参数：无
返回  值：0x68
**************************************************************************/
uint8_t MPU6050_getDeviceID(void) {

    IICreadBytes(devAddr, MPU6050_RA_WHO_AM_I, 1, buffer);
    return buffer[0];
}

/**************************************************************************
Function: Check whether mpu6050 is connected
Input   : none
Output  : 1：Connected；0：Not connected
函数功能：检测MPU6050 是否已经连接
入口参数：无
返回  值：1：已连接；0：未连接
**************************************************************************/
uint8_t MPU6050_testConnection(void) {
   if(MPU6050_getDeviceID() == 0x68)  //0b01101000;
   return 1;
   	else return 0;
}

/**************************************************************************
Function: Setting whether mpu6050 is the host of aux I2C cable
Input   : enable：1，yes；0;not
Output  : none
函数功能：设置 MPU6050 是否为AUX I2C线的主机
入口参数：enable：1，是；0：否
返回  值：无
**************************************************************************/
void MPU6050_setI2CMasterModeEnabled(uint8_t enabled) {
    IICwriteBit(devAddr, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_I2C_MST_EN_BIT, enabled);
}

/**************************************************************************
Function: Setting whether mpu6050 is the host of aux I2C cable
Input   : enable：1，yes；0;not
Output  : none
函数功能：设置 MPU6050 是否为AUX I2C线的主机
入口参数：enable：1，是；0：否
返回  值：无
**************************************************************************/
void MPU6050_setI2CBypassEnabled(uint8_t enabled) {
    IICwriteBit(devAddr, MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_I2C_BYPASS_EN_BIT, enabled);
}

/**************************************************************************
Function: initialization Mpu6050 to enter the available state
Input   : none
Output  : none
函数功能：初始化	MPU6050 以进入可用状态
入口参数：无
返回  值：无
**************************************************************************/
void MPU6050_initialize(void) {
    // printf("0.1\n");
    MPU6050_setClockSource(MPU6050_CLOCK_PLL_YGYRO); //设置时钟 Set the clock
    // printf("0.2\n");
    MPU6050_setFullScaleGyroRange(MPU6050_GYRO_FS_2000);//陀螺仪量程设置 Gyroscope range setting
    // printf("0.3\n");
    MPU6050_setFullScaleAccelRange(MPU6050_ACCEL_FS_2);	//加速度度最大量程 +-2G  Maximum acceleration range +-2G
    // printf("0.4\n");
    MPU6050_setSleepEnabled(0); //进入工作状态  Enter working state
    // printf("0.5\n");
	  MPU6050_setI2CMasterModeEnabled(0);	 //不让MPU6050 控制AUXI2C  Prevent MPU6050 from controlling AUXI2C
    // printf("0.6\n");
	  MPU6050_setI2CBypassEnabled(0);	 //主控制器的I2C与	MPU6050的AUXI2C	直通关闭  I2C of the host controller and AUXI2C of the MPU6050 are directly closed
    // printf("0.7\n");
}
void mul(int A_row, int A_col, int B_row, int B_col, float A[][A_col], float B[][B_col], float C[][B_col])
{
    if (A_col == B_row)
    {
        for (int i = 0; i < A_row; i++)
        {
            for (int j = 0; j < B_col; j++)
            {
                C[i][j] = 0; // 初始化 initialization
                for (int k = 0; k < A_col; k++)
                {
                    C[i][j] += A[i][k]*B[k][j];
                }
            }
        }
    }
    else
    {
        printf("错误：矩阵的尺寸不对！");
    }
}

/**************************************************************************
Function: First order complementary filtering
Input   : acceleration、angular velocity
Output  : none
函数功能：一阶互补滤波
入口参数：加速度获取的角度、角速度
返回  值：x轴角速度
**************************************************************************/
float Complementary_Filter_x(float angle_m, float gyro_m)
{
    static float angle;
    float K1 =0.02; 
    angle = K1 * angle_m+ (1-K1) * (angle + gyro_m * dt);
    return angle;
}
/**************************************************************************
Function: First order complementary filtering
Input   : acceleration、angular velocity
Output  : none
函数功能：一阶互补滤波
入口参数：加速度获取的角度、角速度
返回  值：y轴角速度
**************************************************************************/
float Complementary_Filter_y(float angle_m, float gyro_m)
{
    static float angle;
    float K1 =0.02; 
    angle = K1 * angle_m+ (1-K1) * (angle + gyro_m * dt);
    return angle;
}

/**************************************************************************
Function: Kalman filter
Input: angular velocity, acceleration
Output: None
Function: 卡尔曼滤波
Input   : 角速度，加速度
Output  : 无
**************************************************************************/
float KF_X(float acce_Y, float acce_Z, float gyro_X) // 输入量：Y轴加速度，Z轴加速度，X轴角速度。  Input quantities: Y-axis acceleration, Z-axis acceleration, X-axis angular velocity.
{
    static float x_hat[2][1] = {0}; // 后验估计  Posterior Estimation
    static float x_hat_minus[2][1] = {0}; // 先验估计  Prior estimates
    static float p_hat[2][2] = {{1, 0}, {0, 1}}; // 后验误差协方差矩阵  Posterior error covariance matrix
    static float p_hat_minus[2][2] = {0}; // 先验误差协方差矩阵 Prior error covariance matrix
    static float K[2][1] = {0}; // 卡尔曼增益 Kalman Gain
    const float Ts = 0.005; // 采样间隔(5ms) Sampling interval (5ms)
    const float I[2][2] = {{1, 0}, {0, 1}};
    float u[1][1] = {{gyro_X}};
    float A[2][2] = {{1, -Ts}, {0, 1}}; // A矩阵 A Matrix
    float B[2][1] = {{Ts}, {0}}; // B矩阵 B Matrix
    float C[1][2] = {{1, 0}};// C矩阵 C Matrix
    float Q[2][2] = {{1e-10, 0}, {0, 1e-10}}; // 过程噪声 Process noise
    float R[1][1] = {{1e-4}}; // 测量噪声 Measurement noise
    float A_T[2][2] = {{1, 0}, {-Ts, 1}}; // A矩阵的转置 Transpose of matrix A
    float C_T[2][1] = {{1}, {0}}; // C矩阵的转置 Transpose of the C matrix
    float temp_1[2][1] = {0}; // 用以存储中间计算结果  Used to store intermediate calculation results
    float temp_2[2][1] = {0}; // 用以存储中间计算结果 Used to store intermediate calculation results
    float temp_3[2][2] = {0}; // 用以存储中间计算结果 Used to store intermediate calculation results
    float temp_4[2][2] = {0}; // 用以存储中间计算结果 Used to store intermediate calculation results
    float temp_5[1][2] = {0}; // 用以存储中间计算结果 Used to store intermediate calculation results
    float temp_6[1][1] = {0}; // 用以存储中间计算结果 Used to store intermediate calculation results
    float y = atan2(-acce_Y, acce_Z); // 利用加速度计算角度 Calculating Angle Using Acceleration
    // 预测部分 Prediction part
    // 先验估计公式 Prior estimation formula
    mul(2, 2, 2, 1, A, x_hat, temp_1);
    mul(2, 1, 1, 1, B, u, temp_2);
    x_hat_minus[0][0] = temp_1[0][0] + temp_2[0][0];
    x_hat_minus[1][0] = temp_1[1][0] + temp_2[1][0];
    // 先验误差协方差公式 Prior error covariance formula
    mul(2, 2, 2, 2, A, p_hat, temp_3);
    mul(2, 2, 2, 2, temp_3, A_T, temp_4);
    p_hat_minus[0][0] = temp_4[0][0] + Q[0][0];
    p_hat_minus[0][1] = temp_4[0][1] + Q[0][1];
    p_hat_minus[1][0] = temp_4[1][0] + Q[1][0];
    p_hat_minus[1][1] = temp_4[1][1] + Q[1][1];
    // 校正部分  Correction part
    // 卡尔曼增益公式  Kalman gain formula
    mul(1, 2, 2, 2, C, p_hat_minus, temp_5);
    mul(1, 2, 2, 1, temp_5, C_T, temp_6);
    temp_6[0][0] = 1.0f / (temp_6[0][0] + R[0][0]);
    mul(2, 2, 2, 1, p_hat_minus, C_T, temp_1);
    mul(2, 1, 1, 1, temp_1, temp_6, K);
    // 后验估计公式 Posterior estimation formula
    mul(1, 2, 2, 1, C, x_hat_minus, temp_6);
    temp_6[0][0] = y - temp_6[0][0];
    mul(2, 1, 1, 1, K, temp_6, temp_1);
    x_hat[0][0] = x_hat_minus[0][0] + temp_1[0][0];
    x_hat[1][0] = x_hat_minus[1][0] + temp_1[1][0];
    // 更新误差协方差公式  Update error covariance formula
    mul(2, 1, 1, 2, K, C, temp_3);
    temp_3[0][0] = I[0][0] - temp_3[0][0];
    temp_3[0][1] = I[0][1] - temp_3[0][1];
    temp_3[1][0] = I[1][0] - temp_3[1][0];
    temp_3[1][1] = I[1][1] - temp_3[1][1];
    mul(2, 2, 2, 2, temp_3, p_hat_minus, p_hat);
    // 返回值 Return value
    return x_hat[0][0];
}

/**************************************************************************
Function: Kalman filter
Input: angular velocity, acceleration
Output: None
Function: 卡尔曼滤波
Input   : 角速度，加速度
Output  : 无
**************************************************************************/
float KF_Y(float acce_X, float acce_Z, float gyro_Y) // 输入量：X轴加速度，Z轴加速度，Y轴角速度。 Input quantities: X-axis acceleration, Z-axis acceleration, Y-axis angular velocity.
{
    static float x_hat[2][1] = {0}; // 后验估计 Posterior Estimation
    static float x_hat_minus[2][1] = {0}; // 先验估计 Prior estimates
    static float p_hat[2][2] = {{1, 0}, {0, 1}}; // 后验误差协方差矩阵 Posterior error covariance matrix
    static float p_hat_minus[2][2] = {0}; // 先验误差协方差矩阵 Prior error covariance matrix
    static float K[2][1] = {0}; // 卡尔曼增益 Kalman Gain
    const float Ts = 0.005; // 采样间隔(5ms) Sampling interval (5ms)
    const float I[2][2] = {{1, 0}, {0, 1}};
    float u[1][1] = {{gyro_Y}};
    float A[2][2] = {{1, -Ts}, {0, 1}}; // A矩阵 A Matrix
    float B[2][1] = {{Ts}, {0}}; // B矩阵 B Matrix
    float C[1][2] = {{1, 0}};// C矩阵 C Matrix
    float Q[2][2] = {{1e-10, 0}, {0, 1e-10}}; // 过程噪声 Process noise
    float R[1][1] = {{1e-4}}; // 测量噪声 Measurement noise
    float A_T[2][2] = {{1, 0}, {-Ts, 1}}; // A矩阵的转置 Transpose of matrix A
    float C_T[2][1] = {{1}, {0}}; // C矩阵的转置 transpose of C matrix
    float temp_1[2][1] = {0}; // 用以存储中间计算结果 Used to store intermediate calculation results
    float temp_2[2][1] = {0}; // 用以存储中间计算结果 Used to store intermediate calculation results
    float temp_3[2][2] = {0}; // 用以存储中间计算结果 Used to store intermediate calculation results
    float temp_4[2][2] = {0}; // 用以存储中间计算结果 Used to store intermediate calculation results
    float temp_5[1][2] = {0}; // 用以存储中间计算结果 Used to store intermediate calculation results
    float temp_6[1][1] = {0}; // 用以存储中间计算结果 Used to store intermediate calculation results
    float y = atan2(-acce_X, acce_Z); // 利用加速度计算角度 Calculating Angle Using Acceleration
    // 预测部分 Prediction part
    // 先验估计公式 Prior estimation formula
    mul(2, 2, 2, 1, A, x_hat, temp_1);
    mul(2, 1, 1, 1, B, u, temp_2);
    x_hat_minus[0][0] = temp_1[0][0] + temp_2[0][0];
    x_hat_minus[1][0] = temp_1[1][0] + temp_2[1][0];
    // 先验误差协方差公式 Prior error covariance formula
    mul(2, 2, 2, 2, A, p_hat, temp_3);
    mul(2, 2, 2, 2, temp_3, A_T, temp_4);
    p_hat_minus[0][0] = temp_4[0][0] + Q[0][0];
    p_hat_minus[0][1] = temp_4[0][1] + Q[0][1];
    p_hat_minus[1][0] = temp_4[1][0] + Q[1][0];
    p_hat_minus[1][1] = temp_4[1][1] + Q[1][1];
    // 校正部分  Correction part
    // 卡尔曼增益公式  Kalman gain formula
    mul(1, 2, 2, 2, C, p_hat_minus, temp_5);
    mul(1, 2, 2, 1, temp_5, C_T, temp_6);
    temp_6[0][0] = 1.0f / (temp_6[0][0] + R[0][0]);
    mul(2, 2, 2, 1, p_hat_minus, C_T, temp_1);
    mul(2, 1, 1, 1, temp_1, temp_6, K);
    // 后验估计公式 Posterior estimation formula
    mul(1, 2, 2, 1, C, x_hat_minus, temp_6);
    temp_6[0][0] = y - temp_6[0][0];
    mul(2, 1, 1, 1, K, temp_6, temp_1);
    x_hat[0][0] = x_hat_minus[0][0] + temp_1[0][0];
    x_hat[1][0] = x_hat_minus[1][0] + temp_1[1][0];
    // 更新误差协方差公式  Update error covariance formula
    mul(2, 1, 1, 2, K, C, temp_3);
    temp_3[0][0] = I[0][0] - temp_3[0][0];
    temp_3[0][1] = I[0][1] - temp_3[0][1];
    temp_3[1][0] = I[1][0] - temp_3[1][0];
    temp_3[1][1] = I[1][1] - temp_3[1][1];
    mul(2, 2, 2, 2, temp_3, p_hat_minus, p_hat);
    // 返回值 Return value
    return x_hat[0][0];
}



/**************************************************************************
Function: Initialization of DMP in mpu6050
Input   : none
Output  : none
函数功能：MPU6050内置DMP的初始化
入口参数：无
返回  值：无
**************************************************************************/
void DMP_Init(void)
{ 
   u8 temp[1]={0};
   i2cRead(0x68,0x75,1,temp);
	//  printf("mpu_set_sensor complete ......\r\n");
	if(temp[0]!=0x68)NVIC_SystemReset();
	if(!mpu_init())
  {
	  if(!mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL)){}
	  	//  printf("mpu_set_sensor complete ......\r\n");
	  if(!mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL)){}
	  	//  printf("mpu_configure_fifo complete ......\r\n");
	  if(!mpu_set_sample_rate(DEFAULT_MPU_HZ)){}
	  	//  printf("mpu_set_sample_rate complete ......\r\n");
	  if(!dmp_load_motion_driver_firmware()){}
	  	// printf("dmp_load_motion_driver_firmware complete ......\r\n");
	  if(!dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation))){}
	  	//  printf("dmp_set_orientation complete ......\r\n");
	  if(!dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |
	      DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO |
	      DMP_FEATURE_GYRO_CAL)){}
	  	//  printf("dmp_enable_feature complete ......\r\n");
	  if(!dmp_set_fifo_rate(DEFAULT_MPU_HZ)){}
	  	//  printf("dmp_set_fifo_rate complete ......\r\n");
	  run_self_test();
		if(!mpu_set_dmp_state(1)){}
			//  printf("mpu_set_dmp_state complete ......\r\n");
  }

}
/**************************************************************************
Function: Read the attitude information of DMP in mpu6050
Input   : none
Output  : none
函数功能：读取MPU6050内置DMP的姿态信息
入口参数：无
返回  值：无
**************************************************************************/
float* Read_DMP(void)
{	
    unsigned long sensor_timestamp;
    unsigned char more;
    long quat[4];
    static float angles[9];
    dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors, &more);		//读取DMP数据 Read DMP data
    // Dynamically allocate an array to store Roll, Pitch, Yaw
    // 动态分配一个数组来存储 Roll, Pitch, Yaw
    // float* angles = (float*)malloc(3 * sizeof(float));
    
    if (sensors & INV_WXYZ_QUAT)
    {    
        float q0 = quat[0] / q30;
        float q1 = quat[1] / q30;
        float q2 = quat[2] / q30;
        float q3 = quat[3] / q30; 		//四元数 Quaternions

        Roll = asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3; 	//计算出横滚角 Calculate the roll angle
        Pitch = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3; // 计算出俯仰角 Calculate the pitch angle
        Yaw = atan2(2*(q1*q2 + q0*q3),q0*q0+q1*q1-q2*q2-q3*q3) * 57.3;	 //计算出偏航角 Calculate the yaw angle
        // printf("Roll:%.2f\t Pitch:%.2f\t Yaw:%.2f\r\n",Roll,Pitch,Yaw);
    }
    angles[0] = Roll;
    angles[1] = Pitch;
    angles[2] = Yaw;
    angles[3] = (float)gyro[0];
    angles[4] = (float)gyro[1];
    angles[5] = (float)gyro[2]; 
    angles[6] = (float)accel[0];
    angles[7] = (float)accel[1];
    angles[8] = (float)accel[2];
    return angles; // 返回包含 Roll, Pitch, Yaw 的数组 Returns an array containing Roll, Pitch, Yaw
}



float* MPU6050_getDate(int way){
	float gyro_x,gyro_y,gyro_z,accel_x,accel_y,accel_z;
	float Accel_Y,Accel_Z,Accel_X,Accel_Angle_x,Accel_Angle_y,Gyro_X,Gyro_Z,Gyro_Y;
	if(way==1)                           //DMP的读取在数据采集中断读取，严格遵循时序要求  //The reading of DMP is interrupted during data collection, strictly following the timing requirements
	{	
		return Read_DMP();                      	 //读取加速度、角速度、倾角  //Read acceleration, angular velocity, and tilt angle
	}			
	else
	{
        static float angles[9];
		Gyro_X=(I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_XOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_XOUT_L);    //读取X轴陀螺仪 //Read X-axis gyroscope
		Gyro_Y=(I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_YOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_YOUT_L);    //读取Y轴陀螺仪 //Read Y-axis gyroscope
		Gyro_Z=(I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_ZOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_ZOUT_L);    //读取Z轴陀螺仪 //Read Z-axis gyroscope
		Accel_X=(I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_XOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_XOUT_L); //读取X轴加速度计 //Read X-axis accelerometer
		Accel_Y=(I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_YOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_YOUT_L); //读取X轴加速度计 //Read Y-axis accelerometer
		Accel_Z=(I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_ZOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_ZOUT_L); //读取Z轴加速度计 //Read Z-axis accelerometer
		if(Gyro_X>32768)  Gyro_X-=65536;                 //数据类型转换  也可通过short强制类型转换 Data type conversion can also be enforced through short type conversion
		if(Gyro_Y>32768)  Gyro_Y-=65536;                 //数据类型转换  也可通过short强制类型转换 Data type conversion can also be enforced through short type conversion
		if(Gyro_Z>32768)  Gyro_Z-=65536;                 //数据类型转换 Data type conversion
		if(Accel_X>32768) Accel_X-=65536;                //数据类型转换 Data type conversion
		if(Accel_Y>32768) Accel_Y-=65536;                //数据类型转换 Data type conversion
		if(Accel_Z>32768) Accel_Z-=65536;                //数据类型转换 Data type conversion                           //更新平衡角速度 Update balance angular velocity
		accel_x=Accel_X/1671.84;
		accel_y=Accel_Y/1671.84;
		accel_z=Accel_Z/1671.84;
		gyro_x=Gyro_X/939.8;                              //陀螺仪量程转换 Gyroscope range conversion
		gyro_y=Gyro_Y/939.8;                              //陀螺仪量程转换 Gyroscope range conversion
		gyro_z=Gyro_Z/939.8;                              //陀螺仪量程转换 Gyroscope range conversion
		if(way==2)		  	
		{
            Pitch= KF_X(accel_y,accel_z,-gyro_x)/PI*180;//卡尔曼滤波 Kalman filtering 
            Roll = KF_Y(accel_x,accel_z,gyro_y)/PI*180;
            
            angles[0] = Roll;
            angles[1] = Pitch;
            angles[2] = ERROR_DEGREE;
            angles[3] = -Gyro_X;
            angles[4] = Gyro_Y;
            angles[5] = Gyro_Z;
            angles[6] = Accel_X;
            angles[7] = Accel_Y;
            angles[8] = Accel_Z;
            return angles;
		}
		else if(way==3) 
		{  
            Accel_Angle_x = atan2(Accel_Y,Accel_Z)*180/PI; //用Accel_Y和accel_y的参数得出的角度是一样的，只是边长不同 The angle obtained using Accel_Y and its parameters is the same, only the side length is different
            Accel_Angle_y = atan2(Accel_X,Accel_Z)*180/PI;
            Pitch = -Complementary_Filter_x(Accel_Angle_x,Gyro_X/16.4);//互补滤波 Complementary filtering
            Roll = -Complementary_Filter_y(Accel_Angle_y,Gyro_Y/16.4);

            angles[0] = Roll;
            angles[1] = Pitch;
            angles[2] = ERROR_DEGREE;
            angles[3] = -Gyro_X;
            angles[4] = Gyro_Y;
            angles[5] = Gyro_Z;
            angles[6] = Accel_X;
            angles[7] = Accel_Y;
            angles[8] = Accel_Z;
            return angles;
		}
	}
}




/**************************************************************************
Function: Read mpu6050 built-in temperature sensor data
Input   : none
Output  : Centigrade temperature
函数功能：读取MPU6050内置温度传感器数据
入口参数：无
返回  值：摄氏温度
**************************************************************************/
int Read_Temperature(void)
{	   
    float Temp;
    Temp=(I2C_ReadOneByte(devAddr,MPU6050_RA_TEMP_OUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_TEMP_OUT_L);
    if(Temp>32768) Temp-=65536;	//数据类型转换 Data type conversion
    Temp=(36.53+Temp/340)*10;	  //温度放大十倍存放 Store at ten times the temperature
    return (int)Temp;   
}

void MPU6050_EXTI_Init(void)
{  
    GPIO_InitTypeDef GPIO_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);   //外部中断，需要使能AFIO时钟 //External interrupt, AFIO clock needs to be enabled
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); //使能GPIO端口时钟  //Enable GPIO port clock
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;	          //端口配置   //Port configuration
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;         //上拉输入   //Pull up input
    GPIO_Init(GPIOA, &GPIO_InitStructure);					      //根据设定参数初始化GPIO  //Initialize GPIO according to the set parameters
  	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource12);
  	EXTI_InitStructure.EXTI_Line=EXTI_Line12;
  	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	
  	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;//下降沿触发 //Falling edge trigger
  	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  	EXTI_Init(&EXTI_InitStructure);	 	//根据EXTI_InitStruct中指定的参数初始化外设EXTI寄存器  //Initialize the external EXTI register based on the parameters specified in EXTI_InitStruct
    NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;			//使能按键所在的外部中断通道  //Enable the external interrupt channel where the button is located
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;	//抢占优先级2，    //Seize priority 2,
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;					//子优先级3 			 //Sub priority 3
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;								//使能外部中断通道 //Enable external interrupt channel
  	NVIC_Init(&NVIC_InitStructure); 
}




void MPU6050_Init(void){
    delay_init();
    IIC_MPU6050_Init();
    MPU6050_initialize();	//初始化MPU6050 Initialize MPU6050
    DMP_Init();				//初始化DMP Initialize DMP
    MPU6050_EXTI_Init();
}

//////////////////////// 补充 Replenish //////////////////////////

/**************************************************************************
Function: Matrix multiplication
Input: The two matrices to be multiplied and their sizes
Output: The matrix after multiplication
Function: 矩阵乘法
Input   : 需要相乘的两个矩阵以及它们的尺寸
Output  : 相乘后的矩阵
**************************************************************************/





















//------------------End of File----------------------------
