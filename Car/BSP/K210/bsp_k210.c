#include "bsp_k210.h"


char buf_msg[20] = {'\0'};
uint8_t g_new_flag = 0;
uint8_t g_index = 0;

// 函数功能:解析k210的信息
// 传入函数:recv_msg:串口发来的信息
// Function function: Preserve information of k210
// Incoming function: recv_ Msg: Information sent from serial port
char* K210_Deal_Recv(uint8_t recv_msg)
{
    if (recv_msg == '$' && g_new_flag == 0)
    {
        g_new_flag = 1;
        g_index = 0; // 重置索引 Reset Index
        memset(buf_msg, 0, sizeof(buf_msg)); // 清除旧数据 Clear old data
        return NULL; // 开始接收时返回NULL Returns NULL when receiving starts
    }

    if (g_new_flag == 1)
    {
        if (recv_msg == '#')
        {
            g_new_flag = 0;
            buf_msg[g_index] = '\0'; // 确保字符串以 '\0' 结尾 Make sure the string ends with '\0'
            g_index = 0; // 重置索引等待下次接收 Reset index and wait for next reception
            return buf_msg; // 返回完整的字符串 Returns the complete string
        }
        else if (g_index < BUFFER_SIZE - 1) // 检查是否不会溢出 Check that there is no overflow
        {
            buf_msg[g_index++] = recv_msg; // 存储接收到的字符 Store received characters
        }
        else // 防止数组溢出 Preventing array overflow
        {
            g_index = 0;
            g_new_flag = 0;
            memset(buf_msg, 0, sizeof(buf_msg)); // 清除旧数据 Clear old data
            return NULL; // 如果溢出，则返回NULL If overflow occurs, NULL is returned.
        }
    }
    return NULL; // 在数据接收不完整时返回NULL Returns NULL if data reception is incomplete.
}

void K210_Send_Msg(const char *data_str)
{
    if (data_str != NULL) // Check for null pointer
    {
        uint16_t datasize = strlen(data_str); // Calculate string length
        USART2_Send_ArrayU8((uint8_t *)data_str, datasize); // Cast to uint8_t*
    }
}
