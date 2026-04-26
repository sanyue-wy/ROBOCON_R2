#include "dev_vofa.h"
#include "usart.h"
#include "string.h"
#include "drv_usart.h"


static pid_para_t para;

uint8_t RxBuffer[1];//串口接收缓冲
uint16_t RxLine;//指令长度初始值设置为零
uint8_t DataBuff[commandlength];//指令内容



#if VOFA_DATA_FORMAT==0

void firewater_displaydata(float position_target,float position_actual,float position_out,float speed_target,float speed_actual,float speed_out)
{
    static uint32_t vofa_tick=0;
    if(uwTick-vofa_tick<50)return;
    vofa_tick=uwTick;

    UART_Print("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\r\n",position_target,position_actual,position_out,speed_target,speed_actual,speed_out);

}

#endif



#if  VOFA_DATA_FORMAT==1
/**
 *
 * @brief 将接受到的信息投射到真正的PID参数上更新
 *mode为0时测速度环
 *
 */
void uart_pid_to_pid_update(float kp,float ki,float kd,float target,uint8_t mode)
{
    if (mode==0)
    {
        if (para.speed_kp!=kp)
            kp=para.speed_kp;
        if (para.speed_ki!=ki)
            ki=para.speed_ki;
        if (para.speed_kd!=kd)
            kd=para.speed_kd;
        if (para.speed_target!=target)
            target=para.speed_target;
    }
else
{
    if (para.position_kp!=kp)
        kp=para.position_kp;
    if (para.position_ki!=ki)
        ki=para.position_ki;
    if (para.position_kd!=kd)
        kd=para.position_kd;
    if (para.angle_target!=target)
        target=para.angle_target;
}
}






/*
要点提示:
1. float和unsigned long具有相同的数据结构长度
2. union据类型里的数据存放在相同的物理空间
*/
typedef union
{
    float fdata;
    unsigned long ldata;
} FloatLongType;


/*
将浮点数f转化为4个字节数据存放在byte[4]中
*/
void Float_to_Byte(float f,unsigned char byte[])
{
    FloatLongType fl;
    fl.fdata=f;
    byte[0]=(unsigned char)fl.ldata;
    byte[1]=(unsigned char)(fl.ldata>>8);
    byte[2]=(unsigned char)(fl.ldata>>16);
    byte[3]=(unsigned char)(fl.ldata>>24);
}

void Serial_SendByte(uint8_t Byte)
{
    HAL_UART_Transmit(&huart1,&Byte,sizeof(Byte),HAL_MAX_DELAY);
}


void Serial_SendArray(uint8_t *Array, uint16_t Length)
{
    uint16_t i;
    for (i = 0; i < Length; i ++)
    {
        Serial_SendByte(Array[i]);
    }
}

void vofa_FloatSend(float * date,uint16_t length)
{
    uint8_t byte[4]={0};		//float转化为4个字节数据
    uint8_t tail[4]={0x00, 0x00, 0x80, 0x7f};	//帧尾
    for (uint16_t i=0;i<length;i++)
    {
        Float_to_Byte(*(date+i),byte);
        Serial_SendArray(byte,4);
    }
    Serial_SendArray(tail,4);


}




#endif


/******
 * 将接受的数组传入回调函数实现命名的解析
 *
 */
void Vofa_Callback(uint8_t *data, uint16_t size)
{
    for(uint16_t i=0;i<size;i++)
    {
        RxLine++;                      //每接收到一个数据，进入回调数据长度加1
        DataBuff[RxLine-1]=data[i];  //把每次接收到的数据保存到缓存数组
        if(data[i]==frametail)            //接收结束标志位，这个数据可以自定义，根据实际需求，这里只做示例使用，不一定是0x21
        {
            /**********************
            * 下面是调试代码，可以在串口助手上查看接收的数据
             *********************/
            UART_Print("RXLen=%d\r\n",RxLine);
            for(int j=0;j<RxLine;j++)
               UART_Print("UART DataBuff[%d] = %c\r\n",j,DataBuff[j]);
            USART_PID_Adjust(0);//数据解析和参数赋值函数
            memset(DataBuff,0,sizeof(DataBuff));  //清空缓存数组
            RxLine=0;  //清空接收长度
            break;
        }
    }
}
/*
 * 解析出DataBuff中的数据
 * 返回解析得到的数据
 */
float Get_Data(void)
{
    uint8_t data_Start_Num = 0; // 记录数据位开始的地方
    uint8_t data_End_Num = 0; // 记录数据位结束的地方
    float data_return = 0; // 解析得到的数据
    
    // 查找等号和感叹号的位置
    for(uint8_t i=0;i<commandlength;i++) 
    {
        if(DataBuff[i] == framehead) 
        {
            data_Start_Num = i + 1; // +1是直接定位到数据起始位
        }
        if(DataBuff[i] == frametail)
        {
            data_End_Num = i;
            break;
        }
    }
    
    // 将数据部分转换为浮点数
    if(data_Start_Num < data_End_Num)
    {
        // 临时缓冲区存储数据字符串
        char temp_buff[50] = {0};
        uint8_t j = 0;
        
        // 复制数据部分到临时缓冲区
        for(uint8_t i=data_Start_Num; i<data_End_Num; i++)
        {
            temp_buff[j++] = DataBuff[i];
        }
        temp_buff[j] = '\0'; // 字符串结束符
        
        // 使用标准库函数将字符串转换为浮点数
        data_return = atof(temp_buff);
        UART_Print("data=%.2f\r\n",data_return);
    }
    
    return data_return;
}


/*
 * 根据串口信息进行PID调参
 */
void USART_PID_Adjust(uint8_t Motor_n)
{
    float data_Get = Get_Data(); // 存放接收到的数据
    
    // 打印接收到的完整命令，用于调试
    UART_Print("Received command:");
    for(int i=0; i<RxLine; i++)
    {
        UART_Print("%c", DataBuff[i]);
    }
    UART_Print("\r\n");
    
    if(Motor_n == 0)//电机1
    {
        // 查找命令类型（在等号之前的部分）
        uint8_t cmd_end = 0;
        for(uint8_t i=0; i<commandlength; i++)
        {
            if(DataBuff[i] == framehead)
            {
                cmd_end = i;
                break;
            }
        }
        
        // 根据命令类型进行处理
        if(cmd_end > 0)
        {
            // 临时缓冲区存储命令字符串
            char cmd_buff[20] = {0};
            for(uint8_t i=0; i<cmd_end; i++)
            {
                cmd_buff[i] = DataBuff[i];
            }
            cmd_buff[cmd_end] = '\0';
            
            // 打印命令类型，用于调试
            UART_Print("Command type: %s\r\n", cmd_buff);
            
            // 处理不同类型的命令
            if(strcmp(cmd_buff, "P1") == 0) // 位置环P
            {
                para.speed_kp = data_Get;
                UART_Print("Set P1 (speed_kp) to %.2f\r\n", data_Get);
            }
            else if(strcmp(cmd_buff, "I1") == 0) // 位置环I
            {
                para.speed_ki = data_Get;
                UART_Print("Set I1 (speed_ki) to %.2f\r\n", data_Get);
            }
            else if(strcmp(cmd_buff, "D1") == 0) // 位置环D
            {
                para.speed_kd = data_Get;
                UART_Print("Set D1 (speed_kd) to %.2f\r\n", data_Get);
            }
            else if(strcmp(cmd_buff, "P2") == 0) // 速度环P
            {
                para.position_kp = data_Get;
                UART_Print("Set P2 (position_kp) to %.2f\r\n", data_Get);
            }
            else if(strcmp(cmd_buff, "I2") == 0) // 速度环I
            {
                para.position_ki = data_Get;
                UART_Print("Set I2 (position_ki) to %.2f\r\n", data_Get);
            }
            else if(strcmp(cmd_buff, "D2") == 0) // 速度环D
            {
                para.position_kd = data_Get;
                UART_Print("Set D2 (position_kd) to %.2f\r\n", data_Get);
            }
            else if(strcmp(cmd_buff, "Spe") == 0) //目标速度
            {
                para.speed_target = data_Get*100;
                UART_Print("Set Spe (speed_target) to %.2f\r\n", data_Get*100);
            }
            else if(strcmp(cmd_buff, "Ang") == 0) //目标速度
            {
                para.angle_target = data_Get;
                UART_Print("Set Ang (angle_target) to %.2f\r\n", data_Get);
            }
//后续想要扩展命令从这里添加else if即可
        }
    }
    

}





