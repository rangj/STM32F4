#include "includes.h"

//USART3 communication to PC, for debug.
#define USART3_PORT             GPIOD
#define USART3_RX_PIN           GPIO_Pin_9
#define USART3_TX_PIN           GPIO_Pin_8
#define USART3_RX_PIN_SOURCE    GPIO_PinSource9
#define USART3_TX_PIN_SOURCE    GPIO_PinSource8

__IO uint8_t USART3_DMA_RD_Buff[10];    //USART3 DMA接收缓冲区
__IO uint8_t USART3_DMA_TD_Buff[10];    //USART3 DMA发送缓冲区
INT8U USART3_RX_QueueBuff[USART3_RX_QUEUE_MAX_SIZE];      //定义USART3_RX_Queue buff
ADT_QUEUE_TYPE5_DEF USART3_RX_Queue;                      //定义USART3_RX_Queue FIFO
INT8U USART3_TX_QueueBuff[USART3_TX_QUEUE_MAX_SIZE];     //定义USART3_TX_Queue buff
ADT_QUEUE_TYPE5_DEF USART3_TX_Queue;                     //定义USART3_TX_Queue FIFO

const INT32U UART_BAUDRATE_TABLE[] = {4800,9600,14400,19200,38400,56000,57600,115200};
void USART3_Init(void)
{
    DMA_InitTypeDef     DMA_InitStructure;
    GPIO_InitTypeDef    GPIO_InitStructure;
    NVIC_InitTypeDef    NVIC_InitStructure;
    USART_InitTypeDef   USART_InitStructure;
    //分配USART3管脚
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
    GPIO_InitStructure.GPIO_Pin = USART3_TX_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(USART3_PORT, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = USART3_RX_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(USART3_PORT, &GPIO_InitStructure);
    GPIO_PinAFConfig(USART3_PORT, USART3_TX_PIN_SOURCE, GPIO_AF_USART3);
    GPIO_PinAFConfig(USART3_PORT, USART3_RX_PIN_SOURCE, GPIO_AF_USART3);

    //USART_InitStructure.USART_BaudRate = 115200;
    if(LocalProperty.RS232_1_Baudrate <= 7)
        USART_InitStructure.USART_BaudRate = UART_BAUDRATE_TABLE[LocalProperty.RS232_1_Baudrate];
    else
        USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    /* Configure USART3 */
    USART_Init(USART3, &USART_InitStructure);
    /* Enable the USART3 */
    USART_Cmd(USART3, ENABLE);//仿真看到执行这里，TC标志居然被设置为1了，不知道实际在flash中运行是否是这样
    USART_ClearFlag(USART3, USART_FLAG_TC);
    USART_ClearFlag(USART3, USART_FLAG_LBD);
    USART_ClearFlag(USART3, USART_FLAG_LBD);
    USART_ClearFlag(USART3, USART_FLAG_RXNE);
    //USART_ITConfig(USART4, USART_IT_TC, ENABLE);
    //DMA 请求
    USART_DMACmd(USART3, USART_DMAReq_Rx, ENABLE);
    //USART_DMACmd(USART3, USART_DMAReq_Tx, ENABLE);
    RCC_AHB1PeriphClockCmd(USART3_RX_DMA_RCC, ENABLE);  //记住DMA也要开时钟的
    //USART3 RX DMA Configure
    DMA_DeInit(USART3_RX_DMA_STREAM);
    DMA_InitStructure.DMA_Channel = USART3_RX_DMA_CHANNEL;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&(USART3->DR));
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&USART3_DMA_RD_Buff;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;  //DMA_DIR_PeripheralToMemory
    DMA_InitStructure.DMA_BufferSize = 10;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; //这里是byte
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular; //这里必须设置为这个模式
    DMA_InitStructure.DMA_Priority = DMA_Priority_Low;
    //DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_Init(USART3_RX_DMA_STREAM, &DMA_InitStructure);
    DMA_ITConfig(USART3_RX_DMA_STREAM, DMA_IT_TC, ENABLE);
    /* DMA2_Stream5(USART3_RX) enable */
    DMA_Cmd(USART3_RX_DMA_STREAM, ENABLE);
    //USART3 TX DMA Configure
    DMA_InitStructure.DMA_Channel = USART3_TX_DMA_CHANNEL;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&(USART3->DR));
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&USART3_DMA_TD_Buff;
    DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;  //DMA_DIR_MemoryToPeripheral;
    //DMA_InitStructure.DMA_BufferSize = 10;
    DMA_InitStructure.DMA_BufferSize = 0;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; //这里是byte
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    //DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;//DMA_Priority_Low;
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_Init(USART3_TX_DMA_STREAM, &DMA_InitStructure);
    DMA_ITConfig(USART3_TX_DMA_STREAM, DMA_IT_TC, DISABLE);
    DMA_ITConfig(USART3_TX_DMA_STREAM, DMA_IT_TE, DISABLE);
    DMA_ITConfig(USART3_TX_DMA_STREAM, DMA_IT_FE, DISABLE);
    DMA_ITConfig(USART3_TX_DMA_STREAM, DMA_IT_HT, DISABLE);
    //DMA_Cmd(USART3_TX_DMA_STREAM, ENABLE);
    /* DMA2_Stream7(USART3_TX) enable */
    //DMA_Cmd(DMA2_Stream7, ENABLE);    注意:这里不能使能DMA,否则会在第一帧发送的第一个字节出错的
    //USART3 DMA 接收完成中断配置
    //NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    NVIC_InitStructure.NVIC_IRQChannel = USART3_RX_DMA_IRQN;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    //USART3 DMA 发送完成中断配置
    //NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    NVIC_InitStructure.NVIC_IRQChannel = USART3_TX_DMA_IRQN;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

//USART3 DMA  接收完成中断服务程序
void USART3_RX_DMA_IRQHandler(void)
{

    INT8U q_err;

    if(DMA_GetITStatus(USART3_RX_DMA_STREAM, USART3_RX_DMA_IT_TCIF) != RESET)
    {
        DMA_ClearFlag(USART3_RX_DMA_STREAM, USART3_RX_DMA_FLAG_TCIF);
        DMA_ClearFlag(USART3_RX_DMA_STREAM, USART3_RX_DMA_IT_TCIF);

        q_err = AddQueueType5_Item(&USART3_RX_Queue, USART3_DMA_RD_Buff,10);
        if(q_err == 0)    //如果这帧数据被正确放入FIFO,就发一个消息,否则是不发.
        {
            SysService.ComMsg.Uart3RxFnCnt ++;
        }
    }
}

//USART3 DMA  发送完成中断服务程序
void USART3_TX_DMA_IRQHandler(void)
{
    if(DMA_GetITStatus(USART3_TX_DMA_STREAM, USART3_TX_DMA_IT_TCIF) != RESET)
    {
        //GPIO_SetBits(GPIOD, GPIO_Pin_15);
        DMA_ClearITPendingBit(USART3_TX_DMA_STREAM, USART3_TX_DMA_IT_TCIF);
        USART_DMACmd(USART3, USART_DMAReq_Tx, DISABLE);
        DMA_Cmd(USART3_TX_DMA_STREAM, DISABLE);
    }
}


//USART1 communication to radar, for obstacle avoidance.
#define USART1_PORT             GPIOA
#define USART1_RX_PIN           GPIO_Pin_10
#define USART1_TX_PIN           GPIO_Pin_9
#define USART1_RX_PIN_SOURCE    GPIO_PinSource10
#define USART1_TX_PIN_SOURCE    GPIO_PinSource9

__IO uint8_t USART1_DMA_RD_Buff[10];    //USART1 DMA接收缓冲区
__IO uint8_t USART1_DMA_TD_Buff[10];    //USART1 DMA发送缓冲区
INT8U USART1_RX_QueueBuff[USART1_RX_QUEUE_MAX_SIZE];      //定义USART1_RX_Queue buff
ADT_QUEUE_TYPE5_DEF USART1_RX_Queue;                      //定义USART1_RX_Queue FIFO

void USART1_Init(void)
{
    DMA_InitTypeDef     DMA_InitStructure;
    GPIO_InitTypeDef    GPIO_InitStructure;
    NVIC_InitTypeDef    NVIC_InitStructure;
    USART_InitTypeDef   USART_InitStructure;
    //分配USART1管脚
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
    GPIO_InitStructure.GPIO_Pin = USART1_TX_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(USART1_PORT, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = USART1_RX_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(USART1_PORT, &GPIO_InitStructure);
    GPIO_PinAFConfig(USART1_PORT, USART1_TX_PIN_SOURCE, GPIO_AF_USART1);
    GPIO_PinAFConfig(USART1_PORT, USART1_RX_PIN_SOURCE, GPIO_AF_USART1);

    //USART_InitStructure.USART_BaudRate = 9600;
    if(LocalProperty.RS232_1_Baudrate <= 7)
        USART_InitStructure.USART_BaudRate = UART_BAUDRATE_TABLE[LocalProperty.RS232_2_Baudrate];
    else
        USART_InitStructure.USART_BaudRate = 9600;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    /* Configure USART1 */
    USART_Init(USART1, &USART_InitStructure);
    /* Enable the USART1 */
    USART_Cmd(USART1, ENABLE);//仿真看到执行这里，TC标志居然被设置为1了，不知道实际在flash中运行是否是这样
    USART_ClearFlag(USART1, USART_FLAG_TC);
    //USART_ClearFlag(USART1, USART_FLAG_LBD);
    //USART_ClearFlag(USART1, USART_FLAG_LBD);
    //USART_ClearFlag(USART1, USART_FLAG_RXNE);
    //USART_ITConfig(USART1, USART_IT_TC, ENABLE);
    //DMA 请求
    USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);
    //USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);
    RCC_AHB1PeriphClockCmd(USART1_RX_DMA_RCC, ENABLE);  //记住DMA也要开时钟的
    //USART1 RX DMA Configure
    DMA_InitStructure.DMA_Channel = USART1_RX_DMA_CHANNEL;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&(USART1->DR));
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&USART1_DMA_RD_Buff;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;  //DMA_DIR_PeripheralToMemory
    //DMA_InitStructure.DMA_BufferSize = USART1_DMA_RD_BUFF_SIZE;
    DMA_InitStructure.DMA_BufferSize = SysService.Radar.FrameBytes;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; //这里是byte
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular; //这里必须设置为这个模式
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    //DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_Init(USART1_RX_DMA_STREAM, &DMA_InitStructure);
    DMA_ITConfig(USART1_RX_DMA_STREAM, DMA_IT_TC, ENABLE);
    /* DMA2_Stream5(USART1_RX) enable */
    DMA_Cmd(USART1_RX_DMA_STREAM, ENABLE);
    //USART1 TX DMA Configure
    DMA_InitStructure.DMA_Channel = USART1_TX_DMA_CHANNEL;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&(USART1->DR));
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&USART1_DMA_TD_Buff;
    DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;  //DMA_DIR_MemoryToPeripheral;
    DMA_InitStructure.DMA_BufferSize = USART1_DMA_TD_BUFF_SIZE;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; //这里是byte
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority = DMA_Priority_Low;
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_Init(USART1_TX_DMA_STREAM, &DMA_InitStructure);
    DMA_ITConfig(USART1_TX_DMA_STREAM, DMA_IT_TC, DISABLE);
    DMA_ITConfig(USART1_TX_DMA_STREAM, DMA_IT_TE, DISABLE);
    DMA_ITConfig(USART1_TX_DMA_STREAM, DMA_IT_FE, DISABLE);
    DMA_ITConfig(USART1_TX_DMA_STREAM, DMA_IT_HT, DISABLE);
    /* DMA2_Stream7(USART1_TX) enable */
    //DMA_Cmd(DMA2_Stream7, ENABLE);    注意:这里不能使能DMA,否则会在第一帧发送的第一个字节出错的
    //USART1 DMA 接收完成中断配置
    //NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    NVIC_InitStructure.NVIC_IRQChannel = USART1_RX_DMA_IRQN;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    //USART1 DMA 发送完成中断配置
    //NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    NVIC_InitStructure.NVIC_IRQChannel = USART1_TX_DMA_IRQN;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

//USART1 DMA  接收完成中断服务程序
void USART1_RX_DMA_IRQHandler(void)
{

    INT8U q_err;

    if(DMA_GetITStatus(USART1_RX_DMA_STREAM, USART1_RX_DMA_IT_TCIF) != RESET)
    {
        DMA_ClearFlag(USART1_RX_DMA_STREAM, USART1_RX_DMA_FLAG_TCIF);
        DMA_ClearFlag(USART1_RX_DMA_STREAM, USART1_RX_DMA_IT_TCIF);

        q_err = AddQueueType5_Item(&USART1_RX_Queue, USART1_DMA_RD_Buff,8);

        if(q_err == 0)    //如果这帧数据被正确放入FIFO,就发一个消息,否则是不发.
        {
            SysService.ComMsg.Uart1RxFnCnt ++;
        }
    }
}

