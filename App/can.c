#include "includes.h"

void CAN_Interrupt(void);

#define CAN_TX1_Pin  GPIO_Pin_9
#define CAN_RX1_Pin  GPIO_Pin_8
#define CAN1_GPIO  GPIOB
#define CAN1_RX1_PIN_SOURCE     GPIO_PinSource8
#define CAN1_TX1_PIN_SOURCE     GPIO_PinSource9

#define CAN2_RX_PIN  GPIO_Pin_12
#define CAN2_TX_PIN  GPIO_Pin_13
#define CAN2_GPIO  GPIOB
#define CAN2_RX_PIN_SOURCE     GPIO_PinSource12
#define CAN2_TX_PIN_SOURCE     GPIO_PinSource13

CanTxMsg CAN1_TX;
CanRxMsg CAN1_RX;
CanTxMsg CAN2_TX;
CanRxMsg CAN2_RX;

INT8U CAN2_RX_QueueBuff[CAN2_RX_QUEUE_MAX_SIZE];        //定义CAN1_RX_Queue buff
ADT_QUEUE_TYPE5_DEF CAN2_RX_Queue;                      //定义CAN1_RX_Queue FIFO

void USERCAN_Init(void)
{
    GPIO_InitTypeDef       GPIO_InitStructure;
    //CAN_InitTypeDef        CAN_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2, ENABLE);

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    //CAN1_RX,TX
    GPIO_InitStructure.GPIO_Pin    =  CAN_TX1_Pin | CAN_RX1_Pin;
    GPIO_InitStructure.GPIO_Mode   =  GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed  =  GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd   =  GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_OType  =  GPIO_OType_PP;
    //GPIO_Init(CAN1_GPIO,&GPIO_InitStructure);
    //GPIO_PinAFConfig(CAN1_GPIO, CAN1_RX1_PIN_SOURCE, GPIO_AF_CAN1);
    //GPIO_PinAFConfig(CAN1_GPIO, CAN1_TX1_PIN_SOURCE, GPIO_AF_CAN1);
    //
    //CAN2_RX,TX
    GPIO_InitStructure.GPIO_Pin    =  CAN2_RX_PIN | CAN2_TX_PIN;
    GPIO_InitStructure.GPIO_Mode   =  GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed  =  GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd   =  GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_OType  =  GPIO_OType_PP;
    GPIO_Init(CAN2_GPIO,&GPIO_InitStructure);



    GPIO_PinAFConfig(CAN2_GPIO, CAN2_RX_PIN_SOURCE, GPIO_AF_CAN2);
    GPIO_PinAFConfig(CAN2_GPIO, CAN2_TX_PIN_SOURCE, GPIO_AF_CAN2);
    CAN_Interrupt();
}
/**
  * @brief  Configures the CAN, transmit and receive using interrupt.
  * @param  None
  * @retval PASSED if the reception is well done, FAILED in other case
  */
void CAN_Interrupt(void)
{
  CAN_InitTypeDef        CAN_InitStructure;
  CAN_FilterInitTypeDef  CAN_FilterInitStructure;
  INT16U COB_ID;
  INT16U COB_ID_H;
  INT16U COB_ID_L;
  NVIC_InitTypeDef NVIC_InitStructure;
  /* CAN register init */
  CAN_DeInit(CAN1);
  CAN_DeInit(CAN2);
  /* CAN cell init */
  CAN_InitStructure.CAN_TTCM = DISABLE;
  CAN_InitStructure.CAN_ABOM = DISABLE;
  CAN_InitStructure.CAN_AWUM = DISABLE;
  CAN_InitStructure.CAN_NART = DISABLE;
  CAN_InitStructure.CAN_RFLM = DISABLE;
  CAN_InitStructure.CAN_TXFP = DISABLE;
  CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;
  CAN_InitStructure.CAN_SJW =  CAN_SJW_1tq;

  /* Baudrate = 500 Kbps   Baudrate=1/(1*tq+tbs1+tbs2)=1/((1+5+8)*6*Tpclk)  Tpclk=1/84MHz tq=(BRP[9:0]+1)*Tpclk */
  CAN_InitStructure.CAN_BS1 = CAN_BS1_5tq;
  CAN_InitStructure.CAN_BS2 = CAN_BS2_8tq;
  CAN_InitStructure.CAN_Prescaler = 6;
  CAN_Init(CAN1, &CAN_InitStructure);

  /* CAN2 cell init */
  CAN_Init(CAN2, &CAN_InitStructure);
#if 0
  COB_ID=0x600 | LocalProperty.MyID;  //接收从机ID
  COB_ID=(COB_ID<<5);
  CAN_FilterInitStructure.CAN_FilterNumber = 2;
  CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdList;
  CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_16bit;
  CAN_FilterInitStructure.CAN_FilterIdHigh = COB_ID;
  CAN_FilterInitStructure.CAN_FilterIdLow = COB_ID;
  CAN_FilterInitStructure.CAN_FilterMaskIdHigh = COB_ID; //只接收本机ID 0x40~0x4F
  CAN_FilterInitStructure.CAN_FilterMaskIdLow = COB_ID; //只接收本机ID 0x40~0x4F
  CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_FIFO0;
  CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
  CAN_FilterInit(&CAN_FilterInitStructure);

  COB_ID=0x600 | LocalProperty.MyID;  //接收主机ID
  COB_ID=(COB_ID<<5);
  CAN_SlaveStartBank(15);
  CAN_FilterInitStructure.CAN_FilterNumber = 15;
  CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdList;
  CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_16bit;
  CAN_FilterInitStructure.CAN_FilterIdHigh = COB_ID;
  CAN_FilterInitStructure.CAN_FilterIdLow = COB_ID;
  CAN_FilterInitStructure.CAN_FilterMaskIdHigh = COB_ID; //只接收本机ID 0x40~0x4F  Function cade[10:7]_ID[6:3]_  | ID[2:0]_ptr_ide_exid[17:15]
  CAN_FilterInitStructure.CAN_FilterMaskIdLow  = COB_ID; //只接收本机ID 0x40~0x4F
  CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_FIFO0;
  CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
  CAN_FilterInit(&CAN_FilterInitStructure);
#endif

  COB_ID_L=0x600 | LocalProperty.MyID;  //接收从机ID
  COB_ID_H=COB_ID_L + 1;
  COB_ID_L=(COB_ID_L<<5);
  COB_ID_H=(COB_ID_H<<5);
  CAN_FilterInitStructure.CAN_FilterNumber = 2;
  CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdList;
  CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_16bit;
  CAN_FilterInitStructure.CAN_FilterIdHigh = COB_ID_H;
  CAN_FilterInitStructure.CAN_FilterIdLow = COB_ID_L;
  CAN_FilterInitStructure.CAN_FilterMaskIdHigh = COB_ID_H; //只接收本机ID 0x40~0x4F
  CAN_FilterInitStructure.CAN_FilterMaskIdLow = COB_ID_L; //只接收本机ID 0x40~0x4F
  CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_FIFO0;
  CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
  CAN_FilterInit(&CAN_FilterInitStructure);

  COB_ID_L=0x600 | LocalProperty.MyID;  //接收从机ID
  COB_ID_H=COB_ID_L + 1;
  COB_ID_L=(COB_ID_L<<5);
  COB_ID_H=(COB_ID_H<<5);
  CAN_SlaveStartBank(15);
  CAN_FilterInitStructure.CAN_FilterNumber = 15;
  CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdList;
  CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_16bit;
  CAN_FilterInitStructure.CAN_FilterIdHigh = COB_ID_H;
  CAN_FilterInitStructure.CAN_FilterIdLow = COB_ID_L;
  CAN_FilterInitStructure.CAN_FilterMaskIdHigh = COB_ID_H; //只接收本机ID 0x40~0x4F  Function cade[10:7]_ID[6:3]_  | ID[2:0]_ptr_ide_exid[17:15]
  CAN_FilterInitStructure.CAN_FilterMaskIdLow  = COB_ID_L; //只接收本机ID 0x40~0x4F
  CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_FIFO0;
  CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
  CAN_FilterInit(&CAN_FilterInitStructure);

  CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);
  CAN_ITConfig(CAN1, CAN_IT_TME,  ENABLE);

  CAN_ITConfig(CAN2, CAN_IT_FMP0, ENABLE);
  CAN_ITConfig(CAN2, CAN_IT_TME,  ENABLE);

  NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  NVIC_InitStructure.NVIC_IRQChannel = CAN1_TX_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  NVIC_InitStructure.NVIC_IRQChannel = CAN2_RX0_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  NVIC_InitStructure.NVIC_IRQChannel = CAN2_TX_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}
#if 0
void CAN1_RX0_IRQHandler(void)
{
    GPIO_ResetBits(CAN1_LED_GPIO, CAN1_LED_PIN);
    CAN_Receive(CAN1, CAN_FIFO0, &CAN1_RX);
    CAN_FIFORelease(CAN1, CAN_FIFO0);
    AddCANQueue_Item(&CAN1RxQueue,&CAN1_RX);
    OSQPost(CAN_RX_INT, &CAN1RxQueue);
}

void CAN1_TX_IRQHandler(void)
{
    if(CAN_GetITStatus(CAN1, CAN_IT_TME)!=RESET)
    {
        CAN_ClearITPendingBit(CAN1, CAN_IT_TME);
        GPIO_SetBits(CAN1_LED_GPIO, CAN1_LED_PIN);
    }
}

#endif
void CAN2_RX0_IRQHandler(void)
{
    INT8U q_err;
    INT8U id_temp;
    CAN_Receive(CAN2, CAN_FIFO0, &CAN2_RX);
    CAN_FIFORelease(CAN2, CAN_FIFO0);
    id_temp = CAN2_RX.StdId;
    q_err = AddQueueType5_Item(&CAN2_RX_Queue,&id_temp,1); //put id into queue
    q_err = AddQueueType5_Item(&CAN2_RX_Queue, CAN2_RX.Data,8);
    if(q_err == 0)    //如果这帧数据被正确放入FIFO,就发一个消息,否则是不发.
    {
        SysService.ComMsg.CAN2_RxFnCnt ++;
    }
}

void CAN2_TX_IRQHandler(void)
{
    if(CAN_GetITStatus(CAN2, CAN_IT_TME)!=RESET)
    {
        CAN_ClearITPendingBit(CAN2, CAN_IT_TME);
        //GPIO_WriteBit(BUSY_LED_PORT, BUSY_LED_PIN, !GPIO_ReadOutputDataBit(BUSY_LED_PORT, BUSY_LED_PIN));
    }
}
