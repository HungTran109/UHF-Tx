//*****************************************************************************
//  File Name:    KT_WirelessMicTxdrv.c
//  Function:    KT Wireless Mic Transmitter Products Driver For Customer
//                (KT064xM)
//*****************************************************************************
//        Revision History
//  Version Date        Description
//  V0.1    2016-04-26  For KT0646M_VX
//  V0.2    2016-06-01  ����VX��VX2�ı���ѡ��    
//  V0.3    2016-08-18  For KT0646M_VX4
//  V0.3.1  2016-08-24  ɾ��KT_WirelessMicTx_Init������һЩ�ظ�����
//  V0.4    2016-08-26  ������XTAL_DUAL����
//  V0.5    2016-10-11  �޸��˳�ʼ�������е�һЩ���ã���0x25,0x26�Ĵ��������üӵ��˳�ʼ���������棬
//                  �ڳ�ʼ����������set pilot frequency
//  V0.5.1  2016-10-17  �����޸���һ������ĺ궨�壬������RXISKT0616M����BPSK�ľ�ģʽ��Ȼ���ٸ���
//                  �Ƿ���˫�����KT0616Mѡ����ĳ���ӽ�ȥ��
//  V0.5.2  2016-11-15  ��tunęǰ��lo_fine_vref_sel��Ϊ3��tune���ĳ�0.
//  V0.5.3  2016-12-15  �ڳ�ʼ���а�lo_fine_vref_sel��Ϊ3��������tunę��ʱ���������޸���.Ϊ���¶�
//                  �仯�����������⣬�������DOUBLE_KVCO��Χ�Ĵ���
//  V0.6    2017-02-08  ��ʽ�淶������
//  V1.1    2017-04-27  ɾ����һЩ��ʼ����������û�õĶ���������Щ��Сд��ĸ����ĺ궨��ĳ��˴�д����ĸ
//  V1.2    2017-05-24  ��ʼ���У���HARD_LIMIT��14�ĳ���15��COMPANDOR_TC�ĳ���3����48ms��
//  V1.3    2017-09-18	COMPEN_GAIN��3�ĳ�1,MIC_SENS_GAIN��9�ĳ�5,COMPANDOR_TC_48ms�ĳ�COMPANDOR_TC_12ms
//  V1.4    2017-10-10  echo�رյ�ʱ�򣬲�������echo���ǰ�Echo_Ratioд��0,soft_rst�����ڼĴ���0x3e��bit15�ˣ�������0x1e��bit4.
//  V1.5    2017-12-14  ���ݽ�����KT0616Mʱ����BPSK_NEW_MODE�ĳ�0������Ϊ1
//*****************************************************************************

//-----------------------------------------------------------------------------
// Includes
//-----------------------------------------------------------------------------
#include "KT_WirelessMicTxdrv.h"
#include <math.h>
#include <string.h>
#include "app_debug.h"
#include <stdio.h>
#include <stdlib.h>

#define KT0656M_LONG_TIMEOUT 3000
#define DIRECT_ACCESS 1
#define I2C_ADDR_DIRECT_ACCESS 0x12
#if(DEBUG_I2C_FM)
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
 
/**
  * @brief  Retargets the C library printf function.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
  SEGGER_RTT_Write(0, (uint8_t *)&ch, 1);
  return ch;
}
#define DEBUG_I2C(format_, ...)  DEBUG_RAW(format_, ##__VA_ARGS__)
#else
#define DEBUG_I2C(format_, ...) (void)(0)
#endif



#define UHF_I2C I2C1
/*****************************************************************************
 * Typedef Implementation
 *****************************************************************************/

typedef union {
    struct
    {
        uint8_t lowByte;
        uint8_t highByte;
    } refined;
    uint16_t raw;
} word16_to_bytes;
/*****************************************************************************
 * Private variables
 *****************************************************************************/

static volatile uint32_t KT0656M_Timeout = 3000;
/*****************************************************************************
 * Implementation
 *****************************************************************************/


 /*****************************************************************************
 * Global variables
 *****************************************************************************/


uint8_t MorSSelect = 1;//�����߷ּ�ʱ 1:�� 0:��
  /*****************************************************************************
 * Function  ariables
 *****************************************************************************/
//*****************************************************************************
//  File Name: I2C.c
//  Function:  KT Wireless Mic Transmitter Products Demoboard I2C Function Define
//*****************************************************************************

#define USE_SOFT_I2C 1
#if(!USE_SOFT_I2C)
#include "i2c.h"
#endif
//-----------------------------------------------------------------------------
// Global VARIABLES
//-----------------------------------------------------------------------------
bool Ack_Flag=0; // I2C Ack Flag

//-----------------------------------------------------------------------------
// Function PROTOTYPES
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
//�� �� ����I2C_Delay
//����������I2C��ʱ
//����˵����
//ȫ�ֱ�������
//��    �룺��
//��    �أ���
//�� �� �ߣ�PAE                     ʱ�䣺2012-08-01
//�� �� �ߣ�                        ʱ�䣺
//��    ����V1.0
//-----------------------------------------------------------------------------
void I2C_Delay(void)
{
//    UINT8 i;
    
   for(int i=0;i<=100;i++)
    {
        __nop();
    }
}

//-----------------------------------------------------------------------------
//�� �� ����I2C_Start
//����������I2C����֡��ʼ
//����˵����
//ȫ�ֱ�������
//��    �룺��
//��    �أ���
//�� �� �ߣ�PAE                     ʱ�䣺2012-08-01
//�� �� �ߣ�                        ʱ�䣺
//��    ����V1.0
//-----------------------------------------------------------------------------
void I2C_Start(void)
{
    I2C_Delay();
    I2C_Delay();
    //SDA = 1;
    LL_GPIO_SetOutputPin(UHF_SDA_GPIO_Port, UHF_SDA_Pin);
    I2C_Delay();
    I2C_Delay();
    //SCL = 1;
    LL_GPIO_SetOutputPin(UHF_SCL_GPIO_Port, UHF_SCL_Pin);
    I2C_Delay();
    I2C_Delay();
    //SDA = 0;
    LL_GPIO_ResetOutputPin(UHF_SDA_GPIO_Port, UHF_SDA_Pin);
    I2C_Delay();
    I2C_Delay();
    //SCL = 0;
    LL_GPIO_ResetOutputPin(UHF_SCL_GPIO_Port, UHF_SCL_Pin);
    I2C_Delay();
    I2C_Delay();
}

//-----------------------------------------------------------------------------
//�� �� ����I2C_Senddata
//����������I2C��������
//����˵����
//ȫ�ֱ�������
//��    �룺uchar senddata
//��    �أ���
//�� �� �ߣ�PAE                     ʱ�䣺2012-08-01
//�� �� �ߣ�                        ʱ�䣺
//��    ����V1.0
//-----------------------------------------------------------------------------
void I2C_Senddata(uint8_t senddata)
{
    uint8_t i;
    
    for (i=0;i<8;i++)
    {    
        I2C_Delay();
        if ((senddata & 0x80) != 0x80)
            //SDA = 0;
            LL_GPIO_ResetOutputPin(UHF_SDA_GPIO_Port, UHF_SDA_Pin);
        else 
            //SDA = 1;
            LL_GPIO_SetOutputPin(UHF_SDA_GPIO_Port, UHF_SDA_Pin);
        senddata = senddata << 1;
        I2C_Delay();
        //SCL = 1;
        LL_GPIO_SetOutputPin(UHF_SCL_GPIO_Port, UHF_SCL_Pin);
        I2C_Delay();
        //SCL = 0;
        LL_GPIO_ResetOutputPin(UHF_SCL_GPIO_Port, UHF_SCL_Pin);
    }
    I2C_Delay();
}

//-----------------------------------------------------------------------------
//�� �� ����I2C_Receivedata
//����������I2C��������
//����˵����
//ȫ�ֱ�������
//��    �룺��
//��    �أ�uchar receivedata
//�� �� �ߣ�PAE                     ʱ�䣺2012-08-01
//�� �� �ߣ�                        ʱ�䣺
//��    ����V1.0
//-----------------------------------------------------------------------------
uint8_t I2C_Receivedata(void)
{
    uint8_t i,temp,receivedata=0;
    
    for (i = 0; i < 8; i++)
    {
        I2C_Delay();
        //SCL = 1;
        LL_GPIO_SetOutputPin(UHF_SCL_GPIO_Port, UHF_SCL_Pin);
        LL_GPIO_SetPinMode(UHF_SDA_GPIO_Port, UHF_SDA_Pin, LL_GPIO_MODE_INPUT);
        I2C_Delay();
        
        temp = LL_GPIO_IsInputPinSet(UHF_SDA_GPIO_Port, UHF_SDA_Pin);
        LL_GPIO_SetPinMode(UHF_SDA_GPIO_Port, UHF_SDA_Pin, LL_GPIO_MODE_OUTPUT);
        //SCL = 0;
        LL_GPIO_ResetOutputPin(UHF_SCL_GPIO_Port, UHF_SCL_Pin);
        receivedata = receivedata | temp;
        if (i < 7)
        {
            receivedata = receivedata << 1;
        }
    }
    I2C_Delay();
    return(receivedata);    
}

//-----------------------------------------------------------------------------
//�� �� ����I2C_Ack
//����������I2C_Ack
//����˵����
//ȫ�ֱ�����Ack_Flag
//��    �룺��
//��    �أ���
//�� �� �ߣ�PAE                     ʱ�䣺2012-08-01
//�� �� �ߣ�                        ʱ�䣺
//��    ����V1.0
//-----------------------------------------------------------------------------
void I2C_Ack(void)
{
    //SDA = 1;
    LL_GPIO_SetOutputPin(UHF_SDA_GPIO_Port, UHF_SDA_Pin);
    I2C_Delay();
    I2C_Delay();
    //SCL = 1;
    LL_GPIO_SetOutputPin(UHF_SCL_GPIO_Port, UHF_SCL_Pin);
    LL_GPIO_SetPinMode(UHF_SDA_GPIO_Port, UHF_SDA_Pin, LL_GPIO_MODE_INPUT);
    I2C_Delay();
    Ack_Flag = LL_GPIO_IsInputPinSet(UHF_SDA_GPIO_Port, UHF_SDA_Pin);
    LL_GPIO_SetPinMode(UHF_SDA_GPIO_Port, UHF_SDA_Pin, LL_GPIO_MODE_OUTPUT);
    //SCL = 0;
    LL_GPIO_ResetOutputPin(UHF_SCL_GPIO_Port, UHF_SCL_Pin);
    I2C_Delay();
    I2C_Delay();
}

//-----------------------------------------------------------------------------
//�� �� ����I2C_Stop
//����������I2C����֡����
//����˵����
//ȫ�ֱ�������
//��    �룺��
//��    �أ���
//�� �� �ߣ�PAE                     ʱ�䣺2012-08-01
//�� �� �ߣ�                        ʱ�䣺
//��    ����V1.0
//-----------------------------------------------------------------------------
void I2C_Stop(void)
{
    //SCL = 0;
    LL_GPIO_ResetOutputPin(UHF_SCL_GPIO_Port, UHF_SCL_Pin);
    I2C_Delay();
    I2C_Delay();
   // SDA = 0;
    
    LL_GPIO_ResetOutputPin(UHF_SDA_GPIO_Port, UHF_SDA_Pin);
    I2C_Delay();
    I2C_Delay();
    //SCL = 1;
    LL_GPIO_SetOutputPin(UHF_SCL_GPIO_Port, UHF_SCL_Pin);
    I2C_Delay();I2C_Delay();
    //SDA = 1;
    LL_GPIO_SetOutputPin(UHF_SDA_GPIO_Port, UHF_SDA_Pin);
    I2C_Delay();I2C_Delay();
}

//-----------------------------------------------------------------------------
//�� �� ����I2C_Byte_Write
//����������I2C��Byteд����
//����˵�����Ĵ�����ַ��ΧΪ16λ
//ȫ�ֱ�������
//��    �룺uchar device_address,UINT16 reg_add,uchar writedata
//��    �أ���
//�� �� �ߣ�PAE                     ʱ�䣺2012-08-01
//�� �� �ߣ�                        ʱ�䣺
//��    ����V1.0
//-----------------------------------------------------------------------------
void I2CS_Byte_Write(uint8_t device_address, uint16_t reg_add, uint16_t writedata)
{
    I2C_Start();
    I2C_Senddata(device_address & 0xFE);
    I2C_Ack();
    if (Ack_Flag == 0)
    {
        I2C_Senddata(reg_add);
        I2C_Ack();
        if (Ack_Flag == 0)
        {
            I2C_Senddata(writedata);
            I2C_Ack();
        }
        else
           // SCL = 0;         
            LL_GPIO_ResetOutputPin(UHF_SCL_GPIO_Port, UHF_SCL_Pin);        
    }
    else
        //SCL = 0;
        LL_GPIO_ResetOutputPin(UHF_SCL_GPIO_Port, UHF_SCL_Pin);
    I2C_Stop();  
}

//-----------------------------------------------------------------------------
//�� �� ����I2C_Byte_Read
//����������I2C��Byte������
//����˵�����Ĵ�����ַ��ΧΪ16λ
//ȫ�ֱ�������
//��    �룺uchar device_address,UINT16 reg_add
//��    �أ���ȷ��uchar readdata    ����0x00
//�� �� �ߣ�PAE                     ʱ�䣺2012-08-01
//�� �� �ߣ�                        ʱ�䣺
//��    ����V1.0
//-----------------------------------------------------------------------------
uint16_t I2CS_Byte_Read(uint8_t device_address, uint16_t reg_add)
{
    uint8_t readdata;
    I2C_Start();
    I2C_Senddata(device_address & 0xFE);
    I2C_Ack();
    if (Ack_Flag == 0)
    {
        I2C_Senddata(reg_add);
        I2C_Ack();
        if (Ack_Flag == 0)
        {
            I2C_Start();
            I2C_Senddata(device_address | 0x01);
            I2C_Ack();
            if (Ack_Flag == 0)
            {
                //    SDA pin is high Z
                readdata = I2C_Receivedata();
                I2C_Ack();
                I2C_Stop();    
                return(readdata);
            }
            else
            {
                I2C_Stop();
                return(0x00);
            }
        }
        else
        {
            I2C_Stop();
             return(0x00);
        }            
    }
    else
    {
        I2C_Stop();
        return(0x00);
    }
}

//-----------------------------------------------------------------------------
//�� �� ����I2C_Byte_Write
//����������I2C��Byteд����
//����˵����
//ȫ�ֱ�������
//��    �룺uchar device_address,UINT8 reg_add,uchar writedata
//��    �أ���
//�� �� �ߣ�PAE                     ʱ�䣺2012-08-01
//�� �� �ߣ�                        ʱ�䣺
//��    ����V1.0
//-----------------------------------------------------------------------------
void I2S_Byte_Write(uint8_t device_address, uint8_t reg_add, uint8_t writedata)
{
    I2C_Start();
    I2C_Senddata(device_address & 0xFE); // Ham write => bit cuoi cung bang 0
    I2C_Ack();
    if (Ack_Flag == 0)
    {
        I2C_Senddata(reg_add);
        I2C_Ack();
        if (Ack_Flag == 0)
        {
            I2C_Senddata(writedata);
            I2C_Ack();
        }
        else
            //SCL = 0;        
            LL_GPIO_ResetOutputPin(UHF_SCL_GPIO_Port, UHF_SCL_Pin);        
    }
    else
        //SCL = 0;
            LL_GPIO_ResetOutputPin(UHF_SCL_GPIO_Port, UHF_SCL_Pin);
    I2C_Stop();    
}
void I2C_Word_Write(uint8_t device_address, uint8_t reg_add, uint16_t writeword)
{
	uint8_t writeword_high,writeword_low;

	writeword_low 	=	writeword;
	writeword_high	=	writeword>>8;

	I2C_Start();
	I2C_Senddata(device_address & 0xFE);
	I2C_Ack();
	if (Ack_Flag == 0)
		{
		 I2C_Senddata(reg_add);
		 I2C_Ack();
		 if (Ack_Flag == 0)
			{
			 I2C_Senddata(writeword_high);
			 I2C_Ack();
			 if (Ack_Flag == 0)
				{
				 I2C_Senddata(writeword_low);
				 I2C_Ack();
				}
			 else{
                 LL_GPIO_ResetOutputPin(UHF_SCL_GPIO_Port, UHF_SCL_Pin);}
			}
		 else{
                 LL_GPIO_ResetOutputPin(UHF_SCL_GPIO_Port, UHF_SCL_Pin);}
		}
	else{
        LL_GPIO_ResetOutputPin(UHF_SCL_GPIO_Port, UHF_SCL_Pin);}
	I2C_Stop();	
}
uint16_t I2C_Word_Read(uint8_t device_address, uint8_t reg_add)
{
	unsigned char readdata_low;
	unsigned int readdata,readdata_high,temp=0;
	I2C_Start();
	I2C_Senddata(device_address & 0xFE);
	I2C_Ack();
	if (Ack_Flag == 0)
		{
		 I2C_Senddata(reg_add);
		 I2C_Ack();
		 if (Ack_Flag == 0)
			{
			 I2C_Start();
			 I2C_Senddata(device_address | 0x01);
			 I2C_Ack();
			 if (Ack_Flag == 0)
				{
//				 SDA = 1;//SDA ��Ϊ���룬������
				 readdata_high = I2C_Receivedata();
				 //SDA = 0;
                 LL_GPIO_ResetOutputPin(UHF_SDA_GPIO_Port, UHF_SDA_Pin);
				 I2C_Delay();I2C_Delay();
				 //SCL = 1;
                 LL_GPIO_SetOutputPin(UHF_SCL_GPIO_Port, UHF_SCL_Pin);
				 I2C_Delay();I2C_Delay();
				 //SCL = 0;
                 LL_GPIO_ResetOutputPin(UHF_SCL_GPIO_Port, UHF_SCL_Pin);
				 I2C_Delay();I2C_Delay();
				 //SDA = 1;
                LL_GPIO_SetOutputPin(UHF_SDA_GPIO_Port, UHF_SDA_Pin);

			 	 if (Ack_Flag == 0)
					{
					 readdata_low = I2C_Receivedata();
					 I2C_Ack();
					}
				 else
					{
					 //SCL=0;
                      LL_GPIO_ResetOutputPin(UHF_SCL_GPIO_Port, UHF_SCL_Pin);
					 return(0x0000);
					}
				}
			 else
				{
				 //SCL=0;
                 LL_GPIO_ResetOutputPin(UHF_SCL_GPIO_Port, UHF_SCL_Pin);
				 return(0x0000);
				}
			}
		 else
		 	{
			 //SCL = 0;
             LL_GPIO_ResetOutputPin(UHF_SCL_GPIO_Port, UHF_SCL_Pin);
		 	 return(0x0000);			
			}
		}
	else
		{
		 //SCL = 0;
         LL_GPIO_ResetOutputPin(UHF_SCL_GPIO_Port, UHF_SCL_Pin);
		 return(0x0000);
		}

	I2C_Stop();
		
	temp=readdata_high<<8;
	readdata=temp | readdata_low;
	return(readdata);
}
//-----------------------------------------------------------------------------
//�� �� ����I2C_Byte_Read
//����������I2C��Byte������
//����˵����
//ȫ�ֱ�������
//��    �룺uchar device_address,UINT8 reg_add
//��    �أ���ȷ��uchar readdata    ����0x00
//�� �� �ߣ�PAE                     ʱ�䣺2012-08-01
//�� �� �ߣ�                        ʱ�䣺
//��    ����V1.0
//-----------------------------------------------------------------------------
uint8_t I2S_Byte_Read(uint8_t device_address, uint8_t reg_add)
{
    uint8_t readdata;
    I2C_Start();
    I2C_Senddata(device_address & 0xFE);
    I2C_Ack();
    if (Ack_Flag == 0)
    {
        I2C_Senddata(reg_add);
        I2C_Ack();
        if (Ack_Flag == 0)
        {
            I2C_Start();
            I2C_Senddata(device_address | 0x01);
            I2C_Ack();
            if (Ack_Flag == 0)
            {
                // SDA pin is high Z
                readdata = I2C_Receivedata();
                I2C_Ack();
                I2C_Stop();    
                return(readdata);
            }
            else
            {
                I2C_Stop();
                return(0x00);
            }
        }
        else
        {
            I2C_Stop();
             return(0x00);
        }            
    }
    else
    {
        I2C_Stop();
        return(0x00);
    }
}
/*

Test Software I2C 
*/
//void I2C_Delay(void)
//{
//    uint8_t i;

//    for(i=0;i<=40;i++)
//    {
//        __nop();
//    }
//}

//uint8_t I2C_Receivedata(void)
//{
//    uint8_t i,temp,receivedata=0;

//    for (i=0;i<8;i++)
//    {
//        receivedata = receivedata << 1;
//        I2C_Delay();
//        SCL = 1;
//        I2C_Delay();
//        temp = SDA;
//        SCL = 0;
//        receivedata = receivedata | temp;
//    }
//    I2C_Delay();
//    return(receivedata);    
//}

 void SwapValue(uint8_t *a, uint8_t *b) 
{
   uint8_t t = *a;
   *a = *b;
   *b = t;
}
#define DIRECT_ACCESS 1
uint32_t I2C_Byte_Write(uint8_t device_addr, uint16_t reg_add, uint8_t data)
{
     KT0656M_Timeout  = KT0656M_LONG_TIMEOUT;
#if(USE_SOFT_I2C)
    return 0;
#else

#if(!DIRECT_ACCESS)
     word16_to_bytes buffer[6] = {0};
     KT0656M_Timeout = KT0935_LONG_TIMEOUT;
     //uint16_t buffer[6] ={0};
     memcpy(&buffer[0] , &reg02.raw, 2);
     memcpy(&buffer[1] , &reg03.raw, 2);
     memcpy(&buffer[2] , &reg04.raw, 2);
     memcpy(&buffer[3] , &reg05.raw, 2);
     memcpy(&buffer[4] , &reg06.raw, 2);
     memcpy(&buffer[5] , &reg07.raw, 2);
     for (uint8_t i = 0; i < 6; i++)
     {
         SwapValue(&buffer[i].refined.highByte,&buffer[i].refined.lowByte);
     }
     uint8_t *p_buffer = (uint8_t*)&buffer[0];
        LL_I2C_HandleTransfer(UHF_I2C, I2C_ADDR_FULL_ACCESS, LL_I2C_ADDRSLAVE_7BIT,  12, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_WRITE);
#else
      word16_to_bytes temp;
      temp.raw = data;      
      SwapValue(&temp.refined.highByte,&temp.refined.lowByte);
      static uint8_t buffer[3] = {0};
      buffer[0] = (uint8_t)reg_add;
      memcpy(&buffer[1], &temp.raw, 2);
      //memcpy(&buffer[1], &temp.raw, 2);
      buffer[2] = data;
      uint8_t *p_buffer = (uint8_t*)&buffer[0];
      LL_I2C_HandleTransfer(UHF_I2C, device_addr, LL_I2C_ADDRSLAVE_7BIT,  3, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_WRITE);
#endif
        /**/

      //uint8_t *p_buffer = (uint8_t*)p_rda_write_reg;
      /* (1) Initiate a Start condition to the Slave device ***********************/

      /* Master Generate Start condition for a write request :              */
      /*    - to the Slave with a 7-Bit SLAVE_OWN_ADDRESS                   */
      /*    - with a auto stop condition generation when transmit all bytes */

    /* (2) Loop until end of transfer received (STOP flag raised) ***************/
     /* Loop until STOP flag is raised  */
    while(!LL_I2C_IsActiveFlag_STOP(UHF_I2C))
    {
        /* (2.1) Transmit data (TXIS flag raised) *********************************/

        /* Check TXIS flag value in ISR register */
        if(LL_I2C_IsActiveFlag_TXIS(UHF_I2C))
        {
          /* Write data in Transmit Data register.
          TXIS flag is cleared by writing data in TXDR register */
          //DEBUG_INFO("p_buffer:0x%02x\r\n",*(p_buffer));
          LL_I2C_TransmitData8(UHF_I2C, *(p_buffer++));
        }
        if (LL_SYSTICK_IsActiveCounterFlag()) 
        {
            if(KT0656M_Timeout-- == 0)
            {
                 /* Time-out occurred. */
                 DEBUG_ERROR("I2C Timeout\r\n");
                 return 1;
            }
        }
    }
    /* (3) Clear pending flags, Data consistency are checking into Slave process */

    /* End of I2C_SlaveReceiver_MasterTransmitter Process */
   // LL_I2C_ClearFlag_STOP(UHF_I2C);
    LL_I2C_ClearFlag_NACK(UHF_I2C);

    /* Clear STOP Flag */
    LL_I2C_ClearFlag_STOP(UHF_I2C);
    return 0;
}
uint32_t I2C_Byte_Read(uint8_t device_addr, uint16_t reg_add, uint8_t *p_receive_buff)
{
     uint8_t *buffer = (uint8_t*)&reg_add;
#if(USE_SOFT_I2C)
    return 0 ;
#endif
    // word16_to_bytes read_data;
     //uint8_t *p_read_data = (uint8_t*)&read_data;
     //uint16_t length = 2;
     //memcpy(buffer, &reg_add, 1);
     KT0656M_Timeout  = KT0656M_LONG_TIMEOUT;
      /* (1) Initiate a Start condition to the Slave device ***********************/
      /*Determine the read register*/
    LL_I2C_HandleTransfer(UHF_I2C, device_addr, LL_I2C_ADDRSLAVE_7BIT, 2, LL_I2C_MODE_SOFTEND, LL_I2C_GENERATE_START_WRITE);

    while (!LL_I2C_IsActiveFlag_TXIS(UHF_I2C)) {};
    LL_I2C_TransmitData8(UHF_I2C, buffer[1]);
        
    while (!LL_I2C_IsActiveFlag_TXIS(UHF_I2C)) {};
    LL_I2C_TransmitData8(UHF_I2C, buffer[0]);
       
    while (!LL_I2C_IsActiveFlag_TC(UHF_I2C)) {};

    LL_I2C_ClearFlag_STOP(UHF_I2C);
    LL_I2C_HandleTransfer(UHF_I2C, device_addr, LL_I2C_ADDRSLAVE_7BIT, 2, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_READ);
          /*We have 2 bytes to read -> Loop for two times :D*/
    for (uint8_t i = 0; i < 2; i++)
    {
            while(!LL_I2C_IsActiveFlag_RXNE(UHF_I2C)){};
            {
                if(KT0656M_Timeout-- == 0)
                {
                    //DEBUG_ERROR("[I2C_UHF]: Error timeout read - received length:%d\r\n", length);
                     return 1;
                }
             *(p_receive_buff + 1 - i) = LL_I2C_ReceiveData8(UHF_I2C);
            }
    }
    //SwapValue(read_data.refined.highByte, read_data.re
    while (!LL_I2C_IsActiveFlag_STOP(UHF_I2C)) {};
    LL_I2C_ClearFlag_STOP(UHF_I2C);
    LL_I2C_ClearFlag_NACK(UHF_I2C);
        
    

    //uint16_t rec = 0;
    /**/

    //uint8_t index = reg_add - 10;
    //SwapValue(&read_data.refined.lowByte, &read_data.refined.highByte);
    //memcpy(p_receive_buff, &read_data, 2);
    DEBUG_I2C("[I2C_FM]: Read add:0x0%04x - data 0x%04x\r\n", reg_add, read_data[index]);
    return 0;
#endif
}
//-----------------------------------------------------------------------------
//�� �� ����KT_Bus_Write                                                             
//��������������д����                                                                
//����˵����                                                                        
//ȫ�ֱ�������                                                                        
//��    �룺uint8_t Register_Address, uint16_t Word_Data                                    
//��    �أ���                                                                        
//�� �� �ߣ�Kang Hekai              ʱ�䣺2014-02-13                                         
//�� �� �ߣ�                        ʱ�䣺                                         
//��    ����V1.0                                                                     
//-----------------------------------------------------------------------------
void KT_Bus_Write(uint8_t Register_Address, uint16_t Word_Data)
{
#if(USE_SOFT_I2C)
    //I2CS_Byte_Write(KTWirelessMicTxw_address, Register_Address, Word_Data);
    I2C_Word_Write(KTWirelessMicTxw_address, Register_Address, Word_Data);
#else 
    I2C_Byte_Write(KTWirelessMicTxw_address,Register_Address,Word_Data);
#endif
}

//-----------------------------------------------------------------------------
//�� �� ����KT_Bus_Read                                                                 
//�������������߶�����                                                                
//����˵����                                                                        
//ȫ�ֱ�������                                                                        
//��    �룺uint8_t Register_Address                                                    
//��    �أ�I2C_Word_Read(KTWirelessMicTxr_address, Register_Address)                
//�� �� �ߣ�Kang Hekai              ʱ�䣺2014-02-13                                         
//�� �� �ߣ�                        ʱ�䣺                                         
//��    ����V1.0                                                                     
//-----------------------------------------------------------------------------
uint16_t KT_Bus_Read(uint8_t Register_Address)
{
   // uint8_t read_data[2];
    uint16_t ret = 0;
    
#if(USE_SOFT_I2C)
    //ret = I2CS_Byte_Read(KTWirelessMicTxr_address, Register_Address);
    ret = I2C_Word_Read(KTWirelessMicTxr_address, Register_Address);
#else
    uint16_t err = 0;
    err = I2C_Byte_Read(KTWirelessMicTxr_address, Register_Address, (uint8_t*)&ret);
    if(err)
    {
        DEBUG_ERROR("Failed to Read i2c\r\n");
    }
#endif
    return ret;
}

//-----------------------------------------------------------------------------
// Function Name: KT_WirelessMicTx_PreInit
// Description: Chip initialization program
// Function Description: Check if the chip is powered up correctly and if the I2C bus reads and writes correctly
// Global Variable: INIT_FAIL_TH
// Input: None
// Return: Success: 1   Error: 0
// Designer: Kang Hekai   Date: 2014-02-13
// Modifier:                        Date: 
// Version: V1.0
//-----------------------------------------------------------------------------
bool KT_WirelessMicTx_PreInit(void)              
{
     uint16_t regx;
    uint8_t i;
    for (i = 0; i < INIT_FAIL_TH; i++)
    {
        regx = KT_Bus_Read(0x01); //Read Manufactory ID 
        //regx = ~regx;
        Delay_ms(10);
          if (regx == 0x4B54)
            return(1);
    }
    return(0);
}

//-----------------------------------------------------------------------------
//�� �� ����KT_WirelessMicTx_Init                                                     
//����������оƬ��ʼ������                                                            
//����˵����                                                                        
//ȫ�ֱ�������                                                                        
//��    �룺��                                                                        
//��    �أ���ȷ��1                 ����0                                            
//�� �� �ߣ�Zhou Dongfeng           ʱ�䣺2016-04-26                                         
//�� �� �ߣ�Zhou Dongfeng           ʱ�䣺2016-08-26                                         
//��    ����V0.4  
//��    ����V1.2    HARD_LIMIT��14�ĳ���15��COMPANDOR_TC�ĳ���3����48ms                                                                     
//-----------------------------------------------------------------------------
bool KT_WirelessMicTx_Init(void)
{
    uint16_t regx;

    Delay_ms(50);

    regx = KT_Bus_Read(0x03);
    KT_Bus_Write(0x03, (regx & 0xfffe) | PA_SEL);

    regx = KT_Bus_Read(0x24);
    KT_Bus_Write(0x24, (regx & 0xEFFF) | (BATTERY_METER_DISABLE << 12));

    //ref_vtr_vth_sel = 1
    regx=KT_Bus_Read(0x47);                        
    KT_Bus_Write(0x47,(regx | 0x0200));

    //vref_mon_en=1
    regx = KT_Bus_Read(0x0a);
    KT_Bus_Write(0x0a, regx | 0x0200);            

    regx = KT_Bus_Read(0x1C);
    KT_Bus_Write(0x1C, (regx & 0xF1E1) | ( FDEV_MONITOR_TC_250ms << 10 ) | (COMPANDOR_ENABLE<<4) | 
                 ( PRE_EMPHASIS_ENABLE << 9 ) | ( COMPANDOR_TC_12ms << 1 ));

    KT_WirelessMicTx_Mic_Sens(MIC_SENS_GAIN_5);        

    //cic overflow detect enable
    regx = KT_Bus_Read(0x1e); 
    KT_Bus_Write(0x1e, (regx & 0xffdf)|(1<<5)); 

    regx=KT_Bus_Read(0x30);
    KT_Bus_Write(0x30, ( regx & 0x8000 ) | (AGC_VHIGH << 11) | (AGC_VLOW << 7) | (AGC_ATTACK << 4) | 
                 (AGC_RELEASE << 1) | AGC_DIS );
 
    regx=KT_Bus_Read(0x31);
    KT_Bus_Write( 0x31, ( regx & 0xff00 ) | (GAIN_SEL << 6) | (COMPEN_GAIN << 4) | 
                  (BLANK_EN << 3) | BLANK_TIME );

    regx=KT_Bus_Read(0x39);
    KT_Bus_Write(0x39,(regx & 0xFC00) | (HARD_LIMIT << 6) | (CPRS_1XLPF_BP << 5) | 
                 (CPRS_KNEE_DIS << 4) | CPRS_THRSH);

    KT_Bus_Write( 0x3a,(ALC_DIS << 15) | (ALC_SOFTKNEE << 14) | (ALC_VMAX << 7) );//| 
                  //(ALC_ATTACK << 4) | ALC_RELEASE );

	#ifdef SILENCE_MUTE
    regx=KT_Bus_Read(0x25);
    KT_Bus_Write( 0x25,(regx&0xc000)|(SLNC_MUTE_DIS << 13) | (SLNC_MUTE_TIME << 8) | 
                 (SLNC_MUTE_LOW_LEVEL << 4) | SLNC_MUTE_HIGH_LEVEL );
    regx=KT_Bus_Read(0x26);
    KT_Bus_Write( 0x26,(regx&0xfffb)|(SILENCE_MUTE_ACT_MCU << 2));
	#endif

    #ifdef AUX_CH
        regx=KT_Bus_Read(0x1f);
        KT_Bus_Write(0x1f,(regx&0x80f0)|(AUXCH_EN << 15)|(AUXDATA_EN << 14)|(AUX_REG_NUM << 12) |
                     (AUX_CARRY_NUM << 9)|BPSK_NEW_MODE);
 
        KT_Bus_Write(0x20,(AUX_ADDRB << 8) | AUX_ADDRA);
        KT_Bus_Write(0x21,(AUX_ADDRD << 8) | AUX_ADDRC);   
    #endif

    #ifdef OTHER_RX
        #ifdef XTAL_24M_ONLY
        KT_WirelessMicTx_Set_Pilot_Freq(XTAL_24M_FREQ); //set pilot frequency
        #endif
        #ifdef XTAL_24P576M_ONLY
        KT_WirelessMicTx_Set_Pilot_Freq(XTAL_24P576M_FREQ); //set pilot frequency
        #endif
    #endif

    //for VCO unlock because of temperature change
    regx=KT_Bus_Read(0x10);
    KT_Bus_Write(0x10,(regx|0x0010)); //vco_ldo_calicode=1.4v
    regx=KT_Bus_Read(0x45);
    KT_Bus_Write(0x45,((regx&0xfff0) | 0x0007));

    regx=KT_Bus_Read(0x2d);
    KT_Bus_Write(0x2d,(regx&0xfff8)|3); //lofine_vref_sel=0.4v
    
    #ifdef pll_unlock 
        regx = KT_Bus_Read(0x2f);                
        KT_Bus_Write(0x2f, regx|0x0080); //pll_unlock_en
        regx = KT_Bus_Read(0x44);                
        KT_Bus_Write(0x44, (regx&0x1fff)|(1<<13)); //lo_lock_hth_vsel
        regx = KT_Bus_Read(0x44);                
        KT_Bus_Write(0x44, (regx&0xe3ff)|(3<<10)); //lo_lock_lth_vsel
        regx = KT_Bus_Read(0x44);                
        KT_Bus_Write(0x44, regx&0xfdff); //lo_lock_det_pd
    #endif

    return(1);
}


//-----------------------------------------------------------------------------
//�� �� ����KT_WirelessMicTx_Standby                                                 
//������������������                                                                
//����˵����                                                                        
//ȫ�ֱ�����                                                                        
//��    �룺��                                                                        
//��    �أ���ȷ��1                 ����0                                            
//�� �� �ߣ�Kang Hekai              ʱ�䣺2014-02-13                                         
//�� �� �ߣ�                        ʱ�䣺                                         
//��    ����V1.0                                                                     
//-----------------------------------------------------------------------------
bool KT_WirelessMicTx_Standby(void)
{
    KT_WirelessMicTx_PowerDownProcedure();
    return(1);
}

//-----------------------------------------------------------------------------
//�� �� ����KT_WirelessMicTx_WakeUp                                                     
//�������������ѳ���                                                                
//����˵�����������ģʽ                                                        
//ȫ�ֱ�����                                                                        
//��    �룺��                                                                        
//��    �أ���ȷ��1                 ����0                                            
//�� �� �ߣ�Kang Hekai              ʱ�䣺2014-02-13                                         
//�� �� �ߣ�                        ʱ�䣺                                         
//��    ����V1.0                                                                     
//-----------------------------------------------------------------------------
bool KT_WirelessMicTx_WakeUp(void)
{
    uint16_t reg3;
    reg3 = KT_Bus_Read(0x03);
    KT_Bus_Write(0x03, (reg3 & 0x7FFF) | (WAKEUP << 15)); //Write Standby bit to 0
    Delay_ms(50);
    KT_WirelessMicTx_Init();
//  wakeUp�Ժ���Ҫ��һЩtunę��������Ч������PA�ȹ������ɲο�main.c�����KT_MicTX_Init����
    return(1);
}

//-----------------------------------------------------------------------------
//�� �� ����KT_WirelessMicTx_PASW                                                     
//����������PA�򿪡��رճ���                                                        
//����˵������������оƬ�򿪻��߹ر�PA                                                
//ȫ�ֱ�����                                                                        
//��    �룺bPA_Switch                                                                
//��    �أ���ȷ��1                 ����0                                            
//�� �� �ߣ�Zhou Dongfeng           ʱ�䣺2016-04-05                                         
//�� �� �ߣ�Zhou Dongfeng           ʱ�䣺2016-08-26                                         
//��    ����V0.4                                                                     
//-----------------------------------------------------------------------------
bool KT_WirelessMicTx_PASW(bool bPA_Switch)
{
    uint16_t regF;

    regF = KT_Bus_Read(0x0F);
    KT_Bus_Write(0x0F, (regF & 0xFFF7) | ( (uint8_t)bPA_Switch << 3 ));
    return(1);
}

//-----------------------------------------------------------------------------
//�� �� ����KT_WirelessMicTx_PAGain                                                     
//����������PA�������                                                                
//����˵����cPaGain��ΧΪ0-63����64��                                    
//ȫ�ֱ�����                                                                        
//��    �룺cPaGain                                                                    
//��    �أ���ȷ��1                 ����0                                            
//�� �� �ߣ�Zhou Dongfeng           ʱ�䣺2016-04-05                                         
//�� �� �ߣ�                        ʱ�䣺                                         
//��    ����V0.1                                                                     
//-----------------------------------------------------------------------------
bool KT_WirelessMicTx_PAGain(uint8_t cPaGain)
{
    uint16_t reg11;

    reg11 = KT_Bus_Read(0x11);
    KT_Bus_Write(0x11, (reg11 & 0xFF00) | cPaGain );        

    return(1);    
}

//-----------------------------------------------------------------------------
//�� �� ����KT_WirelessMicTx_Fdev_Monitor                                             
//����������ʵʱƵƫ��ȡ����                                                        
//����˵������ȡ��ǰ״̬��Ƶƫ��С                                                    
//ȫ�ֱ�����                                                                        
//��    �룺��                                                                        
//��    �أ�(reg1C & 0xF000) >> 12 ��Ϊ0-15��������                                    
//�� �� �ߣ�Kang Hekai              ʱ�䣺2014-02-13                                         
//�� �� �ߣ�                        ʱ�䣺                                         
//��    ����V1.0                                                                     
//-----------------------------------------------------------------------------
uint8_t KT_WirelessMicTx_Fdev_Monitor(void)
{
    uint16_t reg1C;

    reg1C = KT_Bus_Read(0x1C);

    return( (reg1C & 0xF000) >> 12 );
}

//-----------------------------------------------------------------------------
//�� �� ����KT_WirelessMicTx_Mic_Sens                                                 
//����������Mic�����ȵ���                                                            
//����˵����cMicSensΪ0-15����16��                                                
//ȫ�ֱ�����                                                                        
//��    �룺cMicSens                                                                
//��    �أ���ȷ��1                 ����0                                            
//�� �� �ߣ�Kang Hekai              ʱ�䣺2014-02-13                                         
//�� �� �ߣ�                        ʱ�䣺                                         
//��    ����V1.0                                                                     
//-----------------------------------------------------------------------------
bool KT_WirelessMicTx_Mic_Sens(uint8_t cMicSens)
{
    uint16_t reg1C;

    reg1C = KT_Bus_Read(0x1C);
    KT_Bus_Write(0x1C, (reg1C & 0xFE1F) | ( (uint16_t)cMicSens << 5 ));

    return(1);
}

//-----------------------------------------------------------------------------
//�� �� ����KT_WirelessMicTx_Comp_Dis                                                 
//����������ѹ�����ܴ򿪡��رճ���                                                    
//����˵������������оƬ�򿪻��߹ر�ѹ������                                        
//ȫ�ֱ�����                                                                        
//��    �룺bComp_Dis                                                                
//��    �أ���ȷ��1                 ����0                                            
//�� �� �ߣ�Kang Hekai              ʱ�䣺2014-02-13                                         
//�� �� �ߣ�                        ʱ�䣺                                         
//��    ����V1.0                                                                     
//-----------------------------------------------------------------------------
//bool KT_WirelessMicTx_Comp_Dis(bool bComp_Dis)                            
//{
//    uint16_t reg1C;
//
//    reg1C = KT_Bus_Read(0x1C);
//    KT_Bus_Write(0x1C, (reg1C & 0xFFEF) | ( (uint8_t)bComp_Dis << 4 ));
//
//    return(1);
//}

//-----------------------------------------------------------------------------
//�� �� ����KT_WirelessMicTx_MuteSel                                                 
//�����������������ܴ򿪡��رճ���                                                    
//����˵��������ѡ��򿪻��߹رվ�������                                            
//ȫ�ֱ�����                                                                        
//��    �룺bMute_Sel                                                                
//��    �أ���ȷ��1                 ����0                                            
//�� �� �ߣ�Kang Hekai              ʱ�䣺2014-02-13                                         
//�� �� �ߣ�                        ʱ�䣺                                         
//��    ����V1.0                                                                     
//-----------------------------------------------------------------------------
bool KT_WirelessMicTx_MuteSel(bool bMute_Sel)
{
    uint16_t reg1C;

    reg1C = KT_Bus_Read(0x1C);
    KT_Bus_Write(0x1C, (reg1C & 0xFFFE) | bMute_Sel); //Write Mute bit

    return(1);
}


bool KT_WirelessMicTx_Pilot(bool bPilot_Dis)
{
    uint16_t reg1F;

    reg1F = KT_Bus_Read(0x1F);
#ifdef OTHER_RX
    KT_Bus_Write(0x1F, (reg1F & 0xFFBF) | ( (uint8_t)bPilot_Dis << 6 ));
#endif
#ifdef KT_RX
    KT_Bus_Write(0x1F, (reg1F & 0x7FFF) | ( (uint16_t)bPilot_Dis << 15 ));
#endif
    Delay_ms(20);

    return(1);
}

bool KT_WirelessMicTx_Pilot_Fdev(uint8_t cPilot_Fdev)
{
    uint16_t reg1F;

    reg1F = KT_Bus_Read(0x1F);
#ifdef OTHER_RX
    KT_Bus_Write(0x1F, (reg1F & 0xFFCF) | (cPilot_Fdev << 4));
#endif
#ifdef KT_RX
    KT_Bus_Write(0x1F, (reg1F & 0xFE7F) | ((uint16_t)cPilot_Fdev << 7));
#endif
    Delay_ms(20);

    return(1);
}

#ifdef OTHER_RX
//-----------------------------------------------------------------------------
//�� �� ����KT_WirelessMicTx_Set_Pilot_Freq                                             
//�������������õ�ƵƵ��                                                            
//����˵�������õ�ƵƵ��                                                
//ȫ�ֱ�����                                                                        
//��    �룺xtal_sel: 0 or 1                                                        
//��    �أ��ɹ���1�� ʧ�ܣ�0                                                 
//�� �� �ߣ�Kang Hekai              ʱ�䣺2014-02-13                                         
//�� �� �ߣ�                        ʱ�䣺                                         
//��    ����V1.0                                                                     
//-----------------------------------------------------------------------------
uint8_t KT_WirelessMicTx_Set_Pilot_Freq(bool bXtal_Sel)
{
    uint16_t regx;
    if (bXtal_Sel == XTAL_24M_FREQ)
    {
        KT_Bus_Write(0x2C, ((PILOT_FREQ << 13) / 9375));
    }
    else
    {
        KT_Bus_Write(0x2C, ((PILOT_FREQ << 13) / 9600));
    }
    return(1);
}
#endif

#ifdef RXISKT0616M_XTAL_DUAL
//-----------------------------------------------------------------------------
//�� �� ����KT_WirelessMicTx_Calc_ChanReg                                    
//���������������Ƶ�ʿ����ּ���                                                
//����˵����������KHzΪ��λ��VCO��Ƶ��;                                            
//            ����������*chan_ptr,*chan_frac_ptr,*chan_frac_msb_ptr��                
//ȫ�ֱ�����                                                                        
//��    �룺Freq ��������KHzΪ��λ��VCOƵ�ʣ�                                        
//��    �أ���ȷ��1    ����0                                                            
//�� �� �ߣ�YANG Pei                ʱ�䣺2012-04-19                            
//�� �� �ߣ�KANG Hekai              ʱ�䣺2013-03-29                            
//��    ����V2.0                                                                    
//          V2.5 �޸�����<40�����xtal-40��bug                                                                    
//-----------------------------------------------------------------------------
bool KT_WirelessMicTx_Calc_ChanReg_Old(int32_t Freq, uint16_t *chan_ptr, INT16 *chan_frac_ptr, 
                                       uint8_t *chan_frac_msb_ptr, uint16_t xtal_freq)
{
    *chan_ptr = Freq / xtal_freq;
    Freq = Freq % xtal_freq; 
    *chan_frac_ptr = (Freq << 16) / xtal_freq;
    if ((Freq <= 40) && (Freq >= 0))
    {
        *chan_frac_ptr = 0xffff;
        *chan_frac_msb_ptr =3;
    }
    else if ((Freq < xtal_freq ) && (Freq >= xtal_freq - 40))
    {
        (*chan_ptr)++; 
        *chan_frac_ptr = 0xffff;
        *chan_frac_msb_ptr = 3;
    }
    else if ( (Freq >= (xtal_freq / 2 - 40)) && (Freq <= (xtal_freq / 2 + 40)) )
    {
        *chan_frac_ptr = 0x7fff;
        *chan_frac_msb_ptr = 0;
    }
    else if ( Freq > (xtal_freq >> 1) )
    {
        (*chan_ptr)++; 
        *chan_frac_msb_ptr = 3;
    }
    else    
    {
        *chan_frac_msb_ptr = 0;
    }
    return(1);
}

bit selectXtalOld(int32_t Freq)
{
    uint16_t chan0,chan1;
    INT16 chan_frac0,chan_frac1;
    uint8_t chan_frac_msb0,chan_frac_msb1;
    INT16 mod0,mod1,mod2,mod3;

    Freq<<=1;
    KT_WirelessMicTx_Calc_ChanReg_Old(Freq, &chan0, &chan_frac0, &chan_frac_msb0,24000);
    KT_WirelessMicTx_Calc_ChanReg_Old(Freq, &chan1, &chan_frac1, &chan_frac_msb1,24576);
    mod0 = chan_frac0;
    mod1 = chan_frac1;
    mod2 = chan_frac0 << 1;
    mod3 = chan_frac1 << 1;
    if(mod0 < 0)
        mod0=~mod0;             //mod0=abs(mod0);
    if(mod1 < 0)
        mod1=~mod1;             //mod1=abs(mod1);
    if(mod2 < 0)
        mod2=~mod2;             //mod2=abs(mod2);
    if(mod3 < 0)
        mod3=~mod3;             //mod3=abs(mod3);
    if(mod2 < mod0)
        mod0 = mod2;
    if(mod3 < mod1)
        mod1 = mod3;
    if(mod0<mod1)
    {
        return(XTAL_24P576M_FREQ);
    }
    else 
        return(XTAL_24M_FREQ);
}
#endif


uint16_t KT_WirelessMicTx_BatteryMeter_Read(void)
{
    uint16_t reg7;

    reg7 = KT_Bus_Read(0x07);

    return( reg7 & 0x07FF );
}


//lopa_div1 * lopa_div2
uint8_t lopa_div_tab[13]=  
{
    6, 8, 10, 12,
      16, 20, 24,
      32, 40, 48,
      64, 80, 96
};

//reg0x08<bit9:8> and <bit7:6>
uint8_t lopa_div_tab2[13][2]= 
{
     {0,0},{0,1}, {0,2}, {0,3},
           {1,1}, {1,2}, {1,3},
           {2,1}, {2,2}, {2,3},
           {3,1}, {3,2}, {3,3}
};

bool KT_WirelessMicTx_Calc_ChanReg(int32_t Freq, uint8_t *lopa_div1_ptr,uint8_t *lopa_div2_ptr,
                                   uint16_t *chan_ptr, uint16_t *chan_frac_ptr, 
                                   uint8_t *chan_frac_msb_ptr, uint32_t *chan_frac_temp_ptr, int32_t xtal_freq)
{
    uint32_t chan_frac;
    int32_t Fvco;
    uint32_t temp, tempMin = xtal_freq;
//    double chan_frac_d;
//    double chan_frac_temp_d;
    uint16_t i = 12;
    uint8_t lopa_div;
    long vco_highth = 4320000; //VCO Range:4.32G-5.76G
    
    //lodiv ����ֵ
    lopa_div = vco_highth / Freq;

    //lodiv ʵ��ֵ
    for(i=0; i<12; i++)
    {
        
        if(lopa_div < lopa_div_tab[i])
        {            
            break;                                                   
        }
    }
    //���lopa_div1��lopa_div2
    *lopa_div1_ptr = lopa_div_tab2[i][1];
    *lopa_div2_ptr = lopa_div_tab2[i][0];
    
    //����VCO Ƶ��
    Fvco = Freq * lopa_div_tab[i];
    //����
    *chan_ptr = Fvco / (xtal_freq * 7) + 1; //dll_mode : *7
    //С��
//    chan_frac_d = Fvco;
//    chan_frac_d = chan_frac_d / (xtal_freq * 7);     //31.1221
//    chan_frac_d = chan_frac_d - (*chan_ptr);
//    chan_frac_d = chan_frac_d * 65536;                 //-57532.95
//    chan_frac = chan_frac_d;
    chan_frac= (((Fvco % (xtal_freq * 7))-(xtal_freq * 7))<<12)/((xtal_freq * 7)>>4);

    *chan_frac_msb_ptr = (chan_frac & 0x00030000) >> 16 ; //reg0x08<bit1:0>
    *chan_frac_ptr = chan_frac & 0x0000ffff; //reg0x09

//    chan_temp = Freq / xtal_freq + 1;
//    chan_frac_temp_d = Freq;
//    chan_frac_temp_d = chan_frac_temp_d / xtal_freq - chan_temp;
//    *chan_frac_temp_ptr = chan_frac_temp_d;

//    *chan_frac_temp_ptr= ((xtal_freq>>1)-abs(((Freq%xtal_freq)-(xtal_freq>>1))))*(3072000/xtal_freq);
    
    for(i = 1; i < 4 ; i++)
    {
        temp = ((xtal_freq/(2*i)) - abs(((Freq%(xtal_freq/i)) - (xtal_freq/(2*i)))))*i;
        if(temp < tempMin) 
        {
            tempMin = temp;
        }
    }
    i = 8;
    temp = ((xtal_freq/(2 * i)) - abs(((Freq%(xtal_freq/i)) - (xtal_freq/(2*i)))))*i;
    if(temp < tempMin)
    {
        tempMin = temp;
    }
    *chan_frac_temp_ptr=tempMin*(3072000 / xtal_freq);        
    return(1);
}
//-----------------------------------------------------------------------------
//�� �� ����KT_WirelessMicTx_Tune                                                     
//��������������Ƶ�����ú���                                                        
//����˵����������KHzΪ��λ�ķ���Ƶ�ʣ�                                                
//ȫ�ֱ�����                                                                        
//��    �룺Freq ��������KHzΪ��λ�ķ���Ƶ�ʣ�                                        
//��    �أ���ȷ��1                 ����0                                            
//�� �� �ߣ�Zhou Dongfeng           ʱ�䣺2016-04-05                                         
//�� �� �ߣ�Zhou Dongfeng           ʱ�䣺2016-08-26                                         
//��    ����V0.1    For KT0646M_VX
//          V0.2    �޸��˱�������ķ�ʽ��������VX��VX2�ĺ궨�壬
//                  �޸��˽��DLL���������ֳ����λ��
//                  ȥ���˲��õ�chan_cfg2����
//                  ȥ����MCU_POWER_OKλ�Ĳ�ѯ����
//                  ȥ����PLL�������ж�
//          V0.4    ͳһ�˱��������ʽ
//-----------------------------------------------------------------------------
//ʹ��24MHz����ķ���Ƶ��
//static const uint32_t use24M[26] = 
//{   490500,492000,516000,541500,556000,565500,566000,590000,614000,615000,639000,651250,688000,
//    688500,712000,712250,712500,722500,736500,760500,762000,787500,810000,811500,835500,859500
//};

//ʹ��24.576MHz����ķ���Ƶ��
//static const uint32_t use24576M[14] = 
//{
//    7500,9000,10000,10500,12000,13500,14000,15000,16000,16500,18000,19500,20000,22000
//};

bool KT_WirelessMicTx_Tune(int32_t Freq)
{
    uint16_t chan0;
    uint16_t chan_frac0;
    uint8_t chan_frac_msb0;
    uint8_t lopa_div10,lopa_div20;
    static uint16_t regx,regy;
    uint32_t chan_frac_temp0;    
    uint16_t state;
#ifdef XTAL_DUAL
    uint16_t chan1;
    uint16_t chan_frac1;
    uint8_t chan_frac_msb1;
    uint8_t lopa_div11,lopa_div21;
    uint16_t use24M_flag=0,use24576M_flag=0;
    uint32_t chan_frac_temp1;
    #ifndef RXISKT0616M_XTAL_DUAL
    uint8_t i;
    uint32_t state_tmp;
    #endif
//    double mod0,mod1,mod2,mod3;
#endif

    #ifdef DOUBLE_KVCO
        regx=KT_Bus_Read(0x2d); //kvco_cali_bps=0
        KT_Bus_Write(0x2d, (regx&0xf7ff));
    
        regx=KT_Bus_Read(0x0a); //locpcali_bps=0
        
        static uint16_t test = 0;

        
        
        
        regx=KT_Bus_Read(0x3d);
        KT_Bus_Write(0x3d, (regx&0xffbf));              

        test = regx & 0xffbf;
        //KT_Bus_Write(0x0a, (regx&0xffef));
        regx = KT_Bus_Read(0x3d); //locpcali_bps=0
        if(regx != test)
        {
            regx = 1;
            DEBUG_RAW("\r\n");
        }
        else
        {
            regx = 2;
            DEBUG_RAW("\r\n");
        }


        regx=KT_Bus_Read(0x3c);
        KT_Bus_Write(0x3c, (regx&0xfffd)); //loamp_cali_bps=0
    #endif    

//    regx = KT_Bus_Read(0x2d);
//    KT_Bus_Write(0x2d, (regx&0xfff8)|0x0003); //lo_fine_vref_sel

#ifdef XTAL_24M_ONLY
    KT_WirelessMicTx_Calc_ChanReg(Freq, &lopa_div10, &lopa_div20, &chan0, &chan_frac0, 
                                  &chan_frac_msb0,&chan_frac_temp0, 24000);
#endif

#ifdef XTAL_24P576M_ONLY
    KT_WirelessMicTx_Calc_ChanReg(Freq, &lopa_div10, &lopa_div20, &chan0, &chan_frac0, 
                                  &chan_frac_msb0,&chan_frac_temp0,24576);
#endif

#ifdef XTAL_DUAL

    KT_WirelessMicTx_Calc_ChanReg(Freq, &lopa_div10, &lopa_div20, &chan0, &chan_frac0, 
                                  &chan_frac_msb0,&chan_frac_temp0,24000);
    KT_WirelessMicTx_Calc_ChanReg(Freq, &lopa_div11, &lopa_div21, &chan1, &chan_frac1, 
                                  &chan_frac_msb1,&chan_frac_temp1,24576);
    #ifndef RXISKT0616M_XTAL_DUAL
        for(i=0;i<26;i++)
        {
            if(Freq==use24M[i])    
            {
                use24M_flag=1;
                break;
            }
        }
    
        state_tmp = Freq%24000;
        for(i = 0; i < 14; i++)
        {
            if(state_tmp==use24576M[i])
            {
                use24576M_flag=1;
                break;
            }
        }
        if (use24M_flag)
        {
            KT_WirelessMicTx_SW_XTAL_Freq(XTAL_24M_FREQ);
        }
        else if(use24576M_flag)
        {
            KT_WirelessMicTx_SW_XTAL_Freq(XTAL_24P576M_FREQ);
            chan0 = chan1;
            lopa_div10 = lopa_div11;
            lopa_div20 = lopa_div21;
            chan_frac0 = chan_frac1;
            chan_frac_msb0 = chan_frac_msb1;
        }
        else 
        {    
            if(chan_frac_temp0>chan_frac_temp1)
            {
                KT_WirelessMicTx_SW_XTAL_Freq(XTAL_24M_FREQ);    
            }
            else
            {
                KT_WirelessMicTx_SW_XTAL_Freq(XTAL_24P576M_FREQ);
                chan0 = chan1;
                lopa_div10 = lopa_div11;
                lopa_div20 = lopa_div21;
                chan_frac0 = chan_frac1;
                chan_frac_msb0 = chan_frac_msb1;    
            }
        }
    #else
        if(selectXtalOld(Freq))
        {
            KT_WirelessMicTx_SW_XTAL_Freq(XTAL_24P576M_FREQ);
            chan0 = chan1;
            lopa_div10 = lopa_div11;
            lopa_div20 = lopa_div21;
            chan_frac0 = chan_frac1;
            chan_frac_msb0 = chan_frac_msb1;
        }
        else
        {
            KT_WirelessMicTx_SW_XTAL_Freq(XTAL_24M_FREQ);
        }
    #endif

#endif

    KT_Bus_Write(0x08,(chan0 << 10) | (lopa_div10 << 8) | (lopa_div20 << 6) | (0 << 5) | 
                 (0 << 4) | (0 <<2) | chan_frac_msb0); //
    KT_Bus_Write(0x09, chan_frac0|0x0001);

    //dllδ�������л��������Ҫrst
    regx = KT_Bus_Read(0x0e);
    KT_Bus_Write(0x0e, regx | 0x0080); //dll_rst is from regbank
    regx = KT_Bus_Read(0x0e);
    KT_Bus_Write(0x0e, regx | 0x0100); //dll_rst=1
    Delay_ms(1);
    regx = KT_Bus_Read(0x0e);
    KT_Bus_Write(0x0e, regx & 0xfeff); //dll_rst=0

    regx=KT_Bus_Read(0x0a);
    KT_Bus_Write(0x0a, regx | 0x0020); //tune

    regx=KT_Bus_Read(0x0d);
    regx = (regx & 0x0800) >> 11;
    uint32_t uhf_start_ms = sys_get_ms();
    while(!regx) //wait pll ready
    {
        if (sys_get_ms() - uhf_start_ms >= 300)
        {
            DEBUG_ERROR("KT_WirelessMicRx_Tune timeout\r\n");
            break;
        }
        regx=KT_Bus_Read(0x0d);
        regx = (regx & 0x0800) >> 11; 
    }

//    regx = KT_Bus_Read(0x2d);
//    KT_Bus_Write(0x2d, (regx&0xfff8)); //lo_fine_vref_sel=0

    #ifdef DOUBLE_KVCO
        regx=KT_Bus_Read(0x2e); //double+16MHz/V locoarse_var_sel
        state=regx&0x1C00;
        state=state>>10;
        if(state >= 3)
        {
            state = 7;                                
        }
        else
        {
            state = (state<<1) + 3;
        }
        regx=(regx&0xe3ff) | (state<<10);
                                           
        regy=KT_Bus_Read(0x2d); //kvco_cali_bps=1
        KT_Bus_Write(0x2d, (regy|0x0800));              
    
        KT_Bus_Write(0x2e, regx); //write locoarse/lofine_var_sel
    
        regx=KT_Bus_Read(0x2a); //write cp_code
        state=(regx&0x007e)>>1;
        regx=KT_Bus_Read(0x3d);
        KT_Bus_Write(0x3d, (regx&0xff80)|(1<<6)|state);
    
        regx=KT_Bus_Read(0x0a); //locpcali_bps=1
        KT_Bus_Write(0x0a, (regx|0x0010));              
    
        regx=KT_Bus_Read(0x3c);
        KT_Bus_Write(0x3c, (regx|0x0002)); //loamp_cali_bps=1
    
        regx=KT_Bus_Read(0x0a);
        KT_Bus_Write(0x0a,regx|0x0020); //tune
    
        regx=KT_Bus_Read(0x0d);
        regx = (regx&0x0800)>>11;
        uhf_start_ms = sys_get_ms();
        while(!regx) //wait pll ready
        {
            if (sys_get_ms() - uhf_start_ms >= 300)
            {
                DEBUG_ERROR("KT_WirelessMicRx_Tune timeout\r\n");
                break;
            }
            regx=KT_Bus_Read(0x0d);
            regx = (regx&0x0800)>>11; 
        }
    #endif

    return(1);
}

//-----------------------------------------------------------------------------
//�� �� ����KT_WirelessMicTx_Set_XTAL
//�������������þ���Ƶ��
//����˵�������þ���Ƶ��
//ȫ�ֱ�����
//��    �룺xtal_sel: 0 or 1
//��    �أ��ɹ���1�� ʧ�ܣ�0
//�� �� �ߣ�YANG Pei                ʱ�䣺2012-04-10
//�� �� �ߣ�Zhou Dongfeng           ʱ�䣺2016-08-26
//��    ����V0.4
//�� �� �ߣ�wu jinfeng	            ʱ�䣺2017-10-19
//��    ����V1.4     soft_rst�����ڼĴ���0x3e��bit15�ˣ�������0x1e��bit4.
//-----------------------------------------------------------------------------
uint8_t KT_WirelessMicTx_Set_XTAL(bool bXtal_Sel)
{
    uint16_t regx;
    
	regx = KT_Bus_Read(0x1E);
    KT_Bus_Write(0x1E,regx|0x0010); //soft_rst=1      rst dsp part
    KT_Bus_Write(0x0E, 0x0002); //au_rst_bypass=1
    regx = KT_Bus_Read(0x0E);
    KT_Bus_Write(0x0E, regx | 0x0004); //au_dig_rst=1
    
    regx = KT_Bus_Read(0x47);
    KT_Bus_Write( 0x47, (regx & 0xFFDF) | ((uint8_t)bXtal_Sel << 5) ); //bXtal_Sel=0

    Delay_ms(50);

    KT_Bus_Write(0x0E, 0x0000); //au_rst_bypass=0    au_dig_rst=0
	regx = KT_Bus_Read(0x1E);
    KT_Bus_Write(0x1E, regx&~0x0010); //soft_rst=0
#ifdef OTHER_RX
    KT_WirelessMicTx_Set_Pilot_Freq(bXtal_Sel); //set pilot frequency
#endif
    return(1);
}

//-----------------------------------------------------------------------------
//�� �� ����KT_WirelessMicTx_SW_XTAL_Freq                                                      
//�����������л�����Ƶ��                                                             
//����˵����                                                                     
//ȫ�ֱ�������                                                                     
//��    �룺xtal_sel;                                                             
//��    �أ��ɹ���1�� ʧ�ܣ�0                                                                 
//�� �� �ߣ�KANG Hekai              ʱ�䣺                                         
//�� �� �ߣ�Zhou Dongfeng           ʱ�䣺2016-08-26                                         
//��    ����V0.4                                                                     
//-----------------------------------------------------------------------------
uint8_t KT_WirelessMicTx_SW_XTAL_Freq(bool bXtal_Sel)
{
    uint16_t regx;

    regx = KT_Bus_Read(0x47);
    if (bXtal_Sel==1) //24.576MHz
    {
        //Display_Ch_Num(13,1);
        if ((regx & 0x0020)==0) //bXtal_Sel=0    24MHz
            KT_WirelessMicTx_Set_XTAL(bXtal_Sel);
    }
    else
    {
        //Display_Ch_Num(13,0);
        if ((regx & 0x0020)!=0) //bXtal_Sel=1     24.576MHz
            KT_WirelessMicTx_Set_XTAL(bXtal_Sel);
    }
    return(1);
}

//-----------------------------------------------------------------------------
//�� �� ����KT_WirelessMicTx_PowerDownProcedure                                                      
//�����������ػ��������                                                             
//����˵����                                                                     
//ȫ�ֱ�������                                                                     
//��    �룺��;                                                             
//��    �أ���                                                                     
//�� �� �ߣ�Kang Hekai              ʱ�䣺2014-02-13                                         
//�� �� �ߣ�Zhou Dongfeng           ʱ�䣺2016-08-26                                         
//��    ����V0.4                                                                     
//-----------------------------------------------------------------------------
void KT_WirelessMicTx_PowerDownProcedure(void)
{
    uint16_t regx;

    KT_WirelessMicTx_MuteSel(AUDIO_MUTE);
//    Delay_ms(50);
    KT_WirelessMicTx_Pilot(PILOT_DISABLE);
//    Delay_ms(500);

//    KT_WirelessMicTx_PAGain(0);
    KT_WirelessMicTx_PASW(PA_OFF);
//    Delay_ms(5);

    regx=KT_Bus_Read(0x0a);
    KT_Bus_Write(0x0a,regx|0x1000); //recali

    regx = KT_Bus_Read(0x03);
    KT_Bus_Write(0x03, regx | (STANDBY << 15)); //Write Standby bit to 1
//    Delay_ms(20);
}

//-----------------------------------------------------------------------------
//�� �� ����KT_WirelessMicTx_ECHO                                        
//����������ECHO�������ó���                                                
//����˵��������ECHO����Ч��                
//ȫ�ֱ�����                                                                        
//��    �룺bEcho_Dis    ��ECHO���أ�
//          Echo_Ratio     ��ECHO������                                        
//          Echo_Delay     ��ECHO��ʱ��
//��    �أ���                                                            
//�� �� �ߣ�Zhou Dongfeng           ʱ�䣺2016-04-26                                         
//�� �� �ߣ�                        ʱ�䣺                                         
//��    ����V0.1    For KT0646M_VX 
//��    ����V0.2    echo�رյ�ʱ�򣬲�������echo���ǰ�Echo_Ratioд��0                                                                  
//-----------------------------------------------------------------------------
void KT_WirelessMicTx_ECHO(bool bEcho_Dis,uint8_t Echo_Ratio,uint8_t Echo_Delay)
{
    uint16_t regx;
	if(bEcho_Dis==1)//echo disable ��ʱ�򣬲�������echo���ǰ�Echo_Ratioд��0����redmine#11824
	{
		KT_Bus_Write( 0x32, (ECHO_MCU << 15) | (0 << 14) | (ECHO_STRU << 13) | 
                 (0 << 8) | (Echo_Delay << 3) | ECHO_GAINUP );
	}
	else
	{
		KT_Bus_Write( 0x32, (ECHO_MCU << 15) | ((uint8_t)bEcho_Dis << 14) | (ECHO_STRU << 13) | 
                 (Echo_Ratio << 8) | (Echo_Delay << 3) | ECHO_GAINUP );
	}   
    regx = KT_Bus_Read(0x33);
    KT_Bus_Write( 0x33, (regx & 0xFFFC) | ECHO_GAINDOWN );                          
}

//-----------------------------------------------------------------------------
//�� �� ����KT_WirelessMicTx_EQSW                                        
//����������EQ���������س���                                                
//����˵��������EQ�����������ر�                
//ȫ�ֱ�����                                                                        
//��    �룺bEq_Dis    ��EQ���أ�
//��    �أ���                                                            
//�� �� �ߣ�Zhou Dongfeng           ʱ�䣺2016-08-26                                         
//�� �� �ߣ�                        ʱ�䣺
//��    ����V0.4
//-----------------------------------------------------------------------------
void KT_WirelessMicTx_EQSW(bool bEq_Dis)
{
    uint16_t regx;

    regx = KT_Bus_Read(0x34);
    KT_Bus_Write(0x34, ((regx & 0x7FFF) | (uint16_t)bEq_Dis << 15));
}

//-----------------------------------------------------------------------------
//�� �� ����KT_WirelessMicTx_EQGAIN                                        
//����������EQ���������ó���                                                
//����˵��������EQ��������Ƶ������Ч��                
//ȫ�ֱ�����                                                                        
//��    �룺Eq_Freq ��EQƵ�ʣ�                                        
//          Eq_Gain ��EQ���棩
//��    �أ���                                                            
//�� �� �ߣ�Zhou Dongfeng           ʱ�䣺2016-04-26                                         
//�� �� �ߣ�Zhou Dongfeng           ʱ�䣺2016-08-26
//��    ����V0.4
//-----------------------------------------------------------------------------
void KT_WirelessMicTx_EQGAIN(uint8_t Eq_Freq,uint8_t Eq_Gain)
{
    uint16_t regx;
    uint8_t temp1,temp2;
    temp1=Eq_Freq/3+0x34;
    temp2=(2-(Eq_Freq%3))*5;
    regx = KT_Bus_Read(temp1);
    
    KT_Bus_Write(temp1, ((regx & ~(0x001f<<temp2)) | ( (uint16_t)Eq_Gain << temp2 )));
/*    switch(Eq_Freq)
    {
        case EQ_25H:
        {
            regx = KT_Bus_Read(0x34);
            KT_Bus_Write(0x34, ((regx & 0x83FF) | ( (uint16_t)Eq_Gain << 10 )));
        }break;
        case EQ_40H:
        {
            regx = KT_Bus_Read(0x34);
            KT_Bus_Write(0x34, ((regx & 0xFC1F) | ( (uint16_t)Eq_Gain << 5 )));
        }break;
        case EQ_63H:
        {
            regx = KT_Bus_Read(0x34);
            KT_Bus_Write(0x34, ((regx & 0xFFE0) | ( (uint16_t)Eq_Gain << 0 )));
        }break;

        case EQ_100H:
        {
            regx = KT_Bus_Read(0x35);
            KT_Bus_Write(0x35, ((regx & 0x83FF) | ( (uint16_t)Eq_Gain << 10 )));
        }break;
        case EQ_160H:
        {
            regx = KT_Bus_Read(0x35);
            KT_Bus_Write(0x35, ((regx & 0xFC1F) | ( (uint16_t)Eq_Gain << 5 )));
        }break;
        case EQ_250H:
        {
            regx = KT_Bus_Read(0x35);
            KT_Bus_Write(0x35, ((regx & 0xFFE0) | ( (uint16_t)Eq_Gain << 0 )));
        }break;

        case EQ_400H:
        {
            regx = KT_Bus_Read(0x36);
            KT_Bus_Write(0x36, ((regx & 0x83FF) | ( (uint16_t)Eq_Gain << 10 )));
        }break;
        case EQ_630H:
        {
            regx = KT_Bus_Read(0x36);
            KT_Bus_Write(0x36, ((regx & 0xFC1F) | ( (uint16_t)Eq_Gain << 5 )));
        }break;
        case EQ_1KH:
        {
            regx = KT_Bus_Read(0x36);
            KT_Bus_Write(0x36, ((regx & 0xFFE0) | ( (uint16_t)Eq_Gain << 0 )));
        }break;

        case EQ_1K6:
        {
            regx = KT_Bus_Read(0x37);
            KT_Bus_Write(0x37, ((regx & 0x83FF) | ( (uint16_t)Eq_Gain << 10 )));
        }break;
        case EQ_2K5:
        {
            regx = KT_Bus_Read(0x37);
            KT_Bus_Write(0x37, ((regx & 0xFC1F) | ( (uint16_t)Eq_Gain << 5 )));
        }break;
        case EQ_4KH:
        {
            regx = KT_Bus_Read(0x37);
            KT_Bus_Write(0x37, ((regx & 0xFFE0) | ( (uint16_t)Eq_Gain << 0 )));
        }break;

        case EQ_6K3:
        {
            regx = KT_Bus_Read(0x38);
            KT_Bus_Write(0x38, ((regx & 0x83FF) | ( (uint16_t)Eq_Gain << 10 )));
        }break;
        case EQ_10K:
        {
            regx = KT_Bus_Read(0x38);
            KT_Bus_Write(0x38, ((regx & 0xFC1F) | ( (uint16_t)Eq_Gain << 5 )));
        }break;
        case EQ_16K:
        {
            regx = KT_Bus_Read(0x38);
            KT_Bus_Write(0x38, ((regx & 0xFFE0) | ( (uint16_t)Eq_Gain << 0 )));
        }break;
        default    :    break; 
    }     */
}

