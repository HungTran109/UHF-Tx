#ifndef __APP_UHF__
#define __APP_UHF__

//-----------------------------------------------------------------------------
// Includes
//-----------------------------------------------------------------------------
#include "main.h"
#include <stdint.h>
#include <stdbool.h>
//-----------------------------------------------------------------------------
// ���ܼ���������
//-----------------------------------------------------------------------------
#define I2C //I2C���ܶ���

#ifdef I2C
    #define I2C_WORD_MODE
#endif

//#define IR        //�����Ƶ
//#define SOUTAI        //�Զ���̨��Ƶ

#ifdef IR
    #define    MIC      MIC_B
    #define    MIC_A    0xA0
    #define    MIC_B    0xB0
#endif

#ifdef SOUTAI
    //�Զ���̨��Ƶ����Ƶ�ʿ��޸ģ����Ƿ��Ȳ���̫�󣬲�����Ҫ��RXһ��
    #define MIC_Frequency     645350    
#endif

//-----------------------------------------------------------------------------
//�����������Ͷ���
//-----------------------------------------------------------------------------
//typedef unsigned char   uint8_t;
//typedef unsigned int    uint16_t;
//typedef unsigned long   uint32_t;
//typedef char            INT8;
//typedef int             INT16;
//typedef long            INT32;
//typedef bool             bool;

//-----------------------------------------------------------------------------
// Global VARIABLES
//-----------------------------------------------------------------------------
static uint32_t  Memery_Frequency; //����Ƶ����Ϣ 
static uint32_t  Load_Frequency;   //flash���汣���Ƶ����Ϣ
//static uint8_t VOLUME = 6;
static uint8_t Key_UP_flag = 0;    //���������������ϼӱ�־״̬
static uint8_t Key_DOWN_flag = 0;  //���������������¼���־״̬
static uint8_t Key_RF_POW_flag = 1; //�����л��Ƿ�ȥ����־
//static bool temp_RF_POW_SW=0;

//-----------------------------------------------------------------------------
//����VOLUME ����
//-----------------------------------------------------------------------------
#define    key_Vol     0
#define    key_Freq    1

#define    Dis_None    0
#define    Dis_Freq    1
#define    Dis_Vol     2

#define    VOLUME_TOP       15 //volume=15
#define    VOLUME_BOTTOM    0  //volume=0

#define    VBAT_FULL       0x190 //�������׼ֵ������Ч�������е���
#define    VBAT_LOW        0x177 //�͵�ѹ����ֵ������Ч�������е���

//#define    LED_ON        0
//#define    LED_OFF       1

//-----------------------------------------------------------------------------
//����LCD & KEY����
//-----------------------------------------------------------------------------
#define RD_MODE     0xC0    // 110 Binary
#define WR_MODE     0xA0    // 101
#define RMW_MODE    0xA0    // 101
#define CMD_MODE    0x80    // 100

#define LCD_COM        4
#define LCD_SEG        32

#define SYS_EN        0x01    // Enable System Clock
#define SYS_DIS       0x00

#define LCD_ON        0x03    // Turn on LCD
#define LCD_OFF       0x02    // Turn off LCD

#define CRYSTAL_32K    0x14
#define INT_256K    0x18
#define EXT_256K    0x1C

#define LCD_BIAS    0x29    // 1/3 bias, 4coms used
//#define LCD_BIAS    0x28    // 1/2 bias, 4coms used

#define FREQ_ZERO_A     0x09
#define FREQ_ZERO_B     0x09
#define FREQ_ZERO_C     0x00
#define FREQ_ZERO_D     0x03/*0*/

#define FREQ_ONE_A      0x00
#define FREQ_ONE_B      0x00
#define FREQ_ONE_C      0x00
#define FREQ_ONE_D      0x03/*1*/

#define FREQ_TWO_A      0x0A
#define FREQ_TWO_B      0x09
#define FREQ_TWO_C      0x02
#define FREQ_TWO_D      0x01/*2*/

#define FREQ_THREE_A    0x00
#define FREQ_THREE_B    0x09/*3*/
#define FREQ_THREE_C    0x02
#define FREQ_THREE_D    0x03/*3*/

#define FREQ_FOUR_A     0x03
#define FREQ_FOUR_B     0x00/*4*/
#define FREQ_FOUR_C     0x02
#define FREQ_FOUR_D     0x03/*4*/

#define FREQ_FIVE_A     0x03
#define FREQ_FIVE_B     0x09/*5*/
#define FREQ_FIVE_C     0x02
#define FREQ_FIVE_D     0x02/*5*/

#define FREQ_SIX_A      0x0B
#define FREQ_SIX_B      0x09/*6*/
#define FREQ_SIX_C      0x02
#define FREQ_SIX_D      0x02/*6*/

#define FREQ_SEVEN_A    0x00
#define FREQ_SEVEN_B    0x01/*7*/
#define FREQ_SEVEN_C    0x00
#define FREQ_SEVEN_D    0x03/*7*/

#define FREQ_EIGHT_A    0x0B
#define FREQ_EIGHT_B    0x09/*8*/
#define FREQ_EIGHT_C    0x02
#define FREQ_EIGHT_D    0x03/*8*/

#define FREQ_NINE_A     0x03
#define FREQ_NINE_B     0x09/*9*/
#define FREQ_NINE_C     0x02
#define FREQ_NINE_D     0x03/*9*/

#define CH_ZERO_A       0x0F       
#define CH_ZERO_B       0x05/*0*/
                               
#define CH_ONE_A        0x06       
#define CH_ONE_B        0x00/*1*/  
                               
#define CH_TWO_A        0x0B       
#define CH_TWO_B        0x06/*2*/  
                               
#define CH_THREE_A      0x0F     
#define CH_THREE_B      0x02/*3*/
                               
#define CH_FOUR_A       0x06     
#define CH_FOUR_B       0x03/*4*/
                               
#define CH_FIVE_A       0x0D     
#define CH_FIVE_B       0x03/*5*/
                               
#define CH_SIX_A        0x0D       
#define CH_SIX_B        0x07/*6*/  
                               
#define CH_SEVEN_A      0x07     
#define CH_SEVEN_B      0x00/*7*/
                               
#define CH_EIGHT_A      0x0F     
#define CH_EIGHT_B      0x07/*8*/
                               
#define CH_NINE_A       0x0F     
#define CH_NINE_B       0x03/*9*/



////-----------------------------------------------------------------------------
////I2C���Ŷ���
////-----------------------------------------------------------------------------
//sbit SDA        =    P3^0;                                            
//sbit SCL        =    P3^1;

////-----------------------------------------------------------------------------
////KEY���Ŷ���
////-----------------------------------------------------------------------------
//sbit RF_POW_SW  =   P4^0; // ��Ƶ��С����
//sbit IR_SEND    =   P1^3;
//sbit powerOn    =   P3^5;
//sbit porN       =   P3^4;

////-----------------------------------------------------------------------------
////LCD���Ŷ���
////-----------------------------------------------------------------------------
//sbit LCD_LED    =    P1^1; // LCD ����
//sbit LCD_WR     =    P1^0; // LCD ����
//sbit LCD_DATA   =    P3^7; // LCD ����
//sbit LCD_CS     =    P4^1; // LCD Ƭѡ

//sbit Key_UP     =    P4^2; // Ƶ�����Ͽ���
//sbit Key_DOWN   =    P3^2; // Ƶ�����¿���
//sbit Key_SET    =    P3^3; // Ƶ���趨

//-----------------------------------------------------------------------------
// SYS Function PROTOTYPES
//-----------------------------------------------------------------------------

void KT_MicTX_RFSwitch (void);
void KT_MicTX_Next_Fre (void); 
void KT_MicTX_Previous_Fre (void); 
void KT_MicTX_Mute (bool mute); 
void KT_MicTX_Init(uint32_t* freq);

void KT_Enter_STANDBY_Mode (void);
void KT_Exit_STANDBY_Mode (void);

int app_uhf_transmit(uint32_t* freq);
void set_echo_level (uint32_t level, uint32_t delay);
void set_active_freq (uint32_t freq);
#endif
