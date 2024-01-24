
#ifndef __KT0646M_HEADER_
#define __KT0646M_HEADER_
//-----------------------------------------------------------------------------
// Includes
//-----------------------------------------------------------------------------
#include "main.h"
#include <stdbool.h>


//-----------------------------------------------------------------------------
//���ܼ���������
//-----------------------------------------------------------------------------
#define KT0646M

#define DOUBLE_KVCO
//#define pll_unlock
#define AUX_CH
#define SILENCE_MUTE

//#define XTAL_DUAL
#define XTAL_24M_ONLY
//#define XTAL_24P576M_ONLY

//#define RXISKT0616M

#ifdef     RXISKT0616M
    #define RXISKT0616M_BPSK
    #ifdef XTAL_DUAL
        #define RXISKT0616M_XTAL_DUAL
    #endif
#endif

#define KT_RX //ʹ��KTоƬ�Ľ��ջ�
//#define OTHER_RX //ʹ�����������Ľ��ջ�

#define KTWirelessMicTxw_address 0x6A
#define KTWirelessMicTxr_address 0x6A


#define XTAL_24M_FREQ 0
#define XTAL_24P576M_FREQ 1



#define INIT_FAIL_TH 3

#ifdef OTHER_RX
    #define PILOT_FREQ     32768
#endif

#define    BAND_TOP        780000
#define    BAND_BOTTOM     730000
#define    BAND_STEP       250

#define PA_SEL 1

#define PA_OFF 0
#define PA_ON  1

#define    AUDIO_UNMUTE    0
#define    AUDIO_MUTE      1

#define    WAKEUP    0
#define    STANDBY   1

#define    FDEV_MONITOR_TC_250ms    0
#define    FDEV_MONITOR_TC_500ms    1
#define    FDEV_MONITOR_TC_1s       2
#define    FDEV_MONITOR_TC_2s       3

#define    PRE_EMPHASIS_ENABLE      0
#define    PRE_EMPHASIS_DISABLE     1

#define    MIC_SENS_GAIN_0     0
#define    MIC_SENS_GAIN_1     1
#define    MIC_SENS_GAIN_2     2
#define    MIC_SENS_GAIN_3     3
#define    MIC_SENS_GAIN_4     4
#define    MIC_SENS_GAIN_5     5
#define    MIC_SENS_GAIN_6     6
#define    MIC_SENS_GAIN_7     7
#define    MIC_SENS_GAIN_8     8
#define    MIC_SENS_GAIN_9     9
#define    MIC_SENS_GAIN_10    10
#define    MIC_SENS_GAIN_11    11
#define    MIC_SENS_GAIN_12    12
#define    MIC_SENS_GAIN_13    13
#define    MIC_SENS_GAIN_14    14
#define    MIC_SENS_GAIN_15    15

#define    COMPANDOR_ENABLE    0
#define    COMPANDOR_DISABLE   1

#define    COMPANDOR_TC_6ms    0
#define    COMPANDOR_TC_12ms   1
#define    COMPANDOR_TC_24ms   2
#define    COMPANDOR_TC_48ms   3
#define    COMPANDOR_TC_93ms   4
#define    COMPANDOR_TC_199ms  5
#define    COMPANDOR_TC_398ms  6
#define    COMPANDOR_TC_796ms  7

#ifdef SILENCE_MUTE
	#define SLNC_MUTE_DIS          0
	#define SLNC_MUTE_TIME         1
	#define SLNC_MUTE_LOW_LEVEL    9
	#define SLNC_MUTE_HIGH_LEVEL   9
	
	#define SILENCE_MUTE_ACT_MCU   0
#endif

#ifdef KT_RX //ʹ��KTоƬ�Ľ��ջ�
    #define    PILOT_ENABLE    1
    #define    PILOT_DISABLE   0
#endif

#ifdef OTHER_RX //ʹ�����������Ľ��ջ�
    #define    PILOT_ENABLE    0
    #define    PILOT_DISABLE   1
#endif
#define    PILOT_FDEV_2P5K     0
#define    PILOT_FDEV_5K       1
#define    PILOT_FDEV_7P5K     2
#define    PILOT_FDEV_10K      3

#define    BATTERY_METER_DISABLE    0
#define    BATTERY_METER_ENABLE     1
//ZDF
#define    AGC_VHIGH                3        //AGC�����ޣ�0-15��ѡ
#define    AGC_VLOW                 4        //AGC�����ޣ�0-15��ѡ
#define    AGC_ATTACK               3        //AGC ATTACKʱ�䣺0-7��ѡ
#define    AGC_RELEASE              4        //AGC RELEASEʱ�䣺0-7��ѡ
#define    AGC_DIS                  1//0        //AGC���ƣ�0-�Զ����ƣ�1-MCU����

#define    GAIN_SEL                 0        //PGA���棺0:��-6dB��,1:0dB,2:6dB,3:12dB
#define    COMPEN_GAIN              0//3     //�������棺0-0dB,1-6dB,2-12dB,3-18dB
#define    BLANK_EN                 1        //BLANK���ƣ�0-DIS��1-EN
#define    BLANK_TIME               3        //BLANKʱ�䣺0-7��ѡ

#define    ECHO_MCU                 1        //ECHO���ƣ�0-��ť���ƣ�1-MCU����
#define    ECHO_ENABLE              0
#define    ECHO_DISABLE             1
#define    ECHO_STRU                1        //������С��0-������1-����С
#define    ECHO_GAINUP              7        //����źŽ������ţ�7-13.1dB
#define    ECHO_GAINDOWN            0        //�����źŽ������ţ�0-��-13dB��

//����������0-�޷�����25-����25/32
#define    ECHO_RATIO_0            0        
#define    ECHO_RATIO_1            1
#define    ECHO_RATIO_2            2
#define    ECHO_RATIO_3            3
#define    ECHO_RATIO_4            4
#define    ECHO_RATIO_5            5
#define    ECHO_RATIO_6            6
#define    ECHO_RATIO_7            7
#define    ECHO_RATIO_8            8
#define    ECHO_RATIO_9            9
#define    ECHO_RATIO_10           10
#define    ECHO_RATIO_11           11
#define    ECHO_RATIO_12           12
#define    ECHO_RATIO_13           13
#define    ECHO_RATIO_14           14
#define    ECHO_RATIO_15           15
#define    ECHO_RATIO_16           16
#define    ECHO_RATIO_17           17
#define    ECHO_RATIO_18           18
#define    ECHO_RATIO_19           19
#define    ECHO_RATIO_20           20
#define    ECHO_RATIO_21           21
#define    ECHO_RATIO_22           22
#define    ECHO_RATIO_23           23
#define    ECHO_RATIO_24           24
#define    ECHO_RATIO_25           25

//�ź���ʱ��0-22ms��24-207ms
#define    ECHO_DELAY_22ms         0        
#define    ECHO_DELAY_24ms         1
#define    ECHO_DELAY_27ms         2
#define    ECHO_DELAY_29ms         3
#define    ECHO_DELAY_32ms         4
#define    ECHO_DELAY_35ms         5
#define    ECHO_DELAY_39ms         6
#define    ECHO_DELAY_43ms         7
#define    ECHO_DELAY_47ms         8
#define    ECHO_DELAY_52ms         9
#define    ECHO_DELAY_57ms         10
#define    ECHO_DELAY_63ms         11
#define    ECHO_DELAY_69ms         12
#define    ECHO_DELAY_76ms         13
#define    ECHO_DELAY_84ms         14
#define    ECHO_DELAY_92ms         15
#define    ECHO_DELAY_101ms        16
#define    ECHO_DELAY_111ms        17
#define    ECHO_DELAY_122ms        18
#define    ECHO_DELAY_135ms        19
#define    ECHO_DELAY_148ms        20
#define    ECHO_DELAY_163ms        21
#define    ECHO_DELAY_179ms        22
#define    ECHO_DELAY_197ms        23


#define    EQ_ENABLE               1
#define    EQ_DISABLE              0

//������Ƶ��
#define    EQ_25H      0            
#define    EQ_40H      1
#define    EQ_63H      2

#define    EQ_100H     3
#define    EQ_160H     4
#define    EQ_250H     5

#define    EQ_400H     6
#define    EQ_630H     7
#define    EQ_1KH      8

#define    EQ_1K6      9
#define    EQ_2K5      10
#define    EQ_4KH      11

#define    EQ_6K3      12
#define    EQ_10K      13
#define    EQ_16K      14

//����������
#define    EQ_GAIN_Neg12dB           0            
#define    EQ_GAIN_Neg11dB           1
#define    EQ_GAIN_Neg10dB           2
#define    EQ_GAIN_Neg9dB            3
#define    EQ_GAIN_Neg8dB            4
#define    EQ_GAIN_Neg7dB            5
#define    EQ_GAIN_Neg6dB            6
#define    EQ_GAIN_Neg5dB            7
#define    EQ_GAIN_Neg4dB            8
#define    EQ_GAIN_Neg3dB            9
#define    EQ_GAIN_Neg2dB            10
#define    EQ_GAIN_Neg1dB            11
#define    EQ_GAIN_0dB               12
#define    EQ_GAIN_Pos1dB            13
#define    EQ_GAIN_Pos2dB            14
#define    EQ_GAIN_Pos3dB            15
#define    EQ_GAIN_Pos4dB            16
#define    EQ_GAIN_Pos5dB            17
#define    EQ_GAIN_Pos6dB            18
#define    EQ_GAIN_Pos7dB            19
#define    EQ_GAIN_Pos8dB            20
#define    EQ_GAIN_Pos9dB            21
#define    EQ_GAIN_Pos10dB           22
#define    EQ_GAIN_Pos11dB           23
#define    EQ_GAIN_Pos12dB           24

#define    HARD_LIMIT               15       //���Ƶƫ���ƣ�0-15��ѡ
#define    CPRS_1XLPF_BP            1        //ѹ���˲������ƣ�0-������1-�ر�
#define    CPRS_KNEE_DIS            0        //�������ƣ�0-������1-�ر�
#define    CPRS_THRSH               8        //�������ޣ�0-15��ѡ 18uV-14mV

#define    ALC_DIS                 0//0      //ALC���ƣ�0-������1-�ر�
#define    ALC_SOFTKNEE            0         //ALC�յ����ͣ�0-Ӳ�յ㣻1-��յ�
#define    ALC_VMAX                1//4     //ALC���ޣ�0-127��ѡ
#define    ALC_ATTACK              3         //ALC ATTACKʱ�䣺0-7��ѡ
#define    ALC_RELEASE             6         //ALC RELEASEʱ�䣺0-11��ѡ

//��ص�ѹ���
#define    BATTERY_MAX             0x7FF
#define    BATTERY_HIGHTH          0x500
#define    BATTERY_MIDDLETH        0x4C0
#define    BATTERY_LOWTH           0x4A0

#define    LOWVOLTAGE_TH           1400
//#define    LOWVOLTAGE_TH        1800

#ifdef RXISKT0616M_BPSK
	#define BPSK_NEW_MODE            0        //1:new mode  0:old mode
#else
	#define BPSK_NEW_MODE            1        //1:new mode  0:old mode
#endif

#define AUXCH_EN                1
#define AUXDATA_EN               1        //BIT 14
#define AUX_REG_NUM              4        //BIT 13:12
#define AUX_CARRY_NUM            3        //BIT 11:9    00:12bit    01:16bit    10:18bit    11:20bit

#define AUX_ADDRB                0x12    //BIT 15:8
#define AUX_ADDRA                0x07    //BIT 7:0

#define AUX_ADDRD                0x13    //BIT 15:8
#define AUX_ADDRC                0x27    //BIT 7:0


//-----------------------------------------------------------------------------
// Function PROTOTYPES
//-----------------------------------------------------------------------------
bool KT_WirelessMicTx_PreInit(void);
bool KT_WirelessMicTx_Init(void);

bool KT_WirelessMicTx_Standby(void);
bool KT_WirelessMicTx_WakeUp(void);
void KT_WirelessMicTx_PowerDownProcedure(void);

bool KT_WirelessMicTx_PASW(bool bPA_Switch);
bool KT_WirelessMicTx_PAGain(uint8_t cPaGain);

uint8_t KT_WirelessMicTx_Fdev_Monitor(void);
bool KT_WirelessMicTx_Mic_Sens(uint8_t cMicSens);
bool KT_WirelessMicTx_Comp_Dis(bool bComp_Dis);
bool KT_WirelessMicTx_Comp_TC(uint8_t cComp_TC);
bool KT_WirelessMicTx_MuteSel(bool bMute_Sel);

bool KT_WirelessMicTx_Pilot(bool bPilot_Dis);
bool KT_WirelessMicTx_Pilot_Fdev(uint8_t cPilot_Fdev);

#ifdef OTHER_RX
uint8_t KT_WirelessMicTx_Set_Pilot_Freq(bool bXtal_Sel);
#endif

bool KT_WirelessMicTx_Tune(int32_t Freq); //in KHz

uint8_t KT_WirelessMicTx_Band_Cali_Res(void);
uint8_t KT_WirelessMicTx_Set_XTAL(bool bXtal_Sel);
uint8_t KT_WirelessMicTx_SW_XTAL_Freq(bool bXtal_Sel);

void KT_Bus_Write(uint8_t Register_Address, uint16_t Word_Data);
uint16_t KT_Bus_Read(uint8_t Register_Address);

void KT_WirelessMicTx_ECHO(bool bEcho_Dis,uint8_t Echo_Ratio,uint8_t Echo_Delay);
void KT_WirelessMicTx_EQSW(bool bEq_Dis);
void KT_WirelessMicTx_EQGAIN(uint8_t Eq_Freq,uint8_t Eq_Gain);
uint16_t KT_WirelessMicTx_BatteryMeter_Read(void);
bool KT_WirelessMicTx_BatteryMeter_SW(bool bBatteryMeter_En);
bool KT_WirelessMicTx_Mic_Sens(uint8_t cMicSens);

#endif
