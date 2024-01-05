/******************************************************************************
 * @file    	InternalFlash.c
 * @author  	Phinht
 * @version 	V1.0.0
 * @date    	05/09/2015
 * @brief   	TueTD Cross STM32L
 ******************************************************************************/
 /******************************************************************************
                                   INCLUDES					    			 
 ******************************************************************************/
#include <stdio.h>
#include "main.h"
#include "app_debug.h"
#include "sys_ctx.h"
#include "internal_flash.h"
#include "iwdg.h"
/******************************************************************************
                                   GLOBAL VARIABLES					    			 
 ******************************************************************************/
 
/******************************************************************************
                                   GLOBAL FUNCTIONS					    			 
 ******************************************************************************/

/******************************************************************************
                                   DATA TYPE DEFINE					    			 
 ******************************************************************************/

/******************************************************************************
                                   PRIVATE VARIABLES					    			 
 ******************************************************************************/

 /******************************************************************************
                                   LOCAL FUNCTIONS					    			 
 ******************************************************************************/
 
/*****************************************************************************/
/**
 * @brief	:  Khoi tao Internal Flash
 * @param	:  
 * @retval	:
 * @author	:	Phinht
 * @created	:	15/09/2015
 * @version	:
 * @reviewer:	
 */
static void InternalFlash_Init(void)
{	
    HAL_FLASH_Unlock();
    /* Clear all FLASH flags */
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_SIZERR |
                           FLASH_FLAG_OPTVERR | FLASH_FLAG_RDERR | FLASH_FLAG_FWWERR |
                           FLASH_FLAG_NOTZEROERR);
    /* Unlock the Program memory */
    HAL_FLASH_Lock();
}

/*****************************************************************************/
/**
 * @brief	: Erase sector
 * @param	:  Dia chi sector can xoa
 * @retval	:	0 if Success, > 0 if Fail
 * @author	:	Phinht
 * @created	:	15/09/2015
 * @version	:
 * @reviewer:	
 */
uint8_t InternalFlash_Prepare(uint32_t Address)
{			
    return 0;
}

/*****************************************************************************/
/**
 * @brief	: Write config
 * @param	:  
 * @retval	:	0 if Success, > 0 if Fail
 * @author	:	Phinht
 * @created	:	15/09/2015
 * @version	:
 * @reviewer:	
 */

#define SAVE_TO_FLASh 1
uint8_t InternalFlash_WriteConfig(void)
{
    uint8_t	 result = 0, retry = 5;
    InternalFlash_Init();
    FLASH_Unlock();
	while(retry)
	{
        HAL_IWDG_Refresh(&hiwdg);
        if ((FLASH_ErasePage(CONFIG_ADDR) != FLASH_COMPLETE) ||(FLASH_ErasePage(CONFIG_FREQ_PAIR) != FLASH_COMPLETE))
        {
            DEBUG_ERROR("CFG: Xoa bo nho cau hinh FAILED!\r\n");
            HAL_Delay(200);
            retry--;
        }
        else
        {
            DEBUG_INFO("CFG: Xoa EEPROM OK!\r\n");
            break;
        }
        
	}
	if(retry > 0)
	{
		
		/* Flag */
		if(FLASH_ProgramWord(CONFIG_ADDR, CONFIG_FLAG_VALUE) != FLASH_COMPLETE)
			result++;
		/*Pair Frequency*/
        if(FLASH_ProgramWord(CONFIG_FREQ_PAIR, (uint32_t)sys_ctx()->uhf_chip_status.Frequency) != FLASH_COMPLETE)
            result++;

		
		//f(result == 0)
			DEBUG_ERROR("CFG: Luu cau hinh : %u - %s\r\n", result, result == 0 ? "OK" : "FAIL");
	}
    else
    {
        DEBUG_ERROR("CFG: Luu cau hinh : %u - %s\r\n", result, result == 0 ? "OK" : "FAIL");
    }
	
	FLASH_Lock();
    return HAL_OK;
}

/*****************************************************************************/
/**
 * @brief	: Read config
 * @param	:  
 * @retval	:	0 if Success, > 0 if Fail
 * @author	:	Phinht
 * @created	:	15/09/2015
 * @version	:
 * @reviewer:	
 */
void InternalFlash_ReadConfig(void)
{
    sys_ctx_init();
	if(*(__IO uint32_t*)(CONFIG_ADDR) != CONFIG_FLAG_VALUE)
	{
        DEBUG_INFO ("fail FREQ FROM FLASH\r\n");
	}
	else 
	{
        
        sys_ctx()->uhf_chip_status.Frequency = *(__IO uint32_t*)CONFIG_FREQ_PAIR;
        DEBUG_INFO ("read freq %d\r\n", sys_ctx()->uhf_chip_status.Frequency);
//         sys_ctx()->uhf_chip_status.Frequency = 772000;
//         sys_ctx()->uhf_chip_status.CurrentChannelNumber = *(__IO uint32_t*)
    }
}


/*****************************************************************************/
/**
 * @brief	:   Write long number to internal flash
 * @param	:   Address, Data
 * @retval	:   0 if success, 1 if error
 * @author	:	
 * @created	:	15/01/2014
 * @version	:
 * @reviewer:	
 */
uint8_t InternalFlash_WriteLong(uint32_t Address, uint32_t Data) 
{
    FLASH_Status Status;
    
	//Unlock Flash
	InternalFlash_Init();
	Status = FLASH_ProgramWord(Address, Data);
	FLASH_Lock();      

	if(Status != FLASH_COMPLETE) 
    {
        return 1;
    }
	return 0;
}
/*****************************************************************************/
/**
 * @brief	: Erase certain partion in embedded flash
 * @param	:  
 * @retval	:	0 if Success, > 0 if Fail
 * @author	:	TueTD
 * @created	:	
 * @version	:
 * @reviewer:	
 */
FLASH_Status HAL_FLASH_Erase(uint32_t address)
{
	uint32_t err;
    HAL_IWDG_Refresh(&hiwdg);
    InternalFlash_Init();
    HAL_IWDG_Refresh(&hiwdg);
	HAL_FLASHEx_DATAEEPROM_Unlock();
//    DEBUG_INFO ("ERASE \r\n");
	for (uint32_t i = 0; i < 128; i++)
	{
       
		err = HAL_FLASHEx_DATAEEPROM_Erase(address + i*4);
        
        HAL_IWDG_Refresh(&hiwdg);
		if (HAL_OK != err)
		{
			DEBUG_ERROR("Erase eeprom failed at addr 0x%08X, err code %08X\r\n", CONFIG_ADDR + i*4, err);
			break;
		}
	}
//    DEBUG_INFO ("ERASE done \r\n");
    HAL_IWDG_Refresh(&hiwdg);
	HAL_FLASHEx_DATAEEPROM_Lock();
    return HAL_OK;
}
/*****************************************************************************/
/**
 * @brief	: 
 * @param	:  
 * @retval	:	0 if Success, > 0 if Fail
 * @author	:	Phinht
 * @created	:	15/09/2015
 * @version	:
 * @reviewer:	
 */
FLASH_Status HAL_Flash_ProgramWord(uint32_t address, uint32_t data)
{
	HAL_FLASHEx_DATAEEPROM_Unlock();
    FLASH_Status ret = HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_WORD, address, data);
    if(ret != 0)
    {
        DEBUG_ERROR("Faild to write at :0x%08x\r\n", address);
    }
    HAL_FLASHEx_DATAEEPROM_Lock();
    //HAL_Delay(100);
    return ret;
}

