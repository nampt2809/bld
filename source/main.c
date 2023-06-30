/*
 * Copyright (c) 2013 - 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "header_files.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/


static void hardwawre_init(void);
static void software_init(void);
static void sys_control(void);

//nampt
#define pflashSectorSize 512 

#define Start_Add_FLASH_EX 0x600000
void sys_jumpApp(void);
flash_config_t s_flashDriver;
static void sys_boot(void);
static void error_trap(void);

uint8_t g_sector_index;
uint32_t cal_addr_sector(uint8_t sector_index);

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/
/*!
 * @brief Main function
 */
volatile bool ftm_isr_flag           = false;
volatile uint32_t g_systickCounter;

void SysTick_Handler(void)
{
    if (g_systickCounter != 0U)
    {
        g_systickCounter--;
    }
}

void SysTick_DelayTicks(uint32_t n)
{
    g_systickCounter = n;
    while (g_systickCounter != 0U)
    {
    }
}
///=============================================================================
int main(void)
{
    ///1. init system===========================================================
    hardwawre_init();
    software_init();
    SYSTEM_VAR_T *p_sys_var = &system_var;
    /* Set systick reload value to generate 1ms interrupt */
    
    if (SysTick_Config(SystemCoreClock / 1000U))
    {
        while (1)
        {
        }
    }
    //nampt
    flash_read_buffer(USER_SPI1_MASTER,&system_var.u8NewFirmFlag1, Start_Add_FLASH_EX,1);
    flash_read_buffer(USER_SPI1_MASTER,&system_var.u8NewFirmFlag2, Start_Add_FLASH_EX,1);
    flash_read_buffer(USER_SPI1_MASTER,&system_var.u8NewFirmFlag2, Start_Add_FLASH_EX,1);
    if(system_var.u8NewFirmFlag1 == 1 || system_var.u8NewFirmFlag2 == 1 || system_var.u8NewFirmFlag3 == 1){
      sys_boot();
    }
    else{
      sys_jumpApp();
    }
    ///2. timer 50ms ===========================================================
    while (1)
    {
        
        if (ftm_isr_flag)
        {
          p_sys_var->sys_cnt++;
          // system control=====================================================
          sys_control();
          // system control=====================================================
          ftm_isr_flag = false;
        }
        __WFI();
    }
    ///3. ======================================================================
}



///1. hardwawre_init============================================================
static void hardwawre_init(void){
  /* Init board hardware. */
  BOARD_InitBootPins();
  BOARD_InitBootClocks();
}

///2.software_init==============================================================
static void software_init(void){
  init_timer(USER_FTM2_BASEADDR,USER_FTM2_IRQ_NUM,TIMER_PERIOD_MS);
  init_rs485(USER_UART2,USER_UART2_IRQn,9600);
  init_flash(USER_SPI1_MASTER);
  init_rtc(RTC_SDA_PORT,RTC_SDA_PIN,RTC_SCL_PORT,RTC_SCL_PIN,MS);
}

static void sys_get_rtc(ST_TIME_FORMAT *p_get_rtc){
  *p_get_rtc = get_time_rtc(RTC_SDA_PORT,RTC_SDA_PIN,RTC_SCL_PORT,RTC_SCL_PIN,MS);
}

///3. sys_control===============================================================
uint8_t  main_test = 0; 
#define  TEST "thanhcm33"
#define    ADDRRRR       0x600000 
uint8_t  buff_read[512];
static void sys_control(void){
  ///1. led run.
  RUN_LED_TOGGLE;
  SYS_NORMAL_LED_TOGGLE;
  sys_get_rtc(&rtc_DS3231);
  ///2.
  ///////////test//////////////////////////////////////////////////////////
  if(main_test ==  5){
    DIR_485_ON;
    SysTick_DelayTicks(5);
    UART_WriteBlocking(USER_UART2, TEST, strlen(TEST));
    SysTick_DelayTicks(5);
    DIR_485_OFF;
    main_test = 0;
  }
  
  ///test flash
  if(main_test == 6){
    flash_write_buffer(USER_SPI1_MASTER,TEST, ADDRRRR, 9);
    main_test =0;
  }else if(main_test == 7){
    flash_erase_sector(USER_SPI1_MASTER,ADDRRRR);  
    main_test = 0;
  }else if(main_test == 8){
    flash_read_buffer(USER_SPI1_MASTER,buff_read, ADDRRRR, 9);
    main_test = 0;
  }
  ///////////test//////////////////////////////////////////////////////////
  
  
}
//nampt
static void sys_boot(void){
  uint32_t result;
  memset(&s_flashDriver,0,sizeof(s_flashDriver));
  FLASH_SetProperty(&s_flashDriver,kFLASH_PropertyFlashClockFrequency, 20000000U);
  result = FLASH_Init(&s_flashDriver);
  if (kStatus_FLASH_Success != result)
    {
        error_trap();
    }
  // xoa 512 bytes trong sector
  result = FLASH_Erase(&s_flashDriver, cal_addr_sector(0), pflashSectorSize, kFLASH_ApiEraseKey);
  if (kStatus_FLASH_Success != result)
    {
        error_trap();
    }
  
}
void sys_jumpApp(void){
  
}

void error_trap(void)
{
    PRINTF("\r\n\r\n\r\n\t---- HALTED DUE TO FLASH ERROR! ----");
    while (1)
    {
    }
}
uint32_t cal_addr_sector(uint8_t sector_index){
  uint32_t retval= 0;
  retval = FSL_FEATURE_FLASH_PFLASH_START_ADDRESS + sector_index * FSL_FEATURE_FLASH_PFLASH_BLOCK_SECTOR_SIZE;
  return retval;
}

























