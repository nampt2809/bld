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
#define    ADDRRRR       0xFF0000
uint8_t  buff_read[10];
static void sys_control(void){
  ///1. led run.
  RUN_LED_TOGGLE;
  SYS_NORMAL_LED_TOGGLE;
  sys_get_rtc(&rtc_DS3231);
  ///2.

  
  
  ///////////test//////////////////////////////////////////////////////////
  if(main_test == 1){
    SYS_ALARM_LED_ON
    main_test = 0;
  }
  
  if(main_test ==  2){
    SYS_ALARM_LED_OFF
    main_test = 0;
  }
  
  if(main_test ==  3){
    SYS_FAULT_LED_ON
    main_test = 0;
  }
  
  if(main_test ==  4){
    SYS_FAULT_LED_OFF
    main_test = 0;
  }
  
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




























