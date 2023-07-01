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
#define Start_Add_FLASH_EX 600000
#define BUFFER_LEN 4
static void hardwawre_init(void);
static void software_init(void);
//nampt

static void sys_jumpApp(void);
static flash_config_t s_flashDriver;
static void sys_boot(void);
static void error_trap(void);
static void read_flashEX(void);


/*variable*/

/*! @brief Buffer for program */
static uint32_t s_buffer[BUFFER_LEN];
/*! @brief Buffer for readback */
static uint32_t s_buffer_rbc[BUFFER_LEN];

uint8_t g_buffer_flashEX[BUFFER_LEN];

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/
/*!
 * @brief Main function
 */
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
    /* Set systick reload value to generate 1ms interrupt */ 
    //nampt
    uint8_t buffTrans[]={1};
    flash_write_buffer(USER_SPI1_MASTER,buffTrans, Start_Add_FLASH_EX,1);
    flash_read_buffer(USER_SPI1_MASTER,&system_var.u8NewFirmFlag1, Start_Add_FLASH_EX,1);
    flash_read_buffer(USER_SPI1_MASTER,&system_var.u8NewFirmFlag2, Start_Add_FLASH_EX,1);
    flash_read_buffer(USER_SPI1_MASTER,&system_var.u8NewFirmFlag2, Start_Add_FLASH_EX,1);
    if(system_var.u8NewFirmFlag1 == 1 || system_var.u8NewFirmFlag2 == 1 || system_var.u8NewFirmFlag3 == 1){
      sys_boot();
    }
    else{
      sys_jumpApp();
    }
    while(1){
    }
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

///3. sys_control===============================================================
uint8_t  main_test = 0; 
#define  TEST "thanhcm33"
#define    ADDRRRR       0x600000 
uint8_t  buff_read[512];

//nampt
static void sys_boot(void){
    status_t result;    /* Return code from each flash driver function */
    uint32_t destAdrss; /* Address of the target location */
    uint32_t i;

    uint32_t pflashBlockBase  = 0;
    uint32_t pflashTotalSize  = 0;
    uint32_t pflashSectorSize = 0;

    /* Init hardware */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitDebugConsole();

    /* Clean up Flash driver Structure*/
    memset(&s_flashDriver, 0, sizeof(flash_config_t));

    FLASH_SetProperty(&s_flashDriver, kFLASH_PropertyFlashClockFrequency, 20000000U);
    /* Setup flash driver structure for device and initialize variables. */
    result = FLASH_Init(&s_flashDriver);
    if (kStatus_FLASH_Success != result)
    {
        error_trap();
    }
    /* Get flash properties*/
    pflashBlockBase  = s_flashDriver.PFlashBlockBase; // 0x00
    pflashTotalSize  = s_flashDriver.PFlashTotalSize;// 128kb
    pflashSectorSize = s_flashDriver.PFlashSectorSize;//512
#ifndef SECTOR_INDEX_FROM_END
#define SECTOR_INDEX_FROM_END 1U
#endif

/* Erase a sector from destAdrss. */
#if defined(FSL_FEATURE_FLASH_HAS_PFLASH_BLOCK_SWAP) && FSL_FEATURE_FLASH_HAS_PFLASH_BLOCK_SWAP
    /* Note: we should make sure that the sector shouldn't be swap indicator sector*/
    destAdrss = pflashBlockBase + (pflashTotalSize - (SECTOR_INDEX_FROM_END * pflashSectorSize * 2));
#else
    destAdrss = pflashBlockBase + (pflashTotalSize - (SECTOR_INDEX_FROM_END * pflashSectorSize));
#endif

    result = FLASH_Erase(&s_flashDriver, destAdrss, pflashSectorSize, kFLASH_ApiEraseKey);
    if (kStatus_FLASH_Success != result)
    {
        error_trap();
    }
    
    
    /* Prepare user buffer. */
    
    read_flashEX();
//    for (i = 0; i < BUFFER_LEN; i++)
//    {
//        s_buffer[i] = i;
//    }
    
    
    /* Program user buffer into flash*/
    result = FLASH_Program(&s_flashDriver, destAdrss, s_buffer, sizeof(s_buffer));
    if (kStatus_FLASH_Success != result)
    {
        error_trap();
    }
    /* Verify programming by reading back from flash directly*/
    for (i = 0; i < BUFFER_LEN; i++)
    {
        s_buffer_rbc[i] = *(volatile uint32_t *)(destAdrss + i * 4);
        if (s_buffer_rbc[i] != s_buffer[i])
        {
            error_trap();
        }
    }
    FLASH_Erase(&s_flashDriver, destAdrss, pflashSectorSize, kFLASH_ApiEraseKey);
}
static void sys_jumpApp(void){
  
}

static void error_trap(void)
{
    PRINTF("\r\n\r\n\r\n\t---- HALTED DUE TO FLASH ERROR! ----");
    while (1)
    {
    }
}


static void read_flashEX(void){
  uint8_t s_buffer_flashEX[4];
  uint32_t i=0;
  s_buffer_flashEX[0] = 0x01;
  s_buffer_flashEX[1] = 0x02;
  s_buffer_flashEX[2] = 0x03;
  s_buffer_flashEX[3] = 0x04;
  
  //ghi vao flash ngoai
  flash_write_buffer(USER_SPI1_MASTER,s_buffer_flashEX, Start_Add_FLASH_EX,4);
  //doc tu flash ngoai
  flash_read_buffer(USER_SPI1_MASTER,g_buffer_flashEX,Start_Add_FLASH_EX,4);
  

  s_buffer[0] = g_buffer_flashEX[i] 
  
  
}























