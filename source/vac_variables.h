//--thanhcm3--12--06--23--

#ifndef _VAC_VARIABLES_H
#define _VAC_VARIABLES_H
//include-----------------------------------------------------------------------
//define------------------------------------------------------------------------
extern volatile bool ftm_isr_flag;

typedef struct{

  uint32_t sys_cnt;
  int16_t  UpdateFirmWare_Flag;
  uint8_t  u8RunBootflag;
  uint8_t  u8NewFirmFlag1;
  uint8_t  u8NewFirmFlag2;
  uint8_t  u8NewFirmFlag3;
  
  
}SYSTEM_VAR_T;
extern SYSTEM_VAR_T system_var;
//function----------------------------------------------------------------------
//code here


#endif /* _VAC_VARIABLES_H */