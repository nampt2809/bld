//--thanhcm3--12--06--23--

#ifndef _VAC_VARIABLES_H
#define _VAC_VARIABLES_H
//include-----------------------------------------------------------------------
//define------------------------------------------------------------------------
extern volatile bool ftm_isr_flag;

typedef struct{

  uint32_t sys_cnt;
  
}SYSTEM_VAR_T;
extern SYSTEM_VAR_T system_var;
//function----------------------------------------------------------------------
//code here


#endif /* _VAC_VARIABLES_H */