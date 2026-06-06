#ifndef USER_CTRL_H
#define USER_CTRL_H

#include "cmsis_os2.h"

void UserCtrl_Init(void);
void StartUserCtrlTask(void *argument);

#endif /* USER_CTRL_H */
