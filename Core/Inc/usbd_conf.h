#ifndef USBD_CONF_H
#define USBD_CONF_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "stm32u5xx_hal.h"
#include "main.h"

#define USBD_MAX_NUM_INTERFACES               1U
#define USBD_MAX_NUM_CONFIGURATION            1U
#define USBD_MAX_STR_DESC_SIZ                 0x100U
#define USBD_SUPPORT_USER_STRING_DESC         1U
#define USBD_DEBUG_LEVEL                      0U
#define USBD_LPM_ENABLED                      0U
#define USBD_SELF_POWERED                     1U

/* MSC class config */
#define MSC_MEDIA_PACKET                      512U

/* malloc / free */
#define USBD_malloc                           malloc
#define USBD_free                             free
#define USBD_memset                           memset
#define USBD_memcpy                           memcpy
#define USBD_Delay                            HAL_Delay

/* Trace logging — disabled */
#define USBD_UsrLog(...)
#define USBD_ErrLog(...)
#define USBD_DbgLog(...)

#endif
