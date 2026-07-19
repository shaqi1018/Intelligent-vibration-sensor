#ifndef _FFCONF
#define _FFCONF 68300

#include "main.h"

#define _FS_READONLY         0
#define _FS_MINIMIZE         0
#define _USE_STRFUNC         0
#define _USE_FIND            0
#define _USE_MKFS            0
#define _USE_FASTSEEK        0
#define _USE_EXPAND          1
#define _USE_CHMOD           0
#define _USE_LABEL           0
#define _USE_FORWARD         0

#define _CODE_PAGE           437
/* 长文件名(LFN)开启：会话目录用 RTC 时间命名 CTBX_YYYY-MM-DD-HH-MM(21 字符,
 * 超 8.3 短名上限)。_USE_LFN=1=静态工作缓冲(仅 logger 任务在会话起止解析路径,
 * 无并发,故 _FS_REENTRANT 保持 0 安全,与参考工程 DATALOG1 一致)。需把
 * option/ccsbcs.c(CP437 码表,提供 ff_convert/ff_wtoupper)加入编译。
 * _MAX_LFN=63：最长路径段 21 字符,63 富余;静态缓冲仅 (63+1)*2=128B。 */
#define _USE_LFN             1
#define _MAX_LFN             63
#define _LFN_UNICODE         0
#define _STRF_ENCODE         3
#define _FS_RPATH            0

#define _VOLUMES             1
#define _STR_VOLUME_ID       0
#define _VOLUME_STRS         "SD"
#define _MULTI_PARTITION     0
#define _MIN_SS              512
#define _MAX_SS              512
#define _USE_TRIM            0
#define _FS_NOFSINFO         0

#define _FS_TINY             0
#define _FS_EXFAT            0
#define _FS_NORTC            0    /* 0=调 get_fattime() 读 RTC 实时时间作为文件时间戳 */
#define _NORTC_MON           1    /* 仅 _FS_NORTC==1 时生效，此处保留不用 */
#define _NORTC_MDAY          1
#define _NORTC_YEAR          2026
#define _FS_LOCK             0
#define _FS_REENTRANT        0
#define _FS_TIMEOUT          1000
#define _SYNC_t              void*

#define _WORD_ACCESS         0

#endif /* _FFCONF */
