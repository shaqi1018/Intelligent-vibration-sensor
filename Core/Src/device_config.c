/**
  ******************************************************************************
  * @file    device_config.c
  * @brief   SD卡设备配置读写模块
  *
  *  设计原则：
  *    - 不引入外部 JSON 库；使用单遍扫描的最小化解析器
  *    - 全部工作缓冲区声明为 static，避免栈压力
  *    - 与 fatfs_sd.c 共用同一套 FatFs_SD_Mount / Unmount 接口
  *    - DeviceCfg_LoadFromSD     ：挂载 → 读/写 → 卸载
  *    - DeviceCfg_WriteSnapshotToSD：挂载 → 写   → 不卸载
  *      （供后续 FatFs_SD_LoggerStart() 复用已挂载状态）
  ******************************************************************************
  */

#include "device_config.h"

#include <stdio.h>
#include <string.h>

#include "diskio.h"
#include "sd_diskio.h"
#include "sdmmc.h"
#include "fatfs_sd.h"
#include "acq_config.h"

/* ============================================================================
 *  内部常量
 * ========================================================================= */

#define DEV_CFG_PATH           "0:/DEVCFG.JSN"
#define DEV_CFG_READ_BUF_SZ    6144U     /* 配置文件读取缓冲区字节数（新模板带中文说明+分段，需 >文件大小，否则尾部字段被截断丢解析） */
#define DEV_CFG_OBJ_BUF_SZ     512U      /* 子对象提取缓冲区字节数   */

/* 上电写入的默认模板（与解析器所支持的字段保持一致）
 * _doc / _options_* 等以 _ 开头的键仅作文档用途，解析器会自动忽略。 */
static const char kCfgTemplate[] =
"{\r\n"
"  \"_file\": \"Sensor Box 设备配置；开机读取并应用。下划线开头的键是注释，解析器忽略。\",\r\n"
"\r\n"
"  \"_s1\": \"==================== 开机行为 ====================\",\r\n"
"  \"boot_acquire\": 0,\r\n"
"  \"_doc_boot_acquire\": \"开机是否自动开始采集：1=自动采，0=需按键/上位机手动启动\",\r\n"
"  \"_options_boot_acquire\": [0, 1],\r\n"
"\r\n"
"  \"sink\": \"SD\",\r\n"
"  \"_doc_sink\": \"数据出口：SD=存卡(无人值守) / USB=传上位机。开机即采推荐 SD\",\r\n"
"  \"_options_sink\": [\"SD\", \"USB\"],\r\n"
"\r\n"
"  \"output_format\": \"BIN\",\r\n"
"  \"_doc_output_format\": \"SD 卡数据格式：CSV=文本(兼容既有) / BIN=二进制(体积小,需解析工具)\",\r\n"
"  \"_options_output_format\": [\"CSV\", \"BIN\"],\r\n"
"\r\n"
"  \"duration_ms\": 0,\r\n"
"  \"_doc_duration_ms\": \"单次采集时长(ms)，到点自动停；0=一直采到关机或手动停\",\r\n"
"\r\n"
"  \"seg_size_mb\": 512,\r\n"
"  \"_doc_seg_size_mb\": \"SD每个数据文件分段大小(MB)：单文件写满即新建续段(如 LSM_IMU.BIN→IMU0002.BIN→...)，各文件独立计数；0=不分段(单文件)。麦克风MIC.WAV不分段\",\r\n"
"\r\n"
"  \"_s2\": \"==================== LSM6DSOX 六轴IMU ====================\",\r\n"
"  \"lsm6dsox\": {\r\n"
"    \"enabled\": 1,\r\n"
"    \"range_g\": 4,\r\n"
"    \"_options_range_g\": [2, 4, 8, 16],\r\n"
"    \"range_dps\": 2000,\r\n"
"    \"_options_range_dps\": [250, 500, 1000, 2000],\r\n"
"    \"odr_hz\": 1666,\r\n"
"    \"_options_odr_hz\": [12, 26, 52, 104, 208, 416, 833, 1666, 3332, 6664]\r\n"
"  },\r\n"
"\r\n"
"  \"_s3\": \"==================== H3LIS100DL 高量程加速度 ====================\",\r\n"
"  \"h3lis100dl\": {\r\n"
"    \"enabled\": 1,\r\n"
"    \"range_g\": 100,\r\n"
"    \"_options_range_g\": [100],\r\n"
"    \"odr_hz\": 400,\r\n"
"    \"_options_odr_hz\": [50, 100, 400]\r\n"
"  },\r\n"
"\r\n"
"  \"_s4\": \"==================== QMA6100P 加速度 ====================\",\r\n"
"  \"qma6100p\": {\r\n"
"    \"enabled\": 1,\r\n"
"    \"range_g\": 4,\r\n"
"    \"_options_range_g\": [2, 4, 8, 16, 32],\r\n"
"    \"odr_hz\": 100,\r\n"
"    \"_options_odr_hz\": [100, 200, 400, 800, 1600]\r\n"
"  },\r\n"
"\r\n"
"  \"_s5\": \"==================== ES8311 麦克风 ====================\",\r\n"
"  \"es8311\": {\r\n"
"    \"enabled\": 1,\r\n"
"    \"sample_rate_hz\": 16000,\r\n"
"    \"_options_sample_rate_hz\": [8000, 16000, 48000, 96000],\r\n"
"    \"bits\": 16,\r\n"
"    \"gain_db\": 33,\r\n"
"    \"_doc_gain_db\": \"麦克风增益 0..42 dB\"\r\n"
"  },\r\n"
"\r\n"
"  \"_s6\": \"==================== AHT20 温湿度 ====================\",\r\n"
"  \"aht20\": {\r\n"
"    \"enabled\": 1,\r\n"
"    \"odr_hz\": 1,\r\n"
"    \"_doc_odr_hz\": \"1Hz\"\r\n"
"  },\r\n"
"\r\n"
"  \"_s7\": \"==================== LIS2MDL 磁力计 ====================\",\r\n"
"  \"lis2mdl\": {\r\n"
"    \"enabled\": 1,\r\n"
"    \"odr_hz\": 100,\r\n"
"    \"_options_odr_hz\": [10, 20, 50, 100]\r\n"
"  },\r\n"
"\r\n"
"  \"_s8\": \"==================== 电池配置 ====================\",\r\n"
"  \"battery\": {\r\n"
"    \"full_mv\": 4200,\r\n"
"    \"_doc_full_mv\": \"电池满电电压(mV)，充满静置10分钟后用万用表测量填入，用于电量百分比计算\"\r\n"
"  }\r\n"
"}\r\n";

/* ============================================================================
 *  最小化 JSON 辅助函数（无堆分配，单遍扫描）
 * ========================================================================= */

/**
 * @brief 在 buf 中查找键 key 对应的值起始位置（冒号后第一个非空白字符）。
 *
 *        扫描规则：
 *          - 逐字符找到 '"'，检查其后内容是否为 "key"
 *          - 若不匹配则跳过整个字符串到对应闭合 '"'
 *          - 匹配后跳过空白和 ':'，返回值起始指针
 *
 * @return 指向值第一个字符的指针；未找到返回 NULL
 */
static const char *json_find_value(const char *buf, const char *key)
{
    size_t      klen = strlen(key);
    const char *p    = buf;

    while (*p != '\0')
    {
        /* 寻找字符串开始引号 */
        if (*p != '"') { p++; continue; }
        p++; /* 跳过开引号 */

        /* 判断是否与目标键匹配 */
        if (strncmp(p, key, klen) == 0 && p[klen] == '"')
        {
            p += klen + 1U; /* 跳过键名和闭引号 */
            /* 跳过空白 */
            while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n') p++;
            if (*p != ':')
            {
                /* 同名字符串出现在值位置，不是键 → 继续扫描 */
                continue;
            }
            p++; /* 跳过冒号 */
            while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n') p++;
            return p;
        }

        /* 跳过当前字符串（到闭合引号） */
        while (*p != '\0' && *p != '"') p++;
        if (*p == '"') p++; /* 跳过闭引号 */
    }
    return NULL;
}

/**
 * @brief 从 p 处解析一个无符号十进制整数。
 *        p 必须指向数字首字符。
 * @return 1=成功；0=无数字
 */
static int json_parse_uint(const char *p, uint32_t *out)
{
    if (p == NULL || *p < '0' || *p > '9') return 0;
    *out = 0U;
    while (*p >= '0' && *p <= '9')
    {
        *out = (*out * 10U) + (uint32_t)(*p - '0');
        p++;
    }
    return 1;
}

/**
 * @brief 从 p 处解析 JSON 字符串值（p 必须指向开引号 '"'）。
 *        最多复制 buf_sz-1 个字符并追加 '\0'。
 * @return 1=成功；0=格式错误
 */
static int json_parse_string(const char *p, char *out, size_t buf_sz)
{
    size_t i = 0U;
    if (p == NULL || *p != '"') return 0;
    p++; /* 跳过开引号 */
    while (*p != '\0' && *p != '"' && i < buf_sz - 1U)
    {
        out[i++] = *p++;
    }
    out[i] = '\0';
    return (*p == '"') ? 1 : 0;
}

/**
 * @brief 提取 buf 中 key 对应的 JSON 对象（{...}）到 out 缓冲区。
 *        正确处理嵌套大括号。
 * @return 1=成功；0=未找到或缓冲区太小
 */
static int json_extract_object(const char *buf, const char *key,
                                char *out, size_t out_sz)
{
    const char *p     = json_find_value(buf, key);
    size_t      depth = 0U;
    size_t      i     = 0U;

    if (p == NULL || *p != '{') return 0;

    do {
        if (i >= out_sz - 1U) return 0; /* 缓冲区不足 */
        out[i++] = *p;
        if      (*p == '{') { depth++; }
        else if (*p == '}') { depth--; if (depth == 0U) break; }
        p++;
    } while (*p != '\0');

    out[i] = '\0';
    return (depth == 0U) ? 1 : 0;
}

/* ============================================================================
 *  FatFs 写入辅助
 * ========================================================================= */

static FRESULT write_exact(FIL *f, const char *s, UINT len)
{
    UINT    written = 0U;
    FRESULT r       = f_write(f, s, len, &written);
    if (r != FR_OK) return r;
    return (written == len) ? FR_OK : FR_INT_ERR;
}

/* ============================================================================
 *  模板写入
 * ========================================================================= */

static FRESULT DeviceCfg_WriteTemplate(void)
{
    FIL     file;
    FRESULT r;

    r = f_open(&file, DEV_CFG_PATH, FA_CREATE_ALWAYS | FA_WRITE);
    if (r != FR_OK)
    {
        printf("[DevCfg] 写模板失败: %s (%d)\r\n",
               FatFs_SD_ResultToString(r), (int)r);
        return r;
    }

    r = write_exact(&file, kCfgTemplate, (UINT)(sizeof(kCfgTemplate) - 1U));
    if (r == FR_OK) r = f_sync(&file);
    (void)f_close(&file);

    if (r == FR_OK)
        printf("[DevCfg] 已写入默认配置模板: %s\r\n", DEV_CFG_PATH);
    else
        printf("[DevCfg] 模板写入失败: %s (%d)\r\n",
               FatFs_SD_ResultToString(r), (int)r);

    return r;
}

/* ============================================================================
 *  JSON 解析并应用到运行时配置
 * ========================================================================= */

/* 返回位掩码：bit0=文件已有 battery 字段，bit1=已有 seg_size_mb 字段。
 * 缺失的位由调用者补写进配置文件。 */
static int DeviceCfg_ParseAndApply(const char *buf)
{
    AcqConfig_t  cfg;
    const char  *p;
    uint32_t     v;
    char         s[16];
    static char  s_obj[DEV_CFG_OBJ_BUF_SZ]; /* 子对象缓冲区，static 避免栈压力 */

    /* 以当前运行时配置为基础，仅覆盖文件中出现的字段 */
    AcqConfig_GetCopy(&cfg);

    /* ---- 顶层标量字段（仅保留真正生效的） ---- */

    p = json_find_value(buf, "boot_acquire");
    if (json_parse_uint(p, &v)) cfg.boot_acquire = (uint8_t)(v != 0U ? 1U : 0U);

    p = json_find_value(buf, "sink");
    if (p != NULL && json_parse_string(p, s, sizeof(s)))
    {
        if      (strcmp(s, "USB")  == 0) cfg.sink_mask = ACQ_SINK_USB;
        else if (strcmp(s, "SD")   == 0) cfg.sink_mask = ACQ_SINK_SD;
        else if (strcmp(s, "BOTH") == 0) cfg.sink_mask = ACQ_SINK_BOTH;
    }

    p = json_find_value(buf, "output_format");
    if (p != NULL && json_parse_string(p, s, sizeof(s)))
    {
        if      (strcmp(s, "CSV") == 0) cfg.output_format = ACQ_OUTPUT_CSV;
        else if (strcmp(s, "BIN") == 0) cfg.output_format = ACQ_OUTPUT_BIN;
    }

    p = json_find_value(buf, "duration_ms");
    if (json_parse_uint(p, &v)) cfg.duration_ms = v;

    /* ---- seg_size_mb 顶层标量（分段大小，MB） ---- */
    int has_seg = 0;
    p = json_find_value(buf, "seg_size_mb");
    if (p != NULL)
    {
        has_seg = 1;
        if (json_parse_uint(p, &v))
        {
            if (v == 0U)
            {
                cfg.seg_size_mb = 0U;          /* 0=不分段 */
            }
            else
            {
                if (v < 16U)   v = 16U;        /* 下限 16MB */
                if (v > 4000U) v = 4000U;      /* 上限 4000MB(<FAT32 4GB) */
                cfg.seg_size_mb = v;
            }
        }
    }

    /* ---- lsm6dsox 子对象 ---- */
    if (json_extract_object(buf, "lsm6dsox", s_obj, sizeof(s_obj)))
    {
        p = json_find_value(s_obj, "enabled");
        if (json_parse_uint(p, &v))
            cfg.lsm6dsox.enabled = (uint8_t)(v != 0U ? 1U : 0U);

        p = json_find_value(s_obj, "range_g");
        if (json_parse_uint(p, &v))
            cfg.lsm6dsox.range = (uint16_t)v;

        p = json_find_value(s_obj, "range_dps");
        if (json_parse_uint(p, &v))
            cfg.lsm6dsox.range2 = (uint16_t)v;

        p = json_find_value(s_obj, "odr_hz");
        if (json_parse_uint(p, &v))
            cfg.lsm6dsox.odr_hz = (uint16_t)v;
    }

    /* ---- h3lis100dl 子对象 ---- */
    if (json_extract_object(buf, "h3lis100dl", s_obj, sizeof(s_obj)))
    {
        p = json_find_value(s_obj, "enabled");
        if (json_parse_uint(p, &v))
            cfg.h3lis100dl.enabled = (uint8_t)(v != 0U ? 1U : 0U);

        p = json_find_value(s_obj, "range_g");
        if (json_parse_uint(p, &v))
            cfg.h3lis100dl.range = (uint16_t)v;

        p = json_find_value(s_obj, "odr_hz");
        if (json_parse_uint(p, &v))
            cfg.h3lis100dl.odr_hz = (uint16_t)v;
    }

    /* ---- qma6100p 子对象 ---- */
    if (json_extract_object(buf, "qma6100p", s_obj, sizeof(s_obj)))
    {
        p = json_find_value(s_obj, "enabled");
        if (json_parse_uint(p, &v))
            cfg.qma6100p.enabled = (uint8_t)(v != 0U ? 1U : 0U);

        p = json_find_value(s_obj, "range_g");
        if (json_parse_uint(p, &v))
            cfg.qma6100p.range = (uint16_t)v;

        p = json_find_value(s_obj, "odr_hz");
        if (json_parse_uint(p, &v))
            cfg.qma6100p.odr_hz = (uint16_t)v;
    }

    /* ---- es8311 子对象（麦克风） ---- */
    if (json_extract_object(buf, "es8311", s_obj, sizeof(s_obj)))
    {
        p = json_find_value(s_obj, "enabled");
        if (json_parse_uint(p, &v))
            cfg.es8311.enabled = (uint8_t)(v != 0U ? 1U : 0U);

        p = json_find_value(s_obj, "sample_rate_hz");
        if (json_parse_uint(p, &v))
            cfg.es8311.sample_rate_hz = v;

        p = json_find_value(s_obj, "bits");
        if (json_parse_uint(p, &v))
            cfg.es8311.bits = (uint16_t)v;

        p = json_find_value(s_obj, "gain_db");
        if (json_parse_uint(p, &v))
            cfg.es8311.gain_db = (uint16_t)v;
    }

    /* ---- aht20 子对象 ---- */
    if (json_extract_object(buf, "aht20", s_obj, sizeof(s_obj)))
    {
        p = json_find_value(s_obj, "enabled");
        if (json_parse_uint(p, &v))
            cfg.aht20.enabled = (uint8_t)(v != 0U ? 1U : 0U);

        p = json_find_value(s_obj, "odr_hz");
        if (json_parse_uint(p, &v))
            cfg.aht20.odr_hz = (uint16_t)v;
    }

    /* ---- lis2mdl 子对象 ---- */
    if (json_extract_object(buf, "lis2mdl", s_obj, sizeof(s_obj)))
    {
        p = json_find_value(s_obj, "enabled");
        if (json_parse_uint(p, &v))
            cfg.lis2mdl.enabled = (uint8_t)(v != 0U ? 1U : 0U);

        p = json_find_value(s_obj, "odr_hz");
        if (json_parse_uint(p, &v))
            cfg.lis2mdl.odr_hz = (uint16_t)v;
    }

    /* ---- battery 子对象 ---- */
    int has_battery = 0;
    if (json_extract_object(buf, "battery", s_obj, sizeof(s_obj)))
    {
        has_battery = 1;
        p = json_find_value(s_obj, "full_mv");
        if (json_parse_uint(p, &v) && v >= 3500U && v <= 4400U)
            cfg.bat_full_mv = v;
    }

    if (AcqConfig_Set(&cfg) == 0)
    {
        printf("[DevCfg] 配置已应用: boot_acquire=%u sink=0x%02X duration=%lu ms\r\n",
               (unsigned int)cfg.boot_acquire,
               (unsigned int)cfg.sink_mask,
               (unsigned long)cfg.duration_ms);
    }
    else
    {
        printf("[DevCfg] 配置应用被拒绝（范围校验未通过），使用默认值\r\n");
    }
    return (has_battery ? 1 : 0) | (has_seg ? 2 : 0);
}

/* ============================================================================
 *  公开接口实现
 * ========================================================================= */

FRESULT DeviceCfg_LoadFromSD(void)
{
    FRESULT     r;
    FIL         file;
    FILINFO     info;
    UINT        read_len = 0U;
    static char s_buf[DEV_CFG_READ_BUF_SZ]; /* static 避免栈压力 */

    if (SDMMC1_IsCardDetected() == 0U)
    {
        printf("[DevCfg] 未检测到SD卡，跳过配置读取\r\n");
        return FR_NOT_READY;
    }

    r = FatFs_SD_Mount();
    if (r != FR_OK)
    {
        printf("[DevCfg] SD挂载失败: %s (%d)\r\n",
               FatFs_SD_ResultToString(r), (int)r);
        return r;
    }

    /* 检查文件是否存在 */
    r = f_stat(DEV_CFG_PATH, &info);
    if (r == FR_NO_FILE)
    {
        printf("[DevCfg] 未找到 %s，写入默认模板\r\n", DEV_CFG_PATH);
        (void)DeviceCfg_WriteTemplate();
        FatFs_SD_Unmount();
        return FR_NO_FILE; /* 不视为致命错误，调用者使用内建缺省 */
    }
    if (r != FR_OK)
    {
        printf("[DevCfg] f_stat 失败: %s (%d)\r\n",
               FatFs_SD_ResultToString(r), (int)r);
        FatFs_SD_Unmount();
        return r;
    }

    r = f_open(&file, DEV_CFG_PATH, FA_READ);
    if (r != FR_OK)
    {
        printf("[DevCfg] 打开配置文件失败: %s (%d)\r\n",
               FatFs_SD_ResultToString(r), (int)r);
        FatFs_SD_Unmount();
        return r;
    }

    r = f_read(&file, s_buf, (UINT)(DEV_CFG_READ_BUF_SZ - 1U), &read_len);
    (void)f_close(&file);

    if (r != FR_OK)
    {
        printf("[DevCfg] 读取配置文件失败: %s (%d)\r\n",
               FatFs_SD_ResultToString(r), (int)r);
        FatFs_SD_Unmount();
        return r;
    }

    s_buf[read_len] = '\0';
    printf("[DevCfg] 已读取 %s (%u bytes)\r\n",
           DEV_CFG_PATH, (unsigned int)read_len);

    int present = DeviceCfg_ParseAndApply(s_buf);  /* bit0=battery, bit1=seg */

    /* 旧配置文件可能缺少 seg_size_mb / battery：在末尾的 } 前一次性补齐缺失项 */
    if (((present & 1) == 0) || ((present & 2) == 0))
    {
        /* 找最后一个 } */
        char *last_brace = NULL;
        char *q = s_buf;
        while (*q != '\0') { if (*q == '}') last_brace = q; q++; }

        if (last_brace != NULL)
        {
            AcqConfig_t cfg2;
            AcqConfig_GetCopy(&cfg2);

            /* 在最后的 } 处截断，改写为追加缺失字段再闭合 */
            *last_brace = '\0';

            static char s_patch[640];
            char  *w     = s_patch;
            size_t cap   = sizeof(s_patch);
            int    adv;
            int    first = 1;   /* 第一个补写段不加前导逗号(分隔逗号已先写) */

            /* 与原有最后一个字段之间的分隔逗号 */
            adv = snprintf(w, cap, ",\r\n");
            if (adv > 0 && (size_t)adv < cap) { w += adv; cap -= (size_t)adv; }

            if ((present & 2) == 0)   /* 补 seg_size_mb */
            {
                adv = snprintf(w, cap,
                    "%s\r\n"
                    "  \"seg_size_mb\": %lu,\r\n"
                    "  \"_doc_seg_size_mb\": \"SD每个数据文件分段大小(MB)：单文件写满即新建续段(LSM_IMU.BIN→IMU0002.BIN...)，各文件独立计数；0=不分段。MIC.WAV不分段\"",
                    first ? "" : ",\r\n", (unsigned long)cfg2.seg_size_mb);
                if (adv > 0 && (size_t)adv < cap) { w += adv; cap -= (size_t)adv; first = 0; }
            }

            if ((present & 1) == 0)   /* 补 battery */
            {
                adv = snprintf(w, cap,
                    "%s\r\n"
                    "  \"_s8\": \"==================== 电池配置 ====================\",\r\n"
                    "  \"battery\": {\r\n"
                    "    \"full_mv\": %lu,\r\n"
                    "    \"_doc_full_mv\": \"电池满电电压(mV)，充满静置10分钟后用万用表测量填入\"\r\n"
                    "  }",
                    first ? "" : ",\r\n", (unsigned long)cfg2.bat_full_mv);
                if (adv > 0 && (size_t)adv < cap) { w += adv; cap -= (size_t)adv; first = 0; }
            }

            /* 闭合外层对象 */
            adv = snprintf(w, cap, "\r\n}\r\n");
            if (adv > 0 && (size_t)adv < cap) { w += adv; cap -= (size_t)adv; }

            int patch_len = (int)(w - s_patch);
            if (patch_len > 0)
            {
                FIL patch_file;
                if (f_open(&patch_file, DEV_CFG_PATH, FA_WRITE | FA_OPEN_EXISTING) == FR_OK)
                {
                    /* 定位到最后的 } 位置覆写 */
                    FSIZE_t patch_offset = (FSIZE_t)(last_brace - s_buf);
                    if (f_lseek(&patch_file, patch_offset) == FR_OK)
                    {
                        UINT written = 0U;
                        (void)f_write(&patch_file, s_patch, (UINT)patch_len, &written);
                        (void)f_truncate(&patch_file);
                        (void)f_sync(&patch_file);
                        printf("[DevCfg] 已向配置文件补写缺失字段(seg/battery)\r\n");
                    }
                    (void)f_close(&patch_file);
                }
            }
        }
    }

    FatFs_SD_Unmount();
    return FR_OK;
}

/* -------------------------------------------------------------------------- */

FRESULT DeviceCfg_WriteCurrentToSD(void)
{
    FRESULT     r;
    FIL         file;
    AcqConfig_t cfg;
    const char *sink_name;
    const char *format_name;
    static char s_line[6144];   /* UTF-8 中文说明占多字节，留足余量 */
    int         line_len;

    AcqConfig_GetCopy(&cfg);

    r = FatFs_SD_Mount();
    if (r != FR_OK)
    {
        printf("[DevCfg] 写回: SD挂载失败 %s (%d)\r\n",
               FatFs_SD_ResultToString(r), (int)r);
        return r;
    }

    format_name = (cfg.output_format == ACQ_OUTPUT_BIN) ? "BIN" : "CSV";

    line_len = snprintf(
        s_line, sizeof(s_line),
        "{\r\n"
        "  \"_file\": \"Sensor Box 设备配置；开机读取并应用。下划线开头的键是注释，解析器忽略。\",\r\n"
        "\r\n"
        "  \"_s1\": \"==================== 开机行为 ====================\",\r\n"
        "  \"boot_acquire\": %u,\r\n"
        "  \"_doc_boot_acquire\": \"开机是否自动开始采集：1=自动采，0=需按键/上位机手动启动\",\r\n"
        "  \"_options_boot_acquire\": [0, 1],\r\n"
        "\r\n"
        "  \"output_format\": \"%s\",\r\n"
        "  \"_doc_output_format\": \"SD 卡数据格式：CSV=文本(兼容既有) / BIN=二进制(体积小,需解析工具)。USB传输始终用CSV\",\r\n"
        "  \"_options_output_format\": [\"CSV\", \"BIN\"],\r\n"
        "\r\n"
        "  \"duration_ms\": %lu,\r\n"
        "  \"_doc_duration_ms\": \"单次采集时长(ms)，到点自动停；0=一直采到关机或手动停\",\r\n"
        "\r\n"
        "  \"seg_size_mb\": %lu,\r\n"
        "  \"_doc_seg_size_mb\": \"SD每个数据文件分段大小(MB)：单文件写满即新建续段(LSM_IMU.BIN→IMU0002.BIN...)，各文件独立计数；0=不分段。MIC.WAV不分段\",\r\n"
        "\r\n"
        "  \"_s2\": \"==================== LSM6DSOX 六轴IMU ====================\",\r\n"
        "  \"lsm6dsox\": {\r\n"
        "    \"enabled\": %u,\r\n"
        "    \"range_g\": %u,\r\n"
        "    \"_options_range_g\": [2, 4, 8, 16],\r\n"
        "    \"range_dps\": %u,\r\n"
        "    \"_options_range_dps\": [250, 500, 1000, 2000],\r\n"
        "    \"odr_hz\": %u,\r\n"
        "    \"_options_odr_hz\": [12, 26, 52, 104, 208, 416, 833, 1666, 3332, 6664]\r\n"
        "  },\r\n"
        "\r\n"
        "  \"_s3\": \"==================== H3LIS100DL 高量程加速度 ====================\",\r\n"
        "  \"h3lis100dl\": {\r\n"
        "    \"enabled\": %u,\r\n"
        "    \"range_g\": %u,\r\n"
        "    \"_options_range_g\": [100],\r\n"
        "    \"odr_hz\": %u,\r\n"
        "    \"_options_odr_hz\": [50, 100, 400]\r\n"
        "  },\r\n"
        "\r\n"
        "  \"_s4\": \"==================== QMA6100P 加速度 ====================\",\r\n"
        "  \"qma6100p\": {\r\n"
        "    \"enabled\": %u,\r\n"
        "    \"range_g\": %u,\r\n"
        "    \"_options_range_g\": [2, 4, 8, 16, 32],\r\n"
        "    \"odr_hz\": %u,\r\n"
        "    \"_options_odr_hz\": [100, 200, 400, 800, 1600]\r\n"
        "  },\r\n"
        "\r\n"
        "  \"_s5\": \"==================== ES8311 麦克风 ====================\",\r\n"
        "  \"es8311\": {\r\n"
        "    \"enabled\": %u,\r\n"
        "    \"sample_rate_hz\": %lu,\r\n"
        "    \"_options_sample_rate_hz\": [8000, 16000, 48000, 96000],\r\n"
        "    \"bits\": %u,\r\n"
        "    \"gain_db\": %u,\r\n"
        "    \"_doc_gain_db\": \"麦克风增益 0..42 dB\"\r\n"
        "  },\r\n"
        "\r\n"
        "  \"_s6\": \"==================== AHT20 温湿度 ====================\",\r\n"
        "  \"aht20\": {\r\n"
        "    \"enabled\": %u,\r\n"
        "    \"odr_hz\": %u,\r\n"
        "    \"_doc_odr_hz\": \"1Hz\"\r\n"
        "  },\r\n"
        "\r\n"
        "  \"_s7\": \"==================== LIS2MDL 磁力计 ====================\",\r\n"
        "  \"lis2mdl\": {\r\n"
        "    \"enabled\": %u,\r\n"
        "    \"odr_hz\": %u,\r\n"
        "    \"_options_odr_hz\": [10, 20, 50, 100]\r\n"
        "  },\r\n"
        "\r\n"
        "  \"_s8\": \"==================== 电池配置 ====================\",\r\n"
        "  \"battery\": {\r\n"
        "    \"full_mv\": %lu,\r\n"
        "    \"_doc_full_mv\": \"电池满电电压(mV)，充满静置10分钟后用万用表测量填入\"\r\n"
        "  }\r\n"
        "}\r\n",
        (unsigned int)cfg.boot_acquire,
        format_name,
        (unsigned long)cfg.duration_ms,
        (unsigned long)cfg.seg_size_mb,
        (unsigned int)cfg.lsm6dsox.enabled,
        (unsigned int)cfg.lsm6dsox.range,
        (unsigned int)cfg.lsm6dsox.range2,
        (unsigned int)cfg.lsm6dsox.odr_hz,
        (unsigned int)cfg.h3lis100dl.enabled,
        (unsigned int)cfg.h3lis100dl.range,
        (unsigned int)cfg.h3lis100dl.odr_hz,
        (unsigned int)cfg.qma6100p.enabled,
        (unsigned int)cfg.qma6100p.range,
        (unsigned int)cfg.qma6100p.odr_hz,
        (unsigned int)cfg.es8311.enabled,
        (unsigned long)cfg.es8311.sample_rate_hz,
        (unsigned int)cfg.es8311.bits,
        (unsigned int)cfg.es8311.gain_db,
        (unsigned int)cfg.aht20.enabled,
        (unsigned int)cfg.aht20.odr_hz,
        (unsigned int)cfg.lis2mdl.enabled,
        (unsigned int)cfg.lis2mdl.odr_hz,
        (unsigned long)cfg.bat_full_mv);

    if ((line_len < 0) || ((size_t)line_len >= sizeof(s_line)))
        return FR_INT_ERR;

    r = f_open(&file, DEV_CFG_PATH, FA_CREATE_ALWAYS | FA_WRITE);
    if (r != FR_OK)
    {
        printf("[DevCfg] 写回: 打开文件失败 %s (%d)\r\n",
               FatFs_SD_ResultToString(r), (int)r);
        return r;
    }

    r = write_exact(&file, s_line, (UINT)line_len);
    if (r == FR_OK) r = f_sync(&file);
    (void)f_close(&file);

    if (r == FR_OK)
        printf("[DevCfg] 配置已写回: %s\r\n", DEV_CFG_PATH);
    else
        printf("[DevCfg] 写回失败: %s (%d)\r\n",
               FatFs_SD_ResultToString(r), (int)r);

    /* 不卸载 SD，供后续 FatFs_SD_LoggerStop() 处理 */
    return r;
}

/* -------------------------------------------------------------------------- */

FRESULT DeviceCfg_WriteConfigToDir(const char *dir)
{
    FRESULT     r;
    FIL         file;
    AcqConfig_t cfg;
    char        path[48];
    static char s_line[1024];
    int         line_len;

    if (dir == NULL) return FR_INVALID_PARAMETER;

    AcqConfig_GetCopy(&cfg);

    line_len = snprintf(
        s_line, sizeof(s_line),
        "{\r\n"
        "  \"lsm6dsox\": {\r\n"
        "    \"enabled\": %u,\r\n"
        "    \"range_g\": %u,\r\n"
        "    \"range_dps\": %u,\r\n"
        "    \"odr_hz\": %u\r\n"
        "  },\r\n"
        "  \"h3lis100dl\": {\r\n"
        "    \"enabled\": %u,\r\n"
        "    \"range_g\": %u,\r\n"
        "    \"odr_hz\": %u\r\n"
        "  },\r\n"
        "  \"qma6100p\": {\r\n"
        "    \"enabled\": %u,\r\n"
        "    \"range_g\": %u,\r\n"
        "    \"odr_hz\": %u\r\n"
        "  },\r\n"
        "  \"es8311\": {\r\n"
        "    \"enabled\": %u,\r\n"
        "    \"sample_rate_hz\": %lu,\r\n"
        "    \"bits\": %u,\r\n"
        "    \"gain_db\": %u\r\n"
        "  },\r\n"
        "  \"aht20\": {\r\n"
        "    \"enabled\": %u,\r\n"
        "    \"odr_hz\": %u\r\n"
        "  },\r\n"
        "  \"lis2mdl\": {\r\n"
        "    \"enabled\": %u,\r\n"
        "    \"odr_hz\": %u\r\n"
        "  }\r\n"
        "}\r\n",
        (unsigned int)cfg.lsm6dsox.enabled,
        (unsigned int)cfg.lsm6dsox.range,
        (unsigned int)cfg.lsm6dsox.range2,
        (unsigned int)cfg.lsm6dsox.odr_hz,
        (unsigned int)cfg.h3lis100dl.enabled,
        (unsigned int)cfg.h3lis100dl.range,
        (unsigned int)cfg.h3lis100dl.odr_hz,
        (unsigned int)cfg.qma6100p.enabled,
        (unsigned int)cfg.qma6100p.range,
        (unsigned int)cfg.qma6100p.odr_hz,
        (unsigned int)cfg.es8311.enabled,
        (unsigned long)cfg.es8311.sample_rate_hz,
        (unsigned int)cfg.es8311.bits,
        (unsigned int)cfg.es8311.gain_db,
        (unsigned int)cfg.aht20.enabled,
        (unsigned int)cfg.aht20.odr_hz,
        (unsigned int)cfg.lis2mdl.enabled,
        (unsigned int)cfg.lis2mdl.odr_hz);

    if ((line_len < 0) || ((size_t)line_len >= sizeof(s_line)))
        return FR_INT_ERR;

    (void)snprintf(path, sizeof(path), "%s/CONFIG.JSN", dir);

    r = f_open(&file, path, FA_CREATE_ALWAYS | FA_WRITE);
    if (r != FR_OK)
    {
        printf("[DevCfg] 创建 %s 失败: %s (%d)\r\n",
               path, FatFs_SD_ResultToString(r), (int)r);
        return r;
    }

    r = write_exact(&file, s_line, (UINT)line_len);
    if (r == FR_OK) r = f_sync(&file);
    (void)f_close(&file);

    if (r == FR_OK)
        printf("[DevCfg] 配置快照已写入: %s\r\n", path);
    else
        printf("[DevCfg] 快照写入失败: %s (%d)\r\n",
               FatFs_SD_ResultToString(r), (int)r);

    return r;
}
