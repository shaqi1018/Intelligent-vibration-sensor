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
#define DEV_CFG_READ_BUF_SZ    2048U     /* 配置文件读取缓冲区字节数 */
#define DEV_CFG_OBJ_BUF_SZ     512U      /* 子对象提取缓冲区字节数   */

/* 上电写入的默认模板（与解析器所支持的字段保持一致）
 * _doc / _options_* 等以 _ 开头的键仅作文档用途，解析器会自动忽略。 */
static const char kCfgTemplate[] =
"{\r\n"
"  \"_doc\": \"Sensor Box device config. On power-up the MCU reads this file and applies range/ODR to sensors.\",\r\n"
"  \"_doc_units\": \"range_g = +/- g | range_dps = +/- deg/s | odr_hz = output data rate (Hz)\",\r\n"
"  \"_doc_unknown_keys\": \"Any key starting with _ is documentation only and silently ignored by the parser.\",\r\n"
"\r\n"
"  \"sample_rate_hz\": 1000,\r\n"
"  \"sink\": \"USB\",\r\n"
"  \"storage_mode\": \"LINEAR\",\r\n"
"  \"trigger_mode\": \"NONE\",\r\n"
"  \"trigger_delay_ms\": 0,\r\n"
"  \"duration_ms\": 0,\r\n"
"  \"sd_ring_max_bytes\": 0,\r\n"
"\r\n"
"  \"lsm6dsox\": {\r\n"
"    \"_doc\": \"ST 6-axis IMU (ACC + GYR + TEMP) on SPI1 (dedicated bus)\",\r\n"
"    \"enabled\": 1,\r\n"
"    \"range_g\": 4,\r\n"
"    \"_options_range_g\": [2, 4, 8, 16],\r\n"
"    \"range_dps\": 2000,\r\n"
"    \"_options_range_dps\": [250, 500, 1000, 2000],\r\n"
"    \"odr_hz\": 1666,\r\n"
"    \"_options_odr_hz\": [12, 26, 52, 104, 208, 416, 833, 1666, 3332, 6664]\r\n"
"  },\r\n"
"\r\n"
"  \"h3lis100dl\": {\r\n"
"    \"_doc\": \"ST high-g accelerometer (+/-100 g fixed) on SPI2 (shared bus)\",\r\n"
"    \"enabled\": 1,\r\n"
"    \"range_g\": 100,\r\n"
"    \"_options_range_g\": [100],\r\n"
"    \"odr_hz\": 400,\r\n"
"    \"_options_odr_hz_normal\": [50, 100, 400],\r\n"
"    \"_options_odr_hz_lowpower\": [1, 2, 5, 10]\r\n"
"  },\r\n"
"\r\n"
"  \"qma6100p\": {\r\n"
"    \"_doc\": \"QST 3-axis accelerometer (up to 1600 Hz, +/-32 g) on SPI2 (shared bus)\",\r\n"
"    \"enabled\": 1,\r\n"
"    \"range_g\": 4,\r\n"
"    \"_options_range_g\": [2, 4, 8, 16, 32],\r\n"
"    \"odr_hz\": 100,\r\n"
"    \"_options_odr_hz\": [100, 200, 400, 800, 1600],\r\n"
"    \"_options_odr_hz_lowpower\": [12, 25, 50]\r\n"
"  },\r\n"
"\r\n"
"  \"es8311\": {\r\n"
"    \"_doc\": \"ES8311 mono microphone codec on I2C2 (shared with RTC), I2S/SAI capture\",\r\n"
"    \"enabled\": 1,\r\n"
"    \"sample_rate_hz\": 16000,\r\n"
"    \"_options_sample_rate_hz\": [8000, 16000, 48000],\r\n"
"    \"bits\": 16,\r\n"
"    \"gain_db\": 24,\r\n"
"    \"_doc_gain_db\": \"mic PGA gain 0..42 dB (approx 3 dB steps)\"\r\n"
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

static void DeviceCfg_ParseAndApply(const char *buf)
{
    AcqConfig_t  cfg;
    const char  *p;
    uint32_t     v;
    char         s[16];
    static char  s_obj[DEV_CFG_OBJ_BUF_SZ]; /* 子对象缓冲区，static 避免栈压力 */

    /* 以当前运行时配置为基础，仅覆盖文件中出现的字段 */
    AcqConfig_GetCopy(&cfg);

    /* ---- 顶层标量字段 ---- */

    p = json_find_value(buf, "sample_rate_hz");
    if (json_parse_uint(p, &v)) cfg.sample_rate_hz = v;

    p = json_find_value(buf, "sink");
    if (p != NULL && json_parse_string(p, s, sizeof(s)))
    {
        if      (strcmp(s, "USB")  == 0) cfg.sink_mask = ACQ_SINK_USB;
        else if (strcmp(s, "SD")   == 0) cfg.sink_mask = ACQ_SINK_SD;
        else if (strcmp(s, "BOTH") == 0) cfg.sink_mask = ACQ_SINK_BOTH;
    }

    p = json_find_value(buf, "storage_mode");
    if (p != NULL && json_parse_string(p, s, sizeof(s)))
    {
        if      (strcmp(s, "LINEAR") == 0) cfg.storage_mode = ACQ_STORAGE_LINEAR;
        else if (strcmp(s, "RING")   == 0) cfg.storage_mode = ACQ_STORAGE_RING;
    }

    p = json_find_value(buf, "trigger_mode");
    if (p != NULL && json_parse_string(p, s, sizeof(s)))
    {
        if      (strcmp(s, "NONE")     == 0) cfg.trigger_mode = ACQ_TRIGGER_NONE;
        else if (strcmp(s, "EXTERNAL") == 0) cfg.trigger_mode = ACQ_TRIGGER_EXTERNAL;
        else if (strcmp(s, "TIMER")    == 0) cfg.trigger_mode = ACQ_TRIGGER_TIMER;
    }

    p = json_find_value(buf, "trigger_delay_ms");
    if (json_parse_uint(p, &v)) cfg.trigger_delay_ms = v;

    p = json_find_value(buf, "duration_ms");
    if (json_parse_uint(p, &v)) cfg.duration_ms = v;

    p = json_find_value(buf, "sd_ring_max_bytes");
    if (json_parse_uint(p, &v)) cfg.sd_ring_max_bytes = v;

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

    if (AcqConfig_Set(&cfg) == 0)
    {
        printf("[DevCfg] 配置已应用: %lu Hz, sink=0x%02X\r\n",
               (unsigned long)cfg.sample_rate_hz,
               (unsigned int)cfg.sink_mask);
    }
    else
    {
        printf("[DevCfg] 配置应用被拒绝（范围校验未通过），使用默认值\r\n");
    }
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

    DeviceCfg_ParseAndApply(s_buf);

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
    const char *storage_str;
    const char *trigger_str;
    static char s_line[2048];
    int         line_len;

    AcqConfig_GetCopy(&cfg);

    r = FatFs_SD_Mount();
    if (r != FR_OK)
    {
        printf("[DevCfg] 写回: SD挂载失败 %s (%d)\r\n",
               FatFs_SD_ResultToString(r), (int)r);
        return r;
    }

    switch (cfg.sink_mask)
    {
        case ACQ_SINK_SD:   sink_name = "SD";   break;
        case ACQ_SINK_BOTH: sink_name = "BOTH"; break;
        default:            sink_name = "USB";  break;
    }

    switch (cfg.storage_mode)
    {
        case ACQ_STORAGE_RING: storage_str = "RING"; break;
        default:               storage_str = "LINEAR"; break;
    }

    switch (cfg.trigger_mode)
    {
        case ACQ_TRIGGER_EXTERNAL: trigger_str = "EXTERNAL"; break;
        case ACQ_TRIGGER_TIMER:    trigger_str = "TIMER";    break;
        default:                   trigger_str = "NONE";     break;
    }

    line_len = snprintf(
        s_line, sizeof(s_line),
        "{\r\n"
        "  \"_doc\": \"Sensor Box device config. On power-up the MCU reads this file and applies range/ODR to sensors.\",\r\n"
        "  \"_doc_units\": \"range_g = +/- g | range_dps = +/- deg/s | odr_hz = output data rate (Hz)\",\r\n"
        "  \"_doc_unknown_keys\": \"Any key starting with _ is documentation only and silently ignored by the parser.\",\r\n"
        "\r\n"
        "  \"sample_rate_hz\": %lu,\r\n"
        "  \"sink\": \"%s\",\r\n"
        "  \"storage_mode\": \"%s\",\r\n"
        "  \"trigger_mode\": \"%s\",\r\n"
        "  \"trigger_delay_ms\": %lu,\r\n"
        "  \"duration_ms\": %lu,\r\n"
        "  \"sd_ring_max_bytes\": %lu,\r\n"
        "\r\n"
        "  \"lsm6dsox\": {\r\n"
        "    \"_doc\": \"ST 6-axis IMU (ACC + GYR + TEMP) on SPI1 (dedicated bus)\",\r\n"
        "    \"enabled\": %u,\r\n"
        "    \"range_g\": %u,\r\n"
        "    \"_options_range_g\": [2, 4, 8, 16],\r\n"
        "    \"range_dps\": %u,\r\n"
        "    \"_options_range_dps\": [250, 500, 1000, 2000],\r\n"
        "    \"odr_hz\": %u,\r\n"
        "    \"_options_odr_hz\": [12, 26, 52, 104, 208, 416, 833, 1666, 3332, 6664]\r\n"
        "  },\r\n"
        "\r\n"
        "  \"h3lis100dl\": {\r\n"
        "    \"_doc\": \"ST high-g accelerometer (+/-100 g fixed) on SPI2 (shared bus)\",\r\n"
        "    \"enabled\": %u,\r\n"
        "    \"range_g\": %u,\r\n"
        "    \"_options_range_g\": [100],\r\n"
        "    \"odr_hz\": %u,\r\n"
        "    \"_options_odr_hz_normal\": [50, 100, 400],\r\n"
        "    \"_options_odr_hz_lowpower\": [1, 2, 5, 10]\r\n"
        "  },\r\n"
        "\r\n"
        "  \"qma6100p\": {\r\n"
        "    \"_doc\": \"QST 3-axis accelerometer (up to 1600 Hz, +/-32 g) on SPI2 (shared bus)\",\r\n"
        "    \"enabled\": %u,\r\n"
        "    \"range_g\": %u,\r\n"
        "    \"_options_range_g\": [2, 4, 8, 16, 32],\r\n"
        "    \"odr_hz\": %u,\r\n"
        "    \"_options_odr_hz\": [100, 200, 400, 800, 1600],\r\n"
        "    \"_options_odr_hz_lowpower\": [12, 25, 50]\r\n"
        "  },\r\n"
        "\r\n"
        "  \"es8311\": {\r\n"
        "    \"_doc\": \"ES8311 mono microphone codec on I2C2 (shared with RTC), I2S/SAI capture\",\r\n"
        "    \"enabled\": %u,\r\n"
        "    \"sample_rate_hz\": %lu,\r\n"
        "    \"_options_sample_rate_hz\": [8000, 16000, 48000],\r\n"
        "    \"bits\": %u,\r\n"
        "    \"gain_db\": %u,\r\n"
        "    \"_doc_gain_db\": \"mic PGA gain 0..42 dB (approx 3 dB steps)\"\r\n"
        "  }\r\n"
        "}\r\n",
        (unsigned long)cfg.sample_rate_hz,
        sink_name,
        storage_str,
        trigger_str,
        (unsigned long)cfg.trigger_delay_ms,
        (unsigned long)cfg.duration_ms,
        (unsigned long)cfg.sd_ring_max_bytes,
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
        (unsigned int)cfg.es8311.gain_db);

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
    static char s_line[768];
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
        (unsigned int)cfg.es8311.gain_db);

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
