/**
  * USB descriptors for the MSC-mode device.
  * VID = STMicroelectronics (0x0483), PID picked from the unused MSC range.
  */

#include "usbd_desc.h"
#include "usbd_core.h"
#include "usbd_conf.h"

#define USBD_VID                      0x0483U
#define USBD_PID                      0x5721U  /* WCID Bulk sensor streaming */
#define USBD_LANGID_STRING            0x0409U  /* English (US) */
#define USBD_MANUFACTURER_STRING      "STMicroelectronics"
#define USBD_PRODUCT_STRING           "Sensor WCID Bulk"
#define USBD_CONFIGURATION_STRING     "WCID Config"
#define USBD_INTERFACE_STRING         "MSC Interface"

static uint8_t *MSC_DeviceDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
static uint8_t *MSC_LangIDStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
static uint8_t *MSC_ManufacturerStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
static uint8_t *MSC_ProductStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
static uint8_t *MSC_SerialStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
static uint8_t *MSC_ConfigStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
static uint8_t *MSC_InterfaceStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);

USBD_DescriptorsTypeDef MSC_Desc = {
  MSC_DeviceDescriptor,
  MSC_LangIDStrDescriptor,
  MSC_ManufacturerStrDescriptor,
  MSC_ProductStrDescriptor,
  MSC_SerialStrDescriptor,
  MSC_ConfigStrDescriptor,
  MSC_InterfaceStrDescriptor,
};

__ALIGN_BEGIN static uint8_t USBD_DeviceDesc[USB_LEN_DEV_DESC] __ALIGN_END = {
  0x12,                       /* bLength */
  USB_DESC_TYPE_DEVICE,       /* bDescriptorType */
  0x00, 0x02,                 /* bcdUSB = 2.00 */
  0x00,                       /* bDeviceClass */
  0x00,                       /* bDeviceSubClass */
  0x00,                       /* bDeviceProtocol */
  USB_MAX_EP0_SIZE,           /* bMaxPacketSize */
  LOBYTE(USBD_VID), HIBYTE(USBD_VID),
  LOBYTE(USBD_PID), HIBYTE(USBD_PID),
  0x00, 0x02,                 /* bcdDevice = 2.00 */
  USBD_IDX_MFC_STR,
  USBD_IDX_PRODUCT_STR,
  USBD_IDX_SERIAL_STR,
  USBD_MAX_NUM_CONFIGURATION
};

__ALIGN_BEGIN static uint8_t USBD_LangIDDesc[USB_LEN_LANGID_STR_DESC] __ALIGN_END = {
  USB_LEN_LANGID_STR_DESC,
  USB_DESC_TYPE_STRING,
  LOBYTE(USBD_LANGID_STRING),
  HIBYTE(USBD_LANGID_STRING),
};

__ALIGN_BEGIN static uint8_t USBD_StrDesc[USBD_MAX_STR_DESC_SIZ] __ALIGN_END;

/* Encode a 96-bit unique device serial (UID 0x0BFA0700 on U5) into 24-char hex string. */
static char g_serial_str[25];

static void Get_SerialNum(void)
{
  uint32_t deviceserial0 = *(uint32_t *)0x0BFA0700U;
  uint32_t deviceserial1 = *(uint32_t *)0x0BFA0704U;
  uint32_t deviceserial2 = *(uint32_t *)0x0BFA0708U;
  static const char hex[] = "0123456789ABCDEF";

  deviceserial0 += deviceserial2;
  for (int i = 0; i < 8; i++)
  {
    g_serial_str[7 - i]  = hex[(deviceserial0 >> (i * 4)) & 0xFU];
    g_serial_str[15 - i] = hex[(deviceserial1 >> (i * 4)) & 0xFU];
    g_serial_str[23 - i] = hex[(deviceserial2 >> (i * 4)) & 0xFU];
  }
  g_serial_str[24] = '\0';
}

static uint8_t *MSC_DeviceDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
  UNUSED(speed);
  *length = sizeof(USBD_DeviceDesc);
  return USBD_DeviceDesc;
}

static uint8_t *MSC_LangIDStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
  UNUSED(speed);
  *length = sizeof(USBD_LangIDDesc);
  return USBD_LangIDDesc;
}

static uint8_t *MSC_ManufacturerStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
  UNUSED(speed);
  USBD_GetString((uint8_t *)USBD_MANUFACTURER_STRING, USBD_StrDesc, length);
  return USBD_StrDesc;
}

static uint8_t *MSC_ProductStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
  UNUSED(speed);
  USBD_GetString((uint8_t *)USBD_PRODUCT_STRING, USBD_StrDesc, length);
  return USBD_StrDesc;
}

static uint8_t *MSC_SerialStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
  UNUSED(speed);
  Get_SerialNum();
  USBD_GetString((uint8_t *)g_serial_str, USBD_StrDesc, length);
  return USBD_StrDesc;
}

static uint8_t *MSC_ConfigStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
  UNUSED(speed);
  USBD_GetString((uint8_t *)USBD_CONFIGURATION_STRING, USBD_StrDesc, length);
  return USBD_StrDesc;
}

static uint8_t *MSC_InterfaceStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
  UNUSED(speed);
  USBD_GetString((uint8_t *)USBD_INTERFACE_STRING, USBD_StrDesc, length);
  return USBD_StrDesc;
}
