#ifndef UX_DEVICE_DESCRIPTORS_H
#define UX_DEVICE_DESCRIPTORS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "ux_api.h"
#include "ux_stm32_config.h"

#define USBD_MAX_NUM_CONFIGURATION            1U
#define USBD_MAX_SUPPORTED_CLASS              1U
#define USBD_MAX_CLASS_ENDPOINTS              3U
#define USBD_MAX_CLASS_INTERFACES             2U
#define USBD_CDC_ACM_CLASS_ACTIVATED          1U
#define USBD_CONFIG_MAXPOWER                  0U
#define USBD_COMPOSITE_USE_IAD                1U
#define USBD_FRAMEWORK_MAX_DESC_SZ            200U
#define USBD_STRING_FRAMEWORK_MAX_LENGTH      256U
#define LANGUAGE_ID_MAX_LENGTH                2U

#define USBD_VID                              0x0483U
#define USBD_PID                              0x5740U
#define USBD_LANGID_STRING                    0x0409U
#define USBD_MANUFACTURER_STRING              "STMicroelectronics"
#define USBD_PRODUCT_STRING                   "STM32U5 USBX CDC ACM"
#define USBD_SERIAL_NUMBER                    "SENSOR001"

#define USB_DESC_TYPE_INTERFACE               0x04U
#define USB_DESC_TYPE_ENDPOINT                0x05U
#define USB_DESC_TYPE_CONFIGURATION           0x02U
#define USB_DESC_TYPE_IAD                     0x0BU

#define USBD_EP_TYPE_BULK                     0x02U
#define USBD_EP_TYPE_INTR                     0x03U

#define USBD_FULL_SPEED                       0x00U
#define USBD_HIGH_SPEED                       0x01U

#define USB_BCDUSB                            0x0200U
#define USBD_IDX_MFC_STR                      0x01U
#define USBD_IDX_PRODUCT_STR                  0x02U
#define USBD_IDX_SERIAL_STR                   0x03U
#define USBD_MAX_EP0_SIZE                     64U

#define USBD_CDCACM_EPINCMD_ADDR              0x82U
#define USBD_CDCACM_EPINCMD_FS_MPS            8U
#define USBD_CDCACM_EPIN_ADDR                 0x81U
#define USBD_CDCACM_EPOUT_ADDR                0x01U
#define USBD_CDCACM_EPIN_FS_MPS               64U
#define USBD_CDCACM_EPOUT_FS_MPS              64U
#define USBD_CDCACM_EPINCMD_FS_BINTERVAL      5U

typedef enum
{
  CLASS_TYPE_NONE = 0,
  CLASS_TYPE_CDC_ACM = 2
} USBD_CompositeClassTypeDef;

typedef struct
{
  uint8_t add;
  uint8_t type;
  uint16_t size;
  uint8_t is_used;
} USBD_EPTypeDef;

typedef struct
{
  USBD_CompositeClassTypeDef ClassType;
  uint32_t ClassId;
  uint8_t InterfaceType;
  uint32_t Active;
  uint32_t NumEps;
  uint32_t NumIf;
  USBD_EPTypeDef Eps[USBD_MAX_CLASS_ENDPOINTS];
  uint8_t Ifs[USBD_MAX_CLASS_INTERFACES];
} USBD_CompositeElementTypeDef;

typedef struct _USBD_DevClassHandleTypeDef
{
  uint8_t Speed;
  uint32_t classId;
  uint32_t NumClasses;
  USBD_CompositeElementTypeDef tclasslist[USBD_MAX_SUPPORTED_CLASS];
  uint32_t CurrDevDescSz;
  uint32_t CurrConfDescSz;
} USBD_DevClassHandleTypeDef;

typedef struct
{
  uint8_t bLength;
  uint8_t bDescriptorType;
  uint16_t bcdUSB;
  uint8_t bDeviceClass;
  uint8_t bDeviceSubClass;
  uint8_t bDeviceProtocol;
  uint8_t bMaxPacketSize;
  uint16_t idVendor;
  uint16_t idProduct;
  uint16_t bcdDevice;
  uint8_t iManufacturer;
  uint8_t iProduct;
  uint8_t iSerialNumber;
  uint8_t bNumConfigurations;
} __PACKED USBD_DeviceDescTypedef;

typedef struct
{
  uint8_t bLength;
  uint8_t bDescriptorType;
  uint8_t bFirstInterface;
  uint8_t bInterfaceCount;
  uint8_t bFunctionClass;
  uint8_t bFunctionSubClass;
  uint8_t bFunctionProtocol;
  uint8_t iFunction;
} __PACKED USBD_IadDescTypedef;

typedef struct
{
  uint8_t bLength;
  uint8_t bDescriptorType;
  uint8_t bInterfaceNumber;
  uint8_t bAlternateSetting;
  uint8_t bNumEndpoints;
  uint8_t bInterfaceClass;
  uint8_t bInterfaceSubClass;
  uint8_t bInterfaceProtocol;
  uint8_t iInterface;
} __PACKED USBD_IfDescTypedef;

typedef struct
{
  uint8_t bLength;
  uint8_t bDescriptorType;
  uint8_t bEndpointAddress;
  uint8_t bmAttributes;
  uint16_t wMaxPacketSize;
  uint8_t bInterval;
} __PACKED USBD_EpDescTypedef;

typedef struct
{
  uint8_t bLength;
  uint8_t bDescriptorType;
  uint16_t wDescriptorLength;
  uint8_t bNumInterfaces;
  uint8_t bConfigurationValue;
  uint8_t iConfiguration;
  uint8_t bmAttributes;
  uint8_t bMaxPower;
} __PACKED USBD_ConfigDescTypedef;

typedef struct
{
  uint8_t bLength;
  uint8_t bDescriptorType;
  uint8_t bDescriptorSubtype;
  uint16_t bcdCDC;
} __PACKED USBD_CDCHeaderFuncDescTypedef;

typedef struct
{
  uint8_t bLength;
  uint8_t bDescriptorType;
  uint8_t bDescriptorSubtype;
  uint8_t bmCapabilities;
  uint8_t bDataInterface;
} __PACKED USBD_CDCCallMgmFuncDescTypedef;

typedef struct
{
  uint8_t bLength;
  uint8_t bDescriptorType;
  uint8_t bDescriptorSubtype;
  uint8_t bmCapabilities;
} __PACKED USBD_CDCACMFuncDescTypedef;

typedef struct
{
  uint8_t bLength;
  uint8_t bDescriptorType;
  uint8_t bDescriptorSubtype;
  uint8_t bMasterInterface;
  uint8_t bSlaveInterface;
} __PACKED USBD_CDCUnionFuncDescTypedef;

uint8_t *USBD_Get_Device_Framework_Speed(uint8_t Speed, ULONG *Length);
uint8_t *USBD_Get_String_Framework(ULONG *Length);
uint8_t *USBD_Get_Language_Id_Framework(ULONG *Length);
uint16_t USBD_Get_Interface_Number(uint8_t class_type, uint8_t interface_type);
uint16_t USBD_Get_Configuration_Number(uint8_t class_type, uint8_t interface_type);

#ifdef __cplusplus
}
#endif

#endif /* UX_DEVICE_DESCRIPTORS_H */
