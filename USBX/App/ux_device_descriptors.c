#include "ux_device_descriptors.h"

#include <string.h>

static USBD_DevClassHandleTypeDef g_usbd_device_fs;
static USBD_DevClassHandleTypeDef g_usbd_device_hs;

static uint8_t g_user_class_instance[USBD_MAX_SUPPORTED_CLASS] =
{
  CLASS_TYPE_CDC_ACM
};

__ALIGN_BEGIN static uint8_t g_dev_framework_desc_fs[USBD_FRAMEWORK_MAX_DESC_SZ] __ALIGN_END;
__ALIGN_BEGIN static uint8_t g_dev_framework_desc_hs[USBD_FRAMEWORK_MAX_DESC_SZ] __ALIGN_END;
__ALIGN_BEGIN static UCHAR g_string_framework[USBD_STRING_FRAMEWORK_MAX_LENGTH] __ALIGN_END;
__ALIGN_BEGIN static UCHAR g_language_id_framework[LANGUAGE_ID_MAX_LENGTH] __ALIGN_END;

static void USBD_Desc_GetString(uint8_t *desc, uint8_t *unicode, uint16_t *len);
static uint8_t USBD_Desc_GetLen(const uint8_t *buf);
static uint8_t *USBD_Device_Framework_Builder(USBD_DevClassHandleTypeDef *pdev,
                                              uint8_t *pDevFrameWorkDesc,
                                              uint8_t *UserClassInstance,
                                              uint8_t Speed);
static void USBD_FrameWork_AddConfDesc(uint8_t *conf, uint32_t *pSze);
static void USBD_FrameWork_AssignEp(USBD_DevClassHandleTypeDef *pdev,
                                    uint8_t add,
                                    uint8_t type,
                                    uint16_t size);
static void USBD_FrameWork_CDCDesc(USBD_DevClassHandleTypeDef *pdev,
                                   uint8_t *pConf,
                                   uint32_t *Sze);

uint8_t *USBD_Get_Device_Framework_Speed(uint8_t Speed, ULONG *Length)
{
  uint8_t *pFrameWork;

  if (Speed == USBD_FULL_SPEED)
  {
    pFrameWork = USBD_Device_Framework_Builder(&g_usbd_device_fs,
                                               g_dev_framework_desc_fs,
                                               g_user_class_instance,
                                               Speed);
    *Length = (ULONG)(g_usbd_device_fs.CurrDevDescSz + g_usbd_device_fs.CurrConfDescSz);
  }
  else
  {
    pFrameWork = USBD_Device_Framework_Builder(&g_usbd_device_hs,
                                               g_dev_framework_desc_hs,
                                               g_user_class_instance,
                                               Speed);
    *Length = (ULONG)(g_usbd_device_hs.CurrDevDescSz + g_usbd_device_hs.CurrConfDescSz);
  }

  return pFrameWork;
}

uint8_t *USBD_Get_String_Framework(ULONG *Length)
{
  uint16_t len;
  uint8_t count = 0U;

  memset(g_string_framework, 0, sizeof(g_string_framework));

  g_string_framework[count++] = (uint8_t)(USBD_LANGID_STRING & 0xFFU);
  g_string_framework[count++] = (uint8_t)(USBD_LANGID_STRING >> 8);
  g_string_framework[count++] = USBD_IDX_MFC_STR;
  USBD_Desc_GetString((uint8_t *)USBD_MANUFACTURER_STRING, g_string_framework + count, &len);

  count = (uint8_t)(count + len + 1U);
  g_string_framework[count++] = (uint8_t)(USBD_LANGID_STRING & 0xFFU);
  g_string_framework[count++] = (uint8_t)(USBD_LANGID_STRING >> 8);
  g_string_framework[count++] = USBD_IDX_PRODUCT_STR;
  USBD_Desc_GetString((uint8_t *)USBD_PRODUCT_STRING, g_string_framework + count, &len);

  count = (uint8_t)(count + len + 1U);
  g_string_framework[count++] = (uint8_t)(USBD_LANGID_STRING & 0xFFU);
  g_string_framework[count++] = (uint8_t)(USBD_LANGID_STRING >> 8);
  g_string_framework[count++] = USBD_IDX_SERIAL_STR;
  USBD_Desc_GetString((uint8_t *)USBD_SERIAL_NUMBER, g_string_framework + count, &len);

  *Length = (ULONG)count + (ULONG)len;
  return g_string_framework;
}

uint8_t *USBD_Get_Language_Id_Framework(ULONG *Length)
{
  g_language_id_framework[0] = (uint8_t)(USBD_LANGID_STRING & 0xFFU);
  g_language_id_framework[1] = (uint8_t)(USBD_LANGID_STRING >> 8);
  *Length = LANGUAGE_ID_MAX_LENGTH;
  return g_language_id_framework;
}

uint16_t USBD_Get_Interface_Number(uint8_t class_type, uint8_t interface_type)
{
  if ((class_type == (uint8_t)CLASS_TYPE_CDC_ACM) && (interface_type == 0U))
  {
    return g_usbd_device_fs.tclasslist[0].Ifs[0];
  }

  return 0U;
}

uint16_t USBD_Get_Configuration_Number(uint8_t class_type, uint8_t interface_type)
{
  (void)class_type;
  (void)interface_type;
  return 1U;
}

static void USBD_Desc_GetString(uint8_t *desc, uint8_t *unicode, uint16_t *len)
{
  uint8_t idx = 0U;
  uint8_t *pdesc = desc;

  *len = USBD_Desc_GetLen(pdesc);
  unicode[idx++] = (uint8_t)(*len);

  while (*pdesc != '\0')
  {
    unicode[idx++] = *pdesc++;
  }
}

static uint8_t USBD_Desc_GetLen(const uint8_t *buf)
{
  uint8_t len = 0U;

  while (buf[len] != '\0')
  {
    len++;
  }

  return len;
}

static uint8_t *USBD_Device_Framework_Builder(USBD_DevClassHandleTypeDef *pdev,
                                              uint8_t *pDevFrameWorkDesc,
                                              uint8_t *UserClassInstance,
                                              uint8_t Speed)
{
  USBD_DeviceDescTypedef *pDevDesc;

  memset(pdev, 0, sizeof(*pdev));
  memset(pDevFrameWorkDesc, 0, USBD_FRAMEWORK_MAX_DESC_SZ);

  pDevDesc = (USBD_DeviceDescTypedef *)pDevFrameWorkDesc;
  pDevDesc->bLength = (uint8_t)sizeof(USBD_DeviceDescTypedef);
  pDevDesc->bDescriptorType = UX_DEVICE_DESCRIPTOR_ITEM;
  pDevDesc->bcdUSB = USB_BCDUSB;
  pDevDesc->bDeviceClass = 0x02U;
  pDevDesc->bDeviceSubClass = 0x02U;
  pDevDesc->bDeviceProtocol = 0x00U;
  pDevDesc->bMaxPacketSize = USBD_MAX_EP0_SIZE;
  pDevDesc->idVendor = USBD_VID;
  pDevDesc->idProduct = USBD_PID;
  pDevDesc->bcdDevice = 0x0200U;
  pDevDesc->iManufacturer = USBD_IDX_MFC_STR;
  pDevDesc->iProduct = USBD_IDX_PRODUCT_STR;
  pDevDesc->iSerialNumber = USBD_IDX_SERIAL_STR;
  pDevDesc->bNumConfigurations = USBD_MAX_NUM_CONFIGURATION;
  pdev->CurrDevDescSz = (uint32_t)sizeof(USBD_DeviceDescTypedef);
  pdev->Speed = Speed;

  if (UserClassInstance[0] == CLASS_TYPE_CDC_ACM)
  {
    pdev->classId = 0U;
    pdev->NumClasses = 1U;
    pdev->tclasslist[0].Active = 1U;
    pdev->tclasslist[0].ClassId = 0U;
    pdev->tclasslist[0].ClassType = CLASS_TYPE_CDC_ACM;
    pdev->tclasslist[0].NumIf = 2U;
    pdev->tclasslist[0].Ifs[0] = 0U;
    pdev->tclasslist[0].Ifs[1] = 1U;
    pdev->tclasslist[0].NumEps = 3U;
    USBD_FrameWork_AssignEp(pdev, USBD_CDCACM_EPOUT_ADDR, USBD_EP_TYPE_BULK, USBD_CDCACM_EPOUT_FS_MPS);
    USBD_FrameWork_AssignEp(pdev, USBD_CDCACM_EPIN_ADDR, USBD_EP_TYPE_BULK, USBD_CDCACM_EPIN_FS_MPS);
    USBD_FrameWork_AssignEp(pdev, USBD_CDCACM_EPINCMD_ADDR, USBD_EP_TYPE_INTR, USBD_CDCACM_EPINCMD_FS_MPS);
    USBD_FrameWork_AddConfDesc(pDevFrameWorkDesc + pdev->CurrDevDescSz, &pdev->CurrConfDescSz);
    USBD_FrameWork_CDCDesc(pdev, pDevFrameWorkDesc + pdev->CurrDevDescSz, &pdev->CurrConfDescSz);
  }

  return pDevFrameWorkDesc;
}

static void USBD_FrameWork_AddConfDesc(uint8_t *conf, uint32_t *pSze)
{
  USBD_ConfigDescTypedef *ptr = (USBD_ConfigDescTypedef *)conf;

  ptr->bLength = (uint8_t)sizeof(USBD_ConfigDescTypedef);
  ptr->bDescriptorType = USB_DESC_TYPE_CONFIGURATION;
  ptr->wDescriptorLength = 0U;
  ptr->bNumInterfaces = 0U;
  ptr->bConfigurationValue = 1U;
  ptr->iConfiguration = 0U;
  ptr->bmAttributes = 0xC0U;
  ptr->bMaxPower = USBD_CONFIG_MAXPOWER;
  *pSze += (uint32_t)sizeof(USBD_ConfigDescTypedef);
}

static void USBD_FrameWork_AssignEp(USBD_DevClassHandleTypeDef *pdev,
                                    uint8_t add,
                                    uint8_t type,
                                    uint16_t size)
{
  uint32_t idx;

  for (idx = 0U; idx < pdev->tclasslist[pdev->classId].NumEps; idx++)
  {
    if (pdev->tclasslist[pdev->classId].Eps[idx].is_used == 0U)
    {
      pdev->tclasslist[pdev->classId].Eps[idx].add = add;
      pdev->tclasslist[pdev->classId].Eps[idx].type = type;
      pdev->tclasslist[pdev->classId].Eps[idx].size = size;
      pdev->tclasslist[pdev->classId].Eps[idx].is_used = 1U;
      break;
    }
  }
}

static void USBD_FrameWork_CDCDesc(USBD_DevClassHandleTypeDef *pdev,
                                   uint8_t *pConf,
                                   uint32_t *Sze)
{
  USBD_IadDescTypedef *pIadDesc;
  USBD_IfDescTypedef *pIfDesc;
  USBD_CDCHeaderFuncDescTypedef *pHeadDesc;
  USBD_CDCCallMgmFuncDescTypedef *pCallMgmDesc;
  USBD_CDCACMFuncDescTypedef *pACMDesc;
  USBD_CDCUnionFuncDescTypedef *pUnionDesc;
  USBD_EpDescTypedef *pEpDesc;

  pIadDesc = (USBD_IadDescTypedef *)(pConf + *Sze);
  pIadDesc->bLength = (uint8_t)sizeof(USBD_IadDescTypedef);
  pIadDesc->bDescriptorType = USB_DESC_TYPE_IAD;
  pIadDesc->bFirstInterface = pdev->tclasslist[0].Ifs[0];
  pIadDesc->bInterfaceCount = 2U;
  pIadDesc->bFunctionClass = 0x02U;
  pIadDesc->bFunctionSubClass = 0x02U;
  pIadDesc->bFunctionProtocol = 0x01U;
  pIadDesc->iFunction = 0U;
  *Sze += (uint32_t)sizeof(USBD_IadDescTypedef);

  pIfDesc = (USBD_IfDescTypedef *)(pConf + *Sze);
  pIfDesc->bLength = (uint8_t)sizeof(USBD_IfDescTypedef);
  pIfDesc->bDescriptorType = USB_DESC_TYPE_INTERFACE;
  pIfDesc->bInterfaceNumber = pdev->tclasslist[0].Ifs[0];
  pIfDesc->bAlternateSetting = 0U;
  pIfDesc->bNumEndpoints = 1U;
  pIfDesc->bInterfaceClass = 0x02U;
  pIfDesc->bInterfaceSubClass = 0x02U;
  pIfDesc->bInterfaceProtocol = 0x01U;
  pIfDesc->iInterface = 0U;
  *Sze += (uint32_t)sizeof(USBD_IfDescTypedef);

  pHeadDesc = (USBD_CDCHeaderFuncDescTypedef *)(pConf + *Sze);
  pHeadDesc->bLength = 0x05U;
  pHeadDesc->bDescriptorType = 0x24U;
  pHeadDesc->bDescriptorSubtype = 0x00U;
  pHeadDesc->bcdCDC = 0x0110U;
  *Sze += 5U;

  pCallMgmDesc = (USBD_CDCCallMgmFuncDescTypedef *)(pConf + *Sze);
  pCallMgmDesc->bLength = 0x05U;
  pCallMgmDesc->bDescriptorType = 0x24U;
  pCallMgmDesc->bDescriptorSubtype = 0x01U;
  pCallMgmDesc->bmCapabilities = 0x00U;
  pCallMgmDesc->bDataInterface = pdev->tclasslist[0].Ifs[1];
  *Sze += 5U;

  pACMDesc = (USBD_CDCACMFuncDescTypedef *)(pConf + *Sze);
  pACMDesc->bLength = 0x04U;
  pACMDesc->bDescriptorType = 0x24U;
  pACMDesc->bDescriptorSubtype = 0x02U;
  pACMDesc->bmCapabilities = 0x02U;
  *Sze += 4U;

  pUnionDesc = (USBD_CDCUnionFuncDescTypedef *)(pConf + *Sze);
  pUnionDesc->bLength = 0x05U;
  pUnionDesc->bDescriptorType = 0x24U;
  pUnionDesc->bDescriptorSubtype = 0x06U;
  pUnionDesc->bMasterInterface = pdev->tclasslist[0].Ifs[0];
  pUnionDesc->bSlaveInterface = pdev->tclasslist[0].Ifs[1];
  *Sze += 5U;

  pEpDesc = (USBD_EpDescTypedef *)(pConf + *Sze);
  pEpDesc->bLength = (uint8_t)sizeof(USBD_EpDescTypedef);
  pEpDesc->bDescriptorType = USB_DESC_TYPE_ENDPOINT;
  pEpDesc->bEndpointAddress = USBD_CDCACM_EPINCMD_ADDR;
  pEpDesc->bmAttributes = USBD_EP_TYPE_INTR;
  pEpDesc->wMaxPacketSize = USBD_CDCACM_EPINCMD_FS_MPS;
  pEpDesc->bInterval = USBD_CDCACM_EPINCMD_FS_BINTERVAL;
  *Sze += (uint32_t)sizeof(USBD_EpDescTypedef);

  pIfDesc = (USBD_IfDescTypedef *)(pConf + *Sze);
  pIfDesc->bLength = (uint8_t)sizeof(USBD_IfDescTypedef);
  pIfDesc->bDescriptorType = USB_DESC_TYPE_INTERFACE;
  pIfDesc->bInterfaceNumber = pdev->tclasslist[0].Ifs[1];
  pIfDesc->bAlternateSetting = 0U;
  pIfDesc->bNumEndpoints = 2U;
  pIfDesc->bInterfaceClass = 0x0AU;
  pIfDesc->bInterfaceSubClass = 0x00U;
  pIfDesc->bInterfaceProtocol = 0x00U;
  pIfDesc->iInterface = 0U;
  *Sze += (uint32_t)sizeof(USBD_IfDescTypedef);

  pEpDesc = (USBD_EpDescTypedef *)(pConf + *Sze);
  pEpDesc->bLength = (uint8_t)sizeof(USBD_EpDescTypedef);
  pEpDesc->bDescriptorType = USB_DESC_TYPE_ENDPOINT;
  pEpDesc->bEndpointAddress = USBD_CDCACM_EPOUT_ADDR;
  pEpDesc->bmAttributes = USBD_EP_TYPE_BULK;
  pEpDesc->wMaxPacketSize = USBD_CDCACM_EPOUT_FS_MPS;
  pEpDesc->bInterval = 0U;
  *Sze += (uint32_t)sizeof(USBD_EpDescTypedef);

  pEpDesc = (USBD_EpDescTypedef *)(pConf + *Sze);
  pEpDesc->bLength = (uint8_t)sizeof(USBD_EpDescTypedef);
  pEpDesc->bDescriptorType = USB_DESC_TYPE_ENDPOINT;
  pEpDesc->bEndpointAddress = USBD_CDCACM_EPIN_ADDR;
  pEpDesc->bmAttributes = USBD_EP_TYPE_BULK;
  pEpDesc->wMaxPacketSize = USBD_CDCACM_EPIN_FS_MPS;
  pEpDesc->bInterval = 0U;
  *Sze += (uint32_t)sizeof(USBD_EpDescTypedef);

  ((USBD_ConfigDescTypedef *)pConf)->bNumInterfaces = 2U;
  ((USBD_ConfigDescTypedef *)pConf)->wDescriptorLength = (uint16_t)(*Sze);
}
