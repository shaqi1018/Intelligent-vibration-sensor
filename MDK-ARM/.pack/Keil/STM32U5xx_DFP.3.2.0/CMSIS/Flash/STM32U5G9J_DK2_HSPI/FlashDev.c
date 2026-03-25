/***********************************************************************/
/*  This file is part of the ARM Toolchain package                     */
/*  Copyright (c) 2020 Keil - An ARM Company. All rights reserved.     */
/***********************************************************************/
/*                                                                     */
/*  FlashDev.c:  Device Description for ST STM32U5G9J-DK2 HSPI Flash  */
/*                                                                     */
/***********************************************************************/

#include "..\FlashOS.h" // FlashOS Structures

#ifdef FLASH_MEM

struct FlashDevice const FlashDevice = {
    FLASH_DRV_VERS,               // Driver Version, do not modify!
    "MX66LM1G45G_STM32U5G9J-DK2", // Device Name
    EXTSPI,                       // Device Type
    0xA0000000,                   // Device Start Address
    0x08000000,                   // Device Size in Bytes (64MB)
    0x00002000,                   // Programming Page Size 8192 Bytes
    0x00,                         // Reserved, must be 0
    0xFF,                         // Initial Content of Erased Memory
    10000,                        // Program Page Timeout 100 mSec
    6000,                         // Erase Sector Timeout 6000 mSec

    // Specify Size and Address of Sectors
    {
        {0x10000, 0x000000}, // Sector Size  64kB, Sector Num : 2048
        {SECTOR_END}}};

#endif // FLASH_MEM