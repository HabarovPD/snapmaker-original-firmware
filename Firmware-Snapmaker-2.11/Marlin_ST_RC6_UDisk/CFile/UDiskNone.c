// Заглушки USB-флешки для чипов без USB OTG Host (STM32F103 / GD32F103).
// Заменяют usbh_usr.c: прошивка ведёт себя так, будто флешка не вставлена.
#include "includes.h"

#ifndef STM32F10X_CL

volatile bool DiskInserted = false;

void UDiskIdleProceess(void)
{
}

// как в usbh_usr.c при USB_OTG_Core.host.ConnSts == 0
char WaitUDisk(void)
{
	return (char)-2;
}

void UDiskRestart(void)
{
}

// диск 0 (UDISK) в FatFs всегда "не готов"; заменяют drivers/fatfs_usb.c
#include "fatfs_usb.h"

DSTATUS TM_FATFS_USB_disk_initialize(void)
{
	return STA_NOINIT | STA_NODISK;
}

DSTATUS TM_FATFS_USB_disk_status(void)
{
	return STA_NOINIT | STA_NODISK;
}

DRESULT TM_FATFS_USB_disk_read(BYTE* buff, DWORD sector, UINT count)
{
	return RES_NOTRDY;
}

DRESULT TM_FATFS_USB_disk_write(const BYTE* buff, DWORD sector, UINT count)
{
	return RES_NOTRDY;
}

DRESULT TM_FATFS_USB_disk_ioctl(BYTE cmd, void* buff)
{
	return RES_NOTRDY;
}

#endif
