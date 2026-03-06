/*-----------------------------------------------------------------------*/
/* Low level disk I/O module skeleton for FatFs     (C)ChaN, 2013        */
/*-----------------------------------------------------------------------*/
/* If a working storage control module is available, it should be        */
/* attached to the FatFs via a glue function rather than modifying it.   */
/* This is an example of glue functions to attach various exsisting      */
/* storage control module to the FatFs module with a defined API.        */
/*-----------------------------------------------------------------------*/

#include "diskio.h"		/* FatFs lower layer API */
#include "ff.h"

#ifndef FATFS_FLASH_SPI
	#define FATFS_FLASH_SPI				0
#endif

#ifndef FATFS_USE_SDIO
	#define FATFS_USE_SDIO			0
#endif

#ifndef FATFS_USE_USB
	#ifdef STM32F10X_CL
	#define FATFS_USE_USB			1
	#endif
#endif

#ifndef FATFS_USE_SPI_SD
	#define FATFS_USE_SPI_SD				1
#endif


/* Set in defines.h file if you want it */
#ifndef TM_FATFS_CUSTOM_FATTIME
	#define TM_FATFS_CUSTOM_FATTIME		0
#endif


/* Include SD card files if is enabled */
#if FATFS_USE_SDIO == 1
	#include "../CFile/fatfs/drivers/fatfs_sd_sdio.h"
#endif

#if FATFS_FLASH_SPI == 1
	#include "./fatfs/drivers/fatfs_flash_spi.h"
#endif

#if FATFS_USE_USB == 1
	#include "../drivers/fatfs_usb.h"
#endif

#if FATFS_USE_SPI_SD == 1
	#include "../drivers/SDCardDriverSPI.h"
#endif


/* Definitions of physical drive number for each media */
#define ATA			    2
#define SPI_FLASH		1
#define UDISK			0
#define SPI_SD			3
/*-----------------------------------------------------------------------*/
/* Inidialize a Drive                                                    */
/*-----------------------------------------------------------------------*/

DSTATUS disk_initialize (
	BYTE pdrv				/* Physical drive nmuber (0..) */
)
{

	DSTATUS status = STA_NOINIT;
	
	switch (pdrv) {
		case ATA:	/* SD CARD */
			#if FATFS_USE_SDIO == 1
				status = TM_FATFS_SD_SDIO_disk_initialize();	/* SDIO communication */
			#endif
			break;
			
		case SPI_FLASH:
			#if	FATFS_FLASH_SPI ==1
			status = TM_FATFS_FLASH_SPI_disk_initialize();	/* SDIO communication */
			#endif
			break;

		case UDISK:
			status = TM_FATFS_USB_disk_initialize();
			break;

		case SPI_SD:
			status = SDCardInit();
		break;

		default:
			status = STA_NOINIT;
	}
	return status;
}



/*-----------------------------------------------------------------------*/
/* Get Disk Status                                                       */
/*-----------------------------------------------------------------------*/

DSTATUS disk_status (
	BYTE pdrv		/* Physical drive nmuber (0..) */
)
{

	DSTATUS status = STA_NOINIT;
	
	switch (pdrv) {
		case ATA:	/* SD CARD */
			#if FATFS_USE_SDIO == 1
				//status = TM_FATFS_SD_SDIO_disk_status();	/* SDIO communication */
				status = TM_FATFS_as
			#endif
			break;
		case SPI_FLASH:
			#if	FATFS_FLASH_SPI ==1
			status = TM_FATFS_FLASH_SPI_disk_status();	/* SDIO communication */
			#endif
			break;

		case UDISK:
			status = TM_FATFS_USB_disk_status();
			break;

		case SPI_SD:
			status = 0;
			break;

		default:
			status = STA_NOINIT;
	}
	return status;
}



/*-----------------------------------------------------------------------*/
/* Read Sector(s)                                                        */
/*-----------------------------------------------------------------------*/

DRESULT disk_read (
	BYTE pdrv,		/* Physical drive nmuber (0..) */
	BYTE *buff,		/* Data buffer to store read data */
	DWORD sector,	/* Sector address (LBA) */
	UINT count		/* Number of sectors to read (1..128) */
)
{
	unsigned long i;
	DRESULT status = RES_PARERR;
	switch (pdrv) {
		case ATA:	/* SD CARD */
			#if FATFS_USE_SDIO == 1
				status = TM_FATFS_SD_SDIO_disk_read(buff, sector, count);	/* SDIO communication */
			#endif
			break;
		case SPI_FLASH:
			#if	FATFS_FLASH_SPI ==1
			status = TM_FATFS_FLASH_SPI_disk_read(buff, sector, count);	/* SDIO communication */
			#endif
		break;

		case UDISK:
			#if FATFS_USE_USB == 1
				status = TM_FATFS_USB_disk_read(buff, sector, count);
			#endif
		break;

		case SPI_SD:
			if(count == 1)
			{
				status = (DRESULT)Rd_Block(sector, buff);
			}
			else
			{
				for(i=0;i<count;i++)
				{
					sector = sector + 512 * i;
					status = (DRESULT)Rd_Block(sector, buff);
					buff = buff + 512;
					if(status != 0)
						break;
				}
			}
		break;
		
		default:
			status = RES_PARERR;
	}
	return status;
}



/*-----------------------------------------------------------------------*/
/* Write Sector(s)                                                       */
/*-----------------------------------------------------------------------*/

#if _USE_WRITE
DRESULT disk_write (
	BYTE pdrv,			/* Physical drive nmuber (0..) */
	const BYTE *buff,	/* Data to be written */
	DWORD sector,		/* Sector address (LBA) */
	UINT count			/* Number of sectors to write (1..128) */
)
{
	unsigned long i;
	DRESULT status = RES_PARERR;
	if (!count) {
		return RES_PARERR;		/* Check parameter */
	}
	
	switch (pdrv) {
		case ATA:	/* SD CARD */
			#if FATFS_USE_SDIO == 1
				//status = TM_FATFS_SD_SDIO_disk_write((BYTE *)buff, sector, count);	/* SDIO communication */
				status = TM_FATFS_USB_disk_write((BYTE *)buff, sector, count);
			#endif
		break;

		case SPI_FLASH:
			#if	FATFS_FLASH_SPI ==1
			status = TM_FATFS_FLASH_SPI_disk_write((BYTE *)buff, sector, count);	/* SDIO communication */
			#endif
		break;

		case UDISK:
			#if	FATFS_USE_USB ==1
			TM_FATFS_USB_disk_write((BYTE *)buff, sector, count);
			#endif
		break;

		case SPI_SD:
			if(count == 1)
			{
				status = (DRESULT)Wt_Block(sector, (BYTE *)buff);
			}
			else
			{
				for(i=0;i<count;i++)
				{
					sector = sector + 512 * i;
					status = (DRESULT)Wt_Block(sector, (BYTE *)buff);
					if(status != 0)
						break;				
				}
			}
		break;
		
		default:
			status = RES_PARERR;
	}
	return status;
}
#endif


/*-----------------------------------------------------------------------*/
/* Miscellaneous Functions                                               */
/*-----------------------------------------------------------------------*/

#if _USE_IOCTL
DRESULT disk_ioctl (
	BYTE pdrv,		/* Physical drive nmuber (0..) */
	BYTE cmd,		/* Control code */
	void *buff		/* Buffer to send/receive control data */
)
{
	DRESULT status = RES_PARERR;
	switch (pdrv) {
		case ATA:	/* SD CARD */
			#if FATFS_USE_SDIO == 1
				status = TM_FATFS_SD_SDIO_disk_ioctl(cmd, (void*)buff);					/* SDIO communication */
			#endif
			break;
		case SPI_FLASH:
			#if	FATFS_FLASH_SPI ==1
			status = TM_FATFS_FLASH_SPI_disk_ioctl(cmd, (void*)buff);	/* SDIO communication */
			#endif
		break;

		case SPI_SD:
			status = TM_FATFS_SD_SPI_disk_ioctl(cmd,(char*)buff);
			break;


		case UDISK:
			status = TM_FATFS_USB_disk_ioctl(cmd, (void*)buff);
		break;
		default:
			status = RES_PARERR;
	}
	return status;
}
#endif

__weak DWORD get_fattime(void) {
	/* Returns current time packed into a DWORD variable */
	return	  ((DWORD)(2013 - 1980) << 25)	/* Year 2013 */
			| ((DWORD)7 << 21)				/* Month 7 */
			| ((DWORD)28 << 16)				/* Mday 28 */
			| ((DWORD)0 << 11)				/* Hour 0 */
			| ((DWORD)0 << 5)				/* Min 0 */
			| ((DWORD)0 >> 1);				/* Sec 0 */
}


