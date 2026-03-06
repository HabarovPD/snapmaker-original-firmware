#ifndef _SCCARD_H_
#define _SCCARD_H_

#ifdef _IN_SDCARDDRIVER
#define EXT_SDCARDDRIVER	
#else
#define EXT_SDCARDDRIVER	extern
#endif

EXT_SDCARDDRIVER BYTE Rd_Block(DWORD Addr, BYTE* Buff);
EXT_SDCARDDRIVER BYTE Wt_Block(U32 Addr, U8 *Buff);
EXT_SDCARDDRIVER BYTE SDCardInit(void);
EXT_SDCARDDRIVER DRESULT TM_FATFS_SD_SPI_disk_ioctl(BYTE cmd, char *buff);

EXT_SDCARDDRIVER DWORD SectorSize;
EXT_SDCARDDRIVER DWORD SectorCount;

#endif
