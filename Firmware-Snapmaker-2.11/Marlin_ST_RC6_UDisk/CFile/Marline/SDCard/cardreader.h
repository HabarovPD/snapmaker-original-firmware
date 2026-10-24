/**
 * Marlin 3D Printer Firmware
 * Copyright (C) 2016 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#ifndef CARDREADER_H
#define CARDREADER_H

#include "ff.h"
#include "typedef.h"

#if ENABLED(SDSUPPORT)


#define MAX_VFAT_ENTRIES (5)

#define MAX_DIR_DEPTH 1          // Maximum folder depth
#define FILENAME_LENGTH	13
#define LONG_FILENAME_LENGTH	(13*MAX_VFAT_ENTRIES)


enum LsAction { LS_SerialPrint, LS_Count, LS_GetFilename };
enum {FIL_GCODE = 0, FIL_CNC};

class CardReader {
public:
  CardReader();

  void initsd();
  void write_command(char *buf);
  //files auto[0-9].g on the sd card are performed in a row
  //this is to delay autostart and hence the initialisaiton of the sd card to some seconds after the normal init, so the device is available quick after a reset

  void StartSelectPrint(char * FileName);
  void openFile(char* name, bool read, bool push_current=false);
  void openLogFile(char* name);
  void closefile(bool store_location=false);
  void release();
  void openAndPrintFile(const char *name);
  void startFileprint();
  void pauseSDPrint();
  void getStatus();
  void printingHasFinished();

  #if ENABLED(LONG_FILENAME_HOST_SUPPORT)
    void printLongPath(char *path);
  #endif

  void getfilename(uint16_t nr, const char* const match=NULL, uint8_t Filter = FIL_GCODE);
  uint16_t getnrfilenames(uint8_t Filter = FIL_GCODE);

  void ls(uint8_t Filter = FIL_GCODE);
  void chdir(const char *relpath);
  void updir();

  inline bool isFileOpen() { return file.fs!=NULL; }
  inline bool eof() { return sdpos >= filesize; }
  char get(char *pBuff);
  inline void setIndex(long index) { sdpos = index; f_lseek(&file, index); }
  inline uint8_t percentDone() { return (isFileOpen() && filesize) ? sdpos / ((filesize + 99) / 100) : 0; }
  inline char* getWorkDirName();

public:
  bool saving, logging, sdprinting, cardOK, filenameIsDir;
  char filename[FILENAME_LENGTH + 1], longFilename[LONG_FILENAME_LENGTH + 1];
  int autostart_index;
  
  
private:
  DIR root;
  DIR *curDir, workDir, workDirParents[MAX_DIR_DEPTH];
  FATFS CardFatfs;
  uint8_t workDirDepth;
  char workdirpath[255];
  char rootpath[10];

  FIL file;
  bool PortInited;

  #define SD_PROCEDURE_DEPTH 1
  #define MAXPATHNAMELENGTH (FILENAME_LENGTH*MAX_DIR_DEPTH + MAX_DIR_DEPTH + 1)
  uint8_t file_subcall_ctr;
  uint32_t filespos[SD_PROCEDURE_DEPTH];
  char proc_filenames[SD_PROCEDURE_DEPTH][MAXPATHNAMELENGTH];
  uint8_t tmpFileBuff[512];
  uint32_t filesize;
  uint32_t sdpos;

  bool autostart_stilltocheck; //the sd start is delayed, because otherwise the serial cannot answer fast enought to make contact with the hostsoftware.

  LsAction lsAction; //stored for recursion.
  uint16_t nrFiles; //counter for the files in the current directory and recycled as position counter for getting the nrFiles'th name in the directory.
  char* diveDirName;
  void lsDive(const char *prepend, DIR *parent, const char * const match=NULL, uint8_t filter=FIL_CNC);
  void SPIInit(void);
};

extern CardReader card;

#define IS_SD_PRINTING (card.sdprinting)


#define IS_SD_INSERTED ((GPIOC->IDR & GPIO_Pin_6) == 0)


#else

#define IS_SD_PRINTING (false)

#endif //SDSUPPORT

#endif //__CARDREADER_H
