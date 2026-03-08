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


#define MAX_VFAT_ENTRIES (5)

#define MAX_DIR_DEPTH 1          // Maximum folder depth
#define FILENAME_LENGTH	13
#define LONG_FILENAME_LENGTH	(13*MAX_VFAT_ENTRIES)


enum LsAction { LS_SerialPrint, LS_Count, LS_GetFilename };
enum {FIL_GCODE = 0, FIL_CNC};

class UDisk
{
public:
  UDisk();

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
  bool FileSeek(int seekpos);
  int32_t GetFileSize();

  #if ENABLED(LONG_FILENAME_HOST_SUPPORT)
    void printLongPath(char *path);
  #endif

  void getfilename(uint16_t nr, const char* const match=NULL, uint8_t Filter = FIL_GCODE);
  uint16_t getnrfilenames(uint8_t Filter = FIL_GCODE);

  void ls(uint8_t Filter = FIL_GCODE);
  char chdir(const char *relpath);
  void updir();

  inline bool isFileOpen() { return file.fs>0?true:false; }
  inline bool eof() { return Udiskpos >= filesize; }
  char get(char *pBuff);
  inline void setIndex(long index) { Udiskpos = index; f_lseek(&file, index); }
  inline uint8_t percentDone() { return (isFileOpen() && filesize) ? Udiskpos / ((filesize + 99) / 100) : 0; }
  char getWorkDirItems();
  bool getWorkDirPath();
  int32_t ReadBytes(uint8_t *pBuff, int32_t Len);
  void ClearCurrentList();

public:
  bool saving, logging, sdprinting, cardOK, filenameIsDir;
  uint8_t LastPercent;
  char filename[FILENAME_LENGTH + 1], longFilename[LONG_FILENAME_LENGTH + 1];
  //Имя файла, которое будет использоваться при приостановке точки останова.
  char currentselectfilename[LONG_FILENAME_LENGTH + 1];
  //Запись местоположения файла, используемая паузой точки останова
  uint32_t postell;
  //Путь к текущему рабочему каталогу
  char workdirpath[255];
private:
  DIR root;
  DIR *curDir, ListDir;
  FILINFO ListFileInfo;
  FATFS CardFatfs;
  uint8_t workDirDepth;
  
  //char rootpath[10];
  
  FIL file;
  bool PortInited;

  #define UDisk_PROCEDURE_DEPTH 1
  #define MAXPATHNAMELENGTH (FILENAME_LENGTH*MAX_DIR_DEPTH + MAX_DIR_DEPTH + 1)
  uint8_t file_subcall_ctr;
  uint32_t filespos[1];
  char proc_filenames[1][MAXPATHNAMELENGTH];
  uint32_t filesize;
  uint32_t Udiskpos;

  bool autostart_stilltocheck; //the sd start is delayed, because otherwise the serial cannot answer fast enought to make contact with the hostsoftware.

  LsAction lsAction; //stored for recursion.
  uint16_t nrFiles; //counter for the files in the current directory and recycled as position counter for getting the nrFiles'th name in the directory.
  char* diveDirName;
  void lsDive(const char *prepend, DIR *parent, const char * const match=NULL, uint8_t filter=FIL_CNC);
};

extern UDisk card;


#endif //__CARDREADER_H
