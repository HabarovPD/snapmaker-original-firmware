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

#include "Marlin.h"
#include "UDisk.h"

#include "stepper.h"
#include "temperature.h"
#include "language.h"
#include "typedef.h"
#include "SystemControl.h"

extern char WaitUDisk(void);
extern void UDiskRestart(void);


static volatile char UdiskBuff[512];
static volatile uint16_t UdiskBuffLen = 0;
static volatile uint16_t UdiskGetIndex;
static volatile uint32_t UdiskSeekPos;

#if ENABLED(SDSUPPORT)

UDisk::UDisk() 
{
  filesize = 0;
  Udiskpos = 0;
  sdprinting = false;
  cardOK = false;
  saving = false;
  logging = false;
  workDirDepth = 0;
  file_subcall_ctr = 0;

  PortInited = 0;

  autostart_stilltocheck = true; //the SD start is delayed, because otherwise the serial cannot answer fast enough to make contact with the host software.
}



char *createFilename(char *buffer, const FILINFO *p) 
{ //buffer > 12characters

	strcpy(buffer, p->fname);
	/*
  char *pos = buffer;
  for (uint8_t i = 0; i < 11; i++) {
    if (p->fname[i] == ' ') continue;
    if (i == 8) *pos++ = '.';
    *pos++ = p->fname[i];
  }
  *pos++ = 0;
  */
  return buffer;
}

/**
 * Dive into a folder and recurse depth-first to perform a pre-set operation lsAction:
 *   LS_Count       - Add +1 to nrFiles for every file within the parent
 *   LS_GetFilename - Get the filename of the file indexed by nrFiles
 *   LS_SerialPrint - Print the full path of each file to serial output
 */

//Modify by Paladin，Add "filter" to select what type you need(FIL_CNC, FIL_GCODE)
void UDisk::lsDive(const char *prepend, DIR *parent, const char * const match/*=NULL*/, uint8_t filter) 
{
	FILINFO fileinfo;
	uint8_t cnt = 0;

	//Каталог не открыт
	if(parent->fs == 0)
		f_opendir(parent, "0:\\");
	
	// Read the next entry from a directory
	fileinfo.lfname = longFilename;
	fileinfo.lfsize = sizeof(longFilename);
	while(1)
	{
		if(WaitUDisk() == 0)
		{
			if(f_readdir(parent, &fileinfo) != FR_OK)
				return;
		}
		else
		{
			return;
		}
		//найти папку
		if((fileinfo.fattrib & AM_DIR) && (lsAction != LS_Count) && (lsAction != LS_GetFilename))
		{
			continue;
			// Get the short name for the item, which we know is a folder
		      char lfilename[FILENAME_LENGTH];
		      createFilename(lfilename, &fileinfo);

			  // Allocate enough stack space for the full path to a folder, trailing slash, and nul
		      bool prepend_is_empty = (prepend[0] == '\0');
		      int len = (prepend_is_empty ? 1 : strlen(prepend)) + strlen(lfilename) + 1 + 1;
		      char path[40];

			  // Append the FOLDERNAME12/ to the passed string.
		      // It contains the full path to the "parent" argument.
		      // We now have the full path to the item in this folder.
		      strcpy(path, prepend_is_empty ? "/" : prepend); // root slash if prepend is empty
		      strcat(path, lfilename); // FILENAME_LENGTH-1 characters maximum
		      strcat(path, "/");

			  // Serial.print(path);

		      // Get a new directory object using the full path
		      // and dive recursively into it.
		      DIR dir;
		      if (FR_OK == f_opendir(parent, lfilename))
			  {
		        if (lsAction == LS_SerialPrint) 
				{
		          SERIAL_ECHO_START;
		          SERIAL_ECHOLN(MSG_SD_CANT_OPEN_SUBDIR);
		          SERIAL_ECHOLN(lfilename);
		        }
		      }
		      lsDive(path, &dir, NULL, filter);
		}
		else 
		{
			uint8_t pn0 = fileinfo.fname[0];
			if (pn0 == 0) break;
			if (pn0 == 0xe5 || pn0 == '.') continue;
			if (longFilename[0] == '.') continue;

			int extpos = strlen(fileinfo.fname) - 3;
			char ExtName[5];
			strcpy(ExtName, &fileinfo.fname[extpos]);
			if(ExtName[0] >= 'a')
				ExtName[0] = ExtName[0] - 'a' + 'A';
			if(ExtName[1] >= 'a')
				ExtName[1] = ExtName[1] - 'a' + 'A';
			if(ExtName[2] >= 'a')
				ExtName[2] = ExtName[2] - 'a' + 'A';
			//файл G-кода
			if(filter == FIL_GCODE)
			{
				if((ExtName[0]!='G') || (ExtName[1]!='C') || (ExtName[2]!='O')) continue;
			}
			else if(filter == FIL_CNC)
			{
				bool condition = (ExtName[0]!='C') || (ExtName[1]!='N') || (ExtName[2]!='C');
				bool Condition2 = (ExtName[0]!='.') || (ExtName[1]!='N') || (ExtName[2]!='C');
				if((condition == true) && (Condition2 == true)) continue;
			}

			switch (lsAction) 
			{
				case LS_Count:
					nrFiles++;
				break;
				
				case LS_SerialPrint:
					//длинное имя файла
					SERIAL_PROTOCOLLN(longFilename);

					//SERIAL_PROTOCOLLN("\r\n");
				break;
				
				case LS_GetFilename:
					createFilename(filename, &fileinfo);
					if (match != NULL) 
					{
						if (strcasecmp(match, filename) == 0) return;
					}
					else if (cnt == nrFiles) return;
					cnt++;
				break;
			}
	    }
	}
}



void UDisk::ls(uint8_t Filter)  
{
	lsAction = LS_SerialPrint;
	if(WaitUDisk() == 0)
	{
		f_opendir(&root, "0:\\");
		lsDive("", &root, NULL, Filter);
	}
}

void UDisk::initsd() 
{
	cardOK = false;

	if(1)
	if(PortInited == false)
	{
		PortInited = true;
	}
	WaitUDisk();
	if (root.fs != 0) f_closedir(&root);
	workdirpath[0] = 0;
	f_mount(&CardFatfs, "0:\\", 0);

	if(WaitUDisk() == 0)
	{
		if(f_mount(&CardFatfs, "0:\\", 1) != FR_OK)
		{
		    //SERIAL_ECHO_START;
		    //SERIAL_ECHOLNPGM(MSG_SD_INIT_FAIL);
			return;
		}
		else
		{
		    cardOK = true;
		    SERIAL_ECHO_START;
		    SERIAL_ECHOLNPGM(MSG_SD_CARD_OK);
		}
		strcpy(workdirpath, "0:");
		WaitUDisk();
		f_opendir(&root, "0:\\");
		ListDir = root;
		curDir = &root;
	}
}

void UDisk::release() {
  sdprinting = false;
  cardOK = false;
}

void UDisk::openAndPrintFile(const char *name) 
{
	char cmd[4 + 39 + 1]; // Room for "M23 ", filename, and null
	sprintf(cmd, "M23 %s\r\n", name);
	for (char *c = &cmd[4]; *c; c++) 
	{
		if((*c >='A') && (*c <= 'Z'))
			*c = *c-'A'+'a';
		else
			*c = *c;
	}
	enqueue_and_echo_command(cmd);
	enqueue_and_echo_commands_P("M24\r\n");
}

void UDisk::startFileprint() {
  uint8_t FileHead[20];
  if (cardOK)
  {
    WaitUDisk();
  	ReadBytes(FileHead, 20);
	if((FileHead[0] == 0x20) && (FileHead[1] == 0x18) && (FileHead[2] == 0x10) && (FileHead[3] == 0x01))
	{
		Udiskpos = (FileHead[19] << 24) | (FileHead[18] << 16) | (FileHead[17] << 8) | FileHead[16];
		WaitUDisk();
		if(FR_OK != f_lseek(&file, Udiskpos))
		{
			sdprinting = false;
			SetCurrentPrinterStatus(STAT_IDLE);
		}
		else
		{
			sdprinting = true;
			SetCurrentPrinterStatus(STAT_PRINTING);
			UdiskSeekPos = Udiskpos;
			postell = Udiskpos;
			#if(HAVE_FALIMENT_SENSOR == 1)
			EnableFilamentCheck();
			#endif
		}
	}
	else
	{
		WaitUDisk();
		f_lseek(&file, 0);
		sdprinting = true;
		SetCurrentPrinterStatus(STAT_PRINTING);
		Udiskpos = 0;
		UdiskSeekPos = 0;
		postell = Udiskpos;
		#if(HAVE_FALIMENT_SENSOR == 1)
		EnableFilamentCheck();
		#endif
	}
  }
}

void UDisk::pauseSDPrint() {
  if (sdprinting) sdprinting = false;
}

void UDisk::openLogFile(char* name) {
  logging = true;
  openFile(name, false);
}

//Modify By Paladin
void UDisk::openFile(char* name, bool read, bool push_current/*=false*/) {
    char tmpFilename[96];
	if((name[1] == ':') && (name[2] == '\\'))
		strcpy(tmpFilename, "");
	else
		strcpy(tmpFilename, "0:\\");
	strcat(tmpFilename, name);
	strcpy(currentselectfilename, tmpFilename);
	if(WaitUDisk() == 0)
	{
		if(FR_OK == f_open(&file, tmpFilename, FA_READ | FA_OPEN_EXISTING))
		{
			filesize = file.fsize;
			//SERIAL_PROTOCOLPGM(MSG_SD_FILE_OPENED);
			//SERIAL_PROTOCOL(name);
			//SERIAL_PROTOCOLPGM(MSG_SD_SIZE);
			//SERIAL_PROTOCOLLN(filesize);
			Udiskpos = 0;
			UdiskBuffLen = 0;
			UdiskGetIndex = 0;
			UdiskSeekPos = 0;

			//SERIAL_PROTOCOLLNPGM(MSG_SD_FILE_SELECTED);
			//getfilename(0, name);
		}
	}
}

void UDisk::getStatus() {
  if (cardOK) {
    SERIAL_PROTOCOLPGM(MSG_SD_PRINTING_BYTE);
    SERIAL_PROTOCOL(Udiskpos);
    SERIAL_PROTOCOLCHAR('/');
    SERIAL_PROTOCOLLN(filesize);
  }
  else {
    SERIAL_PROTOCOLLNPGM(MSG_SD_NOT_PRINTING);
  }
}

void UDisk::write_command(char *buf) 
{
	UINT WRN;
	char* begin = buf;
	char* npos = 0;
	char* end = buf + strlen(buf) - 1;


  if ((npos = strchr(buf, 'N')) != NULL) {
    begin = strchr(npos, ' ') + 1;
    end = strchr(npos, '*') - 1;
  }
  end[1] = '\r';
  end[2] = '\n';
  end[3] = '\0';
  if(WaitUDisk() == 0)
	f_write(&file, begin, strlen(begin), &WRN);
  
}


void UDisk::StartSelectPrint(char *FileName)
{
	/*
	char cmd[30];
	char NewFile[20];
	uint8_t i;
	i = 0;
	while(*FileName)
	{
		if((*FileName >= 'A') && (*FileName <= 'Z'))
			NewFile[i++] = *FileName - 'A' + 'a';
		else
			NewFile[i++] = *FileName;
		FileName++;
	}
	NewFile[i] = 0;
	sprintf(cmd, "M23 %s", NewFile);
	enqueue_and_echo_command(cmd);
	enqueue_and_echo_commands_P("M24");
	*/
	
	openFile(FileName, true);
	if(file.fs >= 0)
	{
		startFileprint();
		print_job_timer.start();
	}
	
}



void UDisk::closefile(bool store_location)
{
	WaitUDisk();
	f_sync(&file);
	WaitUDisk();
	f_close(&file);
	saving = logging = false;

	if (store_location) {
	//future: store printer state, filename and position for continuing a stopped print
	// so one can unplug the printer and continue printing the next day.
	}
}

/**
 * Get the name of a file in the current directory by index
 */
void UDisk::getfilename(uint16_t nr, const char * const match/*=NULL*/, uint8_t Filter) {
  curDir = &ListDir;
  lsAction = LS_GetFilename;
  nrFiles = nr;
  if(WaitUDisk() == 0)
  {
	  f_closedir(curDir);
	  WaitUDisk();
	  f_opendir(curDir, filename);
	  lsDive("", curDir, match, Filter);
  }
}

uint16_t UDisk::getnrfilenames(uint8_t Filter) {
  curDir = &ListDir;
  lsAction = LS_Count;
  nrFiles = 0;
  if(WaitUDisk() == 0)
  {
	  f_closedir(curDir);
	  WaitUDisk();
	  f_opendir(curDir, filename);
	  lsDive("", curDir, NULL, Filter);
  }
  //SERIAL_ECHOLN(nrFiles);
  return nrFiles;
}

/****************************************
Переход к файлу
искатьпос: позиция
*****************************************/
bool UDisk::FileSeek(int seekpos)
{
	UdiskBuffLen = 0;
	WaitUDisk();
	if(FR_OK != f_lseek(&file, seekpos))
	{
		WaitUDisk();
		return false;
	}
	else
	{
		UdiskSeekPos = seekpos;
		postell = seekpos;
		WaitUDisk();
		return true;
	}
	
}

/****************************************
Получить длину файла
Возвращаемое значение: -1 означает, что файл не открыт; другое означает длину файла
****************************************/
int32_t UDisk::GetFileSize()
{
	if(file.fs > 0)
		return file.fsize;
	else
		return -1;
}


char UDisk::get(char *pBuff)
{
	UINT RN;
	char Res;
	int i;
	uint32_t LastPos;
	FRESULT FR;

	LastPos = UdiskSeekPos;
	
	//Данные кэша были прочитаны
	if(UdiskBuffLen == 0)
	{
		//Файл закрыт
		if(card.isFileOpen() == false)
		{
			f_mount(&CardFatfs, "0:\\", 1);
			WaitUDisk();
			f_open(&file, currentselectfilename, FA_OPEN_EXISTING | FA_READ);
			WaitUDisk();
			f_lseek(&file, UdiskSeekPos);
		}
		//Получить текущее местоположение файла
		UdiskSeekPos = f_tell(&file);
		LastPos = UdiskSeekPos;
		Res = WaitUDisk();
		Res = WaitUDisk();
		if(Res == 0)
		{
			FR = f_read(&file, (char*)UdiskBuff, 512, &RN);
			WaitUDisk();
			//Прочитать файл успешно
			if(FR == FR_OK)
			{
				UdiskBuffLen = RN - 1;
				UdiskGetIndex = 1;
				*pBuff = UdiskBuff[0];
				postell = LastPos;
				UdiskSeekPos++;
				Udiskpos = UdiskSeekPos;
				return 0;
			}
			//Чтение не удалось
			else
			{
				SERIAL_ECHO_START;
				SERIAL_ECHOLNPGM("Udisk Read Error!\r\n");
				sprintf((char*)UdiskBuff, "%d\r\n", FR);
				SERIAL_ECHOLNPGM((char*)UdiskBuff);
				file.fs = 0;
			}
		}
		//Не удалось прочитать файл, перемонтируйте USB-диск.
		else if(Res == (char)-1)
		{
			//Сброс USB-накопителя
			UDiskRestart();
			SERIAL_ECHO_START;
			SERIAL_ECHOLNPGM("Udisk Reset!\r\n");
			return (char)-1;
		}
		//USB-диск отключен
		else if(Res == (char)-2)
		{
			f_mount(&CardFatfs, "0:\\", 1);
			WaitUDisk();
			f_open(&file, currentselectfilename, FA_OPEN_EXISTING | FA_READ);
			WaitUDisk();
			f_lseek(&file, UdiskSeekPos);
			if(FR_OK == f_read(&file, (char*)UdiskBuff, 512, &RN))
			{
				UdiskBuffLen = RN - 1;
				*pBuff = UdiskBuff[0];
				UdiskGetIndex = 1;
				postell = LastPos;
				UdiskSeekPos++;
				Udiskpos = UdiskSeekPos;
				SERIAL_ECHO_START;
				SERIAL_ECHOLNPGM("Udisk Retry sucess!\r\n");
				return 0;
			}
			SERIAL_ECHO_START;
			SERIAL_ECHOLNPGM("Udisk Retry fail!\r\n");
			return (char)-1;
		}
		return (char)-1;
	}
	//В кеше все еще есть данные
	else
	{
		*pBuff = UdiskBuff[UdiskGetIndex++];
		UdiskBuffLen--;
		postell = LastPos;
		UdiskSeekPos++;
		Udiskpos = UdiskSeekPos;
		return 0;
	}
}

bool UDisk::getWorkDirPath()
{
	/*
	if(f_getcwd(workdirpath, sizeof(workdirpath)) != FR_OK)
	{
		f_closedir(&workDir);
		f_closedir(&ListDir);
		strcpy(workdirpath, "");
		return false;
	}
	*/
	if(workdirpath[0] == '0')
		return true;
	else
		return false;
}

/********************************************
Очистить индекс чтения каталога
**********************************************/
void UDisk::ClearCurrentList()
{
	ListDir.index = 0;
}

/********************************************
//Получить содержимое каталога
Возвращаемое значение: -1 означает, что содержимого нет; 0 означает файл; 1 означает каталог
**********************************************/
char UDisk::getWorkDirItems()
{
	WaitUDisk();
	if(ListDir.fs == 0)
		return (char)-2;
	WaitUDisk();

	// Read the next entry from a directory
	ListFileInfo.lfname = longFilename;
	ListFileInfo.lfsize = sizeof(longFilename);
	
	if(FR_OK == f_readdir(&ListDir, &ListFileInfo))
	{
		createFilename(filename, &ListFileInfo);
		if(ListFileInfo.fattrib & AM_DIR)
		{
			return 1;
		}
		else
		{
			return 0;
		}
	}
	else
	{
		return (char)-1;
	}	
}


/********************************************
//Получить специальную информацию о заголовке файла
**********************************************/
int32_t UDisk::ReadBytes(uint8_t *pBuff, int32_t Len)
{
	unsigned long RN;
	WaitUDisk();
	if(file.fs == 0)
		return (char)-1;

	if(f_read(&file, pBuff, Len, &RN) == FR_OK)
	{
		return RN;
	}
	else
	{
		return -1;
	}
}


char UDisk::chdir(const char * relpath) 
{
	int pos;
	//DIR *parent = &root;
	//if (ListDir.fs > 0) parent = &ListDir;
	pos = strlen(workdirpath);
	strcat(workdirpath, "\\");
	strcat(workdirpath, relpath);
	if(WaitUDisk() == 0)
	{
		if(FR_OK != f_chdir(workdirpath))
		{
			//SERIAL_ECHO_START;
		    //SERIAL_ECHOPGM(MSG_SD_CANT_ENTER_SUBDIR);
		    //SERIAL_ECHOLN(relpath);
			workdirpath[pos] = 0;
			return 0;
		}
		else 
		{
			f_opendir(&ListDir, workdirpath);
			f_getcwd(workdirpath, sizeof(workdirpath));
			return (char)-1;
		}
	}
	else
	{
		return (char)-1;
	}
}

void UDisk::updir() 
{
  int Len;
  if(f_getcwd(workdirpath, sizeof(workdirpath)) != FR_OK)
  {
	strcpy(workdirpath, "");
	f_opendir(&ListDir, "0:\\");
  }
  //Найти предыдущий каталог
  Len = strlen(workdirpath) - 1;
  while(1)
  {
  	if(Len == 0)
		return;
	if((workdirpath[Len] == '\\') || (workdirpath[Len] == ':'))
		break;
	workdirpath[Len] = 0;
  }
  f_opendir(&ListDir, workdirpath);
}

void UDisk::printingHasFinished() 
{
	st_synchronize();
	if (file_subcall_ctr > 0)
	{ // Heading up to a parent file that called current as a procedure.
		WaitUDisk();
		f_close(&file);
		file_subcall_ctr--;
		WaitUDisk();
		openFile(proc_filenames[file_subcall_ctr], true, true);
		setIndex(filespos[file_subcall_ctr]);
		startFileprint();
	}
	else 
	{
		WaitUDisk();
		f_close(&file);
		sdprinting = false;
		ShieldLedOn();
		ShieldFanOn();
		//if (SD_FINISHED_STEPPERRELEASE)
		//enqueue_and_echo_commands_P(SD_FINISHED_RELEASECOMMAND);
		if(MA_PRINT == MachineSelected);
			autotempShutdown();
	}
}

//UDisk card;


#endif //SDSUPPORT
