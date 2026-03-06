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
#include "cardreader.h"

#include "stepper.h"
#include "temperature.h"
#include "language.h"
#include "typedef.h"


#if ENABLED(SDSUPPORT)

CardReader::CardReader() 
{
  SPIInit();
  filesize = 0;
  sdpos = 0;
  sdprinting = false;
  cardOK = false;
  saving = false;
  logging = false;
  workDirDepth = 0;
  file_subcall_ctr = 0;
  memset(workDirParents, 0, sizeof(workDirParents));
  PortInited = 0;

  autostart_stilltocheck = true; //the SD start is delayed, because otherwise the serial cannot answer fast enough to make contact with the host software.
  autostart_index = 0;
}

void CardReader::SPIInit(void)
{
	SPI_InitTypeDef SPI_InitStruct;

	SPI_I2S_DeInit(SPI2);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC, ENABLE);

	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_10MHz;
	
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_12;
	GPIO_Init(GPIOB, &GPIO_InitStruct);

	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_15;
	GPIO_Init(GPIOB, &GPIO_InitStruct);

	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_14;
	GPIO_Init(GPIOB, &GPIO_InitStruct);

	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6;
	GPIO_Init(GPIOC, &GPIO_InitStruct);

	SPI_InitStruct.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStruct.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStruct.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStruct.SPI_CPHA = SPI_CPHA_2Edge;
	SPI_InitStruct.SPI_CPOL = SPI_CPOL_High;
	SPI_InitStruct.SPI_CRCPolynomial = SPI_CRCPR_CRCPOLY;
	SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStruct.SPI_Mode = SPI_Mode_Master;
	SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;
	SPI_Init(SPI2, &SPI_InitStruct);

	SPI_Cmd(SPI2, ENABLE);
	GPIOB->BSRR = GPIO_Pin_12;
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
void CardReader::lsDive(const char *prepend, DIR *parent, const char * const match/*=NULL*/, uint8_t filter) 
{
	FILINFO fileinfo;
	uint8_t cnt = 0;

	//目录未打开
	if(parent->fs == 0)
		f_opendir(parent, rootpath);
	
	// Read the next entry from a directory
	fileinfo.lfname = longFilename;
	fileinfo.lfsize = sizeof(longFilename);
	while(f_readdir(parent, &fileinfo) == FR_OK)
	{
		//找到文件夹
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
			//Gcode文件
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
					//长文件名
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

void CardReader::ls(uint8_t Filter)  
{
	lsAction = LS_SerialPrint;
	f_opendir(&root, rootpath);
	lsDive("", &root, NULL, Filter);
}

#if ENABLED(LONG_FILENAME_HOST_SUPPORT)

  /**
   * Get a long pretty path based on a DOS 8.3 path
   */
  void CardReader::printLongPath(char *path) {
    lsAction = LS_GetFilename;

    int i, pathLen = strlen(path);

    // SERIAL_ECHOPGM("Full Path: "); SERIAL_ECHOLN(path);

    // Zero out slashes to make segments
    for (i = 0; i < pathLen; i++) if (path[i] == '/') path[i] = '\0';

    SdFile diveDir = root; // start from the root for segment 1
    for (i = 0; i < pathLen;) {

      if (path[i] == '\0') i++; // move past a single nul

      char *segment = &path[i]; // The segment after most slashes

      // If a segment is empty (extra-slash) then exit
      if (!*segment) break;

      // Go to the next segment
      while (path[++i]) { }

      // SERIAL_ECHOPGM("Looking for segment: "); SERIAL_ECHOLN(segment);

      // Find the item, setting the long filename
      diveDir.rewind();
      lsDive("", diveDir, segment);

      // Print /LongNamePart to serial output
      SERIAL_PROTOCOLCHAR('/');
      SERIAL_PROTOCOL(longFilename[0] ? longFilename : "???");

      // If the filename was printed then that's it
      if (!filenameIsDir) break;

      // SERIAL_ECHOPGM("Opening dir: "); SERIAL_ECHOLN(segment);

      // Open the sub-item as the new dive parent
      SdFile dir;
      if (!dir.open(diveDir, segment, O_READ)) {
        SERIAL_EOL;
        SERIAL_ECHO_START;
        SERIAL_ECHOPGM(MSG_SD_CANT_OPEN_SUBDIR);
        SERIAL_ECHO(segment);
        break;
      }

      diveDir.close();
      diveDir = dir;

    } // while i<pathLen

    SERIAL_EOL;
  }

#endif // LONG_FILENAME_HOST_SUPPORT

void CardReader::initsd() 
{
	cardOK = false;

	if(1)
	if(PortInited == false)
	{
		PortInited = true;
		SPIInit();
	}

	if (root.fs != 0) f_closedir(&root);

	f_mount(&CardFatfs, "3:\\", 0);

	if(f_mount(&CardFatfs, "3:\\", 1) != FR_OK)
	{
	    SERIAL_ECHO_START;
	    SERIAL_ECHOLNPGM(MSG_SD_INIT_FAIL);
		return;
	}
	else
	{
	    cardOK = true;
	    SERIAL_ECHO_START;
	    SERIAL_ECHOLNPGM(MSG_SD_CARD_OK);
	}
	strcpy(rootpath, "3:\\");
	f_opendir(&root, rootpath);
	workDir = root;
	curDir = &root;
}

void CardReader::release() {
  sdprinting = false;
  cardOK = false;
}

void CardReader::openAndPrintFile(const char *name) 
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

void CardReader::startFileprint() {
  if (cardOK)
    sdprinting = true;
}

void CardReader::pauseSDPrint() {
  if (sdprinting) sdprinting = false;
}

void CardReader::openLogFile(char* name) {
  logging = true;
  openFile(name, false);
}

//Modify By Paladin
void CardReader::openFile(char* name, bool read, bool push_current/*=false*/) {
    char tmpFilename[64];
	strcpy(tmpFilename, "3:\\");
	strcat(tmpFilename, name);
	if (!cardOK) return;
	if(FR_OK == f_open(&file, tmpFilename, FA_READ | FA_OPEN_EXISTING))
	{
		filesize = file.fsize;
		SERIAL_PROTOCOLPGM(MSG_SD_FILE_OPENED);
		SERIAL_PROTOCOL(name);
		SERIAL_PROTOCOLPGM(MSG_SD_SIZE);
		SERIAL_PROTOCOLLN(filesize);
		sdpos = 0;

		SERIAL_PROTOCOLLNPGM(MSG_SD_FILE_SELECTED);
		getfilename(0, name);
	}
}

void CardReader::getStatus() {
  if (cardOK) {
    SERIAL_PROTOCOLPGM(MSG_SD_PRINTING_BYTE);
    SERIAL_PROTOCOL(sdpos);
    SERIAL_PROTOCOLCHAR('/');
    SERIAL_PROTOCOLLN(filesize);
  }
  else {
    SERIAL_PROTOCOLLNPGM(MSG_SD_NOT_PRINTING);
  }
}

void CardReader::write_command(char *buf) 
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
  f_write(&file, begin, strlen(begin), &WRN);
  
}


void CardReader::StartSelectPrint(char *FileName)
{
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
}



void CardReader::closefile(bool store_location)
{
	f_sync(&file);
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
void CardReader::getfilename(uint16_t nr, const char * const match/*=NULL*/, uint8_t Filter) {
  curDir = &workDir;
  lsAction = LS_GetFilename;
  nrFiles = nr;
  f_closedir(curDir);
  f_opendir(curDir, filename);
  lsDive("", curDir, match, Filter);
}

uint16_t CardReader::getnrfilenames(uint8_t Filter) {
  curDir = &workDir;
  lsAction = LS_Count;
  nrFiles = 0;
  f_closedir(curDir);
  f_opendir(curDir, filename);
  lsDive("", curDir, NULL, Filter);
  //SERIAL_ECHOLN(nrFiles);
  return nrFiles;
}

char CardReader::get(char *pBuff)
{
	UINT RN;
	sdpos = f_tell(&file);
	if(FR_OK == f_read(&file, pBuff, 1, &RN))
		return 0;
	else
		return (char)-1;
}


inline char* CardReader::getWorkDirName()
{
	return workdirpath;
}


void CardReader::chdir(const char * relpath) 
{
	DIR *parent = &root;

	if (workDir.fs > 0) parent = &workDir;

	if(FR_OK != f_opendir(parent, relpath))
	{
		SERIAL_ECHO_START;
	    SERIAL_ECHOPGM(MSG_SD_CANT_ENTER_SUBDIR);
	    SERIAL_ECHOLN(relpath);
	}
	else 
	{
		if (workDirDepth < MAX_DIR_DEPTH)
			workDirParents[workDirDepth++] = *parent;
		workDir = *parent;
	}
}

void CardReader::updir() {
  if (workDirDepth > 0)
    workDir = workDirParents[--workDirDepth];
}

void CardReader::printingHasFinished() 
{
	st_synchronize();
	if (file_subcall_ctr > 0)
	{ // Heading up to a parent file that called current as a procedure.
		f_close(&file);
		file_subcall_ctr--;
		openFile(proc_filenames[file_subcall_ctr], true, true);
		setIndex(filespos[file_subcall_ctr]);
		startFileprint();
	}
	else 
	{
		f_close(&file);
		sdprinting = false;
		//if (SD_FINISHED_STEPPERRELEASE)
		//enqueue_and_echo_commands_P(SD_FINISHED_RELEASECOMMAND);
		autotempShutdown();
	}
}

//CardReader card;


#endif //SDSUPPORT
