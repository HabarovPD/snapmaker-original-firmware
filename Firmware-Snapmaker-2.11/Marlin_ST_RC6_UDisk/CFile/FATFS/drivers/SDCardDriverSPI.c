#define _IN_SDCARDDRIVER
#include "includes.h"
#include "diskio.h"

#define CS_HIGH()	(GPIOB->BSRR = GPIO_Pin_12)
#define CS_LOW()	(GPIOB->BRR = GPIO_Pin_12)

/* Определение типа SD-карты */
#define SD_TYPE_MMC     0
#define SD_TYPE_V1      1
#define SD_TYPE_V2      2
#define SD_TYPE_V2HC    4



static U8 SD_SendCommand(U8 Command, U32 Arg, U8 CRCDat);
U8 SD_SendCommand_NoDeassert(U8 cmd, U32 arg, U8 crc);


void SPIx_SetSpeed(SPI_TypeDef *SPIx, U8 Speed)
{
	SPIx->CR1 &=~((u16)(1<<6)) ; //стоп с п1
	SPIx->CR1 &=~(((u16)7)<<3);
	SPIx->CR1 |=(((u16)Speed)<<3);  //BR:000-FPCLK/2^^^^^111-FPCLK/256
	SPIx->CR1 |=((u16)(1<<6)) ; //включить s p1		

}

char SPI2_ReadWriteByte(U8 *pwdata , U8 *prdata)
{
	U32 timeouttick;
	//U8 res;
	timeouttick = Tick;
	while(!(SPI2->SR & SPI_I2S_FLAG_TXE))
	{
		if((Tick - timeouttick) > 500)
			return (char)-1;
	}
	SPI2->DR = *pwdata;
	timeouttick = Tick;
	while(!(SPI2->SR & SPI_I2S_FLAG_RXNE))
	{
		if((Tick - timeouttick) > 500)
			return (char)-1;
	}
	*prdata = SPI2->DR;
	return 0;
}


#define SD_SPIx_rw_byte(pWdata, pRdata)	SPI2_ReadWriteByte(pWdata, pRdata)
#define VS_SPI_SET_SPEED(n)				SPIx_SetSpeed(SPI2, n)

U8 SD_Type=0;


U8 SDCardInit(void)
{
	U16 i;      // Используется для подсчета циклов
    U8 r1;      // Сохраните возвращаемое значение SD-карты
    U16 retry;  // Используется для подсчета тайм-аута
    U8 buff[6];
	U8 wbuf;
	U8 rbuf;
    //Если вставка карты не обнаружена, выйдите напрямую и верните флаг ошибки.
    /*
    if((IO_SD_DC_PORT->IDR & IO_IN_SD_DC) == IO_IN_SD_DC)
    {
		вернуть STA_NODISK; //Флаг ошибки FatFS: диск не вставлен
    }
    */
	VS_SPI_SET_SPEED(7);
    //Включение SD-карты
    //SD_PWR_ON();

    //Сначала сгенерируйте >74 импульсов и дайте SD-карте инициализироваться.
    wbuf = 0xff;
    for(i=0;i<10;i++)
    {
        SD_SPIx_rw_byte(&wbuf, &rbuf);
    }

    //------------------Сброс SD-карты в режим ожидания-----------------
    //Отправляйте CMD0 непрерывно в цикле, пока SD-карта не вернет 0x01 и не перейдет в состояние IDLE.
    //Выйти напрямую, если произойдет тайм-аут
    retry = 0;
    do
    {
        //Отправьте cm d0, чтобы перевести SD-карту в состояние ожидания.
        r1 = SD_SendCommand(0, 0, 0x95);
        retry++;
    }while((r1 != 0x01) && (retry<250));
	
    //После выхода из цикла проверьте причину: инициализация прошла успешно? или повторить тайм-аут?
    //------------------Сброс SD-карты в режим ожидания-----------------

	

    //Получить информацию о версии SD-карты
    r1 = SD_SendCommand_NoDeassert(8, 0x1aa, 0x87);

    //Если информация о версии карты соответствует версии v1.0, то есть r1=0x05, выполните следующую инициализацию.
    if(r1 == 0x05)
    {
        //Установите тип карты SD v1.0. Если позже она будет определена как карта mmc, измените ее на mmc.
        SD_Type = SD_TYPE_V1;

        //Если это карта V1.0, последующие данные после команды CMD8 отсутствуют.
        //Установите высокий уровень выбора чипа, чтобы завершить эту команду.
        CS_HIGH();
        //Отправьте еще 8 кликов, чтобы SD завершила последующие операции.
		wbuf = 0xff;
		SD_SPIx_rw_byte(&wbuf, &rbuf);

        //Sd-карта, начинается инициализация карты MMC

        //Команда инициализации выпуска карты CMD55+ACMD41
        // Если есть ответ, значит дело в SD-карте и инициализация завершена.
        // Нет ответа, значит это карта ММС, требуется дополнительная инициализация.
        retry = 0;
        do
        {
            //Сначала отправьте cm d55 и он должен вернуть 0x01; в противном случае возникает ошибка
            r1 = SD_SendCommand(55, 0, 0);
            if(r1 != 0x01)
            {
								return r1;  
            }
            //Получив правильный ответ, отправьте acm d41, и вы должны получить возвращаемое значение 0x00, в противном случае повторите попытку 200 раз.
            r1 = SD_SendCommand(41, 0, 0);
            retry++;
        }while((r1!=0x00) && (retry<400));
        // Определите, истекло ли время ожидания или получен правильный ответ.
        // Если есть ответ: это SD-карта; если нет ответа: это карта MMC
        
        //Начинается операция дополнительной инициализации карты MMC.
        if(retry==400)
        {
            retry = 0;
            //Отправьте команду инициализации карты ммс (не проверялось)
            do
            {
                r1 = SD_SendCommand(1, 0, 0);
                retry++;
            }while((r1!=0x00)&& (retry<400));
            if(retry==400)
            {
								return 1;   //Тайм-аут инициализации карты MMC
            }
            //Напишите тип карты
            SD_Type = SD_TYPE_MMC;
        }
        //Операция дополнительной инициализации карты MMC завершена
        
        //Установите spi в высокоскоростной режим
        VS_SPI_SET_SPEED(4);

		wbuf = 0xff;
		SD_SPIx_rw_byte(&wbuf, &rbuf);
        
        //Отключить проверку CRC
        /*
		r1 = SD_SendCommand(CMD59, 0, 0x01);
        если (r1 != 0x00)
        {
            вернуть р1; //Ошибка команды, возвращаем r1
        }
        */    
        //Установить размер сектора
        r1 = SD_SendCommand(16, 512, 0xff);
        if(r1 != 0x00)
        {
			return r1;  //Ошибка команды, верните r1
        }
        //Инициализация SD-карты и MMC-карты завершена

    }//SD-карта инициализирована для версии v1.0.
    

    //Ниже приведена инициализация карты V2.0.
    //Необходимо прочитать данные OCR, чтобы определить, является ли это картой SD2.0 или SD2.0HC.
    else if(r1 == 0x01)
    {
        //Для карт V2.0 после команды cm d8 будут возвращены 4 байта данных. Вам нужно пропустить и завершить эту команду.
        wbuf = 0xff;
		SD_SPIx_rw_byte(&wbuf, &buff[0]);
		SD_SPIx_rw_byte(&wbuf, &buff[1]);
		SD_SPIx_rw_byte(&wbuf, &buff[2]);
		SD_SPIx_rw_byte(&wbuf, &buff[3]);

     
        CS_HIGH();
        //the next 8 clocks
        SD_SPIx_rw_byte(&wbuf, &rbuf);
        
        //Определите, поддерживает ли карта диапазон напряжения 2,7–3,6 В.
        //if(buff[2]==0x01 && buff[3]==0xAA)
        if(1)
        {
            //Поддерживает диапазон напряжения и может работать
            retry = 0;
            //Команда инициализации выпуска карты cm d55+acm d41
    		do
    		{
    			r1 = SD_SendCommand(55, 0, 0);
    			if(r1!=0x01)
    			{
						return r1;
    			}
    			r1 = SD_SendCommand(41, 0x40000000, 0);
                //если(повторить>200)   
                //{
								//		вернуть р1; //возвращаемся в статус r1, когда происходит таймаут
                //}
            }while(r1!=0);
          
            //Отправляется команда инициализации, а затем получается информация OCR.

            //Начинается версия идентификационной карты d2.0
            r1 = SD_SendCommand_NoDeassert(58, 0, 0);
            if(r1!=0x00)
            {
				return r1;  //Если команда не возвращает правильный ответ, выйдите сразу и верните ответ.
            }
            //После выдачи команды чтения ocr за ней следуют 4 байта информации ocr.
			wbuf = 0xff;
			SD_SPIx_rw_byte(&wbuf, &buff[0]);
			SD_SPIx_rw_byte(&wbuf, &buff[1]);
			SD_SPIx_rw_byte(&wbuf, &buff[2]);
			SD_SPIx_rw_byte(&wbuf, &buff[3]);

            //Прием OCR завершен, выбор чипа установлен на высокий уровень
            CS_HIGH();
            SD_SPIx_rw_byte(&wbuf, &buff[2]);

            //Проверьте бит 30 (CCS) в полученном OCR, чтобы определить, SD2.0 это или SDHC.
            //Если CCS=1: SDHC CCS=0: SD2.0
            if(buff[0]&0x40)    //检查CCS
            {
                SD_Type = SD_TYPE_V2HC;
            }
            else
            {
                SD_Type = SD_TYPE_V2;
            }
            //Окончание идентификации версии карты s d2.0 
        }

    }
    return r1;
}

U8 SD_SendCommand_NoDeassert(U8 cmd, U32 arg, U8 crc)
{
	U8 r1;
	U8 Retry;
	U8 wbuf;
	U8 rbuf;
	U8 i;

	VS_SPI_SET_SPEED(6);

	wbuf = 0xff;
	/*
	CS_HIGH();
	SD_SPIx_rw_byte(&wbuf, &rbuf);
	SD_SPIx_rw_byte(&wbuf, &rbuf);
	SD_SPIx_rw_byte(&wbuf, &rbuf);
	*/
	//Установите терминал выбора чипа на низкий уровень, чтобы выбрать SD-карту.
	CS_LOW();

	//Ожидание занято
	wbuf = 0xff;
	for(i=0;i<250;i++)
	{
		rbuf = 0x00;
		SD_SPIx_rw_byte(&wbuf, &rbuf);
		if(rbuf == 0xff)
			break;
	}
	
	//отправлять
	wbuf = cmd | 0x40;
	SD_SPIx_rw_byte(&wbuf, &rbuf);
	wbuf = arg >> 24;
	SD_SPIx_rw_byte(&wbuf, &rbuf);
	wbuf = arg >> 16;
	SD_SPIx_rw_byte(&wbuf, &rbuf);
	wbuf = arg >> 8;
	SD_SPIx_rw_byte(&wbuf, &rbuf);
	wbuf = arg;
	SD_SPIx_rw_byte(&wbuf, &rbuf);
	wbuf = crc;
	SD_SPIx_rw_byte(&wbuf, &rbuf);

	//Дождитесь ответа или выйдите по истечении времени ожидания.
	wbuf = 0xff;
	SD_SPIx_rw_byte(&wbuf, &r1);
	r1 = 0xff;
	Retry = 0;
	while(r1 == 0xff)
	{
		Retry++;
		if(Retry > 200)
		{
		    break;
		}
		r1 = 0xff;
		SD_SPIx_rw_byte(&wbuf, &r1);
	}
	//Возвращаемое значение ответа
	return r1;
}



static U8 SD_SendCommand(U8 Command, U32 Arg, U8 CRCDat)
{
	U8 wdata;
	U8 rdata;
	U8 Cmd[6];
	U8 R1;
	U8 i;
	U8 Retry;

	VS_SPI_SET_SPEED(6);
	
	//Заполните параметры
	Cmd[0] = 0x40 | Command;
	Cmd[1] = Arg >> 24;
	Cmd[2] = Arg >> 16;
	Cmd[3] = Arg >> 8;
	Cmd[4] = Arg;
	Cmd[5] = CRCDat;

	wdata = 0xff;
	/*
	CS_HIGH();
	SD_SPIx_rw_byte(&wdata, &rdata);
	SD_SPIx_rw_byte(&wdata, &rdata);
	SD_SPIx_rw_byte(&wdata, &rdata);
	*/
	CS_LOW();
	//Ожидание занято
	wdata = 0xff;
	for(i=0;i<250;i++)
	{
		rdata = 0x00;
		SD_SPIx_rw_byte(&wdata, &rdata);
		if(rdata == 0xff)
			break;
	}

	//Отправка инструкций и параметров
	for(i=0;i<6;i++)
		SD_SPIx_rw_byte(&Cmd[i], &rdata);

	wdata = 0xff;
	SD_SPIx_rw_byte(&wdata, &R1);
	Retry = 0;
	while(R1 == 0xff)
	{
		if(Retry++ > 200)
			break;
		SD_SPIx_rw_byte(&wdata, &R1);
	}

	CS_HIGH();
	SD_SPIx_rw_byte(&wdata, &rdata);
	return R1;
}

#if _USE_IOCTL

DRESULT TM_FATFS_SD_SPI_disk_ioctl(BYTE cmd, char *buff)
{
	switch (cmd) 
	{
		case GET_SECTOR_SIZE :     // Get R/W sector size (WORD) 
			*(WORD * )buff = 512;
		break;
		case GET_BLOCK_SIZE :      // Get erase block size in unit of sector (DWORD)
			*(DWORD * )buff = 512;
		break;

		/*
		case GET_SECTOR_COUNT:
			*(DWORD * )buff = SDCardInfo.CardCapacity/SDCardInfo.CardBlockSize;
			break;
		*/
		case CTRL_SYNC :
		break;
	}
	return RES_OK;
}

#endif


BYTE Rd_Block(DWORD Addr, BYTE* Buff)
{
	U16 i;
	U16 Retry;
	U8 wdata;
	U8 rdata; 



	if(SD_Type != SD_TYPE_V2HC)
		Addr = Addr << 9;
	
	Retry = 0;
	
	while(SD_SendCommand(17, Addr, 0xff) != 0)
	{
		if(++Retry >= 250)
			return 1;
	}

	if(SD_Type != SD_TYPE_V2HC)
		VS_SPI_SET_SPEED(3);
	else
		VS_SPI_SET_SPEED(4);
	
	CS_LOW();
	
	//for(i=0;i<36;i++);
	//Read Start Byte
	
	rdata = 0xff;
	wdata = 0xff;
	Retry = 0;
	while(rdata != 0xfe)
	{
		rdata = 0xff;
		SD_SPIx_rw_byte(&wdata, &rdata);
		if(++Retry >= 0x2000)
		{
			CS_HIGH();
			wdata = Retry;
			return wdata;
		}
	}
	//Read Data
	for(i=0;i<512;i++)
		SD_SPIx_rw_byte(&wdata, Buff++);
	//Read CRC, Don't Care
	wdata = 0xff;
	SD_SPIx_rw_byte(&wdata, &rdata);
	SD_SPIx_rw_byte(&wdata, &rdata);
	CS_HIGH();
	SD_SPIx_rw_byte(&wdata, &rdata);
	return 0;
}


U8 Wt_Block(U32 Addr, U8 *Buff)
{
	U16 i;
	U16 Retry;
	U8 wdata;
	U8 rdata;

	if(SD_Type != SD_TYPE_V2HC)
		Addr = Addr << 9;
	
	Retry = 0;
	while(SD_SendCommand(24, Addr, 0) != 0)
	{
		if(++Retry >= 500)
			return 1;
	}

	if(SD_Type != SD_TYPE_V2HC)
		VS_SPI_SET_SPEED(3);
	else
		VS_SPI_SET_SPEED(4);
	CS_LOW();
	wdata = 0xff;
	for(i=0;i<3;i++)
		SD_SPIx_rw_byte(&wdata, &rdata);
	wdata = 0xfe;
	SD_SPIx_rw_byte(&wdata, &rdata);

	//записать данные
	for(i=0;i<512;i++)
		SD_SPIx_rw_byte(Buff++, &rdata);

	//написать CRC
	wdata = 0xff;
	SD_SPIx_rw_byte(&wdata, &rdata);
	SD_SPIx_rw_byte(&wdata, &rdata);

	//Записанные данные не были приняты
	rdata = 0;
	SD_SPIx_rw_byte(&wdata, &rdata);
	if((rdata & 0x1f) != 0x05)
	{
		CS_HIGH();
		return 1;
	}
	//Ожидание сохранения данных SD-карты
	rdata = 0;
	while(rdata != 0xff)
	{
		rdata = 0;
		SD_SPIx_rw_byte(&wdata, &rdata);
	}
	CS_HIGH();
	SD_SPIx_rw_byte(&wdata, &rdata);
	return 0;
}

