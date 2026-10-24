#define _IN_SDCARDDRIVER
#include "includes.h"
#include "diskio.h"

#define CS_HIGH()	(GPIOB->BSRR = GPIO_Pin_12)
#define CS_LOW()	(GPIOB->BRR = GPIO_Pin_12)

/* SD卡类型定义 */
#define SD_TYPE_MMC     0
#define SD_TYPE_V1      1
#define SD_TYPE_V2      2
#define SD_TYPE_V2HC    4



static U8 SD_SendCommand(U8 Command, U32 Arg, U8 CRCDat);
U8 SD_SendCommand_NoDeassert(U8 cmd, U32 arg, U8 crc);


void SPIx_SetSpeed(SPI_TypeDef *SPIx, U8 Speed)
{
	SPIx->CR1 &=~((u16)(1<<6)) ; //停SP1
	SPIx->CR1 &=~(((u16)7)<<3);
	SPIx->CR1 |=(((u16)Speed)<<3);  //BR:000-FPCLK/2^^^^^111-FPCLK/256
	SPIx->CR1 |=((u16)(1<<6)) ; //使能SP1		

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
	U16 i;      // 用来循环计数
    U8 r1;      // 存放SD卡的返回值
    U16 retry;  // 用来进行超时计数
    U8 buff[6];
	U8 wbuf;
	U8 rbuf;
    //如果没有检测到卡插入，直接退出，返回错误标志
    /*
    if((IO_SD_DC_PORT->IDR & IO_IN_SD_DC) == IO_IN_SD_DC)
    {
		return STA_NODISK;  //  FatFS错误标志：没有插入磁盘
    }
    */
	VS_SPI_SET_SPEED(7);
    //SD卡上电
    //SD_PWR_ON();

    //先产生>74个脉冲，让SD卡自己初始化完成
    wbuf = 0xff;
    for(i=0;i<10;i++)
    {
        SD_SPIx_rw_byte(&wbuf, &rbuf);
    }

    //-----------------SD卡复位到idle开始-----------------
    //循环连续发送CMD0，直到SD卡返回0x01,进入IDLE状态
    //超时则直接退出
    retry = 0;
    do
    {
        //发送CMD0，让SD卡进入IDLE状态
        r1 = SD_SendCommand(0, 0, 0x95);
        retry++;
    }while((r1 != 0x01) && (retry<250));
	
    //跳出循环后，检查原因：初始化成功？or 重试超时？
    //-----------------SD卡复位到idle结束-----------------

	

    //获取卡片的SD版本信息
    r1 = SD_SendCommand_NoDeassert(8, 0x1aa, 0x87);

    //如果卡片版本信息是v1.0版本的，即r1=0x05，则进行以下初始化
    if(r1 == 0x05)
    {
        //设置卡类型为SDV1.0，如果后面检测到为MMC卡，再修改为MMC
        SD_Type = SD_TYPE_V1;

        //如果是V1.0卡，CMD8指令后没有后续数据
        //片选置高，结束本次命令
        CS_HIGH();
        //多发8个CLK，让SD结束后续操作
		wbuf = 0xff;
		SD_SPIx_rw_byte(&wbuf, &rbuf);

        //-----------------SD卡、MMC卡初始化开始-----------------

        //发卡初始化指令CMD55+ACMD41
        // 如果有应答，说明是SD卡，且初始化完成
        // 没有回应，说明是MMC卡，额外进行相应初始化
        retry = 0;
        do
        {
            //先发CMD55，应返回0x01；否则出错
            r1 = SD_SendCommand(55, 0, 0);
            if(r1 != 0x01)
            {
								return r1;  
            }
            //得到正确响应后，发ACMD41，应得到返回值0x00，否则重试200次
            r1 = SD_SendCommand(41, 0, 0);
            retry++;
        }while((r1!=0x00) && (retry<400));
        // 判断是超时还是得到正确回应
        // 若有回应：是SD卡；没有回应：是MMC卡
        
        //----------MMC卡额外初始化操作开始------------
        if(retry==400)
        {
            retry = 0;
            //发送MMC卡初始化命令（没有测试）
            do
            {
                r1 = SD_SendCommand(1, 0, 0);
                retry++;
            }while((r1!=0x00)&& (retry<400));
            if(retry==400)
            {
								return 1;   //MMC卡初始化超时
            }
            //写入卡类型
            SD_Type = SD_TYPE_MMC;
        }
        //----------MMC卡额外初始化操作结束------------
        
        //设置SPI为高速模式
        VS_SPI_SET_SPEED(4);

		wbuf = 0xff;
		SD_SPIx_rw_byte(&wbuf, &rbuf);
        
        //禁止CRC校验
        /*
		r1 = SD_SendCommand(CMD59, 0, 0x01);
        if(r1 != 0x00)
        {
            return r1;  //命令错误，返回r1
        }
        */    
        //设置Sector Size
        r1 = SD_SendCommand(16, 512, 0xff);
        if(r1 != 0x00)
        {
			return r1;  //命令错误，返回r1
        }
        //-----------------SD卡、MMC卡初始化结束-----------------

    }//SD卡为V1.0版本的初始化结束
    

    //下面是V2.0卡的初始化
    //其中需要读取OCR数据，判断是SD2.0还是SD2.0HC卡
    else if(r1 == 0x01)
    {
        //V2.0的卡，CMD8命令后会传回4字节的数据，要跳过再结束本命令
        wbuf = 0xff;
		SD_SPIx_rw_byte(&wbuf, &buff[0]);
		SD_SPIx_rw_byte(&wbuf, &buff[1]);
		SD_SPIx_rw_byte(&wbuf, &buff[2]);
		SD_SPIx_rw_byte(&wbuf, &buff[3]);

     
        CS_HIGH();
        //the next 8 clocks
        SD_SPIx_rw_byte(&wbuf, &rbuf);
        
        //判断该卡是否支持2.7V-3.6V的电压范围
        //if(buff[2]==0x01 && buff[3]==0xAA)
        if(1)
        {
            //支持电压范围，可以操作
            retry = 0;
            //发卡初始化指令CMD55+ACMD41
    		do
    		{
    			r1 = SD_SendCommand(55, 0, 0);
    			if(r1!=0x01)
    			{
						return r1;
    			}
    			r1 = SD_SendCommand(41, 0x40000000, 0);
                //if(retry>200)   
                //{
								//		return r1;  //超时则返回r1状态
                //}
            }while(r1!=0);
          
            //初始化指令发送完成，接下来获取OCR信息

            //-----------鉴别SD2.0卡版本开始-----------
            r1 = SD_SendCommand_NoDeassert(58, 0, 0);
            if(r1!=0x00)
            {
				return r1;  //如果命令没有返回正确应答，直接退出，返回应答
            }
            //读OCR指令发出后，紧接着是4字节的OCR信息
			wbuf = 0xff;
			SD_SPIx_rw_byte(&wbuf, &buff[0]);
			SD_SPIx_rw_byte(&wbuf, &buff[1]);
			SD_SPIx_rw_byte(&wbuf, &buff[2]);
			SD_SPIx_rw_byte(&wbuf, &buff[3]);

            //OCR接收完成，片选置高
            CS_HIGH();
            SD_SPIx_rw_byte(&wbuf, &buff[2]);

            //检查接收到的OCR中的bit30位（CCS），确定其为SD2.0还是SDHC
            //如果CCS=1：SDHC   CCS=0：SD2.0
            if(buff[0]&0x40)    //检查CCS
            {
                SD_Type = SD_TYPE_V2HC;
            }
            else
            {
                SD_Type = SD_TYPE_V2;
            }
            //-----------鉴别SD2.0卡版本结束----------- 
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
	//片选端置低，选中SD卡
	CS_LOW();

	//等待忙
	wbuf = 0xff;
	for(i=0;i<250;i++)
	{
		rbuf = 0x00;
		SD_SPIx_rw_byte(&wbuf, &rbuf);
		if(rbuf == 0xff)
			break;
	}
	
	//发送
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

	//等待响应，或超时退出
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
	//返回响应值
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
	
	//填写参数
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
	//等待忙
	wdata = 0xff;
	for(i=0;i<250;i++)
	{
		rdata = 0x00;
		SD_SPIx_rw_byte(&wdata, &rdata);
		if(rdata == 0xff)
			break;
	}

	//发送指令与参数
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

	//写数据
	for(i=0;i<512;i++)
		SD_SPIx_rw_byte(Buff++, &rdata);

	//写CRC
	wdata = 0xff;
	SD_SPIx_rw_byte(&wdata, &rdata);
	SD_SPIx_rw_byte(&wdata, &rdata);

	//写入的数据未被接受
	rdata = 0;
	SD_SPIx_rw_byte(&wdata, &rdata);
	if((rdata & 0x1f) != 0x05)
	{
		CS_HIGH();
		return 1;
	}
	//等待SD  卡数据存储
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

