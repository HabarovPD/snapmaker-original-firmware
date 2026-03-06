#ifndef _TYPEDEF_H_
#define _TYPEDEF_H_

#include "Macro.h"
#include "stdint.h"
//#include "Configuration_adv.h"
//#include "Configuration.h"
#define QWORD   uint64_t
#define DWORD	unsigned long
#define CHAR	char
#define WORD	unsigned short
#define BYTE	unsigned char


#define U32	unsigned long
#define U16	unsigned short
#define U8	unsigned char
#define WCHAR unsigned short
//#define bool unsigned long

#define INT int;
#define UINT unsigned long

/* These types MUST be 32 bit */
#define LONG long



typedef  void	(*pFunction)(void);

/*
#define FALSE	0
#define TRUE	(!FALSE)
#define false	0
#define true	(!false)
*/

typedef struct
{
	U16 Head;
	U16 Tail;
	U8 Buff[RINGBUFF_SEND_SIZE];
}strringbuffsend;

typedef struct
{
	U16 Head;
	U16 Tail;
	U8 Buff[RINGBUFF_RECV_SIZE];
}strringbuffrecv;





#endif
