#ifndef _TYPE_DEF_
#define _TYPE_DEF_

#include "common.h"

typedef volatile unsigned char U8;
typedef volatile char S8;
typedef volatile unsigned short U16;
typedef volatile short S16;
typedef volatile unsigned long int U32;
typedef volatile long int S32;
typedef volatile unsigned long long int U64;
typedef volatile long long int S64;

typedef unsigned char u8;
typedef char s8;
typedef unsigned short u16;
typedef short s16;
typedef unsigned long int u32;
typedef long int s32;
typedef unsigned long long int u64;
typedef long long int s64;

#define abs(x) (x>0?x:(-x))
#define Nop asm("nop")


#endif