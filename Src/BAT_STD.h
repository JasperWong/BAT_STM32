#ifndef _BAT_STD_H_
#define _BAT_STD_H_


/* exact-width signed integer types */
typedef   signed          char int8_x;
typedef   signed short     int int16_x;
typedef   signed           int int32_x;


/* exact-width unsigned integer types */
typedef unsigned          char uint8_x;
typedef unsigned short     int uint16_x;
typedef unsigned           int uint32_x;


/* boolean types */
typedef   signed          char bool_x;
#define true_x (!0)
#define false_x (0)


/* define useful NULL */
#ifndef NULL 
#ifdef __cplusplus 
#define NULL    0 
#else 
#define NULL    ((void *)0) 
#endif 
#endif

#endif
