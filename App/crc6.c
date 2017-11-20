#include "includes.h"
/*
* The CRC parameters. Currently configured for CCITT.
* Simply modify these to switch to another CRC Standard.
*/
/*
#define POLYNOMIAL          0x8005
#define INITIAL_REMAINDER   0x0000
#define FINAL_XOR_VALUE     0x0000
*/
#define POLYNOMIAL          0x0043
#define INITIAL_REMAINDER   0xFFFF
#define FINAL_XOR_VALUE     0x0000

/*
#define POLYNOMIAL          0x1021
#define POLYNOMIAL          0xA001
#define INITIAL_REMAINDER   0xFFFF
#define FINAL_XOR_VALUE     0x0000
*/

/*
* The width of the CRC calculation and result.
* Modify the typedef for an 8 or 32-bit CRC standard.
*/
//typedef unsigned short width_t;
#define WIDTH 6
#define TOPBIT (0x0080)
#define POLY (POLYNOMIAL<<(8-WIDTH))

/**
 * Initialize the CRC lookup table.
 * This table is used by crcCompute() to make CRC computation faster.
 */
void crcInit(void);

/**
 * Compute the CRC checksum of a binary message block.
 * @para message, 用来计算的数据
 * @para nBytes, 数据的长度
 * @note This function expects that crcInit() has been called
 *       first to initialize the CRC lookup table.
 */
INT8S crcCompute(INT8U * message, INT16U nBytes);
/*
* An array containing the pre-computed intermediate result for each
* possible byte of input. This is used to speed up the computation.
*/
static INT8S crcTable[256];

/**
 * Initialize the CRC lookup table.
 * This table is used by crcCompute() to make CRC computation faster.
 */
void crcInit(void)
{
    int remainder;
    int dividend;
    int bit;
    /* Perform binary long division, a bit at a time. */
    for(dividend = 0; dividend < 256; dividend++)
    {
        /* Initialize the remainder.  */
        remainder = dividend;
        /* Shift and XOR with the polynomial.   */
        for(bit = 0; bit < 8; bit++)
        {
            /* Try to divide the current data bit.  */
            if(remainder & TOPBIT)
            {
                remainder = (remainder<<1)^POLY;
            }
            else
            {
                remainder = remainder << 1;
            }
        }
        /* Save the result in the table. */
        crcTable[dividend] = (remainder);// >> (8-WIDTH));
    }
} /* crcInit() */

/**
 * Compute the CRC checksum of a binary message block.
 * @para message, 用来计算的数据
 * @para nBytes, 数据的长度
 * @note This function expects that crcInit() has been called
 *       first to initialize the CRC lookup table.
 */
INT8S crcCompute(INT8U * message, INT16U nBytes)
{
    INT16U offset;
    INT8U byte;
    INT8S remaind = 0;// INITIAL_REMAINDER;
    INT8S test;
    byte=0;
    /* Divide the message by the polynomial, a byte at a time. */
    for( offset = 0; offset <= nBytes; offset++)
    {
        remaind = crcTable[byte];
        byte = remaind^*message;
        //test=remaind>>2;
        message--;
    }
    /* The final remainder is the CRC result. */
    return (remaind>>(8-WIDTH));// ^ FINAL_XOR_VALUE);
} /* crcCompute() */
#if 0
main()
{
    int crc_r;
    //char dat1[]={0x5A,0x2D,0x7B,0x18};
    char dat1[]= {0,0x16,0x8b,0x5e,0xc6,0x2d^0x3f};
    char dat2[]= {0,0x1a,0x38,0x7b,0x01,0x0f^0x3f};
    char dat3[]= {0,0x1a,0x38,0x45,0x00,0x33^0x3f};
    crcInit();
    crc_r=crcCompute(dat1,6);
    crc_r=crcCompute(dat2,6);
    crc_r=crcCompute(dat3,6);
    crc_r=crcCompute(dat1,6);
}
void SystemInit (void)
{
}
#endif
