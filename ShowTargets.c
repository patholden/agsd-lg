/*
static char rcsid[] = "$Id$";
*/
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <linux/tcp.h>

#include "BoardComm.h"
#include "comm_loop.h"
#include "parse_data.h"
#include "SensorSearch.h"
#include "Protocol.h"
#include "ShowTargets.h"
#include "LaserInterface.h"

#define kMaxNumberX                                        0x7FFFC000U
#define kMinNumberX                                        0x80000000U
#define kMaxNumberY                                        0x7FFFC000U
#define kMinNumberY                                        0x80000000U

#define kMaxUnsigned  0xFFFFC000U



#define NPOINTS 2048

   /*
    *  size based on default values of gNumberOfSpiral
    *  and gCoarseSearchStep
    */
static int32_t gNumberStep =  64 * 0x00080000U;

static int32_t gCoarse3SearchStep;

static void DoCross( unsigned int currentX, unsigned int currentY,
                char * tmpPtr, int * pIndex);

static void DoSquare( unsigned int currentX, unsigned int currentY,
                char * tmpPtr, int * pIndex);

static void Do1( unsigned int currentX, unsigned int currentY,
                char * tmpPtr, int * pIndex);

static void Do2( unsigned int currentX, unsigned int currentY,
                char * tmpPtr, int * pIndex);

static void Do3( unsigned int currentX, unsigned int currentY,
                char * tmpPtr, int * pIndex);

static void Do4( unsigned int currentX, unsigned int currentY,
                char * tmpPtr, int * pIndex);

static void Do5( unsigned int currentX, unsigned int currentY,
                char * tmpPtr, int * pIndex);

static void Do6( unsigned int currentX, unsigned int currentY,
                char * tmpPtr, int * pIndex);

static void Do7( unsigned int currentX, unsigned int currentY,
                char * tmpPtr, int * pIndex);

static void Do8( unsigned int currentX, unsigned int currentY,
                char * tmpPtr, int * pIndex);

static void Do9( unsigned int currentX, unsigned int currentY,
                char * tmpPtr, int * pIndex);

static void Do10( unsigned int currentX, unsigned int currentY,
                  char * tmpPtr, int * pIndex);

static void Do11( unsigned int currentX, unsigned int currentY,
                  char * tmpPtr, int * pIndex);

static void Do12( unsigned int currentX, unsigned int currentY,
                  char * tmpPtr, int * pIndex);

static void Do13( unsigned int currentX, unsigned int currentY,
                  char * tmpPtr, int * pIndex);

static void Do14( unsigned int currentX, unsigned int currentY,
                  char * tmpPtr, int * pIndex);

static void Do15( unsigned int currentX, unsigned int currentY,
                  char * tmpPtr, int * pIndex);

static void Do16( unsigned int currentX, unsigned int currentY,
                  char * tmpPtr, int * pIndex);

static void Do17( unsigned int currentX, unsigned int currentY,
                  char * tmpPtr, int * pIndex);

static void Do18( unsigned int currentX, unsigned int currentY,
                  char * tmpPtr, int * pIndex);

static void Do19( unsigned int currentX, unsigned int currentY,
                  char * tmpPtr, int * pIndex);

static void Do20( unsigned int currentX, unsigned int currentY,
                  char * tmpPtr, int * pIndex);

static void Do21( unsigned int currentX, unsigned int currentY,
                  char * tmpPtr, int * pIndex);

static void Do22( unsigned int currentX, unsigned int currentY,
                  char * tmpPtr, int * pIndex);

static void Do23( unsigned int currentX, unsigned int currentY,
                  char * tmpPtr, int * pIndex);
static void Do24( unsigned int currentX, unsigned int currentY,
                  char * tmpPtr, int * pIndex);
static void show_move( unsigned int lastX, unsigned int lastY,
                unsigned int currentX, unsigned int currentY,
                char * tmpPtr, int * pIndex);

//Start of local functions
void DoShowTargets(struct lg_master *pLgMaster, char * Parameters, uint32_t respondToWhom )
{
    char * wr_buf;
    char * tmpPtr;
    int maxSize;
    int index;
    int offset;
    double * pExternalAngles;
    int nTargets; 
    double Xtemp;
    double Ytemp;
    uint32_t Xi;
    uint32_t Yi;
    uint32_t Xarr[32];
    uint32_t Yarr[32];
    uint32_t flag[32];
    uint32_t currentX;
    uint32_t currentY;
    uint32_t lastX;
    uint32_t lastY;
    int i;
    uint32_t resp_len=(sizeof(uint32_t) + kCRCSize);
    struct parse_basic_resp *pResp = (struct parse_basic_resp *)pLgMaster->theResponseBuffer;

    memset((char *)pResp, 0, resp_len);
    maxSize = NPOINTS * 24;
    wr_buf = (char *)calloc( NPOINTS * 24, 4 );

    tmpPtr = wr_buf;

    offset = 0;
    nTargets = *(int32_t *)(&(Parameters)[offset]);

    gCoarse3SearchStep = 0x00080000;

#if ZDEBUG
    fprintf( stderr
           , "ShowTarget gCoarseSearchStep %x  (2) %x  (3)  %x\n"
           , gCoarseSearchStep
           , gCoarse2SearchStep
           , gCoarse3SearchStep
           );
    fprintf( stderr
           , "ShowTarget gNumberOfSpirals  %d\n"
           , gNumberOfSpirals
           );
#endif

    for ( i = 0; i < nTargets; i++ ) {
        offset = sizeof(int32_t) + i * (kSizeOf1TargetData);
        pExternalAngles = (double *)(&(Parameters)[offset]);
        Xtemp   = pExternalAngles[ 0 ];
        Ytemp   = pExternalAngles[ 1 ];
        ConvertExternalAnglesToBinary( Xtemp, Ytemp, &Xi, &Yi );
        Xarr[i] = Xi;
        Yarr[i] = Yi;
        offset = sizeof(int32_t) + i * (kSizeOf1TargetData) + 2 * sizeof(double);
        flag[i] = *((uint32_t *)(&(Parameters)[offset]));
    }

    /*
     * draw marker
     */
    index = 0;
    lastX = Xarr[0];
    lastY = Yarr[0];
    for ( i = 0; i < nTargets; i++ ) {
#ifdef ZDEBUG
fprintf(stderr,"ShowTargets i %3d  ptr %08x  dex %5d\n", i, tmpPtr, index );
#endif
        if ( flag[i] == 1 ) {
           currentX = Xarr[i];
           currentY = Yarr[i];
           show_move( lastX, lastY, currentX, currentY, tmpPtr, &index );
           lastX = currentX;
           lastY = currentY;
           Do1( currentX, currentY, tmpPtr, &index );
        }
        if ( flag[i] == 2 ) {
           currentX = Xarr[i];
           currentY = Yarr[i];
           show_move( lastX, lastY, currentX, currentY, tmpPtr, &index );
           lastX = currentX;
           lastY = currentY;
           Do2( currentX, currentY, tmpPtr, &index );
        }
        if ( flag[i] == 3 ) {
           currentX = Xarr[i];
           currentY = Yarr[i];
           show_move( lastX, lastY, currentX, currentY, tmpPtr, &index );
           lastX = currentX;
           lastY = currentY;
           Do3( currentX, currentY, tmpPtr, &index );
        }
        if ( flag[i] == 4 ) {
           currentX = Xarr[i];
           currentY = Yarr[i];
           show_move( lastX, lastY, currentX, currentY, tmpPtr, &index );
           lastX = currentX;
           lastY = currentY;
           Do4( currentX, currentY, tmpPtr, &index );
        }
        if ( flag[i] == 5 ) {
           currentX = Xarr[i];
           currentY = Yarr[i];
           show_move( lastX, lastY, currentX, currentY, tmpPtr, &index );
           lastX = currentX;
           lastY = currentY;
           Do5( currentX, currentY, tmpPtr, &index );
        }
        if ( flag[i] == 6 ) {
           currentX = Xarr[i];
           currentY = Yarr[i];
           show_move( lastX, lastY, currentX, currentY, tmpPtr, &index );
           lastX = currentX;
           lastY = currentY;
           Do6( currentX, currentY, tmpPtr, &index );
        }
        if ( flag[i] == 7 ) {
           currentX = Xarr[i];
           currentY = Yarr[i];
           show_move( lastX, lastY, currentX, currentY, tmpPtr, &index );
           lastX = currentX;
           lastY = currentY;
           Do7( currentX, currentY, tmpPtr, &index );
        }
        if ( flag[i] == 8 ) {
           currentX = Xarr[i];
           currentY = Yarr[i];
           show_move( lastX, lastY, currentX, currentY, tmpPtr, &index );
           lastX = currentX;
           lastY = currentY;
           Do8( currentX, currentY, tmpPtr, &index );
        }
        if ( flag[i] == 9 ) {
           currentX = Xarr[i];
           currentY = Yarr[i];
           show_move( lastX, lastY, currentX, currentY, tmpPtr, &index );
           lastX = currentX;
           lastY = currentY;
           Do9( currentX, currentY, tmpPtr, &index );
        }
        if ( flag[i] == 10 ) {
           currentX = Xarr[i];
           currentY = Yarr[i];
           show_move( lastX, lastY, currentX, currentY, tmpPtr, &index );
           lastX = currentX;
           lastY = currentY;
           Do10( currentX, currentY, tmpPtr, &index );
        }
        if ( flag[i] == 11 ) {
           currentX = Xarr[i];
           currentY = Yarr[i];
           show_move( lastX, lastY, currentX, currentY, tmpPtr, &index );
           lastX = currentX;
           lastY = currentY;
           Do11( currentX, currentY, tmpPtr, &index );
        }
        if ( flag[i] == 12 ) {
           currentX = Xarr[i];
           currentY = Yarr[i];
           show_move( lastX, lastY, currentX, currentY, tmpPtr, &index );
           lastX = currentX;
           lastY = currentY;
           Do12( currentX, currentY, tmpPtr, &index );
        }
        if ( flag[i] == 13 ) {
           currentX = Xarr[i];
           currentY = Yarr[i];
           show_move( lastX, lastY, currentX, currentY, tmpPtr, &index );
           lastX = currentX;
           lastY = currentY;
           Do13( currentX, currentY, tmpPtr, &index );
        }
        if ( flag[i] == 14 ) {
           currentX = Xarr[i];
           currentY = Yarr[i];
           show_move( lastX, lastY, currentX, currentY, tmpPtr, &index );
           lastX = currentX;
           lastY = currentY;
           Do14( currentX, currentY, tmpPtr, &index );
        }
        if ( flag[i] == 15 ) {
           currentX = Xarr[i];
           currentY = Yarr[i];
           show_move( lastX, lastY, currentX, currentY, tmpPtr, &index );
           lastX = currentX;
           lastY = currentY;
           Do15( currentX, currentY, tmpPtr, &index );
        }
        if ( flag[i] == 16 ) {
           currentX = Xarr[i];
           currentY = Yarr[i];
           show_move( lastX, lastY, currentX, currentY, tmpPtr, &index );
           lastX = currentX;
           lastY = currentY;
           Do16( currentX, currentY, tmpPtr, &index );
        }
        if ( flag[i] == 17 ) {
           currentX = Xarr[i];
           currentY = Yarr[i];
           show_move( lastX, lastY, currentX, currentY, tmpPtr, &index );
           lastX = currentX;
           lastY = currentY;
           Do17( currentX, currentY, tmpPtr, &index );
        }
        if ( flag[i] == 18 ) {
           currentX = Xarr[i];
           currentY = Yarr[i];
           show_move( lastX, lastY, currentX, currentY, tmpPtr, &index );
           lastX = currentX;
           lastY = currentY;
           Do18( currentX, currentY, tmpPtr, &index );
        }
        if ( flag[i] == 19 ) {
           currentX = Xarr[i];
           currentY = Yarr[i];
           show_move( lastX, lastY, currentX, currentY, tmpPtr, &index );
           lastX = currentX;
           lastY = currentY;
           Do19( currentX, currentY, tmpPtr, &index );
        }
        if ( flag[i] == 20 ) {
           currentX = Xarr[i];
           currentY = Yarr[i];
           show_move( lastX, lastY, currentX, currentY, tmpPtr, &index );
           lastX = currentX;
           lastY = currentY;
           Do20( currentX, currentY, tmpPtr, &index );
        }
        if ( flag[i] == 21 ) {
           currentX = Xarr[i];
           currentY = Yarr[i];
           show_move( lastX, lastY, currentX, currentY, tmpPtr, &index );
           lastX = currentX;
           lastY = currentY;
           Do21( currentX, currentY, tmpPtr, &index );
        }
        if ( flag[i] == 22 ) {
           currentX = Xarr[i];
           currentY = Yarr[i];
           show_move( lastX, lastY, currentX, currentY, tmpPtr, &index );
           lastX = currentX;
           lastY = currentY;
           Do22( currentX, currentY, tmpPtr, &index );
        }
        if ( flag[i] == 23 ) {
           currentX = Xarr[i];
           currentY = Yarr[i];
           show_move( lastX, lastY, currentX, currentY, tmpPtr, &index );
           lastX = currentX;
           lastY = currentY;
           Do23( currentX, currentY, tmpPtr, &index );
        }
        if ( flag[i] == 24 ) {
           currentX = Xarr[i];
           currentY = Yarr[i];
           show_move( lastX, lastY, currentX, currentY, tmpPtr, &index );
           lastX = currentX;
           lastY = currentY;
           Do24( currentX, currentY, tmpPtr, &index );
        }
        if ( flag[i] == 31 ) {
           currentX = Xarr[i];
           currentY = Yarr[i];
           show_move( lastX, lastY, currentX, currentY, tmpPtr, &index );
           lastX = currentX;
           lastY = currentY;
           DoCross( currentX, currentY, tmpPtr, &index );
        }
        if ( flag[i] == 32 ) {
           currentX = Xarr[i];
           currentY = Yarr[i];
           show_move( lastX, lastY, currentX, currentY, tmpPtr, &index );
           lastX = currentX;
           lastY = currentY;
           DoSquare( currentX, currentY, tmpPtr, &index );
        }
    }
    currentX = Xarr[0];
    currentY = Yarr[0];
    show_move( lastX, lastY, currentX, currentY, tmpPtr, &index );

#ifdef ZDEBUG
    for ( i=0; i<index; i+= 8 ) {
      x = *(int32_t *)(&(tmpPtr[i+0]));
      y = *(int32_t *)(&(tmpPtr[i+4]));
      printf( "%9d %9d\n", x, y );
    }
#endif

    if ( index < maxSize ) {
      JustDoDisplay(pLgMaster, wr_buf, index );

       free( wr_buf );
       pResp->hdr.status = RESPGOOD;
       HandleResponse ( pLgMaster, (sizeof(struct parse_basic_resp)-kCRCSize), respondToWhom );
       return;
    } else {
       free( wr_buf );
       pResp->hdr.status = RESPFAIL;
       HandleResponse ( pLgMaster, (sizeof(struct parse_basic_resp)-kCRCSize), respondToWhom );
       return;
    }
}

static void DoCross(
                  unsigned int currentX
                , unsigned int currentY
                , char * tmpPtr
                , int * pIndex
            ) {
    int index;
    int32_t eolXNeg;
    int32_t eolXPos;
    int32_t eolYNeg;
    int32_t eolYPos;
    uint32_t * Xptr;
    uint32_t * Yptr;
    int32_t x;
    int32_t y;
    int j;

    index = *pIndex;

    if ( (int32_t)currentX >= (int32_t)(kMaxNumberX - gNumberStep) ) {
         eolXPos   =  kMaxNumberX;
    } else {
         eolXPos =  (int32_t)currentX            + gNumberStep; 
    }
    if ( (int32_t)currentX <= (int32_t)(kMinNumberX + gNumberStep) ) {
         eolXNeg   =  kMinNumberX;
    } else {
         eolXNeg =  (int32_t)currentX            - gNumberStep; 
    }
    if ( (int32_t)currentY >= (int32_t)(kMaxNumberY - gNumberStep) ) {
         eolYPos   =  kMaxNumberY;
    } else {
         eolYPos =  (int32_t)currentY            + gNumberStep; 
    }
    if ( (int32_t)currentY <= (int32_t)(kMinNumberY + gNumberStep) ) {
         eolYNeg   =  kMinNumberY;
    } else {
         eolYNeg =  (int32_t)currentY            - gNumberStep; 
    }
    x = (int32_t)currentX;
    y = (int32_t)currentY;
    for ( j = 0; j < 10; j++ ) {
        Xptr = (uint32_t *)(&(tmpPtr[index+0]));
        Yptr = (uint32_t *)(&(tmpPtr[index+4]));
        *Xptr = (uint32_t)x;
        *Yptr = (uint32_t)y;
        SetLowBeam( Xptr, Yptr );
        index += 8; 
    }
    for ( j = 0; j < 2; j++ ) {
        Xptr = (uint32_t *)(&(tmpPtr[index+0]));
        Yptr = (uint32_t *)(&(tmpPtr[index+4]));
        *Xptr = (uint32_t)x;
        *Yptr = (uint32_t)y;
        SetLowBeam( Xptr, Yptr );
        index += 8; 
    }
    for ( x=(int32_t)currentX; x <= eolXPos && x >= (int32_t)currentX; x += 64*gCoarse3SearchStep ) {
        Xptr = (uint32_t *)(&(tmpPtr[index+0]));
        Yptr = (uint32_t *)(&(tmpPtr[index+4]));
        *Xptr = (uint32_t)x;
        *Yptr = (uint32_t)y;
        SetHighBeam( Xptr, Yptr );
        index += 8; 
    }
    x = (int32_t)eolXPos;
    y = (int32_t)currentY;
    for ( j = 0; j < 2; j++ ) {
        Xptr = (uint32_t *)(&(tmpPtr[index+0]));
        Yptr = (uint32_t *)(&(tmpPtr[index+4]));
        *Xptr = (uint32_t)x;
        *Yptr = (uint32_t)y;
        SetHighBeam( Xptr, Yptr );
        index += 8;
    }
    for ( x=eolXPos; x >= eolXNeg && x <= eolXPos; x -= 64*gCoarse3SearchStep ) {
        Xptr = (uint32_t *)(&(tmpPtr[index+0]));
        Yptr = (uint32_t *)(&(tmpPtr[index+4]));
        *Xptr = (uint32_t)x;
        *Yptr = (uint32_t)y;
        SetHighBeam( Xptr, Yptr );
        index += 8; 
    }
    x = (int32_t)eolXNeg;
    y = (int32_t)currentY;
    for ( j = 0; j < 2; j++ ) {
        Xptr = (uint32_t *)(&(tmpPtr[index+0]));
        Yptr = (uint32_t *)(&(tmpPtr[index+4]));
        *Xptr = (uint32_t)x;
        *Yptr = (uint32_t)y;
        SetHighBeam( Xptr, Yptr );
        index += 8;
    }
    for ( x = eolXNeg; x<=(int32_t)currentX && x >= eolXNeg; x += 64*gCoarse3SearchStep ) {
        Xptr = (uint32_t *)(&(tmpPtr[index+0]));
        Yptr = (uint32_t *)(&(tmpPtr[index+4]));
        *Xptr = (uint32_t)x;
        *Yptr = (uint32_t)y;
        SetHighBeam( Xptr, Yptr );
        index += 8; 
    }
    x = (int32_t)currentX;
    y = (int32_t)currentY;
    for ( j = 0; j < 2; j++ ) {
        Xptr = (uint32_t *)(&(tmpPtr[index+0]));
        Yptr = (uint32_t *)(&(tmpPtr[index+4]));
        *Xptr = (uint32_t)x;
        *Yptr = (uint32_t)y;
        SetHighBeam( Xptr, Yptr );
        index += 8;
    }
    for ( y=(int32_t)currentY; y >= eolYNeg && y <= (int32_t)currentY; y -= 64*gCoarse3SearchStep ) {
        Xptr = (uint32_t *)(&(tmpPtr[index+0]));
        Yptr = (uint32_t *)(&(tmpPtr[index+4]));
        *Xptr = (uint32_t)x;
        *Yptr = (uint32_t)y;
        SetHighBeam( Xptr, Yptr );
        index += 8;
    }
    x = (int32_t)currentX;
    y = eolYNeg;
    for ( j = 0; j < 2; j++ ) {
        Xptr = (uint32_t *)(&(tmpPtr[index+0]));
        Yptr = (uint32_t *)(&(tmpPtr[index+4]));
        *Xptr = (uint32_t)x;
        *Yptr = (uint32_t)y;
        SetHighBeam( Xptr, Yptr );
        index += 8;
    }
    for ( y=eolYNeg; y <= eolYPos && y >= eolYNeg; y += 64*gCoarse3SearchStep ) {
        Xptr = (uint32_t *)(&(tmpPtr[index+0]));
        Yptr = (uint32_t *)(&(tmpPtr[index+4]));
        *Xptr = (uint32_t)x;
        *Yptr = (uint32_t)y;
        SetHighBeam( Xptr, Yptr );
        index += 8;
    }
    x = (int32_t)currentX;
    y = eolYPos;
    for ( j = 0; j < 2; j++ ) {
        Xptr = (uint32_t *)(&(tmpPtr[index+0]));
        Yptr = (uint32_t *)(&(tmpPtr[index+4]));
        *Xptr = (uint32_t)x;
        *Yptr = (uint32_t)y;
        SetHighBeam( Xptr, Yptr );
        index += 8;
    }
    for ( y=eolYPos; y >= (int32_t)currentY && y <= eolYPos; y -= 64*gCoarse3SearchStep ) {
        Xptr = (uint32_t *)(&(tmpPtr[index+0]));
        Yptr = (uint32_t *)(&(tmpPtr[index+4]));
        *Xptr = (uint32_t)x;
        *Yptr = (uint32_t)y;
        SetHighBeam( Xptr, Yptr );
        index += 8;
    }
    x = (int32_t)currentX;
    y = (int32_t)currentY;
    for ( j = 0; j < 2; j++ ) {
        Xptr = (uint32_t *)(&(tmpPtr[index+0]));
        Yptr = (uint32_t *)(&(tmpPtr[index+4]));
        *Xptr = (uint32_t)x;
        *Yptr = (uint32_t)y;
        SetLowBeam( Xptr, Yptr );
        index += 8;
    }

    *pIndex = index;
}

static void DoSquare(
                unsigned int currentX,
                unsigned int currentY,
                char * tmpPtr,
                int * pIndex
            ) {
    int index;
    int32_t eolXNeg;
    int32_t eolXPos;
    int32_t eolYNeg;
    int32_t eolYPos;
    uint32_t * Xptr;
    uint32_t * Yptr;
    int32_t x;
    int32_t y;
    int j;

    index = *pIndex;

    if ( (int32_t)currentX >= (int32_t)(kMaxNumberX - gNumberStep) ) {
         eolXPos   =  kMaxNumberX;
    } else {
         eolXPos =  (int32_t)currentX            + gNumberStep; 
    }
    if ( (int32_t)currentX <= (int32_t)(kMinNumberX + gNumberStep) ) {
         eolXNeg   =  kMinNumberX;
    } else {
         eolXNeg =  (int32_t)currentX            - gNumberStep; 
    }
    if ( (int32_t)currentY >= (int32_t)(kMaxNumberY - gNumberStep) ) {
         eolYPos   =  kMaxNumberY;
    } else {
         eolYPos =  (int32_t)currentY            + gNumberStep; 
    }
    if ( (int32_t)currentY <= (int32_t)(kMinNumberY + gNumberStep) ) {
         eolYNeg   =  kMinNumberY;
    } else {
         eolYNeg =  (int32_t)currentY            - gNumberStep; 
    }
    x = (int32_t)eolXNeg;
    y = (int32_t)eolYNeg;
    for ( j = 0; j < 10; j++ ) {
        Xptr = (uint32_t *)(&(tmpPtr[index+0]));
        Yptr = (uint32_t *)(&(tmpPtr[index+4]));
        *Xptr = (uint32_t)x;
        *Yptr = (uint32_t)y;
        SetLowBeam( Xptr, Yptr );
        index += 8; 
    }
    for ( j = 0; j < 2; j++ ) {
        Xptr = (uint32_t *)(&(tmpPtr[index+0]));
        Yptr = (uint32_t *)(&(tmpPtr[index+4]));
        *Xptr = (uint32_t)x;
        *Yptr = (uint32_t)y;
        SetHighBeam( Xptr, Yptr );
        index += 8; 
    }
    for ( x=(int32_t)currentX; x <= eolXPos && x >= (int32_t)currentX; x += 64*gCoarse3SearchStep ) {
        Xptr = (uint32_t *)(&(tmpPtr[index+0]));
        Yptr = (uint32_t *)(&(tmpPtr[index+4]));
        *Xptr = (uint32_t)x;
        *Yptr = (uint32_t)y;
        SetHighBeam( Xptr, Yptr );
        index += 8; 
    }
    x = (int32_t)eolXPos;
    y = (int32_t)eolYNeg;
    for ( j = 0; j < 5; j++ ) {
        Xptr = (uint32_t *)(&(tmpPtr[index+0]));
        Yptr = (uint32_t *)(&(tmpPtr[index+4]));
        *Xptr = (uint32_t)x;
        *Yptr = (uint32_t)y;
        SetHighBeam( Xptr, Yptr );
        index += 8;
    }
    for ( y=eolYNeg; y <= eolYPos && y >= eolYNeg; y += 64*gCoarse3SearchStep ) {
        Xptr = (uint32_t *)(&(tmpPtr[index+0]));
        Yptr = (uint32_t *)(&(tmpPtr[index+4]));
        *Xptr = (uint32_t)x;
        *Yptr = (uint32_t)y;
        SetHighBeam( Xptr, Yptr );
        index += 8; 
    }
    x = (int32_t)eolXPos;
    y = (int32_t)eolYPos;
    for ( j = 0; j < 5; j++ ) {
        Xptr = (uint32_t *)(&(tmpPtr[index+0]));
        Yptr = (uint32_t *)(&(tmpPtr[index+4]));
        *Xptr = (uint32_t)x;
        *Yptr = (uint32_t)y;
        SetHighBeam( Xptr, Yptr );
        index += 8;
    }
    for ( x = eolXPos; x>=eolXNeg && x <= eolXPos; x -= 64*gCoarse3SearchStep ) {
        Xptr = (uint32_t *)(&(tmpPtr[index+0]));
        Yptr = (uint32_t *)(&(tmpPtr[index+4]));
        *Xptr = (uint32_t)x;
        *Yptr = (uint32_t)y;
        SetHighBeam( Xptr, Yptr );
        index += 8; 
    }
    x = (int32_t)eolXNeg;
    y = (int32_t)eolYPos;
    for ( j = 0; j < 5; j++ ) {
        Xptr = (uint32_t *)(&(tmpPtr[index+0]));
        Yptr = (uint32_t *)(&(tmpPtr[index+4]));
        *Xptr = (uint32_t)x;
        *Yptr = (uint32_t)y;
        SetHighBeam( Xptr, Yptr );
        index += 8;
    }
    for ( y=eolYPos; y >= eolYNeg && y <= eolYPos; y -= 64*gCoarse3SearchStep ) {
        Xptr = (uint32_t *)(&(tmpPtr[index+0]));
        Yptr = (uint32_t *)(&(tmpPtr[index+4]));
        *Xptr = (uint32_t)x;
        *Yptr = (uint32_t)y;
        SetHighBeam( Xptr, Yptr );
        index += 8;
    }
    x = (int32_t)eolXNeg;
    y = (int32_t)eolYNeg;
    for ( j = 0; j < 2; j++ ) {
        Xptr = (uint32_t *)(&(tmpPtr[index+0]));
        Yptr = (uint32_t *)(&(tmpPtr[index+4]));
        *Xptr = (uint32_t)x;
        *Yptr = (uint32_t)y;
        SetHighBeam( Xptr, Yptr );
        index += 8;
    }
    for ( j = 0; j < 2; j++ ) {
        Xptr = (uint32_t *)(&(tmpPtr[index+0]));
        Yptr = (uint32_t *)(&(tmpPtr[index+4]));
        *Xptr = (uint32_t)x;
        *Yptr = (uint32_t)y;
        SetLowBeam( Xptr, Yptr );
        index += 8;
    }

    *pIndex = index;
}

void show_move(
                unsigned int lastX,
                unsigned int lastY,
                unsigned int currentX,
                unsigned int currentY,
                char * tmpPtr,
                int * pIndex
            )
{
    int32_t new_xpos;
    int32_t new_ypos;
    int32_t last_xpos;
    int32_t last_ypos;

    last_xpos = (int32_t)lastX;
    last_ypos = (int32_t)lastY;
    new_xpos  = (int32_t)currentX;
    new_ypos  = (int32_t)currentY;

    draw_dark( last_xpos, last_ypos, new_xpos, new_ypos, tmpPtr, pIndex );

}

static void Do1(
                unsigned int currentX,
                unsigned int currentY,
                char * tmpPtr,
                int * pIndex
            ) {
    int32_t XNeg;
    int32_t XPos;
    int32_t YNeg;
    int32_t YPos;

    limitXY(currentX,currentY,&XNeg,&XPos,&YNeg,&YPos);

    Draw1( currentX, YNeg, YPos, tmpPtr, pIndex );
}



static void Do2(
                unsigned int currentX,
                unsigned int currentY,
                char * tmpPtr,
                int * pIndex
            ) {
    int32_t XNeg;
    int32_t XPos;
    int32_t YNeg;
    int32_t YPos;

    limitXY(currentX,currentY,&XNeg,&XPos,&YNeg,&YPos);

    Draw2( XNeg,XPos,YNeg,currentY,YPos, tmpPtr, pIndex );
}


static void Do3(
                unsigned int currentX,
                unsigned int currentY,
                char * tmpPtr,
                int * pIndex
            ) {
    int32_t XNeg;
    int32_t XPos;
    int32_t YNeg;
    int32_t YPos;

    limitXY(currentX,currentY,&XNeg,&XPos,&YNeg,&YPos);

    Draw3( XNeg,XPos,YNeg,currentY,YPos, tmpPtr, pIndex );
}


static void Do4(
                unsigned int currentX,
                unsigned int currentY,
                char * tmpPtr,
                int * pIndex
            ) {
    int32_t XNeg;
    int32_t XPos;
    int32_t YNeg;
    int32_t YPos;

    limitXY(currentX,currentY,&XNeg,&XPos,&YNeg,&YPos);

    Draw4( XNeg,XPos,YNeg,currentY,YPos, tmpPtr, pIndex );
}


static void Do5(
                unsigned int currentX,
                unsigned int currentY,
                char * tmpPtr,
                int * pIndex
            ) {
    int32_t XNeg;
    int32_t XPos;
    int32_t YNeg;
    int32_t YPos;

    limitXY(currentX,currentY,&XNeg,&XPos,&YNeg,&YPos);

    Draw5( XNeg,XPos,YNeg,currentY,YPos, tmpPtr, pIndex );
}

static void Do6(
                unsigned int currentX,
                unsigned int currentY,
                char * tmpPtr,
                int * pIndex
            ) {
    int32_t XNeg;
    int32_t XPos;
    int32_t YNeg;
    int32_t YPos;

    limitXY(currentX,currentY,&XNeg,&XPos,&YNeg,&YPos);

    Draw6( XNeg,XPos,YNeg,currentY,YPos, tmpPtr, pIndex );

}
static void Do7(
                unsigned int currentX,
                unsigned int currentY,
                char * tmpPtr,
                int * pIndex
            ) {
    int32_t XNeg;
    int32_t XPos;
    int32_t YNeg;
    int32_t YPos;

    limitXY(currentX,currentY,&XNeg,&XPos,&YNeg,&YPos);

    Draw7( XNeg,XPos,YNeg,YPos, tmpPtr, pIndex );

}
static void Do8(
                unsigned int currentX,
                unsigned int currentY,
                char * tmpPtr,
                int * pIndex
            ) {
    int32_t XNeg;
    int32_t XPos;
    int32_t YNeg;
    int32_t YPos;

    limitXY(currentX,currentY,&XNeg,&XPos,&YNeg,&YPos);

    Draw8( XNeg,XPos,YNeg,currentY,YPos, tmpPtr, pIndex );

}
static void Do9(
                unsigned int currentX,
                unsigned int currentY,
                char * tmpPtr,
                int * pIndex
            ) {
    int32_t XNeg;
    int32_t XPos;
    int32_t YNeg;
    int32_t YPos;

    limitXY(currentX,currentY,&XNeg,&XPos,&YNeg,&YPos);

    Draw9( XNeg,XPos,YNeg,currentY,YPos, tmpPtr, pIndex );

}



static void Do10(
                unsigned int currentX,
                unsigned int currentY,
                char * tmpPtr,
                int * pIndex
            ) {
    int32_t XNeg;
    int32_t X001;
    int32_t X002;
    int32_t XPos;
    int32_t YNeg;
    int32_t YPos;

    limit2C(currentX,currentY,&XNeg,&X001,&X002,&XPos,&YNeg,&YPos);

    Draw1(XNeg,YNeg,YPos,tmpPtr,pIndex);
    Draw0(X002,XPos,YNeg,YPos,tmpPtr,pIndex);

}


static void Do11(
                unsigned int currentX,
                unsigned int currentY,
                char * tmpPtr,
                int * pIndex
            ) {
    int32_t XNeg;
    int32_t X001;
    int32_t X002;
    int32_t XPos;
    int32_t YNeg;
    int32_t YPos;

    limit2C(currentX,currentY,&XNeg,&X001,&X002,&XPos,&YNeg,&YPos);

    Draw1(XNeg,YNeg,YPos,tmpPtr,pIndex);
    Draw1(XPos,YNeg,YPos,tmpPtr,pIndex);

}


static void Do12(
                unsigned int currentX,
                unsigned int currentY,
                char * tmpPtr,
                int * pIndex
            ) {
    int32_t XNeg;
    int32_t X001;
    int32_t X002;
    int32_t XPos;
    int32_t YNeg;
    int32_t YPos;

    limit2C(currentX,currentY,&XNeg,&X001,&X002,&XPos,&YNeg,&YPos);

    Draw1(XNeg,YNeg,YPos,tmpPtr,pIndex);
    Draw2(X002,XPos,YNeg,currentY,YPos,tmpPtr,pIndex);

}


static void Do13(
                unsigned int currentX,
                unsigned int currentY,
                char * tmpPtr,
                int * pIndex
            ) {
    int32_t XNeg;
    int32_t X001;
    int32_t X002;
    int32_t XPos;
    int32_t YNeg;
    int32_t YPos;

    limit2C(currentX,currentY,&XNeg,&X001,&X002,&XPos,&YNeg,&YPos);

    Draw1(XNeg,YNeg,YPos,tmpPtr,pIndex);
    Draw3(X002,XPos,YNeg,currentY,YPos,tmpPtr,pIndex);

}


static void Do14(
                unsigned int currentX,
                unsigned int currentY,
                char * tmpPtr,
                int * pIndex
            ) {
    int32_t XNeg;
    int32_t X001;
    int32_t X002;
    int32_t XPos;
    int32_t YNeg;
    int32_t YPos;

    limit2C(currentX,currentY,&XNeg,&X001,&X002,&XPos,&YNeg,&YPos);

    Draw1(XNeg,YNeg,YPos,tmpPtr,pIndex);
    Draw4(X002,XPos,YNeg,currentY,YPos,tmpPtr,pIndex);

}


static void Do15(
                unsigned int currentX,
                unsigned int currentY,
                char * tmpPtr,
                int * pIndex
            ) {
    int32_t XNeg;
    int32_t X001;
    int32_t X002;
    int32_t XPos;
    int32_t YNeg;
    int32_t YPos;

    limit2C(currentX,currentY,&XNeg,&X001,&X002,&XPos,&YNeg,&YPos);

    Draw1(XNeg,YNeg,YPos,tmpPtr,pIndex);
    Draw5(X002,XPos,YNeg,currentY,YPos,tmpPtr,pIndex);

}


static void Do16(
                unsigned int currentX,
                unsigned int currentY,
                char * tmpPtr,
                int * pIndex
            ) {
    int32_t XNeg;
    int32_t X001;
    int32_t X002;
    int32_t XPos;
    int32_t YNeg;
    int32_t YPos;

    limit2C(currentX,currentY,&XNeg,&X001,&X002,&XPos,&YNeg,&YPos);

    Draw1(XNeg,YNeg,YPos,tmpPtr,pIndex);
    Draw6(X002,XPos,YNeg,currentY,YPos,tmpPtr,pIndex);

}


static void Do17(
                unsigned int currentX,
                unsigned int currentY,
                char * tmpPtr,
                int * pIndex
            ) {
    int32_t XNeg;
    int32_t X001;
    int32_t X002;
    int32_t XPos;
    int32_t YNeg;
    int32_t YPos;

    limit2C(currentX,currentY,&XNeg,&X001,&X002,&XPos,&YNeg,&YPos);

    Draw1(XNeg,YNeg,YPos,tmpPtr,pIndex);
    Draw7(X002,XPos,YNeg,YPos,tmpPtr,pIndex);

}


static void Do18(
                unsigned int currentX,
                unsigned int currentY,
                char * tmpPtr,
                int * pIndex
            ) {
    int32_t XNeg;
    int32_t X001;
    int32_t X002;
    int32_t XPos;
    int32_t YNeg;
    int32_t YPos;

    limit2C(currentX,currentY,&XNeg,&X001,&X002,&XPos,&YNeg,&YPos);

    Draw1(XNeg,YNeg,YPos,tmpPtr,pIndex);
    Draw8(X002,XPos,YNeg,currentY,YPos,tmpPtr,pIndex);

}


static void Do19(
                unsigned int currentX,
                unsigned int currentY,
                char * tmpPtr,
                int * pIndex
            ) {
    int32_t XNeg;
    int32_t X001;
    int32_t X002;
    int32_t XPos;
    int32_t YNeg;
    int32_t YPos;

    limit2C(currentX,currentY,&XNeg,&X001,&X002,&XPos,&YNeg,&YPos);

    Draw1(XNeg,YNeg,YPos,tmpPtr,pIndex);
    Draw9(X002,XPos,YNeg,currentY,YPos,tmpPtr,pIndex);

}


static void Do20(
                unsigned int currentX,
                unsigned int currentY,
                char * tmpPtr,
                int * pIndex
            ) {
    int32_t XNeg;
    int32_t X001;
    int32_t X002;
    int32_t XPos;
    int32_t YNeg;
    int32_t YPos;

    limit2C(currentX,currentY,&XNeg,&X001,&X002,&XPos,&YNeg,&YPos);

    Draw2(XNeg,X001,YNeg,currentY,YPos,tmpPtr,pIndex);
    Draw0(X002,XPos,YNeg,YPos,tmpPtr,pIndex);

}


static void Do21(
                unsigned int currentX,
                unsigned int currentY,
                char * tmpPtr,
                int * pIndex
            ) {
    int32_t XNeg;
    int32_t X001;
    int32_t X002;
    int32_t XPos;
    int32_t YNeg;
    int32_t YPos;

    limit2C(currentX,currentY,&XNeg,&X001,&X002,&XPos,&YNeg,&YPos);

    Draw2(XNeg,X001,YNeg,currentY,YPos,tmpPtr,pIndex);
    Draw1(XPos,YNeg,YPos,tmpPtr,pIndex);

}


static void Do22(
                unsigned int currentX,
                unsigned int currentY,
                char * tmpPtr,
                int * pIndex
            ) {
    int32_t XNeg;
    int32_t X001;
    int32_t X002;
    int32_t XPos;
    int32_t YNeg;
    int32_t YPos;

    limit2C(currentX,currentY,&XNeg,&X001,&X002,&XPos,&YNeg,&YPos);

    Draw2(XNeg,X001,YNeg,currentY,YPos,tmpPtr,pIndex);
    Draw2(X002,XPos,YNeg,currentY,YPos,tmpPtr,pIndex);

}


static void Do23(
                unsigned int currentX,
                unsigned int currentY,
                char * tmpPtr,
                int * pIndex
            ) {
    int32_t XNeg;
    int32_t X001;
    int32_t X002;
    int32_t XPos;
    int32_t YNeg;
    int32_t YPos;

    limit2C(currentX,currentY,&XNeg,&X001,&X002,&XPos,&YNeg,&YPos);

    Draw2(XNeg,X001,YNeg,currentY,YPos,tmpPtr,pIndex);
    Draw3(X002,XPos,YNeg,currentY,YPos,tmpPtr,pIndex);

}

static void Do24(
                unsigned int currentX,
                unsigned int currentY,
                char * tmpPtr,
                int * pIndex
            ) {
    int32_t XNeg;
    int32_t X001;
    int32_t X002;
    int32_t XPos;
    int32_t YNeg;
    int32_t YPos;

    limit2C(currentX,currentY,&XNeg,&X001,&X002,&XPos,&YNeg,&YPos);

    Draw2(XNeg,X001,YNeg,currentY,YPos,tmpPtr,pIndex);
    Draw4(X002,XPos,YNeg,currentY,YPos,tmpPtr,pIndex);

}

void Draw0(int32_t xmin, int32_t xmax, int32_t ymin,
	   int32_t ymax, char *tmpPtr, int *pIndex)
{
    off_pause(  xmin, ymin,             tmpPtr, pIndex );
    draw_line(  xmin, ymin, xmin, ymax, tmpPtr, pIndex );
    draw_line(  xmin, ymax, xmax, ymax, tmpPtr, pIndex );
    draw_line(  xmax, ymax, xmax, ymin, tmpPtr, pIndex );
    draw_line(  xmax, ymin, xmin, ymin, tmpPtr, pIndex );
    off_pause(  xmin, ymin,             tmpPtr, pIndex );
    return;
}

void
Draw1( int32_t x0
     , int32_t ymin
     , int32_t ymax
     , char * tmpPtr
     , int * pIndex
     )
{
    off_pause(  x0, ymin,             tmpPtr, pIndex );
    draw_line(  x0, ymin, x0, ymax, tmpPtr, pIndex );
    off_pause(  x0, ymin,             tmpPtr, pIndex );
}

void
Draw2( int32_t xmin
     , int32_t xmax
     , int32_t ymin
     , int32_t y0
     , int32_t ymax
     , char * tmpPtr
     , int * pIndex
     )
{

    off_pause(  xmin, ymax,             tmpPtr, pIndex );
    draw_line(  xmin, ymax, xmax, ymax, tmpPtr, pIndex );
    draw_line(  xmax, ymax, xmax, y0,   tmpPtr, pIndex );
    draw_line(  xmax, y0,   xmin, y0,   tmpPtr, pIndex );
    draw_line(  xmin, y0,   xmin, ymin, tmpPtr, pIndex );
    draw_line(  xmin, ymin, xmax, ymin, tmpPtr, pIndex );
    off_pause(  xmax, ymin,             tmpPtr, pIndex );

}

void
Draw3( int32_t xmin
     , int32_t xmax
     , int32_t ymin
     , int32_t y0
     , int32_t ymax
     , char * tmpPtr
     , int * pIndex
     )
{
    off_pause(  xmin, ymax,             tmpPtr, pIndex );
    draw_line(  xmin, ymax, xmax, ymax, tmpPtr, pIndex );
    draw_line(  xmax, ymax, xmax, y0,   tmpPtr, pIndex );
    draw_line(  xmax, y0,   xmin, y0,   tmpPtr, pIndex );
    draw_line(  xmin, y0,   xmax, y0,   tmpPtr, pIndex );
    draw_line(  xmax, y0,   xmax, ymin, tmpPtr, pIndex );
    draw_line(  xmax, ymin, xmin, ymin, tmpPtr, pIndex );
    off_pause(  xmin, ymin,             tmpPtr, pIndex );
}

void
Draw4( int32_t xmin
     , int32_t xmax
     , int32_t ymin
     , int32_t y0
     , int32_t ymax
     , char * tmpPtr
     , int * pIndex
     )
{
    off_pause(  xmin, ymax,             tmpPtr, pIndex );
    draw_line(  xmin, ymax, xmin, y0,   tmpPtr, pIndex );
    draw_line(  xmin, y0,   xmax, y0,   tmpPtr, pIndex );
    draw_line(  xmax, y0,   xmax, ymax, tmpPtr, pIndex );
    draw_line(  xmax, ymax, xmax, ymin, tmpPtr, pIndex );
    off_pause(  xmax, ymin,             tmpPtr, pIndex );
}

void
Draw5( int32_t xmin
     , int32_t xmax
     , int32_t ymin
     , int32_t y0
     , int32_t ymax
     , char * tmpPtr
     , int * pIndex
     )
{
    off_pause(  xmax, ymax,             tmpPtr, pIndex );
    draw_line(  xmax, ymax, xmin, ymax, tmpPtr, pIndex );
    draw_line(  xmin, ymax, xmin, y0,   tmpPtr, pIndex );
    draw_line(  xmin, y0,   xmax, y0,   tmpPtr, pIndex );
    draw_line(  xmax, y0,   xmax, ymin, tmpPtr, pIndex );
    draw_line(  xmax, ymin, xmin, ymin, tmpPtr, pIndex );
    off_pause(  xmin, ymin,             tmpPtr, pIndex );
}

void
Draw6( int32_t xmin
     , int32_t xmax
     , int32_t ymin
     , int32_t y0
     , int32_t ymax
     , char * tmpPtr
     , int * pIndex
     )
{
    off_pause(  xmax, ymax,             tmpPtr, pIndex );
    draw_line(  xmax, ymax, xmin, ymax, tmpPtr, pIndex );
    draw_line(  xmin, ymax, xmin, ymin, tmpPtr, pIndex );
    draw_line(  xmin, ymin, xmax, ymin, tmpPtr, pIndex );
    draw_line(  xmax, ymin, xmax, y0,   tmpPtr, pIndex );
    draw_line(  xmax, y0,   xmin, y0,   tmpPtr, pIndex );
    off_pause(  xmin, y0,               tmpPtr, pIndex );
}

void
Draw7( int32_t xmin
     , int32_t xmax
     , int32_t ymin
     , int32_t ymax
     , char * tmpPtr
     , int * pIndex
     )
{
    off_pause(  xmin, ymin,             tmpPtr, pIndex );
    draw_line(  xmin, ymin, xmax, ymax, tmpPtr, pIndex );
    draw_line(  xmax, ymax, xmin, ymax, tmpPtr, pIndex );
    off_pause(  xmin, ymax,             tmpPtr, pIndex );
}

void
Draw8( int32_t xmin
     , int32_t xmax
     , int32_t ymin
     , int32_t y0
     , int32_t ymax
     , char * tmpPtr
     , int * pIndex
     )
{
    off_pause(  xmin, y0,               tmpPtr, pIndex );
    draw_line(  xmin, y0,   xmin, ymax, tmpPtr, pIndex );
    draw_line(  xmin, ymax, xmax, ymax, tmpPtr, pIndex );
    draw_line(  xmax, ymax, xmax, y0,   tmpPtr, pIndex );
    draw_line(  xmax, y0,   xmin, y0,   tmpPtr, pIndex );
    draw_line(  xmin, y0,   xmin, ymin, tmpPtr, pIndex );
    draw_line(  xmin, ymin, xmax, ymin, tmpPtr, pIndex );
    draw_line(  xmax, ymin, xmax, y0,   tmpPtr, pIndex );
    off_pause(  xmax, y0,               tmpPtr, pIndex );
}

void
Draw9( int32_t xmin
     , int32_t xmax
     , int32_t ymin
     , int32_t y0
     , int32_t ymax
     , char * tmpPtr
     , int * pIndex
     )
{
    off_pause(  xmax, ymin,             tmpPtr, pIndex );
    draw_line(  xmax, ymin, xmax, ymax, tmpPtr, pIndex );
    draw_line(  xmax, ymax, xmin, ymax, tmpPtr, pIndex );
    draw_line(  xmin, ymax, xmin, y0,   tmpPtr, pIndex );
    draw_line(  xmin, y0,   xmax, y0,   tmpPtr, pIndex );
    off_pause(  xmax, y0,               tmpPtr, pIndex );
}

void
draw_line( int32_t x0
         , int32_t y0
         , int32_t x1
         , int32_t y1
         , char * tmpPtr
         , int * pIndex
         )
{
    int index;
    uint32_t *Xptr;
    uint32_t *Yptr;
    int32_t delX;
    int32_t delY;
    int32_t x;
    int32_t y;
    int j;

    index = *pIndex;

    delX = (x1 - x0) / 16;
    delY = (y1 - y0) / 16;

          // pause at the start of line
    for ( j = 0; j < 2; j++ ) {
        Xptr = (uint32_t *)(&(tmpPtr[index+0]));
        Yptr = (uint32_t *)(&(tmpPtr[index+4]));
        *Xptr = (uint32_t)x0;
        *Yptr = (uint32_t)y0;
        SetHighBeam( Xptr, Yptr );
        index += 8;
    }

    for ( j=0; j <= 16; j++ ) {
        x = x0 + j * delX;
        y = y0 + j * delY;
        Xptr = (uint32_t *)(&(tmpPtr[index+0]));
        Yptr = (uint32_t *)(&(tmpPtr[index+4]));
        *Xptr = (uint32_t)x;
        *Yptr = (uint32_t)y;
        SetHighBeam( Xptr, Yptr );
        index += 8;
    }

          // pause at the end of line
    for ( j = 0; j < 2; j++ ) {
        Xptr = (uint32_t *)(&(tmpPtr[index+0]));
        Yptr = (uint32_t *)(&(tmpPtr[index+4]));
        *Xptr = (uint32_t)x1;
        *Yptr = (uint32_t)y1;
        SetHighBeam( Xptr, Yptr );
        index += 8;
    }

    *pIndex = index;
}

void
draw_dark( int32_t x0
         , int32_t y0
         , int32_t x1
         , int32_t y1
         , char * tmpPtr
         , int * pIndex
         )
{
    int index;
    uint32_t *Xptr;
    uint32_t *Yptr;
    int32_t delX;
    int32_t delY;
    int32_t x;
    int32_t y;
    int j;

    index = *pIndex;

    delX = (x1 - x0) / 60;
    delY = (y1 - y0) / 60;

          // pause at the start of line
    for ( j = 0; j < 2; j++ ) {
        Xptr = (uint32_t *)(&(tmpPtr[index+0]));
        Yptr = (uint32_t *)(&(tmpPtr[index+4]));
        *Xptr = (uint32_t)x0;
        *Yptr = (uint32_t)y0;
        *Xptr = (*Xptr & 0xFFFFFF00);
        *Yptr = (*Yptr & 0xFFFFFF00);
        index += 8;
    }

    for ( j=0; j <= 60; j++ ) {
        x = x0 + j * delX;
        y = y0 + j * delY;
        Xptr = (uint32_t *)(&(tmpPtr[index+0]));
        Yptr = (uint32_t *)(&(tmpPtr[index+4]));
        *Xptr = (uint32_t)x;
        *Yptr = (uint32_t)y;
        *Xptr = (*Xptr & 0xFFFFFF00);
        *Yptr = (*Yptr & 0xFFFFFF00);
        index += 8;
    }

          // pause at the end of line
    for ( j = 0; j < 2; j++ ) {
        Xptr = (uint32_t *)(&(tmpPtr[index+0]));
        Yptr = (uint32_t *)(&(tmpPtr[index+4]));
        *Xptr = (uint32_t)x1;
        *Yptr = (uint32_t)y1;
        *Xptr = (*Xptr & 0xFFFFFF00);
        *Yptr = (*Yptr & 0xFFFFFF00);
        index += 8;
    }

    *pIndex = index;
}

void
off_pause( int32_t x1
         , int32_t y1
         , char * tmpPtr
         , int * pIndex
         )
{
    uint32_t *Xptr;
    uint32_t *Yptr;
    int index;
    int j;

    index = *pIndex;

    for ( j = 0; j < 10; j++ ) {
        Xptr = (uint32_t *)(&(tmpPtr[index+0]));
        Yptr = (uint32_t *)(&(tmpPtr[index+4]));
        *Xptr = (uint32_t)x1;
        *Yptr = (uint32_t)y1;
        *Xptr = (*Xptr & 0xFFFFFF00);
        *Yptr = (*Yptr & 0xFFFFFF00);
        //  SetLowBeam( Xptr, Yptr );
        index += 8;
    }

    *pIndex = index;
}

void
limitXY( uint32_t currentX
       , uint32_t currentY
       , int32_t * eolXNeg
       , int32_t * eolXPos
       , int32_t * eolYNeg
       , int32_t * eolYPos
       )
{
    
    if ( (int32_t)currentX >= (int32_t)(kMaxNumberX - gNumberStep) ) {
        *eolXPos   =  kMaxNumberX;
    } else {
        *eolXPos =  (int32_t)currentX            + gNumberStep; 
    }
    if ( (int32_t)currentX <= (int32_t)(kMinNumberX + gNumberStep) ) {
        *eolXNeg   =  kMinNumberX;
    } else {
        *eolXNeg =  (int32_t)currentX            - gNumberStep; 
    }
    if ( (int32_t)currentY >= (int32_t)(kMaxNumberY - gNumberStep) ) {
        *eolYPos   =  kMaxNumberY;
    } else {
        *eolYPos =  (int32_t)currentY            + gNumberStep; 
    }
    if ( (int32_t)currentY <= (int32_t)(kMinNumberY + gNumberStep) ) {
        *eolYNeg   =  kMinNumberY;
    } else {
        *eolYNeg =  (int32_t)currentY            - gNumberStep; 
    }

}

void
limit2C( uint32_t currentX
       , uint32_t currentY
       , int32_t * eolXNeg
       , int32_t * eolX001
       , int32_t * eolX002
       , int32_t * eolXPos
       , int32_t * eolYNeg
       , int32_t * eolYPos
       )
{
    
    if ( (int32_t)currentX >= (int32_t)(kMaxNumberX - gNumberStep) ) {
        *eolXPos   =  kMaxNumberX;
    } else {
        *eolXPos =  (int32_t)currentX            + gNumberStep; 
    }
    if ( (int32_t)currentX >= (int32_t)(kMaxNumberX - gNumberStep/3) ) {
        *eolX002   =  kMaxNumberX;
    } else {
        *eolX002 =  (int32_t)currentX            + gNumberStep/3; 
    }
    if ( (int32_t)currentX <= (int32_t)(kMinNumberX + gNumberStep/3) ) {
        *eolX001   =  kMinNumberX;
    } else {
        *eolX001 =  (int32_t)currentX            - gNumberStep/3; 
    }
    if ( (int32_t)currentX <= (int32_t)(kMinNumberX + gNumberStep) ) {
        *eolXNeg   =  kMinNumberX;
    } else {
        *eolXNeg =  (int32_t)currentX            - gNumberStep; 
    }
    if ( (int32_t)currentY >= (int32_t)(kMaxNumberY - gNumberStep) ) {
        *eolYPos   =  kMaxNumberY;
    } else {
        *eolYPos =  (int32_t)currentY            + gNumberStep; 
    }
    if ( (int32_t)currentY <= (int32_t)(kMinNumberY + gNumberStep) ) {
        *eolYNeg   =  kMinNumberY;
    } else {
        *eolYNeg =  (int32_t)currentY            - gNumberStep; 
    }

}
