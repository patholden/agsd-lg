#include <stdint.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/sysinfo.h>
#include <time.h>
#include <string.h>
#include <unistd.h>

#include "HeadFlash.h"

int
HeadFlash( unsigned char * buffer, unsigned char * filename, int count )
{
    struct sysinfo  info;
    time_t  timenow;
    char *ptr;
    int size;
    int fd;
    int err;

    timenow = time(0);

    ptr = &(buffer[0]); 

    sysinfo( &info );

    ptr += sprintf( ptr, "FlAsHeAdEr001 %25s\n", filename );
    ptr += sprintf( ptr, "FlAsHeAdEr002 %25d\n", (int)(info.uptime) );
    ptr += sprintf( ptr, "FlAsHeAdEr003 %25d\n", (int)(timenow) );
    ptr += sprintf( ptr, "FlAsHeAdEr004 %25d\n", (int)count );
    ptr += sprintf( ptr, "FlAsHeAdEr005 %25d\n", (int)(count+1) );



}
