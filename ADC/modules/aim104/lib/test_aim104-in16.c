/* test_aim104-in16.c - test the aim104-in16 libaim104 routines.

   Usage:
     test_aim104-in16 <device node>
*/

#include <stdio.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include "arcom/libaim104.h"

int main( int argc, char *argv[] )
{
    int fd;
    unsigned short inputs=0;

    if( argc<2 ) {
        fprintf( stderr, "%s: required file name missing\n", argv[0] );
        exit( 1 );
    }

    /* open */
    if( (fd=open( argv[1], O_RDONLY ))<0 ) {
        fprintf( stderr, "%s: %s: %s\n", argv[0], argv[1], strerror( errno ) );
        exit( 1 );
    }

    /* continuously read all 16 inputs */
    for(;;) {
        int new_inputs;
        int chan[2];
        int i;

        for( i = 0; i < 2; i++ ) {
            chan[i] = aim104_in16_inputs( fd, i );
            if( chan[i] < 0 ) {
                fprintf( stderr, "%s: %s: read: %s\n", argv[0], argv[1], 
                         strerror( errno ) );
                exit( 1 );
            }
        }
        new_inputs = ((chan[1] << 8) & chan[0]);

        /* only print inputs when they change */
        if( new_inputs!=inputs ) {
            printf( "inputs: 0x%04x\n", (unsigned)new_inputs );
            inputs=new_inputs;
        }
    }

    if( close(fd)<0 ) {
        fprintf( stderr, "%s: %s: close: %s\n", argv[0], argv[1],
                 strerror( errno ) );
        exit( 1 );
    }
    
    return 0;
}
