/* test_aim104-io16.c - test the aim104-io32 libaim104 routines.

   Usage:
     test_aim104-io32 <device node>

   This code assumes that lines 0-15 are outputs and 16-31 are inputs.
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

static void set_outputs( int fd, unsigned output, const char *exe_name,
                         const char *dev_name );

int main( int argc, char *argv[] )
{
    int fd;
    unsigned i;
    unsigned inputs=0;

    if( argc<2 ) {
        fprintf( stderr, "%s: required file name missing\n", argv[0] );
        exit( 1 );
    }

    /* open */
    if( (fd=open( argv[1], O_RDWR ))<0 ) {
        fprintf( stderr, "%s: %s: %s\n", argv[0], argv[1], strerror( errno ) );
        exit( 1 );
    }

    /* enable outputs */
    printf( "Enable outputs.\n" );
    if( aim104_io32_enable_outputs( fd, 1 ) != AIM104_SUCCESS ) {
        fprintf( stderr, "%s: %s: enable: %s\n", argv[0], argv[1], 
                 strerror( errno ) );
        exit( 1 );
    }

    sleep(1);

    /* set each output on in turn */
    for( i=1; i<=0x8000 && i!=0; i<<=1 ) {
        set_outputs( fd, i, argv[0], argv[1] );
    }
    
    /* continuously read all 16 inputs */
    for(;;) {
        unsigned new_inputs;
        int chan[2];

        for( i = 0; i < 2; i++ ) {
            chan[i] = aim104_io32_inputs( fd, i, 0xff );
            if( chan[i] < 0 ) {
                fprintf( stderr, "%s: %s: read: %s\n", argv[0], argv[1], 
                         strerror( errno ) );
                exit( 1 );
            }
        }
        new_inputs = (chan[1] << 8) & chan[0];

        /* only print inputs when they change */
        if( new_inputs!=inputs ) {
            printf( "inputs: 0x%08x\n", (unsigned)new_inputs );
            inputs=new_inputs;
        }
    }

    /* disable outputs */
    printf( "Disable outputs.\n" );
    if( !aim104_io32_enable_outputs( fd, 0 ) ) {
        fprintf( stderr, "%s: %s: disable: %s\n", argv[0], argv[1], 
                 strerror( errno ) );
        exit( 1 );
    }

    /* close */
    if( close(fd)<0 ) {
        fprintf( stderr, "%s: %s: close: %s\n", argv[0], argv[1], 
                 strerror( errno ) );
        exit( 1 );
    }

    return 0;
}


static void set_outputs( int fd, unsigned outputs, const char *exe_name,
                         const char *dev_name )
{
    int i;
    for( i = 0; i < 2; i++ ) {
        if( aim104_io32_set_all( fd, i, 0x00, (outputs >> i*8) & 0xff ) != AIM104_SUCCESS ) {
            fprintf( stderr, "%s: %s: write: %s\n", exe_name, dev_name, 
                     strerror( errno ) );
            exit( 1 );
        }
    }

    usleep( 100000 );
}
