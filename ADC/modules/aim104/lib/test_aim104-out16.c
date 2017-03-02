/* test_aim104-out16.c - test the aim104-out16 libaim104 routines.

   Usage:
     test_aim104-out16 <device node>
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

static void check_board( int fd, const char *exe_name, const char *dev_name );
static void set_outputs( int fd, unsigned short output, const char *exe_name,
                         const char *dev_name );

int main( int argc, char *argv[] )
{
    int fd;
    int i;

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
    if( !aim104_out16_enable_outputs( fd, 1 ) ) {
        fprintf( stderr, "%s: %s: enable: %s\n", argv[0], argv[1], 
                 strerror( errno ) );
        exit( 1 );
    }

    sleep(1);

    check_board(fd, argv[0], argv[1]);

    /* set each output on in turn */
    for( i=1; i<=0xffff; i<<=1 ) {
        set_outputs( fd, i, argv[0], argv[1] );
        check_board( fd, argv[0], argv[1]);
    }

    /* disable outputs */
    printf( "Disable outputs.\n" );
    if( !aim104_out16_enable_outputs( fd, 0 ) ) {
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


static void check_board( int fd, const char *exe_name, const char *dev_name )
{
    int status;
    int chan[2];
    int i;

    /* check output status */
    for( i = 0; i < 2; i++ ) {
        status = aim104_out16_output_status( fd, i ); 
        if( status < 0 ) {
            fprintf( stderr, "%s: %s: status: %s\n", exe_name, dev_name, 
                     strerror( errno ) );
            exit( 1 );
        }
    }
    printf( "Output status: 0x%04x\n", (unsigned)((chan[1] << 8) & chan[0]) );

    sleep(1);
}

static void set_outputs( int fd, unsigned short outputs, const char *exe_name,
                         const char *dev_name )
{
    int i;

    for( i = 0; i < 2; i++ ) {
        if( !aim104_out16_set_all( fd, i, (outputs >> i*8) & 0xff ) ) {
            fprintf( stderr, "%s: %s: write: %s\n", exe_name, dev_name, 
                     strerror( errno ) );
            exit( 1 );
        }
    }

    sleep(1);
}
