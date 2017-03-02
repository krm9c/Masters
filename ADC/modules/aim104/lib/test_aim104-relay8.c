/* test_aim104-relay8.c - test the aim104-relay8 libaim104 routines.

   Usage:
     test_aim104-relay8 <device node>
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
static void set_relays( int fd, unsigned char output, const char *exe_name,
                        const char *dev_name );

int main( int argc, char *argv[] )
{
    int fd;
    unsigned char inputs;

    if( argc<2 ) {
        fprintf( stderr, "%s: required file name missing\n", argv[0] );
        exit( 1 );
    }

    /* open */
    if( (fd=open( argv[1], O_RDWR ))<0 ) {
        fprintf( stderr, "%s: %s: %s\n", argv[0], argv[1], strerror( errno ) );
        exit( 1 );
    }

    /* enable relays */
    printf( "Enable relays.\n" );
    if( !aim104_relay8_enable_relays( fd, 1 ) ) {
        fprintf( stderr, "%s: %s: enable: %s\n", argv[0], argv[1], 
                 strerror( errno ) );
        exit( 1 );
    }

    sleep(1);

    check_board(fd, argv[0], argv[1]);

    /* set 0-3 relays */
    printf( "Set 0-3 relays.\n" );
    set_relays( fd, 0x0f, argv[0], argv[1] );
    check_board(fd, argv[0], argv[1]);

    /* set 4-7 relays */
    printf( "Set 4-7 relays.\n" );
    set_relays( fd, 0xff, argv[0], argv[1] );
    check_board(fd, argv[0], argv[1]);

    /* clear 0-1,6-7 relays */
    printf( "Clear 0-1,6-7 relays.\n" );
    set_relays( fd, 0x3c, argv[0], argv[1] );
    check_board(fd, argv[0], argv[1]);

    /* clear 2-5 relays */
    printf( "Clear 2-5 relays.\n" );
    set_relays( fd, 0x00, argv[0], argv[1] );
    check_board(fd, argv[0], argv[1]);

    /* disable relays */
    printf( "Disable relays.\n" );
    if( !aim104_relay8_enable_relays( fd, 0 ) ) {
        fprintf( stderr, "%s: %s: disable: %s\n", argv[0], argv[1], 
                 strerror( errno ) );
        exit( 1 );
    }

    /* continuously read all 8 inputs */
    for(;;) {
        int new_inputs;

        new_inputs=aim104_relay8_inputs( fd );
        if(  new_inputs < 0 ) {
            fprintf( stderr, "%s: %s: read: %s\n", argv[0], argv[1], 
                     strerror( errno ) );
            exit( 1 );
        }

        /* only print inputs when they change */
        if( new_inputs!=inputs ) {
            printf( "inputs: 0x%02x\n", (unsigned)new_inputs );
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


static void check_board( int fd, const char *exe_name, const char *dev_name )
{
    int status;

    /* check relay status */
    status = aim104_relay8_relay_status( fd );
    if( status < 0 ) {
        fprintf( stderr, "%s: %s: status: %s\n", exe_name, dev_name, 
                 strerror( errno ) );
        exit( 1 );
    }
    printf( "Relay status: 0x%02x\n", (unsigned)status );

    sleep(1);
}

static void set_relays( int fd, unsigned char outputs, const char *exe_name,
                        const char *dev_name)
{
    if( !aim104_relay8_set_all( fd, outputs ) ) {
        fprintf( stderr, "%s: %s: write: %s\n", exe_name, dev_name, 
                 strerror( errno ) );
        exit( 1 );
    }

    sleep(1);
}
