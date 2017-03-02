/* test_aim104-multi-io.c - test the aim104-multi-io libaim104 routines.

   Usage:
     test_aim104-multi-io <device node>
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

static void test_analog( int fd, const char *exe_name, const char *dev_name );
static void test_digital( int fd, const char *exe_name, const char *dev_name );

int main( int argc, char *argv[] )
{
    int fd;

    if( argc < 2 ) {
        fprintf( stderr, "%s: required file name missing\n", argv[0] );
        exit( 1 );
    }

    /* open */
    if( (fd = open( argv[1], O_RDWR )) < 0 ) {
        fprintf( stderr, "%s: %s: %s\n", argv[0], argv[1], strerror( errno ) );
        exit( 1 );
    }

    test_analog( fd, argv[0], argv[1] );

    test_digital( fd, argv[0], argv[1] );

    /* close */
    if( close( fd ) < 0 ) {
        fprintf( stderr, "%s: %s: close: %s\n", argv[0], argv[1], 
                 strerror( errno ) );
        exit( 1 );
    }

    return 0;
}


static void test_analog( int fd, const char *exe_name, const char *dev_name )
{
    short int output;

    /* Step analog outputs from 0 to 4095 and read from ADC */
    for( output=0; output<=4095; output+=200 ) {
        short int input;
        int chan;
        
        for( chan = 0; chan < 2; chan++ ) {
            if( aim104_multi_io_DAC( fd, chan, output ) < 0 ) {
                fprintf( stderr, "%s: %s: DAC output: %s\n",
                         exe_name, dev_name, strerror( errno ) );
                exit( 1 );
            }
        }

        printf( "ADC " );
        for( chan = 0; chan < 8; chan++ ) {            
            input = aim104_multi_io_ADC( fd, chan, 0 );
            if( input < 0 ) {
                fprintf( stderr, "%s: %s: ADC input: %s\n",
                         exe_name, dev_name, strerror( errno ) );
                exit( 1 );
            }
            printf( "%d: %d  ", chan, input );
        }
        printf( "\n" );
        
        sleep( 1 );
    }
}


static void test_digital( int fd, const char *exe_name, const char *dev_name )
{
    unsigned char inputs;

    /* continuously read all 8 inputs */
    for(;;) {
        short new_inputs;
        
        new_inputs = aim104_multi_io_inputs( fd );
        if( new_inputs < 0 ) {
            fprintf( stderr, "%s: %s: digital inputs: %s\n",
                     exe_name, dev_name, strerror( errno ) );
            exit( 1 );
        }
        
        /* only print inputs when they change */
        if( new_inputs != inputs ) {
            printf( "inputs: 0x%02x\n", (unsigned)new_inputs );
            inputs = new_inputs;
        }
    }
}
