/* test_aim104-multi-io.c - test the aim104-multi-io kernel driver.

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

#include "linux/arcom/aim104.h"

static void test_analog( int fd, const char *exe_name, const char *dev_name );
static void write_to_channel( int channel, short int output, int fd,
                              const char *exe_name, const char *dev_name );
static short int read_from_channel( int channel, int fd, const char *exe_name,
                                    const char *dev_name );
static void test_digital( int fd, const char *exe_name, const char *dev_name );

int main( int argc, char *argv[] )
{
	int fd;

	if( argc<2 ) {
		fprintf( stderr, "%s: required file name missing\n", argv[0] );
		exit( 1 );
	}

	/* open */
	if( (fd=open( argv[1], O_RDWR ))<0 ) {
		fprintf( stderr, "%s: %s: %s\n", argv[0], argv[1], strerror( errno ) );
		exit( 1 );
	}

    test_analog( fd, argv[0], argv[1] );

    test_digital( fd, argv[0], argv[1] );

    /* close */
 	if( close(fd)<0 ) {
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
        
        write_to_channel( AIM104_MIO_DAC0, output, fd, exe_name, dev_name );
        write_to_channel( AIM104_MIO_DAC1, output, fd, exe_name, dev_name );
        
        input=read_from_channel( AIM104_MIO_ADC0D, fd, exe_name, dev_name );
        printf( "ADC 0: %d  ", input );
        input=read_from_channel( AIM104_MIO_ADC1D, fd, exe_name, dev_name );
        printf( "1: %d  ", input );
        input=read_from_channel( AIM104_MIO_ADC2D, fd, exe_name, dev_name );
        printf( "2: %d  ", input );
        input=read_from_channel( AIM104_MIO_ADC3D, fd, exe_name, dev_name );
        printf( "3: %d  ", input );
        input=read_from_channel( AIM104_MIO_ADC4D, fd, exe_name, dev_name );
        printf( "4: %d  ", input );
        input=read_from_channel( AIM104_MIO_ADC5D, fd, exe_name, dev_name );
        printf( "5: %d  ", input );
        input=read_from_channel( AIM104_MIO_ADC6D, fd, exe_name, dev_name );
        printf( "6: %d  ", input );
        input=read_from_channel( AIM104_MIO_ADC7D, fd, exe_name, dev_name );
        printf( "7: %d\n", input );
        
        sleep( 1 );
    }
}


static void write_to_channel( int channel, short int output, int fd,
                              const char *exe_name, const char *dev_name )
{
    /* select channel */
    if( ioctl(fd, AIM104_MIO_IOC_CHANNEL, channel )<0 ) {
        fprintf( stderr, "%s: %s: channel select ioctl: %s\n",
                 exe_name, dev_name, strerror( errno ) );
            exit( 1 );
    }
    
    /* write data */
    if( write(fd, &output, 2)<0 ) {
        fprintf( stderr, "%s: %s: write: %s\n", exe_name, dev_name, 
                 strerror( errno ) );
        exit( 1 );
    }
}

static short int read_from_channel( int channel, int fd, const char *exe_name,
                                    const char *dev_name )
{
    short int input;

    /* select channel */
    if( ioctl(fd, AIM104_MIO_IOC_CHANNEL, channel )<0 ) {
        fprintf( stderr, "%s: %s: channel select ioctl: %s\n",
                 exe_name, dev_name, strerror( errno ) );
            exit( 1 );
    }
    
    /* read data */
    if( read(fd, &input, 2)<0 ) {
        fprintf( stderr, "%s: %s: read: %s\n", exe_name, dev_name, 
                 strerror( errno ) );
        exit( 1 );
    }

    return input;
}

static void test_digital( int fd, const char *exe_name, const char *dev_name )
{
    unsigned char inputs;

    /* select digital inputs */
    printf( "Selecting digital inputs.\n" );
    if( ioctl(fd, AIM104_MIO_IOC_CHANNEL, AIM104_MIO_DIGITAL )<0 ) {
        fprintf( stderr, "%s: %s: channel select ioctl: %s\n",
                 exe_name, dev_name, strerror( errno ) );
        exit( 1 );
    }

    /* continuously read all 8 inputs */
    for(;;) {
        unsigned char new_inputs;
        
        if( read(fd, &new_inputs, 1)<0 ) {
            fprintf( stderr, "%s: %s: read: %s\n", exe_name, dev_name,
                     strerror( errno ) );
            exit( 1 );
        }
        
        /* only print inputs when they change */
        if( new_inputs!=inputs ) {
            printf( "inputs: 0x%02x\n", (unsigned)new_inputs );
            inputs=new_inputs;
        }
    }
}
