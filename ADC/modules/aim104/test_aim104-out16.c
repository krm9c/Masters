/* test_aim104-out16.c - test the aim104-out16 kernel driver.

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

#include "linux/arcom/aim104.h"

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
	if( ioctl(fd, AIM104_OUT16_IOC_ENABLE, 1)<0 ) {
		fprintf( stderr, "%s: %s: enable ioctl: %s\n", argv[0], argv[1], 
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
	if( ioctl(fd, AIM104_OUT16_IOC_ENABLE, 0)<0 ) {
		fprintf( stderr, "%s: %s: disable ioctl: %s\n", argv[0], argv[1], 
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

	/* check output status */
	if( (status=ioctl(fd, AIM104_OUT16_IOC_STATUS))<0 ) {
		fprintf( stderr, "%s: %s: status ioctl: %s\n", exe_name, dev_name, 
				 strerror( errno ) );
		exit( 1 );
	}
	printf( "Output status: 0x%04x\n", status );

	sleep(1);
}

static void set_outputs( int fd, unsigned short outputs, const char *exe_name,
                         const char *dev_name )
{
	if( write(fd, &outputs, 2)<0 ) {
		fprintf( stderr, "%s: %s: write: %s\n", exe_name, dev_name, 
				 strerror( errno ) );
		exit( 1 );
	}

    sleep(1);
}
