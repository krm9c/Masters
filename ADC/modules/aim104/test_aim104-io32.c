/* test_aim104-io16.c - test the aim104-io32 kernel driver.

   Usage:
     test_aim104-io32 <device node>
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

static void set_outputs( int fd, unsigned long output, const char *exe_name,
                         const char *dev_name );

int main( int argc, char *argv[] )
{
	int fd;
    unsigned long i;
    unsigned long inputs = 0;

	if( argc<2 ) {
		fprintf( stderr, "%s: required file name missing\n", argv[0] );
		exit( 1 );
	}

	/* open */
	if( (fd=open( argv[1], O_RDWR ))<0 ) {
		fprintf( stderr, "%s: %s: %s\n", argv[0], argv[1], strerror( errno ) );
		exit( 1 );
	}

    /* turn off outputs */
    set_outputs( fd, 0, argv[0], argv[1] );

	/* enable outputs */
	printf( "Enable outputs.\n" );
	if( ioctl(fd, AIM104_IO32_IOC_ENABLE, 1)<0 ) {
		fprintf( stderr, "%s: %s: enable ioctl: %s\n", argv[0], argv[1], 
				 strerror( errno ) );
		exit( 1 );
	}

	sleep(1);

    /* set each output on in turn */
    for( i=1; i<=0x80000000 && i!=0; i<<=1 ) {
        set_outputs( fd, i, argv[0], argv[1] );
    }
    
	/* disable outputs */
	printf( "Disable outputs.\n" );
	if( ioctl(fd, AIM104_IO32_IOC_ENABLE, 0)<0 ) {
		fprintf( stderr, "%s: %s: disable ioctl: %s\n", argv[0], argv[1], 
				 strerror( errno ) );
		exit( 1 );
	}

    /* turn off outputs which are going to be inputs */
    set_outputs( fd, 0, argv[0], argv[1] );

	/* enable outputs */
	printf( "Enable outputs.\n" );
	if( ioctl(fd, AIM104_IO32_IOC_ENABLE, 1)<0 ) {
		fprintf( stderr, "%s: %s: enable ioctl: %s\n", argv[0], argv[1], 
				 strerror( errno ) );
		exit( 1 );
	}

    /* continuously read all 16 inputs */
    for(;;) {
        unsigned long new_inputs;

        if( read(fd, &new_inputs, 4)<0 ) {
            fprintf( stderr, "%s: %s: read: %s\n", argv[0], argv[1], 
                     strerror( errno ) );
            exit( 1 );
        }

        /* only print inputs when they change */
        if( new_inputs!=inputs ) {
            printf( "inputs: 0x%08lx\n", (unsigned long)new_inputs );
            inputs=new_inputs;
        }
    }

	/* disable outputs */
	printf( "Disable outputs.\n" );
	if( ioctl(fd, AIM104_IO32_IOC_ENABLE, 0)<0 ) {
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


static void set_outputs( int fd, unsigned long outputs, const char *exe_name,
                         const char *dev_name )
{
	if( write(fd, &outputs, 4)<0 ) {
		fprintf( stderr, "%s: %s: write: %s\n", exe_name, dev_name, 
				 strerror( errno ) );
		exit( 1 );
	}

    usleep( 100000 );
}
