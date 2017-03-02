/* test_aim104-in16.c - test the aim104-in16 kernel driver.

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

#include "linux/arcom/aim104.h"

int main( int argc, char *argv[] )
{
	int fd;
    unsigned short inputs=0;
    unsigned low_inputs=0;
    unsigned high_inputs=0;
    int changes=0;

	if( argc<2 ) {
		fprintf( stderr, "%s: required file name missing\n", argv[0] );
		exit( 1 );
	}

	/* open */
	if( (fd=open( argv[1], O_RDONLY ))<0 ) {
		fprintf( stderr, "%s: %s: %s\n", argv[0], argv[1], strerror( errno ) );
		exit( 1 );
	}

    /* Read low then high bytes for 10 input chanages */
    while( changes<10 ) {
        unsigned char new_low_inputs;
        unsigned char new_high_inputs;

        if( read(fd, &new_low_inputs, 1)<0 ) {
            fprintf( stderr, "%s: %s: read: %s\n", argv[0], argv[1], 
                     strerror( errno ) );
            exit( 1 );
        }
        if( read(fd, &new_high_inputs, 1)<0 ) {
            fprintf( stderr, "%s: %s: read: %s\n", argv[0], argv[1], 
                     strerror( errno ) );
            exit( 1 );
        }

        /* only print inputs when they change */
        if( new_low_inputs!=low_inputs || new_high_inputs!=high_inputs  ) {
            printf( "high inputs: 0x%02x  low inputs: 0x%02x\n",
                    (unsigned)new_high_inputs, (unsigned)new_low_inputs );
            low_inputs=new_low_inputs;
            high_inputs=new_high_inputs;
            changes++;
        }
    }

    /* continuously read all 16 inputs */
    for(;;) {
        unsigned short new_inputs;

        if( read(fd, &new_inputs, 2)<0 ) {
            fprintf( stderr, "%s: %s: read: %s\n", argv[0], argv[1], 
                     strerror( errno ) );
            exit( 1 );
        }

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
