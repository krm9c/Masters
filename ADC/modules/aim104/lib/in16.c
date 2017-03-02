/* in16.c - libaim104 source for AIM104-IN16

   Copyright 2002 Arcom Control Systems Ltd

   Released under the terms of the GNU Lesser GPL v2.1.
*/
#include <sys/types.h>
#include <unistd.h>

#include "arcom/libaim104.h"

int aim104_in16_inputs( int fd, int channel )
{
    unsigned char in;
    int err;

    if( channel<0 || channel>1 ) {
        return AIM104_EBAD_CHANNEL;
    }

    err=lseek( fd, channel, SEEK_SET );
    if( err<0 ) {
        return AIM104_EIO;
    }

    err=read( fd, &in, 1 );
    if( err<0 ) {
        return AIM104_EIO;
    }
    return in;
}
