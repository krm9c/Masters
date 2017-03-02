/* out16.c - libaim104 source for AIM104-OUT16

   Copyright 2002 Arcom Control Systems Ltd

   Released under the terms of the GNU Lesser GPL v2.1.
*/
#include <unistd.h>
#include <sys/ioctl.h>

#include "arcom/libaim104.h"

int aim104_out16_enable_outputs( int fd, int enable )
{
    int err;

    err=ioctl( fd, AIM104_OUT16_IOC_ENABLE, enable );

    if( err<0 ) {
        return AIM104_EIO;
    }
    return AIM104_SUCCESS;
}

int aim104_out16_set_all( int fd, int channel, unsigned char set )
{
    int err;

    if( channel<0 || channel>1 ) {
        return AIM104_EBAD_CHANNEL;
    }

    err=write( fd, &set, 2 );
    if( err<0 ) {
        return AIM104_EIO;
    }
    return AIM104_SUCCESS;
}

int aim104_out16_set_masked( int fd, int channel, unsigned char mask,
                             unsigned char set )
{
    int status;

    if( (status=aim104_out16_output_status( fd, channel ))<0 ) {
        return status;
    }

    set=(set & mask) | ((unsigned char)status & ~mask );

    return aim104_out16_set_all( fd, channel, set );
}

int aim104_out16_output_status( int fd, int channel )
{
    unsigned short status;
    int err;

    if( channel<0 || channel>1 ) {
        return AIM104_EBAD_CHANNEL;
    }

    err=ioctl( fd, AIM104_OUT16_IOC_STATUS, &status );
    if( err<0 ) {
        return AIM104_EIO;
    }
    return (unsigned char)(status >> 8*channel);
}
