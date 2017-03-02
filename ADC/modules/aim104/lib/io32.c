/* io32.c - libaim104 source for AIM104-IO32

   Copyright 2002 Arcom Control Systems Ltd

   Released under the terms of the GNU Lesser GPL v2.1.
*/
#include <sys/types.h>
#include <unistd.h>
#include <sys/ioctl.h>

#include "arcom/libaim104.h"

int aim104_io32_enable_outputs( int fd, int enable )
{
    int err;

    err=ioctl( fd, AIM104_IO32_IOC_ENABLE, enable );
    if( err<0 ) {
        return AIM104_EIO;
    }
    return AIM104_SUCCESS;
}

int aim104_io32_set_all( int fd, int channel, unsigned char in_mask,
                         unsigned char set )
{
    if( channel<0 || channel>3 ) {
        return AIM104_EBAD_CHANNEL;
    }

    if( lseek( fd, channel, SEEK_SET )<0 ) {
        return AIM104_EIO;
    }

    /* Make sure the inputs remain low. */
    set &= ~in_mask;

    if( write( fd, &set, 4 )<0 ) {
        return AIM104_EIO;
    }
    return AIM104_SUCCESS;
}

int aim104_io32_inputs( int fd, int channel, unsigned char in_mask )
{
    unsigned char in;

    if( channel<0 || channel>3 ) {
        return AIM104_EBAD_CHANNEL;
    }

    if( lseek( fd, channel, SEEK_SET )<0 ) {
        return AIM104_EIO;
    }

    if( read( fd, &in, 1 )<0 ) {
        return AIM104_EIO;
    }
    /* zero non-masked inputs (as they're probably outputs) */
    in &= in_mask;
    return in;
}
