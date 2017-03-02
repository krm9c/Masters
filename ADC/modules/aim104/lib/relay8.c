/* relay8.c - libaim104 source for AIM104-RELAY8

   Copyright 2002 Arcom Control Systems Ltd

   Released under the terms of the GNU Lesser GPL v2.1.
*/
#include <unistd.h>
#include <sys/ioctl.h>

#include "arcom/libaim104.h"

int aim104_relay8_enable_relays( int fd, int enable )
{
    int err;

    err=ioctl( fd, AIM104_R8_IOC_ENABLE, enable );

    if( err<0 ) {
        return AIM104_EIO;
    }
    return AIM104_SUCCESS;
}

int aim104_relay8_set_all( int fd, unsigned char set )
{
    int err;

    err=write( fd, &set, 1 );

    if( err<0 ) {
        return AIM104_EIO;
    }
    return AIM104_SUCCESS;
}

int aim104_relay8_set_masked( int fd, unsigned char mask, unsigned char set )
{
    int status;

    if( (status=aim104_relay8_relay_status( fd ))<0 ) {
        return status;
    }

    set=(set & mask) | ((unsigned char)status & ~mask );

    return aim104_relay8_set_all( fd, set );
}

int aim104_relay8_inputs( int fd )
{
    unsigned char in;
    int err;

    err=read( fd, &in, 1 );
    
    if( err<0 ) {
        return AIM104_EIO;
    }
    return in;
}

int aim104_relay8_relay_status( int fd  )
{
    unsigned char status;
    int err;

    err=ioctl( fd, AIM104_R8_IOC_STATUS, &status );

    if( err<0 ) {
        return AIM104_EIO;
    }
    return status;
}
