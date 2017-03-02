/* multi-io.c - libaim104 source for AIM104-MULTI-IO

   Copyright 2002 Arcom Control Systems Ltd.

   Released under the terms of the GNU Lesser GPL v2.1.
*/
#include <sys/types.h>
#include <unistd.h>
#include <sys/ioctl.h>

#include "arcom/libaim104.h"

/* Mappings from channel numbers to the channel codes used by the
   kernel driver. */
static const int ADC_Chan_Codes_Single[] = {
    AIM104_MIO_ADC0,
    AIM104_MIO_ADC1,
    AIM104_MIO_ADC2,
    AIM104_MIO_ADC3,
    AIM104_MIO_ADC4,
    AIM104_MIO_ADC5,
    AIM104_MIO_ADC6,
    AIM104_MIO_ADC7,
    AIM104_MIO_ADC8,
    AIM104_MIO_ADC9,
    AIM104_MIO_ADC10,
    AIM104_MIO_ADC11,
    AIM104_MIO_ADC12,
    AIM104_MIO_ADC13,
    AIM104_MIO_ADC14,
    AIM104_MIO_ADC15,
};

static const int ADC_Chan_Codes_Diff[] = {
    AIM104_MIO_ADC0D,
    AIM104_MIO_ADC1D,
    AIM104_MIO_ADC2D,
    AIM104_MIO_ADC3D,
    AIM104_MIO_ADC4D,
    AIM104_MIO_ADC5D,
    AIM104_MIO_ADC6D,
    AIM104_MIO_ADC7D,
};

int aim104_multi_io_inputs( int fd )
{
    unsigned char in;

    /* set input channel to digital inputs */
    if( ioctl( fd, AIM104_MIO_IOC_CHANNEL, AIM104_MIO_DIGITAL ) < 0 ) {
        return AIM104_EIO;
    }

    if( read( fd, &in, 1 ) < 0 ) {
        return AIM104_EIO;
    }

    return in;
}


int aim104_multi_io_ADC( int fd, int channel, int single_ended )
{
    unsigned short in;
    int chan_code;

    if( channel < 0 || channel >= 16 || (channel >=8 && !single_ended) ) {
        return AIM104_EBAD_CHANNEL;
    }

    if( single_ended ) {
        chan_code = ADC_Chan_Codes_Single[channel];
    } else {
        chan_code = ADC_Chan_Codes_Diff[channel];
    }
    if( ioctl( fd, AIM104_MIO_IOC_CHANNEL, chan_code ) < 0 ) {
        return AIM104_EIO;
    }

    if( read( fd, &in, 2 ) < 0 ) {
        return AIM104_EIO;
    }

    return in;
}


int aim104_multi_io_DAC( int fd, int channel, unsigned short output )
{
    int chan_code;

    /* Check output is a valid unsigned 12 bit value. */
    if( output >= (1 << 12) ) {
        return AIM104_ERANGE;
    }

    /* select channel */
    if( channel < 0 || channel >= 2 ) {
        return AIM104_EBAD_CHANNEL;
    }
    if( channel == 0 ) {
        chan_code = AIM104_MIO_DAC0;
    } else {
        chan_code = AIM104_MIO_DAC1;
    }
    if( ioctl( fd, AIM104_MIO_IOC_CHANNEL, chan_code ) < 0 ) {
        return AIM104_EIO;
    }

    if( write( fd, &output, 2 ) < 0 ) {
        return AIM104_EIO;
    }

    return AIM104_SUCCESS;
}
