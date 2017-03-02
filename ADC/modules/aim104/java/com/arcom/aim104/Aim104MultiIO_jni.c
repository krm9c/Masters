/* Aim104MultiIO_jni.c - JNI C code for Aim104MultiIO.java 

   Copyright 2002 Arcom Control Systems Ltd

   Released under the terms of the GNU Lesser GPL v2.1.
*/
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <sys/ioctl.h>

#include <linux/arcom/aim104.h>

#include "com_arcom_aim104_Aim104MultiIO.h"

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

JNIEXPORT jint JNICALL
Java_com_arcom_aim104_Aim104MultiIO_set_1chan_1to_1inputs(
    JNIEnv *env, jobject this, jint fd )
{
    return ioctl( fd, AIM104_MIO_IOC_CHANNEL, AIM104_MIO_DIGITAL );
}

JNIEXPORT jint JNICALL
Java_com_arcom_aim104_Aim104MultiIO_set_1chan_1to_1ADC(
    JNIEnv *env, jobject this, jint fd, jint chan, jboolean se )
{
    int chan_code;

    if( se ) {
        chan_code = ADC_Chan_Codes_Single[chan];
    } else {
        chan_code = ADC_Chan_Codes_Diff[chan];
    }
    return ioctl( fd, AIM104_MIO_IOC_CHANNEL, chan_code );
}

JNIEXPORT jint JNICALL
Java_com_arcom_aim104_Aim104MultiIO_set_1chan_1to_1DAC(
    JNIEnv *env, jobject this, jint fd, jint chan )
{
    int chan_code;

    if( chan == 0 ) {
        chan_code = AIM104_MIO_DAC0;
    } else {
        chan_code = AIM104_MIO_DAC1;
    }
    return ioctl( fd, AIM104_MIO_IOC_CHANNEL, chan_code );
}
