/* Aim104Out16_jni.c - JNI C code for Aim104Out16.java 

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

#include "com_arcom_aim104_Aim104Out16.h"

JNIEXPORT jint JNICALL Java_com_arcom_aim104_Aim104Out16_enable(
    JNIEnv *env, jobject this, jint fd, jboolean en )
{
    return ioctl( fd, AIM104_OUT16_IOC_ENABLE, (en ? 1 : 0) );
}

JNIEXPORT jint JNICALL Java_com_arcom_aim104_Aim104Out16_status(
    JNIEnv *env, jobject this, jint fd )
{
    return ioctl( fd, AIM104_OUT16_IOC_STATUS, 0 );
}
