/* Aim104IO32_jni.c - JNI C code for Aim104IO32.java 

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

#include "com_arcom_aim104_Aim104IO32.h"

JNIEXPORT jint JNICALL Java_com_arcom_aim104_Aim104IO32_enable(
    JNIEnv *env, jobject this, jint fd, jboolean en )
{
    return ioctl( fd, AIM104_IO32_IOC_ENABLE, (en ? 1 : 0) );
}
