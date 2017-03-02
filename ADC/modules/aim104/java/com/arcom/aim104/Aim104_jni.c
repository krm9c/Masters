/* Aim104_jni.c - JNI C code for Aim104.java 

   Copyright 2002 Arcom Control Systems Ltd

   Released under the terms of the GNU Lesser GPL v2.1.
*/
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>

#include "com_arcom_aim104_Aim104.h"

JNIEXPORT jint JNICALL Java_com_arcom_aim104_Aim104_sys_1open_1rw(
    JNIEnv *env, jobject this, jstring d )
{
    const char *d_name;
    int f;

    d_name = (*env)->GetStringUTFChars( env, d, 0 );
    f = open( d_name, O_RDWR );
    (*env)->ReleaseStringUTFChars( env, d, d_name );

    return f;
}

JNIEXPORT jint JNICALL Java_com_arcom_aim104_Aim104_sys_1read(
    JNIEnv *env, jobject this, jint fd, jbyteArray buf, jint len )
{
    jbyte *b;
    int ret;

    b = (*env)->GetByteArrayElements( env, buf, 0 );
    ret = read( fd, b, len );
    (*env)->ReleaseByteArrayElements( env, buf, b, 0 );

    return ret;
}

JNIEXPORT jint JNICALL Java_com_arcom_aim104_Aim104_sys_1write(
    JNIEnv *env, jobject this, jint fd, jbyteArray buf, jint len )
{
    jbyte *b;
    int ret;

    b = (*env)->GetByteArrayElements( env, buf, 0 );
    ret = write( fd, b, len );
    (*env)->ReleaseByteArrayElements( env, buf, b, 0 );

    return ret;
}

JNIEXPORT jint JNICALL Java_com_arcom_aim104_Aim104_sys_1close(
    JNIEnv *env, jobject this, jint fd )
{
    return close( fd );
}

JNIEXPORT jint JNICALL Java_com_arcom_aim104_Aim104_c_1errno(
    JNIEnv *env, jobject this )
{
    return errno;
}

JNIEXPORT jstring JNICALL Java_com_arcom_aim104_Aim104_c_1strerror(
    JNIEnv *env, jobject this, jint e )
{
    return (*env)->NewStringUTF( env, strerror( e ) );
}
