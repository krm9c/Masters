/* Aim104_cni.cc - CNI C++ code for Aim104.java 

   Copyright 2002 Arcom Control Systems Ltd

   Released under the terms of the GNU Lesser GPL v2.1.
*/
#include <gcj/cni.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>

#include "Aim104.h"

using namespace com::arcom::aim104;

jint Aim104::sys_open_rw( jstring file_name )
{
    int len=JvGetStringUTFLength( file_name );
    char *fn=new char[len + 1];
    JvGetStringUTFRegion( file_name, 0, file_name->length(), fn );
    fn[len]='\0';
    
    int f=::open( fn, O_RDWR );
    
    delete fn;
    return f;
}

jint Aim104::sys_read( jint fd, jbyteArray buf, jint len )
{
    return read( fd, elements( buf ), len );
}

jint Aim104::sys_write( jint fd, jbyteArray buf, jint len )
{
    return write( fd, elements( buf ), len );
}

jint Aim104::sys_close( jint fd )
{
    return ::close( fd );
}

jint Aim104::c_errno()
{
    return errno;
}

jstring Aim104::c_strerror( jint e )
{
    return JvNewStringLatin1( strerror( e ) );
}

