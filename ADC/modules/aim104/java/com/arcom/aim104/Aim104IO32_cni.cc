/* Aim104IO32_cni.cc - CNI C++ code for Aim104IO32.java

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
#include <sys/ioctl.h>

#include <linux/arcom/aim104.h>

#include "Aim104.h"
#include "Aim104IO32.h"

using namespace com::arcom::aim104;

jint Aim104IO32::enable( jint fd, jboolean e )
{
    return ioctl( fd, AIM104_IO32_IOC_ENABLE, (e ? 1 : 0) );
}
