#! /bin/sh
# Create all the device nodes for the AIM104 kernel drivers.

if [ $# -gt 0 ] ; then
    DESTDIR=$1
fi

dev_prefix=${DESTDIR}/dev/arcom/aim104
boards="relay8 in16 out16 io32 multi-io"
major_num=240

minor_num=0
for board in $boards; do
    mkdir -p $dev_prefix/$board
    i=0
    while [ $i -lt 8 ]; do
        mknod $dev_prefix/$board/$i c $major_num $minor_num
        let minor_num=minor_num+1
        let i=i+1
    done
done
