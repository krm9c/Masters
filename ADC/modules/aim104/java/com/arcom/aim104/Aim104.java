/* Aim104.java - Java base class for the various boards.

   Copyright 2002 Arcom Control Systems Ltd.

   Released under the terms of the GNU Lesser GPL v2.1.
*/

package com.arcom.aim104;

/**
 * Functions common to all AIM104 boards.
 * <p>
 * To access a board's features the device must first be opened the
 * the open() method.  The close() method must be called when access
 * to the device is no longer needed.
 */
public class Aim104
{
    /**
     * Open the AIM104 device.  This must be called before any of the
     * board's features can be accessed.
     *
     * @param file_name the device file to open.
     * @exception SysIOException on a system I/O or file error.
     */
    public void open( String file_name ) throws SysIOException {
        dev_name = file_name;

        fd=sys_open_rw( dev_name );
        if( fd<0 ) {
            throw new SysIOException( dev_name, "open", 
                                      c_strerror( c_errno() ) );
        }
    }

    /**
     * Close the AIM104 device.
     *
     * @exception SysIOException on a system I/O or file error.
     */
    public void close() throws SysIOException {
        if( sys_close( fd )<0 ) {
            throw new SysIOException( dev_name, "close",
                                      c_strerror( c_errno() ) );
        }
    }

    /**
     * Device file name.
     */
    protected String dev_name;
    /**
     * File descriptor for the open device.
     */
    protected int fd;

    /** Wrapper around the open(..., O_RDRW ) system call. */
    protected static native int sys_open_rw( String file_name );
    /** Wrapper around the close(...) system call. */
    protected static native int sys_close( int fd );
    /** Wrapper around the read(...) system call. */
    protected static native int sys_read( int fd, byte[] buf, int len );
    /** Wrapper around the write(...) system call. */
    protected static native int sys_write( int fd, byte[] buf, int len );
    
    /** Wrapper around the standard C errno variable. */
    protected static native int c_errno();
    /** Wrapper around the standard C strerror() function. */
    protected static native String c_strerror( int e );

    static {
        System.loadLibrary( "aim104_java" );
    }
}
