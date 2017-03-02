/* Aim104Out16.java - Java class for the AIM104-OUT16 board.

   Copyright 2002 Arcom Control Systems Ltd

   Released under the terms of the GNU Lesser GPL v2.1.
*/

package com.arcom.aim104;

/**
 * Class for an AIM104-OUT16 board.
 */
public class Aim104Out16 extends Aim104
{
    /**
     * Enable or disable all the outputs.
     *
     * @param enab true - enable; false - disable.
     * @exception SysIOException on a system I/O or file error.
     */
    public void enable_outputs( boolean enab ) throws SysIOException {
        if( enable( fd, enab )<0 ) {
            throw new SysIOException( dev_name, "enable outputs",
                                      c_strerror( c_errno() ) );
        }
    }

    /**
     * Set all 16 outputs at once.
     *
     * @param s the required output status.
     * @exception SysIOException on a system I/O or file error.
     */
    public void set_all( short s ) throws SysIOException {
        byte[] buf=new byte[2];
        buf[0] = (byte)(s & 0xff);
        buf[1] = (byte)((s >> 8) & 0xff);

        if( sys_write( fd, buf, 2 )<0 ) {
            throw new SysIOException( dev_name, "write",
                                      c_strerror( c_errno() ) );
        }
    }

    /**
     * Set a single output line to high/on or low/off.
     *
     * @param pin the output line (0-15) to set to high.
     * @param on true - high/on; false - low/off.
     * @exception SysIOException on a system I/O or file error.
     */
    public void set( int pin, boolean on ) throws SysIOException {
        short outputs = output_status();
        short o;
        if( on ) {
            o = (short)(outputs | (1 << pin));
        } else {
            o = (short)(outputs & ~(1 << pin));
        }
        set_all( o );
    }

    /**
     * Returns the current status of all 16 outputs.
     *
     * @return The output status.
     * @exception SysIOException on a system I/O or file error.
     */
    public short output_status() throws SysIOException {
        int s;
       
        s=status( fd );
        if( s<0 ) {
            throw new SysIOException( dev_name, "output status",
                                      c_strerror( c_errno() ) );
        }

        return (short)s;
    }

    private static native int enable( int fd, boolean e );
    private static native int status( int fd );
}
