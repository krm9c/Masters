/* Aim104In16.java - Java class for the AIM104-IN16 board.

   Copyright 2002 Arcom Control Systems Ltd

   Released under the terms of the GNU Lesser GPL v2.1.
*/

package com.arcom.aim104;

/**
 * Class for an AIM104-IN16 board.
 */
public class Aim104In16 extends Aim104
{
    /**
     * Returns the status of all 16 inputs.
     *
     * @exception SysIOException on a system I/O or file error.
     */
    public int inputs() throws SysIOException {
        byte[] buf=new byte[2];

        if( sys_read( fd, buf, 2 )<0 ) {
            throw new SysIOException( dev_name, "read", 
                                      c_strerror( c_errno() ) );
        }

        return ((buf[1] & 0xff) << 8) | (buf[0] & 0xff);
    }

    /**
     * Returns the status of a single input line.
     *
     * @param pin the input line (0-15).
     * @exception SysIOException on a system I/O or file error.
     */
    public boolean input( int pin ) throws SysIOException {
        if( (inputs() & (1 << pin )) != 0 ) {
            return true;
        }
        return false;
    }
}

