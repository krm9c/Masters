/* Aim104IO32.java - Java class for the AIM104-OUT16 board.

   Copyright 2002 Arcom Control Systems Ltd

   Released under the terms of the GNU Lesser GPL v2.1.
*/

package com.arcom.aim104;

/**
 * Class for an AIM104-IO32 board.
 */
public class Aim104IO32 extends Aim104
{
    /**
     * Enable or disable all the outputs.
     *
     * @param enab true - enable; false - disable.
     * @exception SysIOException on a system I/O or file error.
     */
    public void enable_outputs( boolean enab ) throws SysIOException {
        if( enable( fd, enab ) < 0 ) {
            throw new SysIOException( dev_name, "enable outputs",
                                      c_strerror( c_errno() ) );
        }
    }

    /**
     * Specify which lines are used as inputs.  These lines will then
     * always be driven low.
     *
     * @param mask bit mask of input lines.
     * @exception SysIOException on a system I/O or file error.
     */
    public void configure_as_inputs( int mask ) throws SysIOException {
        input_mask = mask;
        set_all( outputs );
    }

    /**
     * Set all 32 lines configured as outputs at once.
     *
     * @param s the required output status.
     * @exception SysIOException on a system I/O or file error.
     */
    public void set_all( int s ) throws SysIOException {
        s = s & ~input_mask;

        byte[] buf=new byte[4];
        buf[0] = (byte)(s & 0xff);
        buf[1] = (byte)((s >> 8) & 0xff);
        buf[2] = (byte)((s >> 16) & 0xff);
        buf[3] = (byte)((s >> 24) & 0xff);

        if( sys_write( fd, buf, 4 ) < 0 ) {
            throw new SysIOException( dev_name, "write",
                                      c_strerror( c_errno() ) );
        }
        outputs = s;
    }

    /**
     * Set a single output line high/on or low/off.
     *
     * @param pin the output line (0-31) to set to high.
     * @param on true - high/on; false - low/off.
     * @exception SysIOException on a system I/O or file error.
     */
    public void set( int pin, boolean on ) throws SysIOException {
        int o;
        if( on ) {
            o = outputs | (1 << pin);
        } else {
            o = outputs & ~(1 << pin);
        }
        set_all( o );
    }

    /**
     * Returns the current status of all 32 lines configured as
     * outputs.
     * <p> 
     * Note: The state isn't obtained from the board but is stored
     * in the object.
     *
     * @return The output status.
     */
    public int output_status() {
        return outputs & ~input_mask;
    }

    /**
     * Returns the inputs on the lines configured as inputs.
     *
     * @return The inputs.
     * @exception SysIOException on a system I/O or file error.
     */
    public int inputs() throws SysIOException {
        byte[] b = new byte[4];
        if( sys_read( fd, b, 4 ) < 0 ) {
            throw new SysIOException( dev_name, "read",
                                      c_strerror( c_errno() ) );
        }
        return (((b[3] & 0xff) << 24) | ((b[2] & 0xff) << 16) | ((b[1] & 0xff) << 8) | (b[0] & 0xff))
            & input_mask;
    }

    /**
     * Returns the status of a single input line.
     *
     * @param pin the input line (0-31).
     * @exception SysIOException on a system I/O or file error.
     */
    public boolean input( int pin ) throws SysIOException {
        if( (inputs() & (1 << pin )) != 0 ) {
            return true;
        }
        return false;
    }

    /**
     * Bit mask of lines which are configured as inputs.
     */
    private int input_mask;
    /**
     * Current status of the outputs.  We need to store it here as the
     * AIM104-IO32 board doesn't support reading back output status
     * (unlike the AIM104-OUT16 for example).
     */
    private int outputs;

    private static native int enable( int fd, boolean e );
}
