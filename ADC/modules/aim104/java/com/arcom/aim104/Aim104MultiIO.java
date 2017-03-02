/* Aim104MultiIO.java - Java class for the AIM104-MULTI-IO board.

   Copyright 2002 Arcom Control Systems Ltd

   Released under the terms of the GNU Lesser GPL v2.1.
*/

package com.arcom.aim104;

/**
 * Class for an AIM104-MULTI-IO board.
 */
public class Aim104MultiIO extends Aim104
{
    /**
     * Specifies the configuration of the ADCs (single ended or
     * differential).
     *
     * @param se true - single ended; false - differential. 
     */
    public Aim104MultiIO( boolean se )
    {
        single_ended = se;
    }

    /**
     * Specifies the board's ADCs are configured as single ended.
     */
    public Aim104MultiIO()
    {
        single_ended = true;
    }

    /**
     * Returns the current inputs.
     *
     * @return The inputs.
     * @exception SysIOException on a system I/O or file error.
     */
    public byte inputs() throws SysIOException
    {
        if( set_chan_to_inputs( fd ) < 0 ) {
            throw new SysIOException( dev_name, "set channel",
                                      c_strerror( c_errno() ) );
        }
        byte[] buf = new byte[1];
        if( sys_read( fd, buf, 1 )<0 ) {
            throw new SysIOException( dev_name, "read",
                                      c_strerror( c_errno() ) );
        }

        return buf[0];
    }

    /**
     * Returns the status of a single input line.
     *
     * @param pin the input line (0-7).
     * @exception SysIOException on a system I/O or file error.
     */
    public boolean input( int pin ) throws SysIOException
    {
        if( (inputs() & (1 << pin )) != 0 ) {
            return true;
        }
        return false;
    }

    /**
     * Read an ADC channel.
     *
     * @param channel the channel (0-16 for single ended mode or 0-7
     * for differential mode.
     * @return the 12 bit value read.
     * @exception SysIOException on a system I/O or file error.
     */
    public int read_ADC( int channel ) throws SysIOException
    {
        if( set_chan_to_ADC( fd, channel, single_ended ) < 0 ) {
            throw new SysIOException( dev_name, "set channel",
                                      c_strerror( c_errno() ) );
        }
        byte[] buf = new byte[2];
        if( sys_read( fd, buf, 2 ) < 0 ) {
            throw new SysIOException( dev_name, "read",
                                      c_strerror( c_errno() ) );
        }

        return ((buf[1] & 0xff) << 8) | (buf[0] & 0xff);
    }

    /**
     * Write to a DAC channel.
     *
     * @param channel the DAC channel (0-1) to write to.
     * @param value the 12 bit value to write.
     * @exception SysIOException on a system I/O or file error.
     */
    public void write_to_DAC( int channel, short value ) throws SysIOException
    {
        if( set_chan_to_DAC( fd, channel ) < 0 ) {
            throw new SysIOException( dev_name, "set channel",
                                      c_strerror( c_errno() ) );
        }
        byte[] buf = new byte[2];
        buf[0] = (byte)(value & 0xff);
        buf[1] = (byte)((value >> 8) & 0x0f);
        if( sys_write( fd, buf, 2 ) < 0 ) {
            throw new SysIOException( dev_name, "write",
                                      c_strerror( c_errno() ) );
        }        
    }

    /**
     * ADCs are configured as single ended (not differential).
     */
    private boolean single_ended;

    private static native int set_chan_to_inputs( int fd );
    private static native int set_chan_to_ADC( int fd, int chan,
                                               boolean single_ended );
    private static native int set_chan_to_DAC( int fd, int chan );
}
