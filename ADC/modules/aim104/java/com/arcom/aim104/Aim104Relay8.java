/* Aim104Relay8.java - Java class for the AIM104-Relay8 board.

   Copyright 2002 Arcom Control Systems Ltd

   Released under the terms of the GNU Lesser GPL v2.1.
*/

package com.arcom.aim104;

/**
 * Class for an AIM104-RELAY8 board.
 */
public class Aim104Relay8 extends Aim104
{
    /**
     * Enable or disable all the relays.
     *
     * @param enab true - enable; false - disable.
     * @exception SysIOException on a system I/O or file error.
     */
    public void enable_relays( boolean enab ) throws SysIOException {
        if( enable( fd, enab )<0 ) {
            throw new SysIOException( dev_name, "enable relays",
                                      c_strerror( c_errno() ) );
        }
    }

    /**
     * Set the state of all 8 relays at once.
     *
     * @param s the new state of the relays.
     * @exception SysIOException on a system I/O or file error.
     */
    public void set_all( byte s ) throws SysIOException {
        byte[] buf=new byte[1];
        buf[0]=s;

        if( sys_write( fd, buf, 1 )<0 ) {
            throw new SysIOException( dev_name, "write", 
                                      c_strerror( c_errno() ) );
        }
    }

    /**
     * Set a single relay to on or off.
     *
     * @param index the relay (0-8) to set to on.
     * @param on true - on; false - off.
     * @exception SysIOException on a system I/O or file error.
     */
    public void set( int index, boolean on ) throws SysIOException {
        byte relays = relay_status();
        byte r;
        if( on ) {
            r = (byte)(relays | (1 << index));
        } else {
            r = (byte)(relays & ~(1 << index));
        }
        set_all( r );
    }

    /**
     * Returns the current status of the relays. 
     *
     * @return The relay status.
     * @exception SysIOException on a system I/O or file error.
     */
    public byte relay_status() throws SysIOException {
        int s;
       
        s=status( fd );
        if( s<0 ) {
            throw new SysIOException( dev_name, "relay status", 
                                      c_strerror( c_errno() ) );
        }

        return (byte)s;
    }

    /**
     * Returns the current inputs. 
     *
     * @return The inputs.
     * @exception SysIOException on a system I/O or file error.
     */
    public byte inputs() throws SysIOException {
        byte[] buf=new byte[1];

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
    public boolean input( int pin ) throws SysIOException {
        if( (inputs() & (1 << pin )) != 0 ) {
            return true;
        }
        return false;
    }

    private static native int enable( int fd, boolean e );
    private static native int status( int fd );
}

