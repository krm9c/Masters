/* SysIOException.java

   Copyright 2002 Arcom Control Systems Ltd

   Released under the terms of the GNU Lesser GPL v2.1.
*/

package com.arcom.aim104;

/**
 * This exception is thrown when a low-level problem occurs accesing
 * the Aim104 device.
 */
public class SysIOException extends Exception
{
    /**
     * Constuct the exception with a descriptive message.
     * <p>
     * The format of the message is: <code>f_name: sys_call:
     * mes</code>.
     *
     * @param f_name the file name of the file with which the error
     * occurred.
     * @param sys_call the system call that failed.
     * @param mes a descriptive message indicated the problem.
     *
     * @see com.arcom.aim104.Aim104#c_strerror(int)
     * @see com.arcom.aim104.Aim104#c_errno()
     */
    public SysIOException( String f_name, String sys_call, String mes ) {
        super( f_name + ": " + sys_call + ": " + mes );
    }
}
