/* Test_Aim104Out16.java -- test the Java Aim104 library. */

import java.lang.*;
import com.arcom.aim104.*;

public class Test_Aim104Out16
{
    public static void main( String[] args )
    {
        String dev_name = "/dev/arcom/aim104/out16/0";
        if( args.length > 0 ) {
            dev_name = args[0];
        }

        Aim104Out16 out16=new Aim104Out16();
        try {
            out16.open( dev_name );

            /* Test outputs. */
            out16.set_all( (byte)0x00 );
            out16.enable_outputs( true );
            for( int p = 0; p < 16; p++ ) {
                out16.set( p, true );
                System.out.print( "Output status = 0x" );
                System.out.println( Integer.toHexString( out16.output_status()
                                                         & 0xff ) );
                try { Thread.sleep(500); } catch (Exception e) {}
                out16.set( p, false );
            }
            out16.set_all( (byte)0x00 );
            out16.enable_outputs( false );

            out16.close();
        }
        catch( SysIOException e ) {
            System.out.println( e.getMessage() );
        }
    }
}
