/* Test_Aim104In16.java -- test the Java Aim104 library. */

import java.lang.*;
import com.arcom.aim104.*;

public class Test_Aim104In16
{
    public static void main( String[] args )
    {
        String dev_name = "/dev/arcom/aim104/in16/0";
        if( args.length > 0 ) {
            dev_name = args[0];
        }

        Aim104In16 in16=new Aim104In16();
        try {
            in16.open( dev_name );

            /* Test inputs. */
            for( int i = 0; i < 10; i++ ) {
                System.out.print( "Inputs = 0x" );
                System.out.println( Integer.toHexString( in16.inputs()
                                                         & 0xffff ) );
                try { Thread.sleep(500); } catch (Exception e) {}
            }
        
            in16.close();
        }
        catch( SysIOException e ) {
            System.out.println( e.getMessage() );
        }
    }
}
