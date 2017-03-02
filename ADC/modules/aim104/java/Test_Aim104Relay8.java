/* Test_Aim104Relay8.java -- test the Java Aim104 library. */

import java.lang.*;
import com.arcom.aim104.*;

public class Test_Aim104Relay8
{
    public static void main( String[] args )
    {
        String dev_name = "/dev/arcom/aim104/relay8/0";
        if( args.length > 0 ) {
            dev_name = args[0];
        }

        Aim104Relay8 r8i8=new Aim104Relay8();
        try {
            r8i8.open( dev_name );

            /* Test relays. */
            r8i8.set_all( (byte)0x00 );
            r8i8.enable_relays( true );
            for( int p = 0; p < 8; p++ ) {
                r8i8.set( p, true );
                System.out.print( "Relay status = 0x" );
                System.out.println( Integer.toHexString( r8i8.relay_status()
                                                         & 0xff ) );
                try { Thread.sleep(500); } catch (Exception e) {}
                r8i8.set( p, false );
            }
            r8i8.set_all( (byte)0x00 );
            r8i8.enable_relays( false );

            /* Test inputs. */
            for( int i = 0; i < 10; i++ ) {
                System.out.print( "Inputs = 0x" );
                System.out.println( Integer.toHexString( r8i8.inputs()
                                                         & 0xff ) );
                try { Thread.sleep(500); } catch (Exception e) {}
            }
        
            r8i8.close();
        }
        catch( SysIOException e ) {
            System.out.println( e.getMessage() );
        }
    }
}
