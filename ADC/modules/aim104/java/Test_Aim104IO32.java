/* Test_Aim104IO32.java -- test the Java Aim104 library. */

import java.lang.*;
import com.arcom.aim104.*;

public class Test_Aim104IO32
{
    public static void main( String[] args )
    {
        String dev_name = "/dev/arcom/aim104/io32/0";
        if( args.length > 0 ) {
            dev_name = args[0];
        }

        Aim104IO32 io32=new Aim104IO32();
        try {
            io32.open( dev_name );

            /* Set the first 16 lines 0-15 as outputs and lines 16-31
               as inputs. */
            io32.configure_as_inputs( 0xffff0000 );

            /* Test outputs. */
            io32.set_all( 0x0000 );
            io32.enable_outputs( true );
            for( int p = 0; p < 16; p++ ) {
                io32.set( p, true );
                System.out.print( "Output status (lines 0-15) = 0x" );
                System.out.println( Integer.toHexString( io32.output_status()
                                                         & 0xffff ) );
                try { Thread.sleep(500); } catch (Exception e) {}
                io32.set( p, false );
            }
            io32.set_all( 0x0000 );
            io32.enable_outputs( false );

            /* Test inputs. */
            for( int i = 0; i < 10; i++ ) {
                System.out.print( "Inputs (lines 16-31) = 0x" );
                System.out.println( Integer.toHexString( (io32.inputs() >> 16)
                                                         & 0xffff ) );
                try { Thread.sleep(500); } catch (Exception e) {}
            }
        
            io32.close();
        }
        catch( SysIOException e ) {
            System.out.println( e.getMessage() );
        }
    }
}
