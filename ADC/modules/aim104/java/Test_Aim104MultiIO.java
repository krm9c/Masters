/* Test_Aim104MultiIO.java -- test the Java Aim104 library. */

import java.lang.*;
import com.arcom.aim104.*;

public class Test_Aim104MultiIO
{
    public static void main( String[] args )
    {
        String dev_name = "/dev/arcom/aim104/multi-io/0";
        if( args.length > 0 ) {
            dev_name = args[0];
        }

        Aim104MultiIO mio = new Aim104MultiIO( true );
        try {
            mio.open( dev_name );

            /* Test digital inputs. */
            for( int i = 0; i < 10; i++ ) {
                System.out.print( "Inputs = 0x" );
                System.out.println( Integer.toHexString( mio.inputs()
                                                         & 0xff ) );
                try { Thread.sleep(500); } catch (Exception e) {}
            }

            /* Test ADCs. */
            for( int i = 0; i < 10; i++ ) {
                for( int ch = 0; ch < 16; ch ++ ) {
                    System.out.println( "Channel " + ch + " = "
                                      + mio.read_ADC( ch ) );
                }
                try { Thread.sleep(500); } catch (Exception e) {}
            }

            /* Test DACs. */
            for( short val = 0; val <= 4096; val += 128 ) {
                for( int ch = 0; ch < 2; ch ++ ) {
                    System.out.println( "Setting channel " + ch + " to " 
                                        + val );
                    mio.write_to_DAC( ch, val );
                }
                try { Thread.sleep(500); } catch (Exception e) {}
            }

            mio.close();
        }
        catch( SysIOException e ) {
            System.out.println( e.getMessage() );
        }
    }
}
