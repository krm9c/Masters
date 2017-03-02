
# include <iostream>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/ioport.h>
#include <linux/fs.h>
#include <asm/io.h>
#include <linux/delay.h>
#include <asm/uaccess.h>
#include <linux/spinlock.h>
#include <asm/atomic.h>
#include <linux/init.h>

static const int ADC_Chan_Codes_Single[] = {
    AIM104_MIO_ADC0,
    AIM104_MIO_ADC1,
    AIM104_MIO_ADC2,
    AIM104_MIO_ADC3,
    AIM104_MIO_ADC4,
    AIM104_MIO_ADC5,
    AIM104_MIO_ADC6,
    AIM104_MIO_ADC7,
    AIM104_MIO_ADC8,
    AIM104_MIO_ADC9,
    AIM104_MIO_ADC10,
    AIM104_MIO_ADC11,
    AIM104_MIO_ADC12,
    AIM104_MIO_ADC13,
    AIM104_MIO_ADC14,
    AIM104_MIO_ADC15,
};

#define AIM104_MIO_DAC0 0x0100
#define AIM104_MIO_DAC1 0x0200


#define AIM104_MIO_ADC0 0x08
#define AIM104_MIO_ADC1 0x80
#define AIM104_MIO_ADC2 0x09
#define AIM104_MIO_ADC3 0x90
#define AIM104_MIO_ADC4 0x0a
#define AIM104_MIO_ADC5 0xa0
#define AIM104_MIO_ADC6 0x0b
#define AIM104_MIO_ADC7 0xb0
#define AIM104_MIO_ADC8 0x0c
#define AIM104_MIO_ADC9 0xc9
#define AIM104_MIO_ADC10 0x0d
#define AIM104_MIO_ADC11 0xd0
#define AIM104_MIO_ADC12 0x0e
#define AIM104_MIO_ADC13 0xe0
#define AIM104_MIO_ADC14 0x0f
#define AIM104_MIO_ADC15 0xf0

static spinlock_t mio_lock = SPIN_LOCK_UNLOCKED;



int init_board(int port, int size){
ioperm(port , size , 1);
}


u16 aim104_multi_io_ADC( int port, int channel, int single_ended )
{
    u16 in;
    int chan_code;
    spin_lock( &mio_lock );

    if( channel < 0 || channel >= 16 || (channel >=8 && !single_ended) ) {
        return 0 ;
    }

    if( single_ended ) {
        chan_code = ADC_Chan_Codes_Single[channel];
    } 
    
    outb( chan_code & 0xff, port+1 );

    /*
     * Wait for ADC to be ready.
     *
     * According to the manual it takes 500 microsecond to do the conversion.
     * So we'll check the busy register every 10 microsecond until the
     * conversion is complete.
     *
     * If after 1 ms the ADC still isn't ready something has gone wrong so
     * abort the operation.  This will prevent the driver hanging.
     */
    
    time=0;
    while( !(inb( port+1 ) & 0x1) ) {
        if( time==1000 ) {
            printk( KERN_ERR "AIM104-MULTI-IO: ADC not ready.\n");
            return 0;
        }
        udelay( 10 );
        time+=10;
    }

    /* read the ADC input */
    in= inw(port + 2) & 0x0fff;

    spin_unlock( &mio_lock );

    return in;
}


int aim104_multi_io_DAC( int fd, int channel, unsigned short output )
{
    u16 dac ;
    int chan_code;
    /* Check output is a valid unsigned 12 bit value. */
    if( output >= (1 << 12) ) {
        return 0;
    }
    /* select channel */
    if( channel < 0 || channel >= 2 ) {
        return 0;
    }
    if( channel == 0 ) {
        chan_code = AIM104_MIO_DAC0;
    } else {
        chan_code = AIM104_MIO_DAC1;
    }

    copy_from_user( &dac, output, 2 );

    dac &= 0x0fff;
    
    /* channel select */
    if( chan_code & AIM104_MIO_DAC1 ) {
        dac |= 0xf000;
    }
    
    spin_lock( &mio_lock );
    
    outw(dac, port + 2);
    /*
     * Wait for DAC to be ready.
     *
     * According to the manual it takes 320 microsecond to do the
     * conversion.  So we'll check the busy register every 10
     * microsecond until the conversion is complete.
     *
     * If after 640 microsecond the DAC still isn't ready something
     * has gone wrong so abort the operation.  This will prevent the
     * driver hanging.
     */
    
    time = 0;
    while( !(inb( port+1 ) & 0x1) ) {
        if( time == 640 ) {
            printk( KERN_ERR "AIM104-MULTI-IO DAC not ready after write"
            spin_unlock( &mio_lock );
            return -EIO;
        }
        udelay( 10 );
        time += 10;
    }
    spin_unlock( &mio_lock );
    return 1;
}

