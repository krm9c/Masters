/* aim104-multi-io.c - Kernel module driver for AIM104-MULTI-IO PC104 board.

   Copyright (C) 2004 Arcom Control Systems Ltd.

   Released under the terms of the GNU GPL v2.

   $Id: aim104-multi-io.c 2104 2005-11-03 12:39:34Z dvrabel $
*/

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

#include "linux/arcom/aim104.h"

#define VERSION "$Revision: 2104 $"

static int io_base[AIM104_MAX_BOARDS]={0,};
MODULE_PARM(io_base, "1-" __MODULE_STRING(AIM104_MAX_BOARDS) "i");

static atomic_t is_open[AIM104_MAX_BOARDS];
static short int active_channel[AIM104_MAX_BOARDS];

static spinlock_t mio_lock = SPIN_LOCK_UNLOCKED;

static int aim104_multi_io_channel( int board_num, unsigned long arg )
{
    spin_lock( &mio_lock );

    /* low byte is for the input */
    if( arg & 0xff ) {
        active_channel[board_num]
            = (active_channel[board_num] & 0xff00) | (arg & 0xff);
    }
    /* high byte is for the output */
    if( arg & 0xff00 ) {
        active_channel[board_num]
            = (active_channel[board_num] & 0xff) | (arg & 0xff00);
    }

    spin_unlock( &mio_lock );

    return 0;
}

/*
 * Read data from the board.
 *
 * Digital inputs selected: Read one byte.
 * ADC selected: Read two bytes (trying to read 1 byte is an error).
 */
static ssize_t aim104_multi_io_read( struct file *filp,
                                     char *buf, size_t len, loff_t *ppos )
{
    u8 d;
    u16 adc;
    int board_num = (int)filp->private_data;
    int time;

    if( len<1 ) {
        return 0;
    }

    spin_lock( &mio_lock );

    if( (active_channel[board_num] & 0xff) == AIM104_MIO_DIGITAL ) {
        /* read digital inputs */
        d=inb( io_base[board_num] );

        /* 0 is high so flip bits */
        d ^= 0xff;

        spin_unlock( &mio_lock );

        if( copy_to_user( buf, &d, 1 ) != 0 ) {
            return -EFAULT;
        }

        return 1;
    }

    /*
     * An ADC channel is selected.
     */
    /* only allow 2 byte reads */
    if( len!=2 ) {
        spin_unlock( &mio_lock );
        return -EINVAL;
    }

    /* select channel */
    outb( active_channel[board_num] & 0xff, io_base[board_num]+1 );

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
    while( !(inb( io_base[board_num]+1 ) & 0x1) ) {
        if( time==1000 ) {
            printk( KERN_ERR "AIM104-MULTI-IO (board %d @ 0x%x): "
                    "ADC not ready.\n",
                    board_num, io_base[board_num] );
            spin_unlock( &mio_lock );
            return -EIO;
        }
        udelay( 10 );
        time+=10;
    }

    /* read the ADC input */
    adc = inw(io_base[board_num] + 2) & 0x0fff;

    spin_unlock( &mio_lock );

    if( copy_to_user( buf, &adc, 2 ) != 0 ) {
        return -EFAULT;
    }

    return 2;
}

/*
 * Write 2 bytes (12 bits) to the currently selected DAC channel.
 */
static ssize_t aim104_multi_io_write( struct file *filp, const char *buf,
                                      size_t len, loff_t *ppos )
{
    int board_num = (int)filp->private_data;
    int time;
    u16 dac;

    if( len<1 ) {
        return 0;
    }

    /* only allow 2 byte writes */
    if( len!=2 ) {
        return -EINVAL;
    }

    if( copy_from_user( &dac, buf, 2 ) != 0 ) {
        return -EFAULT;
    }

    dac &= 0x0fff;

    /* channel select */
    if( active_channel[board_num] & AIM104_MIO_DAC1 ) {
        dac |= 0xf000;
    }

    spin_lock( &mio_lock );

    outw(dac, io_base[board_num] + 2);

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
    while( !(inb( io_base[board_num]+1 ) & 0x1) ) {
        if( time == 640 ) {
            printk( KERN_ERR "AIM104-MULTI-IO (board %d @ 0x%x): "
                    "DAC not ready after write.\n",
                    board_num, io_base[board_num] );
            spin_unlock( &mio_lock );
            return -EIO;
        }
        udelay( 10 );
        time += 10;
    }

    spin_unlock( &mio_lock );

    return 2;
}

static int aim104_multi_io_ioctl( struct inode *inode,struct file *filp,
                                  unsigned int cmd, unsigned long arg )
{
    int board_num = (int)filp->private_data;

    if( _IOC_TYPE(cmd) != AIM104_MIO_IOC_MAGIC ) {
        return -ENOTTY;
    }

    /* check for read/write permissions */
    if( (_IOC_DIR(cmd) & _IOC_READ) && !(filp->f_mode & FMODE_READ) ) {
        return -EACCES;
    }
    if( (_IOC_DIR(cmd) & _IOC_WRITE) && !(filp->f_mode & FMODE_WRITE) ) {
        return -EACCES;
    }

    switch( cmd ) {
      case AIM104_MIO_IOC_CHANNEL:
        return aim104_multi_io_channel( board_num, arg );

      default:
        return -ENOTTY;
    }
}

static int aim104_multi_io_open( struct inode *inode, struct file *filp )
{
    int board_num = MINOR(inode->i_rdev) - AIM104_MIO_CHAR_MINOR;

    atomic_inc(&is_open[board_num]);
    if (atomic_read(&is_open[board_num]) > 1) {
        atomic_dec(&is_open[board_num]);
        return -EBUSY;
    }

    filp->private_data=(void*)board_num;

    /* default to digital inputs and DAC0 */
    active_channel[board_num]=AIM104_MIO_DIGITAL|AIM104_MIO_DAC0;

    return 0;
}

static int aim104_multi_io_release( struct inode *inode, struct file *filp )
{
    int board_num = (int)filp->private_data;

    atomic_dec(&is_open[board_num]);

    return 0;
}


static struct file_operations aim104_multi_io_fops={
    .owner   = THIS_MODULE,
    .read    = aim104_multi_io_read,
    .write   = aim104_multi_io_write,
    .ioctl   = aim104_multi_io_ioctl,
    .open    = aim104_multi_io_open,
    .release = aim104_multi_io_release,
};

static void aim104_multi_io_cleanup(void)
{
    int i;

    i=0;
    while( io_base[i]!=0 && i<AIM104_MAX_BOARDS ) {
        remove_aim104_device( AIM104_MIO_CHAR_MINOR+i );

        release_region( io_base[i], AIM104_MIO_IO_SIZE );

        i++;
    }
}

static __init int aim104_multi_io_init(void)
{
    int i;

    printk( KERN_INFO "AIM104-MULTI-IO driver "VERSION"\n" );

    /* If there were no parameters, assume a single board with a default
     * port. */
    if( io_base[0]==0 ) {
        io_base[0]=AIM104_MIO_DEF_PORT;
    }

    i=0;
    while( io_base[i]!=0 && i<AIM104_MAX_BOARDS ) {
        int err;
        err=add_aim104_device( AIM104_MIO_CHAR_MINOR+i,
                               &aim104_multi_io_fops );
        if( err<0 ) {
            io_base[i]=0;
            aim104_multi_io_cleanup();
            return err;
        }

        if( !request_region( io_base[i], AIM104_MIO_IO_SIZE,
                             "AIM104-MULTI-IO" ) ) {
            printk( KERN_ERR
                    "aim104-multi-io: "
                    "I/O ports 0x%x-0x%x are already in use.\n",
                    io_base[i], io_base[i]+AIM104_MIO_IO_SIZE-1 );
            remove_aim104_device( AIM104_MIO_CHAR_MINOR+i );
            io_base[i]=0;
            aim104_multi_io_cleanup();
            return -EBUSY;
        }

        atomic_set(&is_open[i], 0);

        i++;
    }

    return 0;
}

static __exit void aim104_multi_io_exit(void)
{
    aim104_multi_io_cleanup();
}

module_init(aim104_multi_io_init);
module_exit(aim104_multi_io_exit);

MODULE_AUTHOR("Arcom Control Systems Ltd.");
MODULE_LICENSE("GPL");

