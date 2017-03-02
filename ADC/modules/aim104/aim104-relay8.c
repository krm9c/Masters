/* aim104-relay8.c - Kernel module driver for AIM104-RELAY8 PC104 board.

   Copyright (C) 2004 Arcom Control Systems Ltd.

   Released under the terms of the GNU GPL v2.

   $Id: aim104-relay8.c 1403 2004-11-23 18:14:29Z dvrabel $
*/

#include <linux/types.h>
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/ioport.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <asm/atomic.h>
#include <linux/init.h>

#include "linux/arcom/aim104.h"

#define VERSION "$Revision: 1403 $"

static int io_base[AIM104_MAX_BOARDS]={0,};
MODULE_PARM(io_base, "1-" __MODULE_STRING(AIM104_MAX_BOARDS) "i");

static atomic_t is_open[AIM104_MAX_BOARDS];

static int aim104_relay8_enable( int board_num, unsigned long arg )
{
    u8 d;

    d=arg ? 1 : 0;
    outb( d, io_base[board_num]+1 );

    return 0;
}

static int aim104_relay8_status( int board_num )
{
    u8 d;

    d=inb( io_base[board_num] );

    return d;
}

static ssize_t aim104_relay8_read( struct file *filp,
                                   char *buf, size_t len, loff_t *ppos )
{
    int board_num = (int)filp->private_data;
    char d;
    char kbuf[1];

    if( len>0 ) {
        /* Read all 8 of the opto-isolated inputs. */
        d=inb( io_base[board_num]+1 );

        /* Flip bits as 0 mean high. */
        kbuf[0]=d ^ 0xff;

        if( copy_to_user( buf, kbuf, 1 ) != 0 ) {
            return -EFAULT;
        }

        /* Only read one byte. */
        return 1;
    }
    return 0;
}

static ssize_t aim104_relay8_write( struct file *filp, const char *buf,
                                    size_t len, loff_t *ppos )
{
    int board_num = (int)filp->private_data;
    char kbuf[1];

    if( copy_from_user( kbuf, buf, 1 ) != 0 ) {
        return -EFAULT;
    }

    if( len>0 ) {
        /* write first data byte to relays */
        outb( kbuf[0], io_base[board_num] );

        return 1;
    }

    return 0;
}

static int aim104_relay8_ioctl( struct inode *inode,struct file *filp,
                                unsigned int cmd, unsigned long arg )
{
    int board_num = (int)filp->private_data;

    if( _IOC_TYPE(cmd) != AIM104_R8_IOC_MAGIC ) {
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
      case AIM104_R8_IOC_ENABLE:
        return aim104_relay8_enable( board_num, arg );

      case AIM104_R8_IOC_STATUS:
        return aim104_relay8_status( board_num );

      default :
        return -ENOTTY;
    }
}

static int aim104_relay8_open( struct inode *inode, struct file *filp )
{
    int board_num = MINOR(inode->i_rdev) - AIM104_R8_CHAR_MINOR;

    atomic_inc(&is_open[board_num]);
    if (atomic_read(&is_open[board_num]) > 1) {
        atomic_dec(&is_open[board_num]);
        return -EBUSY;
    }

    filp->private_data=(void*)board_num;

    return 0;
}

static int aim104_relay8_release( struct inode *inode, struct file *filp )
{
    int board_num = (int)filp->private_data;

    atomic_dec(&is_open[board_num]);

    return 0;
}


static struct file_operations aim104_relay8_fops={
    .owner   = THIS_MODULE,
    .read    = aim104_relay8_read,
    .write   = aim104_relay8_write,
    .ioctl   = aim104_relay8_ioctl,
    .open    = aim104_relay8_open,
    .release = aim104_relay8_release,
};

static void aim104_relay8_cleanup(void)
{
    int i;

    i=0;
    while( io_base[i]!=0 && i<AIM104_MAX_BOARDS ) {
        remove_aim104_device( AIM104_R8_CHAR_MINOR+i );

        release_region( io_base[i], AIM104_R8_IO_SIZE );

        i++;
    }
}

static __init int aim104_relay8_init(void)
{
    int i;

    printk( KERN_INFO "AIM104-RELAY8 driver "VERSION"\n" );

    /* If there were no parameters assume a single board with a default port */
    if( io_base[0]==0 ) {
        io_base[0]=AIM104_R8_DEF_PORT;
    }

    i=0;
    while( io_base[i]!=0 && i<AIM104_MAX_BOARDS ) {
        int err;
        err=add_aim104_device( AIM104_R8_CHAR_MINOR+i, &aim104_relay8_fops );
        if( err<0 ) {
            io_base[i]=0;
            aim104_relay8_cleanup();
            return err;
        }

        if( !request_region( io_base[i], AIM104_R8_IO_SIZE,
                             "AIM104-RELAY8" ) ) {
            printk( KERN_ERR
                    "aim104-relay8: "
                    "I/O ports 0x%x-0x%x are already in use.\n",
                    io_base[i], io_base[i]+AIM104_R8_IO_SIZE-1 );
            remove_aim104_device( AIM104_R8_CHAR_MINOR+i );
            io_base[i]=0;
            aim104_relay8_cleanup();
            return -EBUSY;
        }

        atomic_set(&is_open[i], 0);

        i++;
    }

    return 0;
}

static __exit void aim104_relay8_exit(void)
{
    aim104_relay8_cleanup();
}

module_init(aim104_relay8_init);
module_exit(aim104_relay8_exit);

MODULE_AUTHOR("Arcom Control Systems Ltd.");
MODULE_LICENSE("GPL");
