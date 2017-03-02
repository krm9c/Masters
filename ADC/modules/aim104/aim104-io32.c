/* aim104-io32.c - Kernel module driver for AIM104-IO32 PC104 board.

   Copyright 2002 Arcom Control Systems Ltd.

   Released under the terms of the GNU GPL v2.

   $Id: aim104-io32.c 1403 2004-11-23 18:14:29Z dvrabel $
*/

#include <linux/types.h>
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/ioport.h>
#include <linux/fs.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <linux/spinlock.h>
#include <asm/atomic.h>
#include <linux/init.h>

#include "linux/arcom/aim104.h"

#define VERSION "$Revision: 1403 $"

static int io_base[AIM104_MAX_BOARDS]={0,};
MODULE_PARM(io_base, "1-" __MODULE_STRING(AIM104_MAX_BOARDS) "i");

static atomic_t is_open[AIM104_MAX_BOARDS];

static spinlock_t io32_lock = SPIN_LOCK_UNLOCKED;

static int aim104_io32_enable( int board_num, unsigned long arg )
{
    u8 d;

    /* enable/disable */
    d=arg ? 1 : 0;
    outb( d, io_base[board_num]+4 );

    return 0;
}

/*
 * Read 1-4 bytes from the board (input state or output status).
 *
 * 1st byte is input 0-7
 * 2nd byte is input 8-15
 * etc.
 *
 * The file pointer then wraps back to 0
 */
static ssize_t aim104_io32_read( struct file *filp,
                                 char *buf, size_t len, loff_t *ppos )
{
    u8 d;
    int board_num = (int)filp->private_data;
    int i;
    int max_len;
    char kbuf[4];

    max_len=4-filp->f_pos;

    spin_lock( &io32_lock );

    for( i=0; i<(len>max_len ? max_len : len); i++, filp->f_pos++ ) {
        /* read inputs */
        d = inb(io_base[board_num] + (int)filp->f_pos);

        /* 1 is high */
        kbuf[i]=d;
    }

    /* fallen off the end? */
    if( filp->f_pos>=4 ) {
        filp->f_pos=0;
    }

    spin_unlock( &io32_lock );

    if( copy_to_user( buf, kbuf, i ) != 0 ) {
        return -EFAULT;
    }

    return i;
}

/*
 * Write 1-4 bytes to the outputs.
 *
 * 1st byte is output 0-7
 * 2nd byte is output 8-15
 * etc.
 *
 * The file pointer then wraps back to 0
 */
static ssize_t aim104_io32_write( struct file *filp,
                                  const char *buf, size_t len, loff_t *ppos )
{
    int board_num = (int)filp->private_data;
    int i;
    int max_len;
    char kbuf[4];

    max_len=4-filp->f_pos;
    if( len > max_len ) {
        len = max_len;
    }

    if( copy_from_user( kbuf, buf, len ) != 0 ) {
        return -EFAULT;
    }

    spin_lock( &io32_lock );

    for( i=0; i < len; i++, filp->f_pos++ ) {
        u8 d;

        /* The outputs on an IO32 aren't inverted but we invert them for
         * backward compatibility with older version of the drivers. */
        d = kbuf[i] ^ 0xff;

        /* write outputs */
        outb(d, io_base[board_num] + (int)filp->f_pos);
    }

    /* fallen off the end? */
    if( filp->f_pos>=4  ) {
        filp->f_pos=0;
    }

    spin_unlock( &io32_lock );

    return i;
}

static int aim104_io32_ioctl( struct inode *inode,struct file *filp,
                              unsigned int cmd, unsigned long arg )
{
    int board_num = (int)filp->private_data;

    if( _IOC_TYPE(cmd) != AIM104_IO32_IOC_MAGIC ) {
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
      case AIM104_IO32_IOC_ENABLE:
        return aim104_io32_enable( board_num, arg );

      default:
        return -ENOTTY;
    }
}

static int aim104_io32_open( struct inode *inode, struct file *filp )
{
    int board_num = MINOR(inode->i_rdev) - AIM104_IO32_CHAR_MINOR;

    atomic_inc(&is_open[board_num]);
    if (atomic_read(&is_open[board_num]) > 1) {
        atomic_dec(&is_open[board_num]);
        return -EBUSY;
    }

    filp->private_data=(void*)board_num;
    filp->f_pos=0;

    return 0;
}

static int aim104_io32_release( struct inode *inode, struct file *filp )
{
    int board_num = (int)filp->private_data;

    atomic_dec(&is_open[board_num]);

    return 0;
}


static struct file_operations aim104_io32_fops = {
    .owner   = THIS_MODULE,
    .read    = aim104_io32_read,
    .write   = aim104_io32_write,
    .ioctl   = aim104_io32_ioctl,
    .open    = aim104_io32_open,
    .release = aim104_io32_release,
};

static void aim104_io32_cleanup(void)
{
    int i;

    i=0;
    while( io_base[i]!=0 && i<AIM104_MAX_BOARDS ) {
        remove_aim104_device( AIM104_IO32_CHAR_MINOR+i );

        release_region( io_base[i], AIM104_IO32_IO_SIZE );

        i++;
    }
}

static __init int aim104_io32_init(void)
{
    int i;

    printk( KERN_INFO "AIM104-IO32 driver "VERSION"\n" );

    /* If there were no parameters, assume a single board with a default
     * port. */
    if( io_base[0]==0 ) {
        io_base[0]=AIM104_IO32_DEF_PORT;
    }

    i=0;
    while( io_base[i]!=0 && i<AIM104_MAX_BOARDS ) {
        int err;
        err=add_aim104_device( AIM104_IO32_CHAR_MINOR+i, &aim104_io32_fops );
        if( err<0 ) {
            io_base[i]=0;
            aim104_io32_cleanup();
            return err;
        }

        if( !request_region( io_base[i], AIM104_IO32_IO_SIZE,
                             "AIM104-IO32" ) ) {
            printk( KERN_ERR
                    "aim104-io32: I/O ports 0x%x-0x%x are already in use.\n",
                    io_base[i], io_base[i]+AIM104_IO32_IO_SIZE-1 );
            remove_aim104_device( AIM104_IO32_CHAR_MINOR+i );
            io_base[i]=0;
            aim104_io32_cleanup();
            return -EBUSY;
        }

        atomic_set(&is_open[i], 0);

        i++;
    }

    return 0;
}

static __exit void aim104_io32_exit(void)
{
    aim104_io32_cleanup();
}

module_init(aim104_io32_init);
module_exit(aim104_io32_exit);

MODULE_AUTHOR("Arcom Control Systems Ltd.");
MODULE_LICENSE("GPL");
