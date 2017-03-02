/* aim104-in16.c - Kernel module driver for AIM104-IN16 PC104 board.

   Copyright (C) 2004 Arcom Control Systems Ltd.

   Released under the terms of the GNU GPL v2.

   $Id: aim104-in16.c 1403 2004-11-23 18:14:29Z dvrabel $
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

static spinlock_t in16_lock = SPIN_LOCK_UNLOCKED;

/*
 * Read 1 or 2 bytes from the inputs.
 *
 * 1st byte is input 0-7
 * 2nd byte is input 8-15
 *
 * The file pointer then wraps back to 0
 */
static ssize_t aim104_in16_read( struct file *filp,
                                 char *buf, size_t len, loff_t *ppos )
{
    int board_num = (int)filp->private_data;
    u8 d;
    int i;
    int max_len;
    char kbuf[2];

    max_len=(filp->f_pos>0 ? 1 : 2 );

    spin_lock( &in16_lock );

    for( i=0; i<(len>max_len ? max_len : len); i++, filp->f_pos++ ) {
        /* read inputs */
        d = inb(io_base[board_num] + (int)filp->f_pos);

        /* flip bits as 0 mean high */
        kbuf[i]=d ^ 0xff;
    }

    /* fallen off the end? */
    if( filp->f_pos>1 ) {
        filp->f_pos=0;
    }

    spin_unlock( &in16_lock );

    if( copy_to_user( buf, kbuf, i ) != 0 ) {
        return -EFAULT;
    }

    return i;
}

static int aim104_in16_open( struct inode *inode, struct file *filp )
{
    int board_num = MINOR(inode->i_rdev) - AIM104_IN16_CHAR_MINOR;

    atomic_inc(&is_open[board_num]);
    if (atomic_read(&is_open[board_num]) > 1) {
        atomic_dec(&is_open[board_num]);
        return -EBUSY;
    }

    filp->private_data=(void*)board_num;
    filp->f_pos=0;

    return 0;
}

static int aim104_in16_release( struct inode *inode, struct file *filp )
{
    int board_num = (int)filp->private_data;

    atomic_dec(&is_open[board_num]);

    return 0;
}


static struct file_operations aim104_in16_fops = {
    .owner   = THIS_MODULE,
    .read    = aim104_in16_read,
    .open    = aim104_in16_open,
    .release = aim104_in16_release,
};

static void aim104_in16_cleanup(void)
{
    int i;

    i=0;
    while( io_base[i]!=0 && i<AIM104_MAX_BOARDS ) {
        remove_aim104_device( AIM104_IN16_CHAR_MINOR+i );

        release_region( io_base[i], AIM104_IN16_IO_SIZE );

        i++;
    }
}

static __init int aim104_in16_init(void)
{
    int i;

    printk( KERN_INFO "AIM104-IN16 driver "VERSION"\n" );

    /* If there were no parameters assume a single board with a default port */
    if( io_base[0]==0 ) {
        io_base[0]=AIM104_IN16_DEF_PORT;
    }

    i=0;
    while( io_base[i]!=0 && i<AIM104_MAX_BOARDS ) {
        int err;
        err=add_aim104_device( AIM104_IN16_CHAR_MINOR+i, &aim104_in16_fops );
        if( err<0 ) {
            io_base[i]=0;
            aim104_in16_cleanup();
            return err;
        }

        if( !request_region( io_base[i], AIM104_IN16_IO_SIZE,
                             "AIM104-IN16" ) ) {
            printk( KERN_ERR
                    "aim104-in16: I/O ports 0x%x-0x%x are already in use.\n",
                    io_base[i], io_base[i]+AIM104_IN16_IO_SIZE-1 );
            remove_aim104_device( AIM104_IN16_CHAR_MINOR+i );
            io_base[i]=0;
            aim104_in16_cleanup();
            return -EBUSY;
        }

        atomic_set(&is_open[i], 0);

        i++;
    }

    return 0;
}

static __exit void aim104_in16_exit(void)
{
    aim104_in16_cleanup();
}

module_init(aim104_in16_init);
module_exit(aim104_in16_exit);

MODULE_AUTHOR("Arcom Control Systems Ltd.");
MODULE_LICENSE("GPL");

