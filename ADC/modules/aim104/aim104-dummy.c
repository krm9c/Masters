/* aim104-dummy.c - Dummy kernel module driver for an AIM104 board.
 
   Copyright (C) 2004 Arcom Control Systems Ltd.

   Licensed under the terms of the GNU GPL v2.

   This dummy driver implements a driver that looks like a generic AIM104
   board.  All writes are no-ops and reads all return zeros.  This means
   application code can be tested on development machines without any 
   AIM104 hardware.

   When this module is loaded the 'minor_num' module parameter can be
   specified to make this module substitute any one of the real drivers.

   $Id: aim104-dummy.c 1403 2004-11-23 18:14:29Z dvrabel $
*/

#include <linux/types.h>
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/ioport.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <asm/atomic.h>
#include <linux/init.h>

#include "linux/arcom/aim104.h"

#define VERSION "$Revision: 1403 $"

static int minor_num[AIM104_MAX_BOARDS] = { -1, -1, -1, -1, -1, -1, -1, -1 };
MODULE_PARM(minor_num, "1-" __MODULE_STRING(AIM104_MAX_BOARDS) "i");

static atomic_t is_open[AIM104_MAX_BOARDS];

/*
 * Fake a read from a board. 
 */
static ssize_t aim104_dummy_read( struct file *filp,
                                     char *buf, size_t len, loff_t *ppos )
{
    return len;
}

/*
 * Fake a write to a board.
 */
static ssize_t aim104_dummy_write( struct file *filp, const char *buf,
                                      size_t len, loff_t *ppos )
{
    return len;
}

/*
 * Fake an ioctl.
 *
 * All ioctls are no ops which are always successful.  Permission checks are
 * still performed.
 */
static int aim104_dummy_ioctl( struct inode *inode,struct file *filp,
                               unsigned int cmd, unsigned long arg )
{
	/* check for read/write permissions */
	if( (_IOC_DIR(cmd) & _IOC_READ) && !(filp->f_mode & FMODE_READ) ) {
		return -EACCES;
	}
	if( (_IOC_DIR(cmd) & _IOC_WRITE) && !(filp->f_mode & FMODE_WRITE) ) {
		return -EACCES;
	}

    return 0;
}

static int aim104_dummy_open( struct inode *inode, struct file *filp )
{
    int i;
    int board_num = -1;

    /* Find which board number we're faking. */
    i = 0;
    while( minor_num[i] >= 0 && i < AIM104_MAX_BOARDS ) {
        if( minor_num[i] == MINOR(inode->i_rdev) ) {
            board_num = i;
            break;
        }
        i++;
    }
    
    /* The open function in aim104.c should check the (fake) board exists but
     * just in case... */
    if( board_num == -1 ) {
        printk(KERN_ERR "AIM104 dummy: unexpectedly failed to find device: %d\n",
               MINOR(inode->i_rdev));
        return -ENODEV;
    }

    /* Don't permit multiple opens (as in all the real board drivers). */
    atomic_inc(&is_open[board_num]);
    if (atomic_read(&is_open[board_num]) > 1) {
        atomic_dec(&is_open[board_num]);
        return -EBUSY;
    }

    filp->private_data = (void*)board_num;

    return 0;
}

static int aim104_dummy_release( struct inode *inode, struct file *filp )
{
    int board_num = (int)filp->private_data;

    atomic_dec(&is_open[board_num]);

    return 0;
}


static struct file_operations aim104_dummy_fops={
    .owner   = THIS_MODULE,
    .read    = aim104_dummy_read,
    .write   = aim104_dummy_write,
    .ioctl   = aim104_dummy_ioctl,
    .open    = aim104_dummy_open,
    .release = aim104_dummy_release,
};

static void aim104_dummy_cleanup(void)
{
    int i;

    i = 0;
    while( minor_num[i] >= 0 && i < AIM104_MAX_BOARDS ) {
        remove_aim104_device( minor_num[i] );
        i++;
    }
}

static __init int aim104_dummy_init(void)
{
    int i;

    printk( KERN_INFO "AIM104 dummy driver "VERSION"\n" );

    /* If there were no parameters, fake a single board with a minor number
       of 0. */
    if( minor_num[0] == -1 ) {
        minor_num[0] = 0;
    }

    i=0;
    while( minor_num[i] >= 0 && i<AIM104_MAX_BOARDS ) {
        int err;
        err=add_aim104_device( minor_num[i], &aim104_dummy_fops );
        if( err < 0 ) {
            minor_num[i] = -1;
            aim104_dummy_cleanup();
            return err;
        }

        atomic_set(&is_open[i], 0);

        i++;
    }
    
    return 0;
}

static __exit void aim104_dummy_exit(void)
{
    aim104_dummy_cleanup();
}

module_init(aim104_dummy_init);
module_exit(aim104_dummy_exit);

MODULE_AUTHOR("Arcom Control Systems Ltd.");
MODULE_LICENSE("GPL");
