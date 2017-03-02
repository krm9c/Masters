/* aim104.c - kernel driver wrapper for all the AIM104 boards

   Copyright (C) 2004 Arcom Control Systems Ltd

   Released under the terms of the GNU GPL v2.

   $Id: aim104.c 1403 2004-11-23 18:14:29Z dvrabel $
*/

#include <linux/config.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/major.h>
#include <linux/fs.h>
#include <linux/ioctl.h>
#include <linux/errno.h>
#include <asm/semaphore.h>
#include <linux/init.h>

#include "linux/arcom/aim104.h"

#define VERSION "$Revision: 1403 $"

static LIST_HEAD(aim104_dev_list);

/* Mutex to protect access to device list. */
static DECLARE_MUTEX(aim104_dev_list_sem);

static struct aim104_dev_t *aim104_find_dev(int minor)
{
    struct list_head *p;

    list_for_each (p, &aim104_dev_list) {
        struct aim104_dev_t *d = list_entry(p, struct aim104_dev_t, list);
        if (d->minor_num == minor)
            return d;
    }
    return NULL;
}

extern int add_aim104_device( int minor, struct file_operations *fops )
{
    struct aim104_dev_t *new;

    /* allocate linked list node. */
    new = kmalloc(sizeof(struct aim104_dev_t), GFP_KERNEL);
    if (new == NULL)
        return -ENOMEM;

    /* fill it */
    new->minor_num = minor;
    new->fops = fops;
    INIT_LIST_HEAD(&new->list);

    down(&aim104_dev_list_sem);

    /* stick it on the head */
    list_add(&new->list, &aim104_dev_list);

    up(&aim104_dev_list_sem);

    return 0;
}

extern int remove_aim104_device( int minor )
{
    struct aim104_dev_t *d;

    down(&aim104_dev_list_sem);

    d = aim104_find_dev(minor);
    if (d) {
        list_del(&d->list);
        kfree(d);
    }

    up(&aim104_dev_list_sem);

    return 0;
}

EXPORT_SYMBOL(add_aim104_device);
EXPORT_SYMBOL(remove_aim104_device);

static int aim104_open(struct inode *inode, struct file *file)
{
    struct aim104_dev_t *d;
    int ret = -ENODEV;
    struct file_operations *old_fops;

    down(&aim104_dev_list_sem);

    /* find board and get correct fops */
    d = aim104_find_dev(MINOR(inode->i_rdev));
    if (d) {
        old_fops = file->f_op;
        file->f_op = fops_get(d->fops);
        if (file->f_op->open) {
            ret = file->f_op->open(inode, file);
            if (ret) {
                fops_put(file->f_op);
                file->f_op = fops_get(old_fops);
            }
        }
        fops_put(old_fops);
    }

    up(&aim104_dev_list_sem);

    return ret;
}


static struct file_operations aim104_fops = {
    .owner = THIS_MODULE,
    .open  = aim104_open,
};

static __init int aim104_init(void)
{
    int err = 0;

    printk( KERN_INFO "AIM104 driver " VERSION "\n" );

    err = register_chrdev( AIM104_CHAR_MAJOR, "AIM104", &aim104_fops );
    if (err < 0)
        printk(KERN_ERR "aim104: unable to obtain major number (%d)\n", AIM104_CHAR_MAJOR);

    return err;
}

static __exit void aim104_exit(void)
{
    int err;

    err = unregister_chrdev( AIM104_CHAR_MAJOR, "AIM104" );
    if (err < 0)
        printk(KERN_ERR "aim104: error unregistering major number (%d)\n", AIM104_CHAR_MAJOR);

    /* FIXME: other clean up? */
}

module_init(aim104_init);
module_exit(aim104_exit);

MODULE_AUTHOR("Arcom Control Systems Ltd.");
MODULE_LICENSE("GPL");
