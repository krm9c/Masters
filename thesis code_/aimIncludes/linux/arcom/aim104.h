/* aim104.h - AIM104 kernel driver

   Copyright 2002 Arcom Control Systems Ltd.

   Released under the terms of the GNU GPL v2.
*/

#ifndef LINUX_ARCOM_AIM104_H
#define LINUX_ARCOM_AIM104_H

#ifdef __KERNEL__

#include <linux/ioctl.h>
#include <linux/list.h>

#endif /* #ifdef __KERNEL__ */

/* Maximum number of each type of board. */
#define AIM104_MAX_BOARDS 8

/*
 * Each type of board is allocated a consecutive set of minor numbers
 * sufficient for the allowed number of boards.
 *
 * AIM104-RELAY8    minor 0-7
 * AIM104-IN16      minor 8-15
 * AIM104-OUT16     minor 16-23
 * AIM104-IO32      minor 24-31
 * AIM104-MULTI-IO  minor 32-39
 * etc.
 *
 * Note: A major number reserved for local/experimental use is currently being
 * used.
 */
#define AIM104_CHAR_MAJOR 240
#define AIM104_R8_CHAR_MINOR 0
#define AIM104_IN16_CHAR_MINOR (AIM104_R8_CHAR_MINOR+AIM104_MAX_BOARDS)
#define AIM104_OUT16_CHAR_MINOR (AIM104_IN16_CHAR_MINOR+AIM104_MAX_BOARDS)
#define AIM104_IO32_CHAR_MINOR (AIM104_OUT16_CHAR_MINOR+AIM104_MAX_BOARDS)
#define AIM104_MIO_CHAR_MINOR (AIM104_IO32_CHAR_MINOR+AIM104_MAX_BOARDS)

/*
 * Ioctls for the AIM104-RELAY8 driver
 *
 * AIM104_R8_IOC_ENABLE - Enable all 8 relays on the board
 * AIM104_R8_IOC_STATUS - Return relay status
 */
#define AIM104_R8_IOC_MAGIC 'r'

#define AIM104_R8_IOC_ENABLE _IOW(AIM104_R8_IOC_MAGIC, 1, int)
#define AIM104_R8_IOC_STATUS _IOR(AIM104_R8_IOC_MAGIC, 3, int)

/*
 * Ioctls for the AIM104-OUT16 driver
 *
 * AIM104_OUT16_IOC_ENABLE - Enable all 16 outputs
 * AIM104_OUT16_IOC_STATUS - Return output status
 */
#define AIM104_OUT16_IOC_MAGIC 'o'

#define AIM104_OUT16_IOC_ENABLE _IOW(AIM104_OUT16_IOC_MAGIC, 1, int)
#define AIM104_OUT16_IOC_STATUS _IOR(AIM104_OUT16_IOC_MAGIC, 2, int)

/*
 * Ioctls for the AIM104-IO32 driver
 *
 * AIM104_IO32_IOC_ENABLE - Enable all 32 outputs
 */
#define AIM104_IO32_IOC_MAGIC 'i'

#define AIM104_IO32_IOC_ENABLE _IOW(AIM104_IO32_IOC_MAGIC, 1, int)

/*
 * Ioctls for the AIM104-MULTI-IO driver
 *
 * AIM104_MIO_IOC_CHANNEL - Select an I/O channel to read/write from/to.
 *     The channel(s) to select are specified using a 16 bit word. The
 *     low byte specifies the input (digital, ADC0-ADC15 or
 *     ADC0D-ADC7D).  The high byte specified the output (DAC0-DAC1).
 *     Leaving the high or low byte as zero mean the selected channel
 *     (output or input respectively) is not changed.
 */
#define AIM104_MIO_IOC_MAGIC 'm'

#define AIM104_MIO_IOC_CHANNEL _IOW(AIM104_MIO_IOC_MAGIC, 1, int)

/*
 * AIM104-MULTI-IO channels (for use with the AIM104_MIO_IOC_CHANNEL ioctl)
 *
 * Or (|) the input channel and output channel specfier together (to select
 * both at once) or just use an input or output (to select them seperately).
 *
 * The odd values for the ADC are the values to be written to the board to
 * select the appropriate channel.
 */
/* digital inputs */
#define AIM104_MIO_DIGITAL 0x01
/* ADC single ended mode inputs */
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
/* ADC differential mode inputs */
#define AIM104_MIO_ADC0D 0x88
#define AIM104_MIO_ADC1D 0x99
#define AIM104_MIO_ADC2D 0xaa
#define AIM104_MIO_ADC3D 0xbb
#define AIM104_MIO_ADC4D 0xcc
#define AIM104_MIO_ADC5D 0xdd
#define AIM104_MIO_ADC6D 0xee
#define AIM104_MIO_ADC7D 0xff
/* DAC outputs */
#define AIM104_MIO_DAC0 0x0100
#define AIM104_MIO_DAC1 0x0200

#ifdef __KERNEL__

/*
 * Individual boards register with the AIM104 driver, giving their minor number
 * and their own file operations.
 *
 * Registered devices are stored in a singly linked list.
 *
 * Boards should unregister when the board driver module is unloaded.
 */
struct aim104_dev_t {
    struct list_head        list;
    int                     minor_num;
    struct file_operations *fops;
};

extern int add_aim104_device( int minor,
                              struct file_operations *fops );
extern int remove_aim104_device( int minor );

/* Default ports and other settings for different types of boards. */
#define AIM104_R8_DEF_PORT 0x180
#define AIM104_R8_IO_SIZE 2 /* byte */

#define AIM104_IN16_DEF_PORT 0x180
#define AIM104_IN16_IO_SIZE 2 /* byte */

#define AIM104_OUT16_DEF_PORT 0x180
#define AIM104_OUT16_IO_SIZE 4 /* byte */

#define AIM104_IO32_DEF_PORT 0x180
#define AIM104_IO32_IO_SIZE 8 /* byte */

#define AIM104_MIO_DEF_PORT 0x180
#define AIM104_MIO_IO_SIZE 4 /* byte */

#endif /* #ifdef __KERNEL__ */

#endif /* #ifndef LINUX_AIM104_H */
