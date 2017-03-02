/* libaim104.h - libaim104 header file

   Copyright 2002 Arcom Control Systems Ltd

   Released under the terms of the GNU GPL v2.
*/
#ifndef ARCOM_LIBAIM104_H
#define ARCOM_LIBAIM104_H

#include <linux/arcom/aim104.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Error codes
 */
enum {
    AIM104_SUCCESS = 0,
    AIM104_EBAD_CHANNEL = -10,
    AIM104_EIO = -20,
    AIM104_ERANGE = -30,
};

/*
 * Functions to access the features of different boards.
 *
 * errno will be set to the error code of the last failed system call.
 */

/* AIM104-RELAY8/IN8 */
extern int aim104_relay8_enable_relays( int fd, int enable );
extern int aim104_relay8_set_all( int fd, unsigned char set );
extern int aim104_relay8_set_masked( int fd, unsigned char mask, unsigned char set );
extern int aim104_relay8_inputs( int fd );
extern int aim104_relay8_relay_status( int fd );

/* AIM104-IN16 */
extern int aim104_in16_inputs( int fd, int channel );

/* AIM104-OUT16 */
extern int aim104_out16_enable_outputs( int fd, int enable );
extern int aim104_out16_set_all( int fd, int channel, unsigned char set );
extern int aim104_out16_set_masked( int fd, int channel, unsigned char masked,
                                    unsigned char set );
extern int aim104_out16_output_status( int fd, int channel );

/* AIM104-IO32 */
extern int aim104_io32_enable_outputs( int fd, int enable );
extern int aim104_io32_set_all( int fd, int channel, unsigned char in_mask,
                                unsigned char set );
extern int aim104_io32_inputs( int fd, int channel, unsigned char in_mask );

/* AIM104-MULTI-IO */
extern int aim104_multi_io_inputs( int fd ); /* digital inputs */
extern int aim104_multi_io_ADC( int fd, int channel, int single_ended );
extern int aim104_multi_io_DAC( int fd, int channel, unsigned short output );

#ifdef __cplusplus
}
#endif

#endif /* #ifndef ARCOM_LIBAIM104_H */
