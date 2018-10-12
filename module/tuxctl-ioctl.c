/*
 * tuxctl-ioctl.c
 *
 * Driver (skeleton) for the mp2 tuxcontrollers for ECE391 at UIUC.
 *
 * Mark Murphy 2006
 * Andrew Ofisher 2007
 * Steve Lumetta 12-13 Sep 2009
 * Puskar Naha 2013
 */

#include <asm/current.h>
#include <asm/uaccess.h>

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/file.h>
#include <linux/miscdevice.h>
#include <linux/kdev_t.h>
#include <linux/tty.h>
#include <linux/spinlock.h>

#include "tuxctl-ld.h"
#include "tuxctl-ioctl.h"
#include "mtcp.h"

#define debug(str, ...) printk(KERN_DEBUG "%s: " str, __FUNCTION__, ## __VA_ARGS__)

unsigned char* button_packet[2];

/************************ Protocol Implementation *************************/

/* tuxctl_handle_packet()
 * IMPORTANT : Read the header for tuxctl_ldisc_data_callback() in
 * tuxctl-ld.c. It calls this function, so all warnings there apply
 * here as well.
 */
void tuxctl_handle_packet (struct tty_struct* tty, unsigned char* packet) {
    unsigned a, b, c;
    char opcode;

    a = packet[0]; /* Avoid printk() sign extending the 8-bit */
    b = packet[1]; /* values when printing them. */
    c = packet[2];

    switch (a) {
      case MTCP_ERROR:
        break;
      case MTCP_ACK:
        break;
      case MTCP_RESET:
        opcode = MTCP_LED_USR
        tuxctl_ldisc_put(tty, opcode, 1);
        opcode = MTCP_BIOC_ON
        tuxctl_ldisc_put(tty, opcode, 1);
        break;
      case MTCP_BIOC_EVT
        button_packet[0] = b;
        button_packet[1] = c;
        break;
    }




    /*printk("packet : %x %x %x\n", a, b, c); */
}

  void tux_init(struct tty_struct* tty){

      char *buf[2];
      buf[0] = MTCP_LED_USR;
      buf[1] = MTCP_BIOC_ON;
      tuxctl_ldisc_put(tty, buf, 2);

}

/******** IMPORTANT NOTE: READ THIS BEFORE IMPLEMENTING THE IOCTLS ************
 *                                                                            *
 * The ioctls should not spend any time waiting for responses to the commands *
 * they send to the controller. The data is sent over the serial line at      *
 * 9600 BAUD. At this rate, a byte takes approximately 1 millisecond to       *
 * transmit; this means that there will be about 9 milliseconds between       *
 * the time you request that the low-level serial driver send the             *
 * 6-byte SET_LEDS packet and the time the 3-byte ACK packet finishes         *
 * arriving. This is far too long a time for a system call to take. The       *
 * ioctls should return immediately with success if their parameters are      *
 * valid.                                                                     *
 *                                                                            *
 ******************************************************************************/

int tuxctl_ioctl(struct tty_struct* tty, struct file* file,
                 unsigned cmd, unsigned long arg) {
    switch (cmd) {
    /*Takes no arguments. Initializes any variables associated with the driver
       and returns 0. Assume that any user-level code that interacts with your
       device will call this ioctl before any others. */
        case TUX_INIT:
            if(arg == NULL) return -EINVAL;
            tux_init(tty);
            return 0;
    /*The argument is a 32-bit integer of the following form: The low 16-bits
      specify a number whose hexadecimal value is to be displayed on the 7-segment
      displays. The low 4 bits of the third byte specifies which LED’s should be
      turned on. The low 4 bits of the highest byte (bits 27:24) specify whether
      the corresponding decimal points should be turned on.
      This ioctl should return 0. */
        case TUX_BUTTONS:
            if(arg == NULL) return -EINVAL;

            break;
    /*Takes a pointer to a 32-bit integer. Returns -EINVAL error if this pointer
    is not valid. Otherwise, sets the bits of the low byte corresponding to the
    currently pressed buttons */
        case TUX_SET_LED:
            if(arg == NULL) return -EINVAL;

            break;
        default:
            return -EINVAL;
    }
}
