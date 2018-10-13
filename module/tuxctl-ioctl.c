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

unsigned char* button_packet[3];


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
          opcode = MTCP_LED_USR;
          tuxctl_ldisc_put(tty, &opcode, 1);
          opcode = MTCP_BIOC_ON;
          tuxctl_ldisc_put(tty, &opcode, 1);
          break;
      case MTCP_BIOC_EVT:
          button_packet[1] = b;
          button_packet[2] = c;
          break;
      default:
          return;
    }




    /*printk("packet : %x %x %x\n", a, b, c); */
}

  unsigned char hex_display(char hex, char dp_on){
      unsigned char led_byte;
      switch (hex){
          case 0x0:
              led_byte = 0xE7;
              break;
          case 0x1:
              led_byte = 0x06;
              break;
          case 0x2:
              led_byte = 0xCB;
              break;
          case 0x3:
              led_byte = 0x8F;
              break;
          case 0x4:
              led_byte = 0x2E;
              break;
          case 0x5:
              led_byte = 0xAD;
              break;
          case 0x6:
              led_byte = 0xED;
              break;
          case 0x7:
              led_byte = 0x86;
              break;
          case 0x8:
              led_byte = 0xEF;
              break;
          case 0x9:
              led_byte = 0xAF;
              break;
          case 0xA:
              led_byte = 0xEE;
              break;
          case 0xB:
              led_byte = 0x6D; // b:0x6D B:0xEF
              break;
          case 0xC:
              led_byte = 0xE1;
              break;
          case 0xD:
              led_byte = 0x4F; // d:0x4F D:0xE7
              break;
          case 0xE:
              led_byte = 0xE9;
              break;
          case 0xF:
              led_byte = 0xE8;
              break;
          default:
              return 0;
      }
      if(dp_on){
          led_byte += 0x10;
      }
      return led_byte;
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
            char *ini_buf[2];
            ini_buf[0] = MTCP_LED_USR;
            ini_buf[1] = MTCP_BIOC_ON;
            tuxctl_ldisc_put(tty, ini_buf, 2);
            return 0;
    /*Takes a pointer to a 32-bit integer. Returns -EINVAL error if this pointer
    is not valid. Otherwise, sets the bits of the low byte corresponding to the
    currently pressed buttons */
        case TUX_BUTTONS:
            if(arg == NULL) return -EINVAL;
            unsigned char* button_set[1];
            button_set[0] = ((button_packet[1] & 0x0F) | ((button_packet[2]<<4) & 0xF0));
            //button info ready to send
            return 0;
    /*The argument is a 32-bit integer of the following form: The low 16-bits
      specify a number whose hexadecimal value is to be displayed on the 7-segment
      displays. The low 4 bits of the third byte specifies which LED’s should be
      turned on. The low 4 bits of the highest byte (bits 27:24) specify whether
      the corresponding decimal points should be turned on.
      This ioctl should return 0. */
        case TUX_SET_LED:
            char led_on, led_mask, led_dp, hex_mask, cur_led, cur_dp, cur_hex;
            char *led_buf[6];
            unsigned long hex_arg = arg;
            int buf_idx;
            led_buf[0] = MTCP_LED_SET;
            led_on = (arg >> 16) & 0x0F;
            led_buf[1] = led_on;
            led_dp = (arg >> 24) & 0x0F;

            led_mask = 0x01;
            hex_mask = 0x000F;
            buf_idx = 2;
            for(int i = 0; i < 4; i++){
                cur_led = led_on & led_mask;
                if(cur_led == led_mask){
                    cur_dp = led_dp & led_mask;
                    cur_hex = hex_arg & hex_mask;
                    led_buf[buf_idx] = hex_display(cur_hex, cur_dp);
                    buf_idx += 1;
                }
                led_mask <<= 1;
                hex_arg >>= 4;
            }

            tuxctl_ldisc_put(tty, led_buf, buf_idx);
            return 0;

        default:
            return -EINVAL;
    }
}
