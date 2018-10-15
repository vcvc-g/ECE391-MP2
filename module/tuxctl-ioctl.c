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

static uint8_t button_packet[3];
static unsigned long led_state;
static unsigned long hold = 0;

static void mtcp_reset(struct tty_struct* tty);
static void tux_init(struct tty_struct* tty);
static void set_led(struct tty_struct* tty, unsigned long arg);
static int tux_buttons(struct tty_struct* tty, unsigned long arg);
static uint8_t hex_display(char hex, char dp_on);

static spinlock_t button_lock = SPIN_LOCK_UNLOCKED;

/************************ Protocol Implementation *************************/

/* tuxctl_handle_packet()
 * IMPORTANT : Read the header for tuxctl_ldisc_data_callback() in
 * tuxctl-ld.c. It calls this function, so all warnings there apply
 * here as well.
 */
void tuxctl_handle_packet (struct tty_struct* tty, unsigned char* packet) {
    unsigned a, b, c;


    a = packet[0]; /* Avoid printk() sign extending the 8-bit */
    b = packet[1]; /* values when printing them. */
    c = packet[2];

    switch (a) {
      case MTCP_ERROR:
          return;
      case MTCP_ACK:
          hold = 0;
          return;
      case MTCP_RESET:
          mtcp_reset(tty);
          return;
      case MTCP_BIOC_EVENT:
          button_packet[1] = b;
          button_packet[2] = c;
          return;
      default:
          return;
    }
    /*printk("packet : %x %x %x\n", a, b, c); */
}

/*
 * mtcp_reset
 *   DESCRIPTION: it will reset the tux and restore last led state
 *   INPUTS: struct tty_struct* tty
 *   OUTPUTS: none
 *   RETURN VALUE: none
 */
void mtcp_reset(struct tty_struct* tty){
    uint8_t opcode[2];        //opcode array for 2 opcode
    opcode[0] = MTCP_LED_USR; //fill in 1st opcode 0
    opcode[1] = MTCP_BIOC_ON; //fill in 2nd opcode 1
    tuxctl_ldisc_put(tty, opcode, 2); //put 2 opcode on tux

    set_led(tty, led_state);   //set led last state
    return;
}

/*
 * mtcp_reset
 *   DESCRIPTION: it will translate the input hex value to LED display
 *   base on the given hex and decimal point indicator
 *   INPUTS: char hex, char dp_on
 *   OUTPUTS: none
 *   RETURN VALUE: HEX display on 7 segment LED
 */
  uint8_t hex_display(char hex, char dp_on){
      uint8_t led_byte;
      switch (hex){
          case 0x0: //HEX 0X0
              led_byte = 0xE7; //7segment display
              break;
          case 0x1: //HEX 0X1
              led_byte = 0x06; //7segment display
              break;
          case 0x2: //HEX 0X2
              led_byte = 0xCB; //7segment display
              break;
          case 0x3: //HEX 0X3
              led_byte = 0x8F; //7segment display
              break;
          case 0x4: //HEX 0X4
              led_byte = 0x2E; //7segment display
              break;
          case 0x5: //HEX 0X5
              led_byte = 0xAD; //7segment display
              break;
          case 0x6: //HEX 0X6
              led_byte = 0xED; //7segment display
              break;
          case 0x7: //HEX 0X7
              led_byte = 0x86; //7segment display
              break;
          case 0x8: //HEX 0X8
              led_byte = 0xEF; //7segment display
              break;
          case 0x9: //HEX 0X9
              led_byte = 0xAF; //7segment display
              break;
          case 0xA: //HEX 0XA
              led_byte = 0xEE; //7segment display
              break;
          case 0xB: //HEX 0XB
              led_byte = 0x6D; // b:0x6D B:0xEF
              break;
          case 0xC: //HEX 0XC
              led_byte = 0xE1; //7segment display
              break;
          case 0xD: //HEX 0XD
              led_byte = 0x4F; // d:0x4F D:0xE7
              break;
          case 0xE: //HEX 0XE
              led_byte = 0xE9; //7segment display
              break;
          case 0xF: //HEX 0XF
              led_byte = 0xE8; //7segment display
              break;
          default:  //default case
              return 0;
      }
      if(dp_on){
          led_byte += 0x10; // light up dp by adding 0x10 to hex display
      }
      return led_byte;
}


/*
 * set_led
 *   DESCRIPTION: it will set LED base on given arg
 *   INPUTS: struct tty_struct* tty, unsigned long arg
 *   OUTPUTS: none
 *   RETURN VALUE: none
 */
void set_led(struct tty_struct* tty, unsigned long arg){
    char led_on, led_mask, led_dp, cur_led, cur_dp, cur_hex;
    uint8_t led_buf[6];   //tux commands sending array, 6 commands at most
    unsigned long hex_arg, hex_mask;
    int i, buf_idx;

    hex_arg = arg;

    led_buf[0] = MTCP_LED_SET;   //LED_SET opcode at command array index 0
    led_on = (arg >> 16) & 0x0F; //right shift arg 16 bits then bit operation with mask get which led is on
    led_buf[1] = led_on;         //LED_on/off at command array index 1
    led_dp = (arg >> 24) & 0x0F; //right shift arg 24 bits then bit operation with mask get which decimal point on

    led_mask = 0x01;    //mask for last 1 bit
    hex_mask = 0x000F;  //mask for last 4 bit
    buf_idx = 2;        //command array offset by 2
    for(i = 0; i < 4; i++){ //loop for checking 4 led on/ff
        cur_led = led_on & led_mask;
        if(cur_led == led_mask){  // if led on
            cur_dp = led_dp & led_mask;
            cur_hex =(char)(hex_arg & hex_mask);
            led_buf[buf_idx] = hex_display(cur_hex, cur_dp);
            buf_idx += 1; // add this on led info to command array and CA length +1
        }
        led_mask <<= 1;   // left shift 1 bit check next led on off
        hex_arg >>= 4;    // right shift 4 bit ready for next hex info
    }
    if(hold ==1){
        return 0;
    }
    else{
      hold = 1;
      led_state = arg;
      tuxctl_ldisc_put(tty, led_buf, buf_idx);
    }

  }


  /*
   * tux_init
   *   DESCRIPTION: it Initializes the tux, set led to 0
   *   INPUTS: struct tty_struct* tty
   *   OUTPUTS: none
   *   RETURN VALUE: none
   */
void tux_init(struct tty_struct* tty){
      uint8_t ini_buf[2]; //command array for 2 opcode
      hold = 1;
      ini_buf[0] = MTCP_LED_USR; //MTCP_LED_USR opcode at CA idx 0
      ini_buf[1] = MTCP_BIOC_ON; //MTCP_BIOC_ON opcode at CA idx 1
      tuxctl_ldisc_put(tty, ini_buf, 2); //put CA with size 2
      led_state = 0xFFFF0000;  // led stae for 0000
      set_led(tty, led_state);
      return;
}


/*
 * tux_buttons
 *   DESCRIPTION: it pack up the lastest button info and copy to user
 *   INPUTS: struct tty_struct* tty, unsigned long arg
 *   OUTPUTS: none
 *   RETURN VALUE: 0 if success, -EINVAL if failed
 */
int tux_buttons(struct tty_struct* tty, unsigned long arg){
      uint8_t button_set[1];  //button packet of size 1
      unsigned long *ptr;

      ptr = (unsigned long *)arg;
      if(ptr == NULL){
          return -EINVAL;
      }
      spin_lock_irq(&button_lock);
      //button packt 1 mask last 4 bit(0x0F) and button packt 2 left shift 4 bit then mask first 4 bit(0xF0)
      //button set :  right down left up C B A S
      button_set[0] = ((button_packet[1] & 0x0F) | ((button_packet[2]<<4) & 0xF0));
      //button info ready to send
      copy_to_user(ptr, button_set, 1);
      spin_unlock_irq(&button_lock);
      return 0;

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
            tux_init(tty);
            return 0;
    /*Takes a pointer to a 32-bit integer. Returns -EINVAL error if this pointer
    is not valid. Otherwise, sets the bits of the low byte corresponding to the
    currently pressed buttons */
        case TUX_BUTTONS:
            return tux_buttons(tty, arg);
    /*The argument is a 32-bit integer of the following form: The low 16-bits
      specify a number whose hexadecimal value is to be displayed on the 7-segment
      displays. The low 4 bits of the third byte specifies which LED’s should be
      turned on. The low 4 bits of the highest byte (bits 27:24) specify whether
      the corresponding decimal points should be turned on.
      This ioctl should return 0. */
        case TUX_SET_LED:
            set_led(tty, arg);
            return 0;

        default:
            return -EINVAL;
    }
}
