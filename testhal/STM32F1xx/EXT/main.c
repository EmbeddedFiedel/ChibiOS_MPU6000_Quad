/*
    ChibiOS/RT - Copyright (C) 2006,2007,2008,2009,2010,
                 2011,2012 Giovanni Di Sirio.

    This file is part of ChibiOS/RT.

    ChibiOS/RT is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 3 of the License, or
    (at your option) any later version.

    ChibiOS/RT is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

                                      ---

    A special exception to the GPL can be applied should you wish to distribute
    a combined work that includes ChibiOS/RT, without being obliged to provide
    the source code for any proprietary components. See the file exception.txt
    for full details of how and when the exception can be applied.
*/

#include "ch.h"
#include "hal.h"

static VirtualTimer vt;

/* LED set to OFF after 200mS.*/
static void ledoff(void *arg) {

  (void)arg;
  palSetPad(GPIOC, GPIOC_LED);
}

/* Triggered when the button is pressed or released. The LED is set to ON.*/
static void extcb1(EXTDriver *extp, expchannel_t channel) {

  (void)extp;
  (void)channel;
  palClearPad(GPIOC, GPIOC_LED);
  chSysLockFromIsr();
  if (!chVTIsArmedI(&vt))
    chVTSetI(&vt, MS2ST(200), ledoff, NULL);
  chSysUnlockFromIsr();
}

/* Triggered when the LED goes OFF.*/
static void extcb2(EXTDriver *extp, expchannel_t channel) {

  (void)extp;
  (void)channel;
}

static const EXTConfig extcfg = {
  {
   {EXT_CH_MODE_BOTH_EDGES | EXT_CH_MODE_AUTOSTART, extcb1},
   {EXT_CH_MODE_DISABLED, NULL},
   {EXT_CH_MODE_DISABLED, NULL},
   {EXT_CH_MODE_DISABLED, NULL},
   {EXT_CH_MODE_DISABLED, NULL},
   {EXT_CH_MODE_DISABLED, NULL},
   {EXT_CH_MODE_DISABLED, NULL},
   {EXT_CH_MODE_DISABLED, NULL},
   {EXT_CH_MODE_DISABLED, NULL},
   {EXT_CH_MODE_DISABLED, NULL},
   {EXT_CH_MODE_DISABLED, NULL},
   {EXT_CH_MODE_DISABLED, NULL},
   {EXT_CH_MODE_RISING_EDGE | EXT_CH_MODE_AUTOSTART, extcb2},
   {EXT_CH_MODE_DISABLED, NULL},
   {EXT_CH_MODE_DISABLED, NULL},
   {EXT_CH_MODE_DISABLED, NULL},
  },
  EXT_MODE_EXTI(EXT_MODE_GPIOA,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                EXT_MODE_GPIOC,
                0,
                0,
                0)
};

/*
 * Application entry point.
 */
int main(void) {

  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();
  chSysInit();

  /*
   * Activates the EXT driver 1.
   */
  extStart(&EXTD1, &extcfg);

  /*
   * Normal main() thread activity, in this demo it enables and disables the
   * button EXT channel using 5 seconds intervals.
   */
  while (TRUE) {
    chThdSleepMilliseconds(5000);
    extChannelDisable(&EXTD1, 0);
    chThdSleepMilliseconds(5000);
    extChannelEnable(&EXTD1, 0);
  }
}
