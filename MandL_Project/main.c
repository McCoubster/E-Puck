#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <main.h>

#include "chprintf.h"
#include "usbcfg.h"


// LED
#include "leds.h"
#include "spi_comm.h"

//Motor
#include "motors.h"

//Selector
#include "selector.h"

//Sensor
#include "sensors/proximity.h"

messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

int main(void)
{

    halInit();
    chSysInit();
    mpu_init();

    messagebus_init(&bus, &bus_lock, &bus_condvar);

    //Initiating LED
    clear_leds();
    spi_comm_start();

    //Initiating Motor
    motors_init();

    //Initiating selector
    int get_selector();

    //Initiating Sensors
    proximity_start();
    calibrate_ir();

    //Initiating sensor comunication
    usb_start();


    /* Infinite loop. */
    while (1) {


    		//Null
    		if(get_selector() == 0){
    			right_motor_set_speed(0);
    			left_motor_set_speed(0);

    		// Left Right and LEd
    		} else if (get_selector() == 1){
				set_led(LED7, 1);
				set_led(LED3, 0);
				right_motor_set_speed(200);
				left_motor_set_speed(-200);
				chThdSleepMilliseconds(1000);

				set_led(LED7, 0);
				set_led(LED3, 1);
				left_motor_set_speed(200);
				right_motor_set_speed(-200);
				chThdSleepMilliseconds(1000);

			// Sensor
    		} else if (get_selector() == 2){
    			int prox_values[7];
    			for(int i = 0; i<=7; i += 1){
    				prox_values[i] = get_calibrated_prox(i);
    				if (SDU1.config->usbp->state == USB_ACTIVE) {
    				chprintf((BaseSequentialStream *)&SDU1, "%4d,", prox_values[i]);
    				}
    			}
    		}



    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}