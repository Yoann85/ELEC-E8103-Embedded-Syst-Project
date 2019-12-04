/*
 * Copyright (c) 2018-2019, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ======== i2ctmp116.c ========
 */
#include <stdint.h>
#include <stddef.h>
#include <unistd.h>
#include <pthread.h>
#include <math.h>

/* Driver Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/I2C.h>
#include <ti/display/Display.h>
#include <ti/drivers/SPI.h>

/* Driver configuration */
#include "ti_drivers_config.h"
#include "uart_term.h"

#define TASKSTACKSIZE       640

/*
 *  ======== Accelerometer BMA222 Registers ========
 */
#define ACC_REG_X        0x0003  /* Acceleration on x-axis */
#define ACC_REG_Y        0x0005  /* Acceleration on y-axis */
#define ACC_REG_Z        0x0007  /* Acceleration on z-axis */
#define ACC_ADDR         0x18;   /* I2C Salve address of Acceleration sensor BMA222 */
#define ACC_1G           64;     /* 1g value */


extern void *mqttMain();

/*
 *  ======== mainThread ========
 */
void *mainThread(void *arg0)
{
    uint16_t        sample;
    int8_t          acc_x, acc_y, acc_z;
    double          acc_z_scaled, angleRad, angleDeg;
    uint8_t         txBuffer[1];
    uint8_t         rxBuffer[1];
    I2C_Handle      i2c;
    I2C_Params      i2cParams;
    I2C_Transaction i2cTransaction;
    UART_Handle tUartHndl;
    pthread_t thread;
    pthread_attr_t pAttrs;
    struct sched_param priParam;
    int retc;
    int detachState;

    /*----------------------Call driver init functions----------------------*/
    GPIO_init();
    SPI_init();
    I2C_init();

    /*----------------------Configure the UART----------------------*/
    tUartHndl = InitTerm();
    /*remove uart receive from LPDS dependency                               */
    UART_control(tUartHndl, UART_CMD_RXDISABLE, NULL);

    UART_PRINT("Starting main_sync_app\n\r");

    /*----------------------I2C and sensor initialization----------------------*/
    /* Create I2C for usage */
    I2C_Params_init(&i2cParams);
    i2cParams.bitRate = I2C_400kHz;
    i2c = I2C_open(0, &i2cParams);
    if (i2c == NULL) {
        UART_PRINT("Error Initializing I2C\n\r");
        while (1);
    }
    UART_PRINT("I2C Initialized!\n\r");

    /*I2C transaction setup */
    i2cTransaction.writeBuf   = txBuffer;
    i2cTransaction.writeCount = 1;  //1 axis read each time
    i2cTransaction.readBuf    = rxBuffer;
    i2cTransaction.readCount  = 1;  //1 axis' data is over 1 byte

    /*Test to get x-axis data*/
    txBuffer[0] = ACC_REG_X;
    i2cTransaction.slaveAddress = ACC_ADDR;
    if (!I2C_transfer(i2c, &i2cTransaction)) {
        /* Could not resolve a sensor, error */
        UART_PRINT("Error when testing sensor.\n\r");
        while(1);
    }

    UART_PRINT("Detected sensor.\n\r");


    /*----------------------Mqtt main thread creating----------------------*/
    pthread_attr_init(&pAttrs);
    priParam.sched_priority = 1;    //Set thread priority
    detachState = PTHREAD_CREATE_DETACHED;
    retc = pthread_attr_setdetachstate(&pAttrs, detachState);
    if(retc != 0)
    {
        UART_PRINT("main_sync_app: MQTT main thread create fail (detach state)\n\r");
        return(NULL);
    }

    retc |= pthread_attr_setschedparam(&pAttrs, &priParam); //Assign schedule parameters
    retc |= pthread_attr_setstacksize(&pAttrs, 4096);       //Set the amount of stack memory to allocate
    if(retc != 0)
    {
        UART_PRINT("main_sync_app: MQTT main thread create fail (schedule param or stack size)\n\r");
        return(NULL);
    }

    retc = pthread_create(&thread, &pAttrs, mqttMain, NULL);    //Create the thread with all previous parameters


    /*----------------------Read sensor----------------------*/
    /* Take 40 samples and print them out onto the console */
    for (sample = 0; sample < 40; sample++) {
        /* x axis reading */
        txBuffer[0] = ACC_REG_X;
        if (I2C_transfer(i2c, &i2cTransaction)) {
            acc_x=rxBuffer[0];
        }
        else {
            UART_PRINT("I2C Bus fault.\n\r");
        }

        /* y axis reading */
        txBuffer[0] = ACC_REG_Y;
        if (I2C_transfer(i2c, &i2cTransaction)) {
            acc_y=rxBuffer[0];
        }
        else {
            UART_PRINT("I2C Bus fault.\n\r");
        }

        /* z axis reading */
        txBuffer[0] = ACC_REG_Z;
        if (I2C_transfer(i2c, &i2cTransaction)) {
            acc_z=rxBuffer[0];
        }
        else {
            UART_PRINT("I2C Bus fault.\n\r");
        }

        /* Theta angle compute */
        acc_z_scaled=(double)acc_z/ACC_1G;  //scaling according to 1g's value
        angleRad=acos(acc_z_scaled);        //compute angle in radians
        if(acc_y<0) angleRad=-angleRad;     //Compute direction according to y-axis orientation
        angleDeg=angleRad*180/M_PI;         //compute angle in degrees

        UART_PRINT("Sample %u: z=%d ; scaled=%.2f g ; angle=%.2f rad ; angle=%.2f deg\n\r",
                        sample, acc_z, acc_z_scaled, angleRad, angleDeg);
        //UART_PRINT("Sample %u: x=%d ; y=%d ; z=%d ; angle=%.2f deg",
        //                sample, acc_x, acc_y, acc_z, angle);

        /* Sleep for 1 second */
        sleep(1);
    }

    /*----------------------Task end, clean up----------------------*/
    I2C_close(i2c);
    UART_PRINT("I2C closed!\n\r");

    return (NULL);
}



