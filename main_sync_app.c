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
#define ACC_ADDR         0x18    /* I2C Salve address of Acceleration sensor BMA222 */
#define ACC_1G           64      /* 1g value */


extern void *mqttMain();
void angleDeg(double accX, double accY, double accZ, double* angleX, double* angleY, double* angleZ);
void angleSatur(int8_t* accX, int8_t* accY, int8_t* accZ, int8_t AbsoluteMaxVal);

/*
 *  ======== mainThread ========
 */
void *mainThread(void *arg0)
{
    uint16_t        sample;
    int8_t          acc_x, acc_y, acc_z;
    double          angle_X_Deg, angle_Y_Deg, angle_Z_Deg;
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

    txBuffer[0] = ACC_REG_X;    //The register we want to read
    i2cTransaction.slaveAddress = ACC_ADDR; //The sensors I2C address
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
        while(1);
    }

    retc |= pthread_attr_setschedparam(&pAttrs, &priParam); //Assign schedule parameters
    retc |= pthread_attr_setstacksize(&pAttrs, 4096);       //Set the amount of stack memory to allocate
    if(retc != 0)
    {
        UART_PRINT("main_sync_app: MQTT main thread create fail (schedule param or stack size)\n\r");
        while(1);
    }

    //retc = pthread_create(&thread, &pAttrs, mqttMain, NULL);    //Create the thread with all previous parameters


    /*----------------------Read sensor----------------------*/
    /* Take 40 samples and print them out onto the console */
    for (;;/*sample = 0; sample < 40; sample++*/) {
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

        /* Saturation of acceleration */
        angleSatur(&acc_x, &acc_y, &acc_z, ACC_1G);

        /* Angles compute */
        angleDeg((double)acc_x, (double)acc_y, (double)acc_z, &angle_X_Deg, &angle_Y_Deg, &angle_Z_Deg);

        //Raw accelerations print
        //UART_PRINT("Sample %u: x=%d ; y=%d ; z=%d\n\r",
                        //sample, acc_x, acc_y, acc_z);
        //Angle print
        UART_PRINT("Sample %u: x angle=%.2f deg ; y angle=%.2f deg ; z angle=%.2f deg\n\r",
                                sample, angle_X_Deg, angle_Y_Deg, angle_Z_Deg);

        /* Sleep for 1 second */
        sleep(1);
    }

    /*----------------------Task end, clean up----------------------*/
    I2C_close(i2c);
    UART_PRINT("I2C closed!\n\r");

    return (NULL);
}

//Compute angles in degrees from accelerations
void angleDeg(double accX, double accY, double accZ, double* angleX, double* angleY, double* angleZ){
    *angleX=atan((double)accX/(sqrt(pow(accY,2)+pow(accZ,2))))*180/M_PI; //x angle
    *angleY=atan((double)accY/(sqrt(pow(accX,2)+pow(accZ,2))))*180/M_PI; //y angle
    *angleZ=atan((double)accZ/(sqrt(pow(accX,2)+pow(accY,2))))*180/M_PI; //z angle
}

//Saturate 3-axis accelerations while keeping the sign
void angleSatur(int8_t* accX, int8_t* accY, int8_t* accZ, int8_t AbsoluteMaxVal){
    if(*accX>AbsoluteMaxVal){
        *accX=AbsoluteMaxVal;
    }
    else if(*accX<-AbsoluteMaxVal){
        *accX=-AbsoluteMaxVal;
    }

    if(*accY>AbsoluteMaxVal){
        *accY=AbsoluteMaxVal;
    }
    else if(*accY<-AbsoluteMaxVal){
        *accY=-AbsoluteMaxVal;
    }

    if(*accZ>AbsoluteMaxVal){
        *accZ=AbsoluteMaxVal;
    }
    else if(*accZ<-AbsoluteMaxVal){
        *accZ=-AbsoluteMaxVal;
    }
}



