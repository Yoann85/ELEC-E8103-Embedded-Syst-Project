#include <stdint.h>
#include <stddef.h>
#include <unistd.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>

/* Driver Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/I2C.h>
#include <ti/display/Display.h>
#include <ti/drivers/SPI.h>
#include <string.h>



/* Driver configuration */
#include "ti_drivers_config.h"
#include "uart_term.h"
#include "mailboxConf.h"
#include "main_sync_app.h"
#include "mqtt_client.h"
/*
 *  ======== Accelerometer BMA222 Registers ========
 */
#define ACC_REG_X        0x0003  /* Acceleration on x-axis */
#define ACC_REG_Y        0x0005  /* Acceleration on y-axis */
#define ACC_REG_Z        0x0007  /* Acceleration on z-axis */
#define ACC_ADDR         0x18    /* I2C Salve address of Acceleration sensor BMA222 */
#define ACC_1G           64      /* 1g value */


extern void *mqttMain();
MailboxMsgObj mailboxBuffer[NUMMSGS];
Mailbox_Handle mbxHandle;
Mailbox_Params mbxParams;

/*
 *  ======== mainThread ========
 */
#define REFBOARD false
void *mainThread(void *arg0)
{
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
    int sampling_count = 0;

    UART_PRINT("Starting main_sync_app\n\r");

    /*----------------------I2C and sensor initialization----------------------*/
    /* Create I2C for usage */
    I2C_Params_init(&i2cParams);
    i2cParams.bitRate = I2C_400kHz;
    i2c = I2C_open(0, &i2cParams);
    while (i2c == NULL) {
        UART_PRINT("Error Initializing I2C\n\r");
        //while (1);
        sleep(2);
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
    while (!I2C_transfer(i2c, &i2cTransaction)) {

        /* Could not resolve a sensor, error */
        UART_PRINT("Error when testing sensor.\n\r");
        //while(1);
        sleep(1);
    }
    UART_PRINT("Detected sensor.\n\r");

    double averageArray[50*3] = {NULL};

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
    retc = pthread_create(&thread, &pAttrs, mqttMain, NULL);    //Create the thread with all previous parameters
    const char *publish_topic = { PUBLISH_TOPIC1 };


    /*----------------------Mailbox configuration----------------------*/
    Mailbox_Params_init(&mbxParams);
    mbxParams.buf = (Ptr)mailboxBuffer;
    mbxParams.bufSize = sizeof(mailboxBuffer);
    Mailbox_construct(&mbxStruct, sizeof(MsgObj), NUMMSGS, &mbxParams, NULL);
    mbxHandle = Mailbox_handle(&mbxStruct);
    bool justBooted = true;

    // Main logic loops for both boards
#if REFBOARD == false // If we build for reference board
    while(1)
    {
        /*----------------------Read sensor----------------------*/
        /* Take 40 samples and print them out onto the console */
        double * rotation;
        rotation = getCurrentAngles(txBuffer, i2c, i2cTransaction, rxBuffer);
        //UART_PRINT("ROT: %lf, %lf, %lf \n\r", rotation[0], rotation[1], rotation[2]);
        // payload = message from client thread with reference rotation (received from mailbox)
        while(gMqttClient == NULL)
        {
            sleep(3);
        }
        /*char publish_data[30];
        sprintf(publish_data, "%.2lf,%.2lf,%.2lf", rotation[0], rotation[1], rotation[2]);
        MQTTClient_publish(gMqttClient, (char*) publish_topic, strlen(
                                              (char*)publish_topic),
                                          (char*)publish_data,
                                          strlen((char*) publish_data), MQTT_QOS_2 |
                                          ((RETAIN_ENABLE) ? MQTT_PUBLISH_RETAIN : 0));

                    UART_PRINT("\n\r CC3200 Publishes the following message \n\r");
                    UART_PRINT("Topic: %s\n\r", publish_topic);
                    UART_PRINT("Data: %s\n\r", publish_data);
*/
        if(sampling_count > 147)
        {
            double * angleAverage = calculateAngleAvg(averageArray, 50);
            UART_PRINT("%.2lf,%.2lf,%.2lf,%.2lf \n\r", angleAverage[0],angleAverage[1],angleAverage[2], angleAverage[3]);
            char publish_data[30];
            sprintf(publish_data, "%.2lf,%.2lf,%.2lf,%.2lf", angleAverage[0],angleAverage[1],angleAverage[2], angleAverage[3]);
            MQTTClient_publish(gMqttClient, (char*) publish_topic, strlen(
                                                          (char*)publish_topic),
                                                      (char*)publish_data,
                                                      strlen((char*) publish_data), MQTT_QOS_2 |
                                                      ((RETAIN_ENABLE) ? MQTT_PUBLISH_RETAIN : 0));

                                //UART_PRINT("\n\r CC3200 Publishes the following message \n\r");
                                //UART_PRINT("Topic: %s\n\r", publish_topic);
                                //UART_PRINT("Data: %s\n\r", publish_data);
           sampling_count = 0;
        }
        averageArray[sampling_count] = rotation[0];
        averageArray[(sampling_count+1)] = rotation[1];
        averageArray[(sampling_count+2)] = rotation[2];
        sampling_count = sampling_count + 3;
        Task_sleep(15);
    }
#else  // if we build for the syncing board
    while(1)
    {
        MsgObj msg;
        char receivedMsg[30];
        // currently this is run just 1 time since we are sending just init msg from ref board. If you want to test just comment 3 lines below
        int pendingMessages = 0;
        pendingMessages = Mailbox_getNumPendingMsgs(mbxHandle);
        UART_PRINT("Number of pending messages: %d", pendingMessages);
        if(justBooted == false)
        {
            Task_sleep(5);
            //UART_PRINT("Mailbox Read from sync_app thread: ID = %d and message = '%s'.\n",msg.id, receivedMsg);

           /*----------------------Read sensor----------------------*/
           /* Take 40 samples and print them out onto the console */
           double * rotation;
           rotation = getCurrentAngles(txBuffer, i2c, i2cTransaction, rxBuffer);
           // payload = message from client thread with reference rotation (received from mailbox)
           while(gMqttClient == NULL)
           {
               sleep(3);
           }
           //int maxdif = 180;
           //calculateAndDrawSyncronization(receivedMsg,rotation,maxdif);

           // add measurements to array
           if(sampling_count > 147)
           {
               sampling_count = 0;
           }
           averageArray[sampling_count] = rotation[0];
           averageArray[(sampling_count+1)] = rotation[1];
           averageArray[(sampling_count+2)] = rotation[2];
           sampling_count = sampling_count + 3;
           if(pendingMessages > 0 && averageArray[-1] != NULL)
           {
               Mailbox_pend(mbxHandle, &msg, BIOS_NO_WAIT);
               strncpy(receivedMsg, msg.message, 30);
               double * syncBoardAverages = {NULL};
               syncBoardAverages = calculateAngleAvg(averageArray, 50);
               double * refBoardAverages = {NULL};
               refBoardAverages = ParseMessage(receivedMsg);
               compareAngleSync(syncBoardAverages, refBoardAverages);
           }
        }
        else
        {
            Mailbox_pend(mbxHandle, &msg, BIOS_NO_WAIT);
            justBooted = false;
            sleep(1);
        }
    }
    #endif
    /*----------------------Task end, clean up----------------------*/
    I2C_close(i2c);
    UART_PRINT("I2C closed!\n\r");

    /* Sleep for 1 second */
    return (NULL);
}

double * ParseMessage(char * refBoardMessage)
{
    double xval, yval, zval, totalVal;

    char * messageString = strtok(refBoardMessage, ",");
    xval = atof(messageString);
    messageString = strtok(NULL, ",");
    yval = atof(messageString);
    messageString = strtok(NULL, ",");
    zval = atof(messageString);
    messageString = strtok(NULL, ",");
    totalVal = atof(messageString);

    static double result[4];
    result[0]=xval;
    result[1]=yval;
    result[2]=zval;
    result[3]=totalVal;
    return result;
}

double * calculateAngleAvg (double * own_values, int sample_cnt)
{
    int i;
    double x_own = 0.0, y_own = 0.0, z_own = 0.0, x_avg = 0.0, y_avg = 0.0, z_avg = 0.0, own_sync= 0.0;
    for (i=0; i<150; i = i+3)
    {
        if(isfinite(own_values[i]) != 0 && isfinite(own_values[i+1]) != 0 && isfinite(own_values[i+2]) != 0)
        {
        x_own += own_values[i];
        y_own += own_values[i+1];
        z_own += own_values[i+2];
        }
        else //failsafe
        {
            x_own += 0;
            y_own += 0;
            z_own += 0;
        }
    }

    x_avg = x_own / 50.0;
    y_avg = y_own / 50.0;
    z_avg = z_own / 50.0;

    own_sync = x_avg + y_avg + z_avg;

    static double result[4];
    result[0]=x_avg;
    result[1]=y_avg;
    result[2]=z_avg;
    result[3]=own_sync;

    return result;
}

void compareAngleSync (double * own_value, double * ref_value)
{
    UART_PRINT("%u Diff X: %.1lf,  Diff Y: %.1lf,   Diff Z: %.1lf \n\r",(unsigned long) time(NULL),fabs(own_value[0] - ref_value[0]), fabs(own_value[1] - ref_value[1]),fabs(own_value[2] - ref_value[2]));
    UART_PRINT("%u TOTAL DIFF: %.1lf \n\r", (unsigned long) time(NULL), fabs(own_value[3] - ref_value[3]));
    UART_PRINT("\n\r");
}


//*****************************************************************************
//
//! Function for determining board synchronization. Shows this via UART prints
//! and toggling the LEDs. UART prints a type of progress bar based on proximity
//! and the red LED is toggled when difference is <50%.
//!
//! The MQTT payload should be a comma-separated string of angle values
//! in order (x,y,z) in degrees. Each angle value is a "double".
//!
//! \param[in]  payload     -   contains the mqtt payload string
//! \param[in] ref_x        -   reference x angle in degrees
//! \param[in] ref_y        -   reference y angle in degrees
//! \param[in] ref_z        -   reference z angle in degrees
//! \param[in] maxdif       -   maximum difference when measuring percentages
//!
//! \return None
//!
//*****************************************************************************
void calculateAndDrawSyncronization (char payload[], double * currRot, int maxdif)
{
    double xval, yval, zval, xdif, ydif, zdif, tdif;
    char barx[15], bary[15],barz[15],bart[15], tmpBuf[30];
    double cur_x_rot = currRot[0];
    double cur_y_rot = currRot[1];
    double cur_z_rot = currRot[2];
    char *p;
    int isfin;

    /* Copy the payload into a temporary variable so the original doesn't change */
    strcpy(tmpBuf, payload);
    /* Split the payload, convert into doubles */
    char* cval = strtok(tmpBuf, ",");
    xval = atof(cval);
    cval = strtok(NULL, ",");
    yval = atof(cval);
    cval = strtok(NULL, ",");
    zval = atof(cval);
    /* Calculate absolute differences in angle, flip angle if difference is smaller the other way around */
    xdif = fabs(cur_x_rot - xval);
    if (xdif > 180)
    {
        xdif = 360 - xdif;
    }

    ydif = fabs(cur_y_rot - yval);
    if (ydif > 180)
    {
        ydif = 360 - ydif;
    }

    zdif = fabs(cur_z_rot - zval);
    if (zdif > 180)
    {
        zdif = 360 - zdif;
    }
    /* Convert differences into percentages */
    /* Percentage corresponds to how much the angle is off from an exact match */
    xdif = round((1 - (xdif / maxdif)) * 100 );
    ydif = round((1 - (ydif / maxdif)) * 100 );
    zdif = round((1 - (zdif / maxdif)) * 100 );
    /* Total difference percentage */
    tdif = (xdif + ydif + zdif) / 3;
    /* Get bars */
    p = drawSyncBar(xdif);
    strcpy(barx, p);
    p = drawSyncBar(ydif);
    strcpy(bary, p);
    p = drawSyncBar(zdif);
    strcpy(barz, p);
    p = drawSyncBar(tdif);
    strcpy(bart, p);
    isfin = isfinite(tdif);
    /* Various debug prints, can be turned off to reduce terminal bloat */
    //UART_PRINT("Current rotation values: x: %lf, y: %lf, z: %lf.\n\r", cur_x_rot, cur_y_rot, cur_z_rot);
    //UART_PRINT("Reference rotation values: x: %lf, y: %lf, z: %lf.\n\r", xval, yval, zval);
    //UART_PRINT("Difference: x: %lf%%, y: %lf%%, z: %lf%%.\n\r", xdif, ydif, zdif);
    //UART_PRINT("Total difference: %lf%%.\n\r", tdif);

        /* Print "progress bar" and toggle LEDs based on how close the angles are */
        switch(isfin)
        {
        case 1:
            UART_PRINT("\n\r");
            UART_PRINT("X sync:");
            UART_PRINT(barx);
            UART_PRINT("\n\r");
            UART_PRINT("Y sync:");
            UART_PRINT(bary);
            UART_PRINT("\n\r");
            UART_PRINT("Z sync:");
            UART_PRINT(barz);
            UART_PRINT("\n\r");
            UART_PRINT("t sync:");
            UART_PRINT(bart);
            UART_PRINT("\n\r");
            if(tdif >= 95.00)
            {
                GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);
            }
            else
            {
                GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
            }
            break;
        default:
            /* For the infinites and other weirdness */
            break;
        }
}

//*****************************************************************************
//
//! Function for building the bar used to represent the level of synchronization.
//! Takes the match percentage, returns a bar that represents the value.
//!
//! \param[in]  sync_percentage     -   percentage for angle match
//!
//! \return bar                     -   e.g. [|||.......]
//!
//*****************************************************************************

char *drawSyncBar (double sync_percentage)
{
    /* Init the bar, count full tenths */
    static char bar[15];
    int tenths = floor(sync_percentage / 10);
    int dots = 10 - tenths;
    /* Create the bar */
    memset(bar, '[', sizeof(char));
    memset(bar + 1, '|', tenths*sizeof(char));
    if (tenths < 10)
    {
        memset(bar + 1 + tenths, '.', dots*sizeof(char));
    }
    memset(bar + 11, ']', sizeof(char));
    memset(bar + 12, '\0', sizeof(char));

    return bar;
}

/*----------------------Read sensor----------------------*/
/* Take 40 samples and print them out onto the console */
double * getCurrentAngles(uint8_t txBuffer[1], I2C_Handle i2c, I2C_Transaction i2cTransaction, uint8_t rxBuffer[1])
{

    int8_t          acc_x, acc_y, acc_z;
    double          angle_X_Deg, angle_Y_Deg, angle_Z_Deg;
    uint16_t        sample;


    for (;;)
    {
        /* x axis reading */
        txBuffer[0] = ACC_REG_X;
        if (I2C_transfer(i2c, &i2cTransaction))
        {
            acc_x = rxBuffer[0];
        }
        else
        {
            UART_PRINT("I2C Bus fault.\n\r");
        }
        /* y axis reading */
        txBuffer[0] = ACC_REG_Y;
        if (I2C_transfer(i2c, &i2cTransaction))
        {
            acc_y = rxBuffer[0];
        }
        else
        {
            UART_PRINT("I2C Bus fault.\n\r");
        }
        /* z axis reading */
        txBuffer[0] = ACC_REG_Z;
        if (I2C_transfer(i2c, &i2cTransaction))
        {
            acc_z = rxBuffer[0];
        }
        else
        {
            UART_PRINT("I2C Bus fault.\n\r");
        }
        /* Saturation of acceleration */
        //angleSatur(&acc_x, &acc_y, &acc_z, ACC_1G);

        /* Angles compute */
        angleDeg((double)acc_x, (double)acc_y, (double)acc_z, &angle_X_Deg, &angle_Y_Deg, &angle_Z_Deg);

        //Raw accelerations print
        //UART_PRINT("Sample %u: x=%d ; y=%d ; z=%d\n\r",
        //sample, acc_x, acc_y, acc_z);
        //angle print
        /*UART_PRINT(
                "Sample %u: x angle=%.2f deg ; y angle=%.2f deg ; z angle=%.2f deg\n\r",
                sample, angle_X_Deg, angle_Y_Deg, angle_Z_Deg);
        */
        // TODO: clean this
        static double result[3];
        result[0]=angle_X_Deg;
        result[1]=angle_Y_Deg;
        result[2]=angle_Z_Deg;
        return result;


    }
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

//Compute angles in degrees from accelerations
void angleDeg(double accX, double accY, double accZ, double* angleX, double* angleY, double* angleZ){
    *angleX=atan((double)accX/(sqrt(pow(accY,2)+pow(accZ,2.0))))*180.0/M_PI; //x angle
    *angleY=atan((double)accY/(sqrt(pow(accX,2)+pow(accZ,2.0))))*180.0/M_PI; //y angle
    *angleZ=atan((sqrt(pow(accX,2)+pow(accY,2.0)))/(double)accZ)*180.0/M_PI; //z angle
}


