/*
 * mqtt_client.h
 *
 *  Created on: 30 Nov 2019
 *      Author: Yoann
 */

#ifndef MQTT_CLIENT_H_
#define MQTT_CLIENT_H_

/* Standard includes                                                         */
#include <stdlib.h>
#include <pthread.h>
#include <mqueue.h>
#include <time.h>
#include <unistd.h>

/* TI-Driver includes                                                        */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/SPI.h>

/* Simplelink includes                                                       */
#include <ti/drivers/net/wifi/simplelink.h>

/* SlNetSock includes                                                        */
#include <ti/drivers/net/wifi/slnetifwifi.h>

/* MQTT Library includes                                                     */
#include <ti/net/mqtt/mqttclient.h>

/* Common interface includes                                                 */
#include "network_if.h"
#include "uart_term.h"

/* TI-DRIVERS Header files */
#include "ti_drivers_config.h"

/* Application includes                                                      */
#include "client_cbs.h"

#define PUBLISH_TOPIC1           "/sync/measurements"
#define SUBSCRIPTION_TOPIC_COUNT 4
#define RETAIN_ENABLE            1
volatile MQTTClient_Handle gMqttClient;
MQTTClient_Params MqttClientExmple_params;

#endif /* MQTT_CLIENT_H_ */
