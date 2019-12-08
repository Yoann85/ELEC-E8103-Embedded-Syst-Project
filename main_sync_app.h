/*
 * main_sync_app.h
 *
 *  Created on: 8 Dec 2019
 *      Author: Roope
 */

#ifndef MAIN_SYNC_APP_H_
#define MAIN_SYNC_APP_H_

void calculateAndDrawSyncronization (char payload[], double * currRot, int maxdif);
char *drawSyncBar (double sync_percentage);
double * getCurrentAngles(uint8_t txBuffer[1], I2C_Handle i2c, I2C_Transaction i2cTransaction, uint8_t rxBuffer[1]);
void angleDeg(double accX, double accY, double accZ, double* angleX, double* angleY, double* angleZ);
void angleSatur(int8_t* accX, int8_t* accY, int8_t* accZ, int8_t AbsoluteMaxVal);

#endif /* MAIN_SYNC_APP_H_ */
