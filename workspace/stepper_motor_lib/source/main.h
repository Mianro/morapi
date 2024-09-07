/*
 * main.h
 *
 *  Created on: 12 jun 2024
 *      Author: rmoratilla
 */

#ifndef SOURCE_MAIN_H_
#define SOURCE_MAIN_H_


//DEFINES E INCLUES ANTES DE LAS FUNCIONES



/**
 * @brief Configure all the pins for the program
 *
 * @param none
 */
void pin_config (void);

/**
 * @brief Sends an 8 bit command to the
 * SPI device
 *
 * @param spi_device_handle_t spi
 * @param uint8_t command to transmit
 *
 * @note Before send the command, it is needed
 * to put the CHIP_SELECT pin to '0' and after
 * sending the command put it back to '1'
 */
void spi_send_command(spi_device_handle_t spi, uint8_t command);

/**
 * @brief Configure the SPI handlers in order
 * to establish the connection between devices
 *
 * @param none
 */
void spi_init (void);

/**
 * @brief Main program to move the stepper motor
 *
 * @param none
 */
void app_main (void);

#endif /* SOURCE_MAIN_H_ */
