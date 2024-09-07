#include <stdio.h>
#include <stdbool.h>
#include <unistd.h>
#include <string.h>

//PRIVATE INCLUDES
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "driver/spi_common.h"
#include "driver/spi_slave.h"

#include "driver/timer.h"



//DECLARACION DE PINES Y VARIABLES
#define HIGH		1
#define LOW 		0

#define LIGHT       16

#define VSPI_MISO   2
#define VSPI_MOSI   4
#define VSPI_SCLK   0
#define VSPI_SS     33

#define HSPI_MISO   26
#define HSPI_MOSI   27
#define HSPI_SCLK   25
#define HSPI_SS     32

#define SPI_FREQUENCY   1000000 // 1 MHz

spi_device_handle_t vspi;
spi_device_handle_t hspi;


//FUNCIONES AUXILIARES
void configure_pin (void){
    gpio_reset_pin(LIGHT);
    gpio_reset_pin(HSPI_MISO);
    gpio_reset_pin(HSPI_MOSI);
    gpio_reset_pin(HSPI_SCLK);
    gpio_reset_pin(HSPI_SS);

    gpio_set_direction(LIGHT,		GPIO_MODE_OUTPUT);
    gpio_set_direction(HSPI_MISO, 	GPIO_MODE_OUTPUT);
    gpio_set_direction(HSPI_MOSI, 	GPIO_MODE_OUTPUT);
    gpio_set_direction(HSPI_SCLK, 	GPIO_MODE_OUTPUT);
    gpio_set_direction(HSPI_SS, 	GPIO_MODE_OUTPUT);
}

void spi_command(spi_device_handle_t spi, uint8_t data){
	spi_transaction_t t;
	memset(&t, 0, sizeof(t));
	t.length = 8; // 8 bits
	t.tx_buffer = &data;
	spi_device_queue_trans(spi, &t, portMAX_DELAY);

	spi_device_polling_transmit(spi, &t);
}

void spi_init(void){
    spi_bus_config_t buscfg = {
        .miso_io_num = HSPI_MISO,
        .mosi_io_num = HSPI_MOSI,
        .sclk_io_num = HSPI_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = SPI_FREQUENCY,
        .mode = 0,
        .spics_io_num = HSPI_SS,
        .queue_size = 1,
    };

    spi_bus_initialize(HSPI_HOST, &buscfg, 1);
    spi_bus_add_device(HSPI_HOST, &devcfg, &hspi);
}


//PROGRAMA PRINCIPAL
void app_main(void)
{
	spi_init();
	configure_pin();

	gpio_set_level(HSPI_MOSI, LOW);
	vTaskDelay(500);

	spi_command(hspi, 0b10111000); //ACTIVAR EL ENABLE DEL DRIVER

    while (1) {
        gpio_set_level(25, HIGH); // Activar reloj del motor
        vTaskDelay(25);
        gpio_set_level(25, LOW); // Desactivar reloj del motor
        vTaskDelay(25);
    }
}
