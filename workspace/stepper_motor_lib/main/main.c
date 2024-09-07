#include <stdio.h>
#include <string.h>
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

//PRIVATE INCLUDES
#include "main.h"
#include "motor.h"


#define VSPI_MISO 19
#define VSPI_MOSI 23
#define VSPI_SCLK 18
#define VSPI_SS 5

#define HSPI_MISO 12
#define HSPI_MOSI 13
#define HSPI_SCLK 14
#define HSPI_SS 15

#define STCK 25
#define DIR 27
#define CHIP_SELECT 5
//#define PIN4 35

#define LED 		 	16

#define BACKWARD		1
#define FORWARD			0

static const int spiClk = 1000000; // 1 MHz

spi_device_handle_t vspi;
spi_device_handle_t hspi;

void spi_init() {
    spi_bus_config_t buscfg = {
        .miso_io_num = VSPI_MISO,
        .mosi_io_num = VSPI_MOSI,
        .sclk_io_num = VSPI_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4094,
    };

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = spiClk,           // Clock out at 1 MHz
        .mode = 0,                          // SPI mode 0
        .spics_io_num = VSPI_SS,            // CS pin
        .queue_size = 7,                    // Queue size
    };

    // Initialize the SPI bus
    spi_bus_initialize(VSPI_HOST, &buscfg, 1);
    // Attach the VSPI device to the SPI bus
    spi_bus_add_device(VSPI_HOST, &devcfg, &vspi);

    buscfg.miso_io_num = HSPI_MISO;
    buscfg.mosi_io_num = HSPI_MOSI;
    buscfg.sclk_io_num = HSPI_SCLK;
    devcfg.spics_io_num = HSPI_SS;

    // Initialize the SPI bus
    spi_bus_initialize(HSPI_HOST, &buscfg, 1);
    // Attach the HSPI device to the SPI bus
    spi_bus_add_device(HSPI_HOST, &devcfg, &hspi);
}

void spi_send_command(spi_device_handle_t spi, uint8_t command) {
    spi_transaction_t trans;
    memset(&trans, 0, sizeof(trans));
    trans.length = 8;
    trans.tx_buffer = &command;
    spi_device_transmit(spi, &trans);
}

void pin_config() {
    gpio_reset_pin(STCK);
    gpio_reset_pin(DIR);
    gpio_reset_pin(CHIP_SELECT);
//    gpio_reset_pin(PIN4);
    gpio_reset_pin(LED);


    gpio_set_direction(STCK, GPIO_MODE_OUTPUT);
    gpio_set_direction(DIR, GPIO_MODE_OUTPUT);
    gpio_set_direction(CHIP_SELECT, GPIO_MODE_OUTPUT);
//    gpio_set_direction(PIN4, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED, GPIO_MODE_OUTPUT);

    // ELEGIMOS LA DIRECCION DE MOVIMIENTO
    gpio_set_level(DIR, BACKWARD);

    gpio_set_level(CHIP_SELECT, 0); // SELECCIONAMOS CON EL CHIP SELECT A NIVEL BAJO
    vTaskDelay(500 / portTICK_PERIOD_MS);
}

/******************************************************//**
 * @brief  Inicializa el movimiento del motor
 * @param[in] None
 * @retval None
 * @note Para que funcione el movimiento del motor tenemos
 * que hacer varios pasos en un orden determinado. Primero
 * configuramos los pines, donde tambien elegimos la
 * direccion de giro del motor y bajamos el chip-select.
 * Luego inicializamos y añadimos al bus spi los elementos
 * hspi y vspi.
 * Por último mandamos los comandos de ENABLE y activamos el
 * chip-select para deseleccionar el dispositivo.
 * En el bucle while(1) generamos la señal de reloj para los
 * "steps" que activaran el movimiento del motor.
 **********************************************************/
void app_main(void) {
    pin_config();
    spi_init();

    spi_send_command(vspi, 0b10111000); // ENVIAMOS EL COMANDO DE ENABLE
    spi_send_command(hspi, 0b10111000);

    gpio_set_level(CHIP_SELECT, 1); // DESSELECCIONAMOS CON EL CHIP SELECT A NIVEL ALTO
//    int i = 0;
    while (1) {
        gpio_set_level(STCK, 1); 	// SEÑAL CUADRADA A '1'
        gpio_set_level(LED, 1);		// ENCENDEMOS LUZ
//        i++;
        vTaskDelay(1); // Wait for 250 microseconds

        gpio_set_level(STCK, 0); 	// SEÑAL CUADRADA A '0'
        gpio_set_level(LED, 0);		// APAGAMOS LUZ
        vTaskDelay(1); // Wait for 250 microseconds

//        if (i == 100){
//            gpio_set_level(DIR, FORWARD);
//            i = 0;
//        }
    }
}
