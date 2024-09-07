#include <stdio.h>
#include "driver/gpio.h"
#include "driver/adc.h"
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>

#include "esp_log.h"

#include "driver/adc.h"

#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"


#define NPN_PIN 			17 //SENAL DIGITAL -> '1'/'0' LOGICO
#define OPA_PIN				2 //SEÑAL QUE VA AL PIN CON ADC2 INTERNO DEL MICRO
#define HIGH 				1
#define LOW					0

//int sensorValue = analogRead(OPA_PIN);
int *raw_out;

static void configure_pin(void){
	gpio_reset_pin(NPN_PIN);
	gpio_reset_pin(OPA_PIN);
	gpio_set_direction(NPN_PIN, GPIO_MODE_INPUT);
	gpio_set_direction(OPA_PIN, GPIO_MODE_INPUT);
}

//static void init_ADC(void){
//	adc2_config_channel_atten(OPA_PIN, 0);
//}



void app_main(void)
{
    ESP_LOGI("INFO:", "COMIENZO DEL PROGRAMA");
	configure_pin();


    while (1) {
    	int sensorValue  = adc2_get_raw(OPA_PIN, 12, &raw_out);

        ESP_LOGI("INFO:", "valor del fotodiodo, sensorValue=%d", sensorValue);
        vTaskDelay(150);
    }
}
