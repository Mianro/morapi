#include <stdint.h>
#include <stdio.h>
#include <inttypes.h>
#include <string.h>
#include <sys/unistd.h>
#include <sys/stat.h>


#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "memspi_host_driver.h"

#include "esp_chip_info.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_log.h"
#include "esp_partition.h"
#include "esp_netif_sntp.h"
//#include "esp_netif_sntp.h"
#include "esp_sntp.h"

#include "nvs_flash.h"
#include "mqtt_client.h"

#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"

#include "driver/gpio.h"
#include "driver/i2c.h"
//#include "MLX90640_API.h"

#include "lwip/sys.h"
//#include "lwip/ip_addr.h"
#include "lwip/netdb.h"
#include "lwip/api.h"

// motor
#include "driver/spi_master.h"

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

#define BACKWARD		1
#define FORWARD			0

static const int spiClk = 1000000; // 1 MHz

spi_device_handle_t vspi;
spi_device_handle_t hspi;
// motor


#define DEFAULT_SSID "DIGIFIBRA-ZSuS"
#define DEFAULT_PWD  "hfG6uKh4cu2X"

#define LED 		 	16
#define IR_GPIO			17
#define LEVER			4

#define HIGH 			1
#define LOW				0
//#define TA_SHIFT 	 8
//#define MLX90640_ADDR                 0x33
//#define MLX90640_WHO_AM_I_REG_ADDR    0x8000

//#define PIN_NUM_MISO   13
//#define PIN_NUM_MOSI   11
//#define PIN_NUM_CLK    12
//#define PIN_NUM_CS     10
//#define PIN_NUM_QUADWP 14
//#define PIN_NUM_QUADHD 9


//#define MOUNT_POINT "/sd"
//
//float emissivity = 0.95;
//float tr;
//static uint16_t eeMLX90640[832];
//static uint16_t mlx90640Frame[834];
//paramsMLX90640 mlx90640;
//static float mlx90640To[768];
//int status;
const TickType_t xDelay = 40 / portTICK_PERIOD_MS;
//TickType_t xStart, xEnd, xDifference;


/*
 * TIENE QUE SER UNA VARIABLE GLOBAL PARA QUE LA
 * PUEDA UTILIZAR MAS DE UNA TAREA SIMULTANEAMENTE
 */
bool boxRunning = 0;
bool lightOn = 0;
bool leverPushed = 0;
bool alreadyMoved = 0;
//bool cameraRunning = 0;
//bool dataOK = 0;


typedef struct {
    esp_mqtt_client_handle_t *client; // Client handle
    QueueHandle_t queue;
    SemaphoreHandle_t sem;
    bool dataOk;
    bool dataCentroidOK;
    float Tth;

} TaskParams_t;


typedef struct {
	TickType_t  xMlxStart;
	TickType_t  xMlxAfter;
	TickType_t  xMlxBefore;
} timestamp_t;



//char mount_point[] = MOUNT_POINT;

const char CONFIG_BROKER_URL[] = "mqtt://51f4e816-d778-441d-8668-2f8941197eeb:waZYWRxPg6Fm1CW7SxfcfrXbOunz1PThYNMZHnvO@broker.qubitro.com";

wifi_config_t wifi_config = {
        .sta = {
            .ssid = DEFAULT_SSID,
            .password = DEFAULT_PWD
        },
    };
//spi_bus_config_t bus_cfg = {
//        .mosi_io_num = PIN_NUM_MOSI,
//        .miso_io_num = PIN_NUM_MISO,
//        .sclk_io_num = PIN_NUM_CLK,
//        .quadwp_io_num = -1,
//        .quadhd_io_num = -1,
//        .max_transfer_sz = 4000,
//    };
static const char *TAG = "LightIR-Test";

static void moveMotor(uint32_t steps);


/*
 * @brief Initialize wifi in ESP32
 */
static void init_wifi(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );

    esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();
    assert(sta_netif);
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_connect());

}

/*
 * @brief This function is a Task that samples the camera, then parses the data into JSON format
 * Finally stores the data in an SD and publishes it with mqtt
 *
 * @param parameter void pointer which converts into mqtt client handle
 */
//void mlx_capture(void *parameter){
//
//	// Mqtt client declaration and tick variables
//	TaskParams_t *params = (TaskParams_t *) parameter;
//	esp_mqtt_client_handle_t client = *params->client;
//	TickType_t xMlxAfter, xMlxStart;
//	int numDataRead = 0;
//
//	for(;;){
//
//		// Initialization of variables
//		time_t now;
//		struct tm timeinfo;
//		char ruta[100];
//		FILE *f = NULL;
//		timestamp_t timestamp;
//		xMlxStart = xTaskGetTickCount();
//		timestamp.xMlxStart = xMlxStart;
//		timestamp.xMlxBefore = xMlxStart;
//
//		// Obtain time data from SNTP server configured in main
//		time(&now);
//		setenv("TZ", "UTC-2", 1); // Set Timezone. Adjust this in summer or in winter (Spain)
//		tzset();
//		localtime_r(&now, &timeinfo);
//		char strftime_buf[64];
//		strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
//
//		// Create a txt file for storage data in SD, based in actual date time data (day-month-hour-min)
//		memset(ruta, 0, sizeof(ruta));
//		strcat(ruta, MOUNT_POINT"/camaraIR");
//		char dateTime[64];
//		snprintf(dateTime, sizeof(dateTime),"/%d%d%d%d.txt", timeinfo.tm_mday, timeinfo.tm_mon+1, timeinfo.tm_hour, timeinfo.tm_min );
//		strcat(ruta, dateTime);
//
//		// If the MQTT server send True into camaraIR topic execute this code
//		while(cameraRunning){
//
//			// See if the file is open, is not open it
//			if (!f) {
//				f = fopen(ruta, "a");
//			    if (!f) {
//					ESP_LOGE(TAG, "Failed to open file %s", ruta);
//					vTaskDelay(xDelay);
//			    }
//			}
//			float fps;
//			char string[200];
//			char datos[5000];
//			memset(datos, 0, sizeof(datos));
//
//			// Capture data from MLX90640
//			MLX90640_data(&fps);
//
//			// Communication to the string convert data  Task
//			params->dataOk = true;
//			// Communication to the centroid Task
//			params->dataCentroidOK = true;
//
//
//			//mlx_imageProcess(&centroid, 36.0);
//			//printf("%d, %d\n", params->centroid.x, params->centroid.y);
//			/*
//			for(int i=0; i < 768; i++){
//				printf("\nX: %d, Y: %d", hotBody[i].x, hotBody[i].y);
//			}
//			*/
//
//			xMlxAfter = xTaskGetTickCount(); // Time capture for fps and timestamp
//			numDataRead++;
//			// Read 5 samples
//			if(numDataRead == 5) {
//			   xSemaphoreGive(params->sem);
//			   numDataRead = 0; // Reset the counter
//
//				// Wait until there is data
//			   if(xQueueReceive(params->queue, datos, portMAX_DELAY) == pdTRUE && xSemaphoreTake(params->sem, portMAX_DELAY) == pdTRUE){
//				   // Add timestamp and fps (Including parse data and process time)
//				   timestamp.xMlxAfter = xMlxAfter;
//				   fps = (float) ((timestamp.xMlxAfter - timestamp.xMlxBefore) * portTICK_PERIOD_MS)/1000;
//				   fps = 1/fps;
//			       strcat(datos, "\",\"timeStamp\": ");
//				   snprintf(string, 200, "%ld", timestamp.xMlxAfter - timestamp.xMlxStart );
//				   strcat(datos, string);
//				   strcat(datos, ",\"fps\": ");
//				   snprintf(string, 20, "%.2f", fps);
//				   strcat(datos, string);
//				   snprintf(string, 20, "\"(%d,%d)\"", params->centroid.x, params->centroid.y);
//				   strcat(datos, "\",\"centroidHotBody\": ");
//				   strcat(datos, string);
//				   strcat(datos, "}");
//				   printf("%.2f\n", fps);
//				   timestamp.xMlxBefore = xMlxAfter;
//				   // Write in SD
//				   fprintf(f, datos);
//				   // Publish data in camaraIR/data MQTT topic
//				   esp_mqtt_client_publish(client, "camaraIR/data", datos, 0, 0, 0);
//				}
//			}
//
//
//	}
//		// If the file is open close it
//		if(f){
//			fclose(f);
//		}
//		// Delay to give time to the IDLE Task
//		vTaskDelay(xDelay);
//	}
//
//
//}
/*
 * @brief This function is a task that converts data from float to string in JSON format
 *
 * @param parameter pointer to TakParams_t structure
 *
 */
// void mlx_stringData( void  *parameter ){
//	 TaskParams_t *params = (TaskParams_t *) parameter;
//	 for(;;){
//		// Initialization of variables
//	 if(params->dataOk){
//			struct tm timeinfo;
//			time_t now;
//			float fps;
//
//			char time_str[50];
//			time(&now);
//			localtime_r(&now, &timeinfo);
//
//			/*// Calculate fps with ticks after and before captureData and parseData
//			fps = (float) ((timestamp.xMlxAfter - timestamp.xMlxBefore) * portTICK_PERIOD_MS)/1000;
//			fps = 1/fps;
//			printf("%.2f\n", fps);*/
//
//			// Formatting data into JSON
//			char datos[5000];
//			char *temp_ptr = datos;
//			snprintf(time_str, sizeof(time_str), "%d-%d-%d %d:%d:%d", timeinfo.tm_mday,timeinfo.tm_mon+1, timeinfo.tm_year +1900, timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec );
//			temp_ptr += snprintf(temp_ptr, 17, "{ \"dateTime\": \"");
//			temp_ptr += snprintf(temp_ptr, 20, "%s", time_str);/*
//			temp_ptr += snprintf(temp_ptr, 17, "\",\"timeStamp\": ");
//			temp_ptr += snprintf(temp_ptr, 200, "%ld", timestamp.xMlxAfter - timestamp.xMlxStart );
//			sprintf(fps_c, "%.2f ", fps);
//			temp_ptr += snprintf(temp_ptr, 17, ",\"fps\": ");
//			temp_ptr += snprintf(temp_ptr, 17, "%s", fps_c);*/
//			temp_ptr += snprintf(temp_ptr, 17, ",\"frame\": \"");
//
//
//			// Iterate over mlx90640To to convert float data into String
//			// This operation consumes time but it's necessary to send data through MQTT and save data in SD Card
//			for (uint8_t h = 0; h < 24; h++) {
//				for (uint8_t w = 0; w < 32; w++) {
//					 temp_ptr += snprintf(temp_ptr, 20, "%.2f ", mlx90640To[h * 32 + w]);
//				}
//				if (h < 23) {
//					temp_ptr += snprintf(temp_ptr, 3, " ;");
//				} else {
//					temp_ptr += snprintf(temp_ptr, 2, "\"");
//				}
//			}
//			//temp_ptr += snprintf(temp_ptr, 17, "}");
//			xQueueSend(params->queue, datos, portMAX_DELAY);
//			params->dataOk = false;
//		}
//		vTaskDelay(xDelay);
//	}
//}

/*
 * @brief This function is a Task that samples the camera, then parses the data into JSON format
 * Finally stores the data in an SD and publishes it with mqtt
 *
 * @param parameter void pointer which converts into mqtt client handle
 */
void box_start(void *parameter){
	// Mqtt client declaration and tick variables
	TaskParams_t *params = (TaskParams_t *) parameter;
	esp_mqtt_client_handle_t client = *params->client;

	char data[50];
	memset(data, 0, sizeof(data));

	int numVecesDetectado = 0;
	bool detected = 0;
	TickType_t timeInit;
	TickType_t timePassed;
	TickType_t timeDetection = 0;;
	TickType_t timeDiff;
	bool initTime = true;

	for(;;){
		while(boxRunning){
			if (initTime){
				timeInit = xTaskGetTickCount();
				initTime = false;
			}
			

			if (lightOn) {
				gpio_set_level(LED, HIGH);
				if (gpio_get_level(LEVER) && (leverPushed == 0) && (alreadyMoved == 0)) {
					sprintf(data, "1");
					printf("%s\n", data);
					esp_mqtt_client_publish(client, "lever", data, 0, 0, 0);
					moveMotor(1000);
					leverPushed = 1;
					alreadyMoved = 1;
				} else if (!gpio_get_level(LEVER) && (leverPushed == 1)) {
					sprintf(data, "0");
					printf("%s\n", data);
					esp_mqtt_client_publish(client, "lever", data, 0, 0, 0);
					leverPushed = 0;			
				}	
			}
			else {
				gpio_set_level(LED, LOW);
				alreadyMoved = 0;
			}
			
			if (!gpio_get_level(LEVER) && (leverPushed == 1) ) {
				sprintf(data, "0");
				printf("%s\n", data);
				esp_mqtt_client_publish(client, "lever", data, 0, 0, 0);
				leverPushed = 0;
			}
			
			if (gpio_get_level(IR_GPIO) == LOW && detected ) { //RECIBO LUZ -> PIN A GND -> ENCIENDO LED
				timeDiff = xTaskGetTickCount();
				//gpio_set_level(LED, LOW);
				sprintf(data, "NO DETECTADO\r\n");
				printf("%s\n", data);
// 				printf("RATON NO DETECTADO\r\n");
				esp_mqtt_client_publish(client, "skinnerbox_output", data, 0, 0, 0);
				detected = !detected;
			} else if (gpio_get_level(IR_GPIO) == HIGH && !detected ){	//NO RECIBO LUZ -> PIN A VDD -> APAGO LED
				timePassed = xTaskGetTickCount();
				//gpio_set_level(LED, HIGH);
				timeDetection = timePassed - timeInit;
//				printf("RATON DETECTADO %i, Time: %.2f\r\n", ++numVecesDetectado, (float)((timePassed - timeInit)*portTICK_PERIOD_MS));
//				Si divido entre 1000 tengo los segundos (no olvidar poner %lds o %ldms
//				sprintf(data, "RATON DETECTADO %i\nTime: %ldms\nDiff: %ldms\r\n", ++numVecesDetectado, (timeDetection * portTICK_PERIOD_MS), (timePassed - timeDiff)*portTICK_PERIOD_MS);
				sprintf(data, "RATON DETECTADO %i\nTime: %ldms\r\n", ++numVecesDetectado, (timeDetection * portTICK_PERIOD_MS));
				printf("%s\n", data);
//				printf("RATON DETECTADO %i, Time: %ldms\r\n", ++numVecesDetectado, ((timePassed - timeInit)*portTICK_PERIOD_MS));
				esp_mqtt_client_publish(client, "skinnerbox_output", data, 0, 0, 0);
				detected = !detected;

			} else {
				break;
			}
		}

		if (!boxRunning){
			numVecesDetectado = 0;
			detected = 0;
			initTime = true;
			//lightOn = false;
			gpio_set_level(LED, LOW);
//			sprintf(data, "EXPERIMENTO NO COMENZADO");
//			esp_mqtt_client_publish(client, "skinnerbox_output", data, 0, 0, 0);
		}
		vTaskDelay(xDelay);
	}
}

//while(boxRunning){
//	if (initTime){
//		timeInit = xTaskGetTickCount();
//		initTime = false;
//	}
//	if (gpio_get_level(IN_GPIO) == LOW && detected ) { //RECIBO LUZ -> PIN A GND -> ENCIENDO LED
//		timePassed = xTaskGetTickCount();
//		gpio_set_level(LED, LOW);
//		timeDetection = timePassed - timeInit;
////				printf("RATON DETECTADO %i, Time: %.2f\r\n", ++numVecesDetectado, (float)((timePassed - timeInit)*portTICK_PERIOD_MS));
////				Si divido entre 1000 tengo los segundos (no olvidar poner %lds o %ldms
////				sprintf(data, "RATON DETECTADO %i\nTime: %ldms\nDiff: %ldms\r\n", ++numVecesDetectado, (timeDetection * portTICK_PERIOD_MS), (timePassed - timeDiff)*portTICK_PERIOD_MS);
//		sprintf(data, "RATON DETECTADO %i\nTime: %ldms\r\n", ++numVecesDetectado, (timeDetection * portTICK_PERIOD_MS));
//		printf("%s\n", data);
////				printf("RATON DETECTADO %i, Time: %ldms\r\n", ++numVecesDetectado, ((timePassed - timeInit)*portTICK_PERIOD_MS));
//		esp_mqtt_client_publish(client, "skinnerbox_output", data, 0, 0, 0);
//		detected = !detected;
//	} else if (gpio_get_level(IN_GPIO) == HIGH && !detected ){			 //NO RECIBO LUZ -> PIN A VDD -> APAGO LED
//		timeDiff = xTaskGetTickCount();
//		gpio_set_level(LED, HIGH);
//		sprintf(data, "NO DETECTADO\r\n");
//		printf("%s\n", data);
//// 				printf("RATON NO DETECTADO\r\n");
//		esp_mqtt_client_publish(client, "skinnerbox_output", data, 0, 0, 0);
//		detected = !detected;
//	} else {
//		break;
//	}
//}

/*
 * @brief This function is a Task that samples the camera, then parses the data into JSON format
 * Finally stores the data in an SD and publishes it with mqtt
 *
 * @param parameter void pointer which converts into mqtt client handle
 */
void check_lever(void *parameter){
	// Mqtt client declaration and tick variables
	TaskParams_t *params = (TaskParams_t *) parameter;
	esp_mqtt_client_handle_t client = *params->client;

	char data[100];
	memset(data, 0, sizeof(data));


	for(;;){
		while(boxRunning){
			if (LEVER == 0){
				sprintf(data, "PULSADA\r\n");
				esp_mqtt_client_publish(client, "lever", data, 0, 0, 0);
			} else if (LEVER == 1){
				sprintf(data, "NO PULSADA\r\n");
				esp_mqtt_client_publish(client, "lever", data, 0, 0, 0);
			}

		}
//		if (!boxRunning){
//			break;
//		}
		vTaskDelay(xDelay);
	}
}

// motor

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


static void moveMotor (uint32_t steps){
	for (uint32_t i=0; i<steps; i++) {
		uint32_t count;
		uint32_t count2;
        gpio_set_level(STCK, 1); 	// SEÑAL CUADRADA A '1'
        for (uint32_t j=0; j<=1000; j++){
			count++;
			if (count % 100 == 0){
				count2++;
			}
		}
		count = 0;
		if(count2 != 0){
			count2 = 0;
		}
        gpio_set_level(STCK, 0); 	// SEÑAL CUADRADA A '0'
       for (uint32_t j=0; j<=1000; j++){
			count++;
			if (count % 10 == 0){
				count2++;
			}
		}
		count = 0;
		if(count2 != 0){
			count2 = 0;
		}
	}
}

// motor


/*
 * @brief mqtt callback for connected, disconnected, suscribed, unsuscribed, published and data
 *
 * @param event_data
 * @param event_id  MQTT_EVENT_CONNECTED,
 * 					MQTT_EVENT_DISCONNECTED,
 * 					MQTT_EVENT_SUBSCRIBED,
 * 					MQTT_EVENT_UNSUBSCRIBED,
 * 					MQTT_EVENT_PUBLISHED,
 * 					MQTT_EVENT_DATA or
 * 					MQTT_EVENT_ERROR
 */
static void mqtt_callback(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data){
	esp_mqtt_event_handle_t event = event_data;
	esp_mqtt_client_handle_t client = event->client;
	int msg_id;
	switch ((esp_mqtt_event_id_t)event_id) {
	    case MQTT_EVENT_CONNECTED:
	        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
	        msg_id = esp_mqtt_client_publish(client, "/topic/qos1", "data_3", 0, 1, 0);
	        ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);

//	        msg_id = esp_mqtt_client_subscribe(client, "camaraIR", 0);
//	        ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

//	        msg_id = esp_mqtt_client_subscribe(client, "output", 0);
//	        ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

	        msg_id = esp_mqtt_client_subscribe(client, "skinnerbox", 0);
	        ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);
	        
	        msg_id = esp_mqtt_client_subscribe(client, "light", 0);
	        ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

	        //NO HACE FALTA QUE ME SUBSCRIBA A LO QUE TENGO QUE ENVIAR
//	        msg_id = esp_mqtt_client_subscribe(client, "skinnerbox_output", 0);
//	        ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

//	        msg_id = esp_mqtt_client_subscribe(client, "/topic/qos1", 1);
//	        ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

//	        msg_id = esp_mqtt_client_unsubscribe(client, "/topic/qos1");
//	        ESP_LOGI(TAG, "sent unsubscribe successful, msg_id=%d", msg_id);
	        break;
	    case MQTT_EVENT_DISCONNECTED:
	        ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
	        break;

	    case MQTT_EVENT_SUBSCRIBED:
	        ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
	        msg_id = esp_mqtt_client_publish(client, "camaraIR", "data", 0, 0, 0);
	        msg_id = esp_mqtt_client_publish(client, "output", "data", 0, 0, 0);
	        ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
	        break;
	    case MQTT_EVENT_UNSUBSCRIBED:
	        ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
	        break;
	    case MQTT_EVENT_PUBLISHED:
	        ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
	        break;
	    case MQTT_EVENT_DATA:
	        ESP_LOGI(TAG, "MQTT_EVENT_DATA");
	        if (event->topic[0] == 0x73) {
		        char datos= *event->data;
		        if(datos == 't'){
	//	        	gpio_set_level(LED,HIGH);
		        	boxRunning = 1;
	//	        	cameraRunning = 1;
		        }
		        else{
	//	        	gpio_set_level(LED,LOW);
		        	boxRunning = 0;
	//	        	cameraRunning = 0;
		        }
			}
	        if (event->topic[0] == 0x6C) {
		        char datos= *event->data;
		        if(datos == 't'){
	//	        	gpio_set_level(LED,HIGH);
		        	lightOn = true;
	//	        	cameraRunning = 1;
		        }
		        else{
	//	        	gpio_set_level(LED,LOW);
		        	lightOn = false;
	//	        	cameraRunning = 0;
		        }
			}


	        printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
	        printf("DATA=%.*s\r\n", event->data_len, event->data);
	        break;
	    case MQTT_EVENT_ERROR:
	        ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
	        if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
	            ESP_LOGI(TAG, "Last errno string (%s)", strerror(event->error_handle->esp_transport_sock_errno));

	        }
	        break;
	    default:
	        ESP_LOGI(TAG, "Other event id:%d", event->event_id);
	        break;
	    }
}
/*
 * @brief this function starts mqtt callback task and mlx90640 sample, stringConvert and processData task
 */
static void mqtt_app_start(void){
	esp_mqtt_client_config_t mqtt_cfg = {
	        .broker.address.uri = CONFIG_BROKER_URL,
	    };
	esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
	esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_callback, NULL);
	esp_mqtt_client_start(client);

	QueueHandle_t dataQueue = xQueueCreate(1, sizeof(char)*5000);
	SemaphoreHandle_t dataSemaphore = xSemaphoreCreateBinary();
	// Reservar memoria para la estructura que tendrá los datos de las tareas
	TaskParams_t *task_params = malloc(sizeof(TaskParams_t));
	*task_params->client = client;
	task_params->queue = dataQueue;
	task_params->sem = dataSemaphore;
	task_params->dataOk = false;
	task_params->dataCentroidOK = false;
	task_params->Tth = 36.0;

	xTaskCreate(&box_start, "box_start", 2000, (void *) task_params, 1, NULL);
//	xTaskCreate(&check_lever, "check_lever", 2500, (void *) task_params, 1, NULL);

//	xTaskCreate(&mlx_capture, "mlx_capture", 24576, (void *) task_params , 1, NULL);
//	xTaskCreate(&mlx_stringData, "mlx_stringData", 10000, (void *) task_params, 1, NULL);
//	xTaskCreate(&mlx_imageProcess, "mlx_imageProcess", 2500, (void *) task_params, 1, NULL);
}

void pin_config() {
	
	gpio_set_direction(LED, GPIO_MODE_OUTPUT);
	gpio_set_direction(IR_GPIO, GPIO_MODE_INPUT);
	gpio_set_direction(LEVER, GPIO_MODE_INPUT);
	
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


/*
 * @brief Main Function. System initialization and execution
 */
void app_main(void)
{
	pin_config();
	nvs_flash_init();
	esp_netif_init();
	init_wifi();
	
    spi_init();
    spi_send_command(vspi, 0b10111000); // ENVIAMOS EL COMANDO DE ENABLE
    spi_send_command(hspi, 0b10111000);
    
    gpio_set_level(CHIP_SELECT, 1); // DESSELECCIONAMOS CON EL CHIP SELECT A NIVEL ALTO



	mqtt_app_start();


    //deinitialize the bus after all devices are removed
    //spi_bus_free(host.slot);

}
