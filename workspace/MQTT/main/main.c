#include <stdio.h>
#include <inttypes.h>
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

#define DEFAULT_SSID "B105_net"
#define DEFAULT_PWD  "FiNaO?21"
#define LED 		 16
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
//const TickType_t xDelay = 40 / portTICK_PERIOD_MS;
//TickType_t xStart, xEnd, xDifference;
//bool cameraRunning = 0;
//bool dataOK = 0;
//
//
//typedef struct {
//    uint8_t x;
//    uint8_t y;
//} Coord_t;
//
//typedef struct {
//    esp_mqtt_client_handle_t *client; // Client handle
//    QueueHandle_t queue;
//    SemaphoreHandle_t sem;
//    bool dataOk;
//    bool dataCentroidOK;
//    float Tth;
//    Coord_t centroid;
//} TaskParams_t;
//
//typedef struct {
//	TickType_t  xMlxStart;
//	TickType_t  xMlxAfter;
//	TickType_t  xMlxBefore;
//} timestamp_t;
//


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
static const char *TAG = "camaraIR-Test";

//void mlx_parseData(char* datos, timestamp_t timestamp);
//void mlx_imageProcess(Coord_t *centroid, float Tth);


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
 * @brief Initialize the communication with the MLX camera
 */
//static void mlx_init(){
//
//	MLX_I2CInit();
//	status = MLX90640_DumpEE (MLX90640_ADDR, eeMLX90640);
//	status = MLX90640_ExtractParameters(eeMLX90640, &mlx90640);
//	MLX90640_SetRefreshRate(MLX90640_ADDR, MLX90640_64_HZ);
//
//}
/*
 * @brief Capture temperature data from two frames using MLX90650_API Library after initializing the camera.
 * Additionally this function gives the frames per second of the capture
 *
 * @param fps pointer to fps float data
 * @return void
 */
//static void MLX90640_data(float* fps){
//
//	xStart =xTaskGetTickCount(); // Start time for measure fps
//	// Get temperature data from two frames of Mlx90640
//	// 1st frame
//	MLX90640_GetSubFrameData (MLX90640_ADDR, mlx90640Frame);
//	tr = MLX90640_GetTa(mlx90640Frame, &mlx90640) - TA_SHIFT;
//	MLX90640_CalculateTo(mlx90640Frame, &mlx90640, emissivity, tr, mlx90640To);
//	// 2nd frame
//	MLX90640_GetSubFrameData (MLX90640_ADDR, mlx90640Frame);
//	tr = MLX90640_GetTa(mlx90640Frame, &mlx90640) - TA_SHIFT;
//	MLX90640_CalculateTo(mlx90640Frame, &mlx90640, emissivity, tr, mlx90640To);
//	// Measure fps after getting data from Mlx90640
//	xEnd =xTaskGetTickCount();
//	xDifference = (xEnd-xStart);
//	float fps_p;
//	fps_p= (float) xDifference*portTICK_PERIOD_MS;
//	fps_p= fps_p/1000;
//	fps_p = 1/fps_p;
//	*fps=fps_p;
//
//}
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
  * @brief Capturate the pixels positions of the hot body then calculate the centroid of them
  *
  * @param centroid pointer of the struct of centroid data
  * @param Tth Temperature threshold
  */
//void mlx_imageProcess(void * parameter){
//	TaskParams_t *params = (TaskParams_t *) parameter;
//	Coord_t centroid = params->centroid;
//	centroid.x = 0;
//	centroid.y = 0;
//	for(;;){
//		if(xSemaphoreTake(params->sem, portMAX_DELAY) == pdTRUE){
//			Coord_t hotBody[768];
//			int numPixel = 0;
//			for (int h = 0; h < 24; h++) {
//				for (int w = 0; w < 32; w++) {
//					if (mlx90640To[h * 32 + w] > params->Tth){
//						hotBody[numPixel].x = w;
//						hotBody[numPixel].y = h;
//						numPixel++;
//					}
//				}
//			}
//			for(int i = 0; i < numPixel; i++){
//				centroid.x += hotBody[i].x;
//				centroid.y += hotBody[i].y;
//			}
//			if(numPixel > 0){
//				centroid.x /= numPixel + 1;
//				centroid.y /= numPixel + 1;
//			}
//			params->centroid = centroid;
//			xSemaphoreGive(params->sem);
//		}
//		vTaskDelay(xDelay);
//	}
//}
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

	        msg_id = esp_mqtt_client_subscribe(client, "camaraIR", 0);
	        ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

	        msg_id = esp_mqtt_client_subscribe(client, "output", 0);
	        ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

	        msg_id = esp_mqtt_client_subscribe(client, "/topic/qos1", 1);
	        ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

	        msg_id = esp_mqtt_client_unsubscribe(client, "/topic/qos1");
	        ESP_LOGI(TAG, "sent unsubscribe successful, msg_id=%d", msg_id);
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
	        char datos= *event->data;
	        if(datos== 't'){
	        	gpio_set_level(LED,1);
//	        	cameraRunning = 1;
	        }
	        else{
	        	gpio_set_level(LED,0);
//	        	cameraRunning = 0;
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

//	QueueHandle_t dataQueue = xQueueCreate(1, sizeof(char)*5000);
//	SemaphoreHandle_t dataSemaphore = xSemaphoreCreateBinary();
//	// Reservar memoria para la estructura que tendrá los datos de las tareas
//	TaskParams_t *task_params = malloc(sizeof(TaskParams_t));
//	*task_params->client = client;
//	task_params->queue = dataQueue;
//	task_params->sem = dataSemaphore;
//	task_params->dataOk = false;
//	task_params->dataCentroidOK = false;
//	task_params->Tth = 36.0;
//
//	xTaskCreate(&mlx_capture, "mlx_capture", 24576, (void *) task_params , 1, NULL);
//	xTaskCreate(&mlx_stringData, "mlx_stringData", 10000, (void *) task_params, 1, NULL);
//	xTaskCreate(&mlx_imageProcess, "mlx_imageProcess", 2500, (void *) task_params, 1, NULL);
}
//static esp_err_t sd_write_file(const char *path, char *data)
//{
//    ESP_LOGI(TAG, "Opening file %s", path);
//    FILE *f = fopen(path, "a");
//    if (f == NULL) {
//        ESP_LOGE(TAG, "Failed to open file for writing");
//        return ESP_FAIL;
//    }
//    fprintf(f, data);
//    fclose(f);
//    ESP_LOGI(TAG, "File written");
//
//    return ESP_OK;
//}
//static esp_err_t sd_read_file(const char *path)
//{
//    ESP_LOGI(TAG, "Reading file %s", path);
//    FILE *f = fopen(path, "r");
//    if (f == NULL) {
//        ESP_LOGE(TAG, "Failed to open file for reading");
//        return ESP_FAIL;
//    }
//    char line[64];
//    fgets(line, sizeof(line), f);
//    fclose(f);
//
//    // strip newline
//    char *pos = strchr(line, '\n');
//    if (pos) {
//        *pos = '\0';
//    }
//    ESP_LOGI(TAG, "Read from file: '%s'", line);
//
//    return ESP_OK;
//}
/*
 * @brief this function mount the filesystem from the SPI microSD
 */
//static esp_err_t sd_mount(char* mount_point){
//	esp_vfs_fat_sdmmc_mount_config_t mount_config = {
//		#ifdef CONFIG_EXAMPLE_FORMAT_IF_MOUNT_FAILED
//		        .format_if_mount_failed = true,
//		#else
//		        .format_if_mount_failed = false,
//		#endif // EXAMPLE_FORMAT_IF_MOUNT_FAILED
//		        .max_files = 5,
//		        .allocation_unit_size = 16 * 1024
//		    };
//		sdmmc_card_t *card;
//		sdmmc_host_t host = SDSPI_HOST_DEFAULT();
//		esp_err_t ret = spi_bus_initialize(host.slot, &bus_cfg, SDSPI_DEFAULT_DMA);
//		if (ret != ESP_OK) {
//		        ESP_LOGE(TAG, "Failed to initialize bus.");
//		        return ret;
//		    }
//
//	    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
//	    slot_config.gpio_cs = PIN_NUM_CS;
//	    slot_config.host_id = host.slot;
//
//	    ESP_LOGI(TAG, "Mounting filesystem");
//	    ret = esp_vfs_fat_sdspi_mount(mount_point, &host, &slot_config, &mount_config, &card);
//
//	    if (ret != ESP_OK) {
//	        if (ret == ESP_FAIL) {
//	            ESP_LOGE(TAG, "Failed to mount filesystem. "
//	                     "If you want the card to be formatted, set the CONFIG_EXAMPLE_FORMAT_IF_MOUNT_FAILED menuconfig option.");
//	        } else {
//	            ESP_LOGE(TAG, "Failed to initialize the card (%s). "
//	                     "Make sure SD card lines have pull-up resistors in place.", esp_err_to_name(ret));
//	        }
//
//	        return ret;
//	    }
//	    sdmmc_card_print_info(stdout, card);
//	    return ret;
//}

/*
 * @brief Main Function. System initialization and execution
 */
void app_main(void)
{
	nvs_flash_init();
	esp_netif_init();
	init_wifi();
	gpio_set_direction(LED, GPIO_MODE_OUTPUT);
//	mlx_init();

//	esp_sntp_config_t config = ESP_NETIF_SNTP_DEFAULT_CONFIG("pool.ntp.org");
//	esp_netif_sntp_init(&config);

//	if (esp_netif_sntp_sync_wait(pdMS_TO_TICKS(100000)) != ESP_OK) {
//	    printf("Failed to update system time within 10s timeout");
//	}
//
//	sd_mount(mount_point);


	mqtt_app_start();

    //printf("Hello world!\n");
    /*esp_vfs_fat_sdcard_unmount(mount_point, card);
    ESP_LOGI(TAG, "Card unmounted");

    //deinitialize the bus after all devices are removed
    spi_bus_free(host.slot);*/

}
