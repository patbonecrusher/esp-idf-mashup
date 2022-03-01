
// #include <driver/gpio.h>
// // Include FreeRTOS for delay
// #include <freertos/FreeRTOS.h>
// #include <freertos/task.h>

// #ifndef LED_BUILTIN
// #define LED_BUILTIN 13
// #endif

// #define LED 5 // LED connected to GPIO2
// int app_main() {
//     // Configure pin
//     gpio_config_t io_conf;
//     io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
//     io_conf.mode = GPIO_MODE_OUTPUT;
//     io_conf.pin_bit_mask = (1ULL << LED);
//     io_conf.pull_down_en = 0;
//     io_conf.pull_up_en = 0;
//     gpio_config(&io_conf);
//     // Main loop
//     while(true) {
//         gpio_set_level(LED, 0);
//         vTaskDelay(500 / portTICK_RATE_MS);
//         gpio_set_level(LED, 1);
//         vTaskDelay(500 / portTICK_RATE_MS);
//     }
// }

/*
 * JoB ESP32 LEDC breathe example
 *
 * Setup LEDC so led fades in and out repeatedly without further intervention.
 */

#include <stdio.h>                   // printf()
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"           // for task and timing
#include "driver/ledc.h"             // for ledc led fading api

// #define LED_PIN   5
// #define CHANNEL   LEDC_CHANNEL_0
// #define MODE      LEDC_HIGH_SPEED_MODE
// #define TIMER     LEDC_TIMER_0
// #define FADE_MS   4000
// #define DUTY_BITS 12

// #define MAX_DUTY  (1<<DUTY_BITS)

// startregion: a
#define LEDC_CHANNELS           (6)
#define LEDC_DEFAULT_CLK         0
#define log_e(format, ...) do {} while(0)

uint8_t channels_resolution[LEDC_CHANNELS] = {0};
double ledcSetup(uint8_t chan, double freq, uint8_t bit_num)
{
    if(chan >= LEDC_CHANNELS){
        log_e("No more LEDC channels available! You can have maximum %u", LEDC_CHANNELS);
        return 0;
    }
    uint8_t group=(chan/8), timer=((chan/2)%4);

    ledc_timer_config_t ledc_timer = {
        .speed_mode       = group,
        .timer_num        = timer,
        .duty_resolution  = bit_num,
        .freq_hz          = freq,
        .clk_cfg          = LEDC_DEFAULT_CLK
    };
    ledc_timer_config(&ledc_timer);
    channels_resolution[chan] = bit_num;

    return ledc_get_freq(group,timer);
}

void ledcAttachPin(uint8_t pin, uint8_t chan)
{
     if(chan >= LEDC_CHANNELS){
        return;
    }
    uint8_t group=(chan/8), channel=(chan%8), timer=((chan/2)%4);
    
    ledc_channel_config_t ledc_channel = {
        .speed_mode     = group,
        .channel        = channel,
        .timer_sel      = timer,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = pin,
        .duty           = 0,
        .hpoint         = 0
    };
    ledc_channel_config(&ledc_channel);
}


unsigned long IRAM_ATTR millis()
{
    return (unsigned long) (esp_timer_get_time() / 1000ULL);
}

void ledcWrite(uint8_t chan, uint32_t duty)
{
    if(chan >= LEDC_CHANNELS){
        return;
    }
    uint8_t group=(chan/8), channel=(chan%8);

    //Fixing if all bits in resolution is set = LEDC FULL ON
    uint32_t max_duty = (1 << channels_resolution[chan]) - 1;

    if(duty == max_duty){
        duty = max_duty + 1;
    }

    ledc_set_duty(group, channel, duty);
    ledc_update_duty(group, channel);
}

// endregion

const int freq = 5000;
const int ledChannel = 0;
const int resolution = 8;
void ledc_init(int pin, int ledChannel) {
  ledcSetup(ledChannel, freq, resolution);
  ledcAttachPin(pin, ledChannel);
}

void show_timing( bool inverted ) {
  static TickType_t started = 0;
  TickType_t now = xTaskGetTickCount();
  printf("%u ms: fading %s\n", (now-started)*portTICK_PERIOD_MS, inverted ? "out" : "in");
  started = now;
}

void ledc_fade() {
	float val;
	for(;;) {
		val = (exp(sin(millis()/2000.0*M_PI)) - 0.36787944)*108.0;
		printf("%f\n", val);
		ledcWrite(ledChannel, val);
		 vTaskDelay(1);
	}
}

void app_main()
{
  printf("Hello Breathe LEDC!\n");

  ledc_init(5, ledChannel);

  printf("Start LEDC Breathing\n");

  xTaskCreate(ledc_fade, "ledc_fade", 24*configMINIMAL_STACK_SIZE, NULL, 2, NULL);


  printf("LEDC Task running\n");
}


class LedChannel {

};