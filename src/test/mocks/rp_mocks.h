#include <stdint.h>

#define GPIO_IN 0
#define GPIO_OUT 0

void gpio_init (uint8_t gpio);
void gpio_set_dir (uint8_t gpio, uint8_t out);
void gpio_pull_up (uint8_t gpio);

void tight_loop_contents();

uint64_t time_us_64();

void gpio_put(uint32_t gpio, int value);
int gpio_get(uint32_t gpio);

typedef uint64_t mutex_t;
void mutex_enter_blocking(mutex_t *mtx);

void mutex_exit(mutex_t *mtx);

void mutex_init(mutex_t *mtx);
