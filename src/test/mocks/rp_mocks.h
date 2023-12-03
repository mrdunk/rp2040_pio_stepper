#include <stdint.h>

void tight_loop_contents();

uint64_t time_us_64();

void gpio_put(uint32_t gpio, int value);

typedef uint64_t mutex_t;
void mutex_enter_blocking(mutex_t *mtx);
