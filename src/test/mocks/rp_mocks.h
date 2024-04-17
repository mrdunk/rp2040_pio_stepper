#ifndef MOCKS_RP_MOCKS__H
#define MOCKS_RP_MOCKS__H


#include <stdint.h>
#include <stddef.h>

#define GPIO_IN 0
#define GPIO_OUT 0

#define UART_UARTFR_BUSY_BITS 0

#define uart1 NULL

enum  gpio_function {
  GPIO_FUNC_XIP = 0 , GPIO_FUNC_SPI = 1 , GPIO_FUNC_UART = 2 , GPIO_FUNC_I2C = 3 ,
  GPIO_FUNC_PWM = 4 , GPIO_FUNC_SIO = 5 , GPIO_FUNC_PIO0 = 6 , GPIO_FUNC_PIO1 = 7 ,
  GPIO_FUNC_GPCK = 8 , GPIO_FUNC_USB = 9 , GPIO_FUNC_NULL = 0x1f
};


void gpio_init (uint8_t gpio);
void gpio_set_dir (uint8_t gpio, uint8_t out);
void gpio_pull_up (uint8_t gpio);
void gpio_set_function (size_t gpio, enum gpio_function fn);

void tight_loop_contents();

uint64_t time_us_64();

void gpio_put(uint32_t gpio, int value);
int gpio_get(uint32_t gpio);

typedef uint64_t mutex_t;
void mutex_enter_blocking(mutex_t *mtx);

void mutex_exit(mutex_t *mtx);

void mutex_init(mutex_t *mtx);

void uart_tx_wait_blocking (size_t *uart);
void uart_write_blocking (size_t *uart, const uint8_t *src, size_t len);
size_t uart_init (size_t *uart, size_t baudrate);
void uart_deinit (size_t *uart);
uint8_t uart_is_readable (size_t *uart);
char uart_getc (size_t *uart);

struct uart_hw_t {
  size_t fr;
};
struct uart_hw_t *uart_get_hw(size_t *uart);



#endif  // MOCKS_RP_MOCKS__H
