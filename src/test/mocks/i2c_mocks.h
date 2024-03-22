#ifndef _MOCK_I2C_H
#define _MOCK_I2C_H



typedef struct timeout_state {
    uint64_t param;
} timeout_state_t;

typedef bool (*check_timeout_fn)(timeout_state_t *ts);

typedef struct {
    int enable;
} i2c_hw_t;

struct i2c_inst {
    i2c_hw_t *hw;
    bool restart_on_next;
};

typedef struct i2c_inst i2c_inst_t;


#endif  // _MOCK_I2C_H
