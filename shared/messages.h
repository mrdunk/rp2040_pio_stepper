#ifndef UPDATE_TYPES__H
#define UPDATE_TYPES__H


#define MSG_NONE                     0
#define MSG_SET_GLOAL_UPDATE_RATE    1
#define MSG_SET_AXIS_ABS_POS         2
#define MSG_SET_AXIS_REL_POS         3
#define MSG_SET_AXIS_MAX_SPEED       4
#define MSG_SET_AXIS_MAX_ACCEL       5
#define MSG_SET_AXIS_ABS_POS_AT_TIME 6

#define MSG_GET_GLOBAL_CONFIG        7
#define MSG_GET_AXIS_CONFIG          8
#define MSG_GET_AXIS_POS             9

struct Message {
  uint type;
};

struct Message_uint {
  uint type;
  uint value0;
};

struct Message_uint_uint {
  uint type;
  uint value0;
  uint value1;
};

#endif  // UPDATE_TYPES__H
