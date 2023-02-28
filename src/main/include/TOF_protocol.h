// TOF protocol version 1.0

enum TOF_RIO_msgs_enum {
  RESERVED = 0,
  RANGE = 1,
  HISTOGRAM = 2,
  ARM_ANGLE = 3,
  RAW_PIXEL_DATA = 4,
};

enum target_range_enum {
  TARGET_NOT_PRESENT = 0,
  TARGET_IN_RANGE = 1,
  TARGET_TOO_FAR = 2,
  TARGET_TOO_CLOSE = 3,
};

enum RIO_TOF_msgs_enum {
  RESERVED_RIO = 0,
  TARGET_TYPE = 1,
  HISTOGRAM_ENABLE = 2,
  ARM_ANGLE_OFFSET = 3,
  RAW_PIXEL_DATA_ENABLE = 4,
};

enum target_type_enum {
  CONE = 0,
  CUBE = 1,
} ;
