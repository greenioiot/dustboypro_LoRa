// Device Address (Modbus)
#define ID_WEATHER     1

// 7 in 1 Register Address (Modbus)
#define _WINDSP   500
#define _WINDDI   502
#define _HUM      504
#define _TEMP     505
#define _PM2_5    507
#define _PM10     508
#define _PRESS    509


uint16_t const Address_7IN1[7] = {
  _WINDSP,
  _WINDDI,
  _HUM,
  _TEMP,
  _PM2_5,
  _PM10,
  _PRESS
};
