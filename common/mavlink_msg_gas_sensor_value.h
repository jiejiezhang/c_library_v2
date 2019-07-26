#pragma once
// MESSAGE GAS_SENSOR_VALUE PACKING

#define MAVLINK_MSG_ID_GAS_SENSOR_VALUE 10000

MAVPACKED(
typedef struct __mavlink_gas_sensor_value_t {
 uint32_t SO2; /*<  The unit is ppb*/
 uint32_t VOC; /*<  The unit is ppb*/
 uint32_t NO2; /*<  The unit is ppb*/
 uint32_t CO; /*<  The unit is ppm*/
 uint32_t NH3; /*<  The unit is ppm*/
 uint32_t O3; /*<  The unit is ppb*/
 uint32_t PRESSURE; /*<  The unit is pa*/
 uint16_t PM25; /*<  The unit is ug/cm3*/
 uint16_t PM10; /*<  The unit is ug/cm3*/
 uint16_t TEMPERATURE; /*<  The unit is C*/
 uint8_t HUMIDITY; /*<  The unit is %RH*/
}) mavlink_gas_sensor_value_t;

#define MAVLINK_MSG_ID_GAS_SENSOR_VALUE_LEN 35
#define MAVLINK_MSG_ID_GAS_SENSOR_VALUE_MIN_LEN 35
#define MAVLINK_MSG_ID_10000_LEN 35
#define MAVLINK_MSG_ID_10000_MIN_LEN 35

#define MAVLINK_MSG_ID_GAS_SENSOR_VALUE_CRC 108
#define MAVLINK_MSG_ID_10000_CRC 108



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_GAS_SENSOR_VALUE { \
    10000, \
    "GAS_SENSOR_VALUE", \
    11, \
    {  { "PM25", NULL, MAVLINK_TYPE_UINT16_T, 0, 28, offsetof(mavlink_gas_sensor_value_t, PM25) }, \
         { "PM10", NULL, MAVLINK_TYPE_UINT16_T, 0, 30, offsetof(mavlink_gas_sensor_value_t, PM10) }, \
         { "SO2", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_gas_sensor_value_t, SO2) }, \
         { "VOC", NULL, MAVLINK_TYPE_UINT32_T, 0, 4, offsetof(mavlink_gas_sensor_value_t, VOC) }, \
         { "NO2", NULL, MAVLINK_TYPE_UINT32_T, 0, 8, offsetof(mavlink_gas_sensor_value_t, NO2) }, \
         { "CO", NULL, MAVLINK_TYPE_UINT32_T, 0, 12, offsetof(mavlink_gas_sensor_value_t, CO) }, \
         { "NH3", NULL, MAVLINK_TYPE_UINT32_T, 0, 16, offsetof(mavlink_gas_sensor_value_t, NH3) }, \
         { "O3", NULL, MAVLINK_TYPE_UINT32_T, 0, 20, offsetof(mavlink_gas_sensor_value_t, O3) }, \
         { "PRESSURE", NULL, MAVLINK_TYPE_UINT32_T, 0, 24, offsetof(mavlink_gas_sensor_value_t, PRESSURE) }, \
         { "HUMIDITY", NULL, MAVLINK_TYPE_UINT8_T, 0, 34, offsetof(mavlink_gas_sensor_value_t, HUMIDITY) }, \
         { "TEMPERATURE", NULL, MAVLINK_TYPE_UINT16_T, 0, 32, offsetof(mavlink_gas_sensor_value_t, TEMPERATURE) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_GAS_SENSOR_VALUE { \
    "GAS_SENSOR_VALUE", \
    11, \
    {  { "PM25", NULL, MAVLINK_TYPE_UINT16_T, 0, 28, offsetof(mavlink_gas_sensor_value_t, PM25) }, \
         { "PM10", NULL, MAVLINK_TYPE_UINT16_T, 0, 30, offsetof(mavlink_gas_sensor_value_t, PM10) }, \
         { "SO2", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_gas_sensor_value_t, SO2) }, \
         { "VOC", NULL, MAVLINK_TYPE_UINT32_T, 0, 4, offsetof(mavlink_gas_sensor_value_t, VOC) }, \
         { "NO2", NULL, MAVLINK_TYPE_UINT32_T, 0, 8, offsetof(mavlink_gas_sensor_value_t, NO2) }, \
         { "CO", NULL, MAVLINK_TYPE_UINT32_T, 0, 12, offsetof(mavlink_gas_sensor_value_t, CO) }, \
         { "NH3", NULL, MAVLINK_TYPE_UINT32_T, 0, 16, offsetof(mavlink_gas_sensor_value_t, NH3) }, \
         { "O3", NULL, MAVLINK_TYPE_UINT32_T, 0, 20, offsetof(mavlink_gas_sensor_value_t, O3) }, \
         { "PRESSURE", NULL, MAVLINK_TYPE_UINT32_T, 0, 24, offsetof(mavlink_gas_sensor_value_t, PRESSURE) }, \
         { "HUMIDITY", NULL, MAVLINK_TYPE_UINT8_T, 0, 34, offsetof(mavlink_gas_sensor_value_t, HUMIDITY) }, \
         { "TEMPERATURE", NULL, MAVLINK_TYPE_UINT16_T, 0, 32, offsetof(mavlink_gas_sensor_value_t, TEMPERATURE) }, \
         } \
}
#endif

/**
 * @brief Pack a gas_sensor_value message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param PM25  The unit is ug/cm3
 * @param PM10  The unit is ug/cm3
 * @param SO2  The unit is ppb
 * @param VOC  The unit is ppb
 * @param NO2  The unit is ppb
 * @param CO  The unit is ppm
 * @param NH3  The unit is ppm
 * @param O3  The unit is ppb
 * @param PRESSURE  The unit is pa
 * @param HUMIDITY  The unit is %RH
 * @param TEMPERATURE  The unit is C
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gas_sensor_value_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint16_t PM25, uint16_t PM10, uint32_t SO2, uint32_t VOC, uint32_t NO2, uint32_t CO, uint32_t NH3, uint32_t O3, uint32_t PRESSURE, uint8_t HUMIDITY, uint16_t TEMPERATURE)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GAS_SENSOR_VALUE_LEN];
    _mav_put_uint32_t(buf, 0, SO2);
    _mav_put_uint32_t(buf, 4, VOC);
    _mav_put_uint32_t(buf, 8, NO2);
    _mav_put_uint32_t(buf, 12, CO);
    _mav_put_uint32_t(buf, 16, NH3);
    _mav_put_uint32_t(buf, 20, O3);
    _mav_put_uint32_t(buf, 24, PRESSURE);
    _mav_put_uint16_t(buf, 28, PM25);
    _mav_put_uint16_t(buf, 30, PM10);
    _mav_put_uint16_t(buf, 32, TEMPERATURE);
    _mav_put_uint8_t(buf, 34, HUMIDITY);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GAS_SENSOR_VALUE_LEN);
#else
    mavlink_gas_sensor_value_t packet;
    packet.SO2 = SO2;
    packet.VOC = VOC;
    packet.NO2 = NO2;
    packet.CO = CO;
    packet.NH3 = NH3;
    packet.O3 = O3;
    packet.PRESSURE = PRESSURE;
    packet.PM25 = PM25;
    packet.PM10 = PM10;
    packet.TEMPERATURE = TEMPERATURE;
    packet.HUMIDITY = HUMIDITY;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GAS_SENSOR_VALUE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_GAS_SENSOR_VALUE;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_GAS_SENSOR_VALUE_MIN_LEN, MAVLINK_MSG_ID_GAS_SENSOR_VALUE_LEN, MAVLINK_MSG_ID_GAS_SENSOR_VALUE_CRC);
}

/**
 * @brief Pack a gas_sensor_value message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param PM25  The unit is ug/cm3
 * @param PM10  The unit is ug/cm3
 * @param SO2  The unit is ppb
 * @param VOC  The unit is ppb
 * @param NO2  The unit is ppb
 * @param CO  The unit is ppm
 * @param NH3  The unit is ppm
 * @param O3  The unit is ppb
 * @param PRESSURE  The unit is pa
 * @param HUMIDITY  The unit is %RH
 * @param TEMPERATURE  The unit is C
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gas_sensor_value_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint16_t PM25,uint16_t PM10,uint32_t SO2,uint32_t VOC,uint32_t NO2,uint32_t CO,uint32_t NH3,uint32_t O3,uint32_t PRESSURE,uint8_t HUMIDITY,uint16_t TEMPERATURE)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GAS_SENSOR_VALUE_LEN];
    _mav_put_uint32_t(buf, 0, SO2);
    _mav_put_uint32_t(buf, 4, VOC);
    _mav_put_uint32_t(buf, 8, NO2);
    _mav_put_uint32_t(buf, 12, CO);
    _mav_put_uint32_t(buf, 16, NH3);
    _mav_put_uint32_t(buf, 20, O3);
    _mav_put_uint32_t(buf, 24, PRESSURE);
    _mav_put_uint16_t(buf, 28, PM25);
    _mav_put_uint16_t(buf, 30, PM10);
    _mav_put_uint16_t(buf, 32, TEMPERATURE);
    _mav_put_uint8_t(buf, 34, HUMIDITY);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GAS_SENSOR_VALUE_LEN);
#else
    mavlink_gas_sensor_value_t packet;
    packet.SO2 = SO2;
    packet.VOC = VOC;
    packet.NO2 = NO2;
    packet.CO = CO;
    packet.NH3 = NH3;
    packet.O3 = O3;
    packet.PRESSURE = PRESSURE;
    packet.PM25 = PM25;
    packet.PM10 = PM10;
    packet.TEMPERATURE = TEMPERATURE;
    packet.HUMIDITY = HUMIDITY;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GAS_SENSOR_VALUE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_GAS_SENSOR_VALUE;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_GAS_SENSOR_VALUE_MIN_LEN, MAVLINK_MSG_ID_GAS_SENSOR_VALUE_LEN, MAVLINK_MSG_ID_GAS_SENSOR_VALUE_CRC);
}

/**
 * @brief Encode a gas_sensor_value struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param gas_sensor_value C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gas_sensor_value_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_gas_sensor_value_t* gas_sensor_value)
{
    return mavlink_msg_gas_sensor_value_pack(system_id, component_id, msg, gas_sensor_value->PM25, gas_sensor_value->PM10, gas_sensor_value->SO2, gas_sensor_value->VOC, gas_sensor_value->NO2, gas_sensor_value->CO, gas_sensor_value->NH3, gas_sensor_value->O3, gas_sensor_value->PRESSURE, gas_sensor_value->HUMIDITY, gas_sensor_value->TEMPERATURE);
}

/**
 * @brief Encode a gas_sensor_value struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param gas_sensor_value C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gas_sensor_value_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_gas_sensor_value_t* gas_sensor_value)
{
    return mavlink_msg_gas_sensor_value_pack_chan(system_id, component_id, chan, msg, gas_sensor_value->PM25, gas_sensor_value->PM10, gas_sensor_value->SO2, gas_sensor_value->VOC, gas_sensor_value->NO2, gas_sensor_value->CO, gas_sensor_value->NH3, gas_sensor_value->O3, gas_sensor_value->PRESSURE, gas_sensor_value->HUMIDITY, gas_sensor_value->TEMPERATURE);
}

/**
 * @brief Send a gas_sensor_value message
 * @param chan MAVLink channel to send the message
 *
 * @param PM25  The unit is ug/cm3
 * @param PM10  The unit is ug/cm3
 * @param SO2  The unit is ppb
 * @param VOC  The unit is ppb
 * @param NO2  The unit is ppb
 * @param CO  The unit is ppm
 * @param NH3  The unit is ppm
 * @param O3  The unit is ppb
 * @param PRESSURE  The unit is pa
 * @param HUMIDITY  The unit is %RH
 * @param TEMPERATURE  The unit is C
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_gas_sensor_value_send(mavlink_channel_t chan, uint16_t PM25, uint16_t PM10, uint32_t SO2, uint32_t VOC, uint32_t NO2, uint32_t CO, uint32_t NH3, uint32_t O3, uint32_t PRESSURE, uint8_t HUMIDITY, uint16_t TEMPERATURE)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GAS_SENSOR_VALUE_LEN];
    _mav_put_uint32_t(buf, 0, SO2);
    _mav_put_uint32_t(buf, 4, VOC);
    _mav_put_uint32_t(buf, 8, NO2);
    _mav_put_uint32_t(buf, 12, CO);
    _mav_put_uint32_t(buf, 16, NH3);
    _mav_put_uint32_t(buf, 20, O3);
    _mav_put_uint32_t(buf, 24, PRESSURE);
    _mav_put_uint16_t(buf, 28, PM25);
    _mav_put_uint16_t(buf, 30, PM10);
    _mav_put_uint16_t(buf, 32, TEMPERATURE);
    _mav_put_uint8_t(buf, 34, HUMIDITY);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GAS_SENSOR_VALUE, buf, MAVLINK_MSG_ID_GAS_SENSOR_VALUE_MIN_LEN, MAVLINK_MSG_ID_GAS_SENSOR_VALUE_LEN, MAVLINK_MSG_ID_GAS_SENSOR_VALUE_CRC);
#else
    mavlink_gas_sensor_value_t packet;
    packet.SO2 = SO2;
    packet.VOC = VOC;
    packet.NO2 = NO2;
    packet.CO = CO;
    packet.NH3 = NH3;
    packet.O3 = O3;
    packet.PRESSURE = PRESSURE;
    packet.PM25 = PM25;
    packet.PM10 = PM10;
    packet.TEMPERATURE = TEMPERATURE;
    packet.HUMIDITY = HUMIDITY;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GAS_SENSOR_VALUE, (const char *)&packet, MAVLINK_MSG_ID_GAS_SENSOR_VALUE_MIN_LEN, MAVLINK_MSG_ID_GAS_SENSOR_VALUE_LEN, MAVLINK_MSG_ID_GAS_SENSOR_VALUE_CRC);
#endif
}

/**
 * @brief Send a gas_sensor_value message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_gas_sensor_value_send_struct(mavlink_channel_t chan, const mavlink_gas_sensor_value_t* gas_sensor_value)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_gas_sensor_value_send(chan, gas_sensor_value->PM25, gas_sensor_value->PM10, gas_sensor_value->SO2, gas_sensor_value->VOC, gas_sensor_value->NO2, gas_sensor_value->CO, gas_sensor_value->NH3, gas_sensor_value->O3, gas_sensor_value->PRESSURE, gas_sensor_value->HUMIDITY, gas_sensor_value->TEMPERATURE);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GAS_SENSOR_VALUE, (const char *)gas_sensor_value, MAVLINK_MSG_ID_GAS_SENSOR_VALUE_MIN_LEN, MAVLINK_MSG_ID_GAS_SENSOR_VALUE_LEN, MAVLINK_MSG_ID_GAS_SENSOR_VALUE_CRC);
#endif
}

#if MAVLINK_MSG_ID_GAS_SENSOR_VALUE_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_gas_sensor_value_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint16_t PM25, uint16_t PM10, uint32_t SO2, uint32_t VOC, uint32_t NO2, uint32_t CO, uint32_t NH3, uint32_t O3, uint32_t PRESSURE, uint8_t HUMIDITY, uint16_t TEMPERATURE)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, SO2);
    _mav_put_uint32_t(buf, 4, VOC);
    _mav_put_uint32_t(buf, 8, NO2);
    _mav_put_uint32_t(buf, 12, CO);
    _mav_put_uint32_t(buf, 16, NH3);
    _mav_put_uint32_t(buf, 20, O3);
    _mav_put_uint32_t(buf, 24, PRESSURE);
    _mav_put_uint16_t(buf, 28, PM25);
    _mav_put_uint16_t(buf, 30, PM10);
    _mav_put_uint16_t(buf, 32, TEMPERATURE);
    _mav_put_uint8_t(buf, 34, HUMIDITY);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GAS_SENSOR_VALUE, buf, MAVLINK_MSG_ID_GAS_SENSOR_VALUE_MIN_LEN, MAVLINK_MSG_ID_GAS_SENSOR_VALUE_LEN, MAVLINK_MSG_ID_GAS_SENSOR_VALUE_CRC);
#else
    mavlink_gas_sensor_value_t *packet = (mavlink_gas_sensor_value_t *)msgbuf;
    packet->SO2 = SO2;
    packet->VOC = VOC;
    packet->NO2 = NO2;
    packet->CO = CO;
    packet->NH3 = NH3;
    packet->O3 = O3;
    packet->PRESSURE = PRESSURE;
    packet->PM25 = PM25;
    packet->PM10 = PM10;
    packet->TEMPERATURE = TEMPERATURE;
    packet->HUMIDITY = HUMIDITY;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GAS_SENSOR_VALUE, (const char *)packet, MAVLINK_MSG_ID_GAS_SENSOR_VALUE_MIN_LEN, MAVLINK_MSG_ID_GAS_SENSOR_VALUE_LEN, MAVLINK_MSG_ID_GAS_SENSOR_VALUE_CRC);
#endif
}
#endif

#endif

// MESSAGE GAS_SENSOR_VALUE UNPACKING


/**
 * @brief Get field PM25 from gas_sensor_value message
 *
 * @return  The unit is ug/cm3
 */
static inline uint16_t mavlink_msg_gas_sensor_value_get_PM25(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  28);
}

/**
 * @brief Get field PM10 from gas_sensor_value message
 *
 * @return  The unit is ug/cm3
 */
static inline uint16_t mavlink_msg_gas_sensor_value_get_PM10(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  30);
}

/**
 * @brief Get field SO2 from gas_sensor_value message
 *
 * @return  The unit is ppb
 */
static inline uint32_t mavlink_msg_gas_sensor_value_get_SO2(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field VOC from gas_sensor_value message
 *
 * @return  The unit is ppb
 */
static inline uint32_t mavlink_msg_gas_sensor_value_get_VOC(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  4);
}

/**
 * @brief Get field NO2 from gas_sensor_value message
 *
 * @return  The unit is ppb
 */
static inline uint32_t mavlink_msg_gas_sensor_value_get_NO2(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  8);
}

/**
 * @brief Get field CO from gas_sensor_value message
 *
 * @return  The unit is ppm
 */
static inline uint32_t mavlink_msg_gas_sensor_value_get_CO(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  12);
}

/**
 * @brief Get field NH3 from gas_sensor_value message
 *
 * @return  The unit is ppm
 */
static inline uint32_t mavlink_msg_gas_sensor_value_get_NH3(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  16);
}

/**
 * @brief Get field O3 from gas_sensor_value message
 *
 * @return  The unit is ppb
 */
static inline uint32_t mavlink_msg_gas_sensor_value_get_O3(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  20);
}

/**
 * @brief Get field PRESSURE from gas_sensor_value message
 *
 * @return  The unit is pa
 */
static inline uint32_t mavlink_msg_gas_sensor_value_get_PRESSURE(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  24);
}

/**
 * @brief Get field HUMIDITY from gas_sensor_value message
 *
 * @return  The unit is %RH
 */
static inline uint8_t mavlink_msg_gas_sensor_value_get_HUMIDITY(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  34);
}

/**
 * @brief Get field TEMPERATURE from gas_sensor_value message
 *
 * @return  The unit is C
 */
static inline uint16_t mavlink_msg_gas_sensor_value_get_TEMPERATURE(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  32);
}

/**
 * @brief Decode a gas_sensor_value message into a struct
 *
 * @param msg The message to decode
 * @param gas_sensor_value C-struct to decode the message contents into
 */
static inline void mavlink_msg_gas_sensor_value_decode(const mavlink_message_t* msg, mavlink_gas_sensor_value_t* gas_sensor_value)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    gas_sensor_value->SO2 = mavlink_msg_gas_sensor_value_get_SO2(msg);
    gas_sensor_value->VOC = mavlink_msg_gas_sensor_value_get_VOC(msg);
    gas_sensor_value->NO2 = mavlink_msg_gas_sensor_value_get_NO2(msg);
    gas_sensor_value->CO = mavlink_msg_gas_sensor_value_get_CO(msg);
    gas_sensor_value->NH3 = mavlink_msg_gas_sensor_value_get_NH3(msg);
    gas_sensor_value->O3 = mavlink_msg_gas_sensor_value_get_O3(msg);
    gas_sensor_value->PRESSURE = mavlink_msg_gas_sensor_value_get_PRESSURE(msg);
    gas_sensor_value->PM25 = mavlink_msg_gas_sensor_value_get_PM25(msg);
    gas_sensor_value->PM10 = mavlink_msg_gas_sensor_value_get_PM10(msg);
    gas_sensor_value->TEMPERATURE = mavlink_msg_gas_sensor_value_get_TEMPERATURE(msg);
    gas_sensor_value->HUMIDITY = mavlink_msg_gas_sensor_value_get_HUMIDITY(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_GAS_SENSOR_VALUE_LEN? msg->len : MAVLINK_MSG_ID_GAS_SENSOR_VALUE_LEN;
        memset(gas_sensor_value, 0, MAVLINK_MSG_ID_GAS_SENSOR_VALUE_LEN);
    memcpy(gas_sensor_value, _MAV_PAYLOAD(msg), len);
#endif
}
