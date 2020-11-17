#pragma once
// MESSAGE FLOWMETER PACKING

#define MAVLINK_MSG_ID_FLOWMETER 13050

MAVPACKED(
typedef struct __mavlink_flowmeter_t {
 float period_us; /*<  PWM input capture period .*/
 float frequency; /*<  PWM input capture frequency.*/
 float flowrate_raw; /*<  Original flow rate of flowmeter.uint: L/min*/
 float flowrate_filter; /*<  Filtered flow rate of flowmeter.uint: L/min*/
 float dosage_raw; /*<  Original dosage. uint: L*/
 float dosage_filter; /*<  Filtered dosage.uint: L */
 uint8_t status; /*<  flow meter status, see FLOWMETER_STATUS enum*/
}) mavlink_flowmeter_t;

#define MAVLINK_MSG_ID_FLOWMETER_LEN 25
#define MAVLINK_MSG_ID_FLOWMETER_MIN_LEN 25
#define MAVLINK_MSG_ID_13050_LEN 25
#define MAVLINK_MSG_ID_13050_MIN_LEN 25

#define MAVLINK_MSG_ID_FLOWMETER_CRC 31
#define MAVLINK_MSG_ID_13050_CRC 31



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_FLOWMETER { \
    13050, \
    "FLOWMETER", \
    7, \
    {  { "period_us", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_flowmeter_t, period_us) }, \
         { "frequency", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_flowmeter_t, frequency) }, \
         { "flowrate_raw", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_flowmeter_t, flowrate_raw) }, \
         { "flowrate_filter", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_flowmeter_t, flowrate_filter) }, \
         { "dosage_raw", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_flowmeter_t, dosage_raw) }, \
         { "dosage_filter", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_flowmeter_t, dosage_filter) }, \
         { "status", NULL, MAVLINK_TYPE_UINT8_T, 0, 24, offsetof(mavlink_flowmeter_t, status) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_FLOWMETER { \
    "FLOWMETER", \
    7, \
    {  { "period_us", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_flowmeter_t, period_us) }, \
         { "frequency", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_flowmeter_t, frequency) }, \
         { "flowrate_raw", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_flowmeter_t, flowrate_raw) }, \
         { "flowrate_filter", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_flowmeter_t, flowrate_filter) }, \
         { "dosage_raw", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_flowmeter_t, dosage_raw) }, \
         { "dosage_filter", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_flowmeter_t, dosage_filter) }, \
         { "status", NULL, MAVLINK_TYPE_UINT8_T, 0, 24, offsetof(mavlink_flowmeter_t, status) }, \
         } \
}
#endif

/**
 * @brief Pack a flowmeter message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param period_us  PWM input capture period .
 * @param frequency  PWM input capture frequency.
 * @param flowrate_raw  Original flow rate of flowmeter.uint: L/min
 * @param flowrate_filter  Filtered flow rate of flowmeter.uint: L/min
 * @param dosage_raw  Original dosage. uint: L
 * @param dosage_filter  Filtered dosage.uint: L 
 * @param status  flow meter status, see FLOWMETER_STATUS enum
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_flowmeter_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float period_us, float frequency, float flowrate_raw, float flowrate_filter, float dosage_raw, float dosage_filter, uint8_t status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_FLOWMETER_LEN];
    _mav_put_float(buf, 0, period_us);
    _mav_put_float(buf, 4, frequency);
    _mav_put_float(buf, 8, flowrate_raw);
    _mav_put_float(buf, 12, flowrate_filter);
    _mav_put_float(buf, 16, dosage_raw);
    _mav_put_float(buf, 20, dosage_filter);
    _mav_put_uint8_t(buf, 24, status);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_FLOWMETER_LEN);
#else
    mavlink_flowmeter_t packet;
    packet.period_us = period_us;
    packet.frequency = frequency;
    packet.flowrate_raw = flowrate_raw;
    packet.flowrate_filter = flowrate_filter;
    packet.dosage_raw = dosage_raw;
    packet.dosage_filter = dosage_filter;
    packet.status = status;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_FLOWMETER_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_FLOWMETER;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_FLOWMETER_MIN_LEN, MAVLINK_MSG_ID_FLOWMETER_LEN, MAVLINK_MSG_ID_FLOWMETER_CRC);
}

/**
 * @brief Pack a flowmeter message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param period_us  PWM input capture period .
 * @param frequency  PWM input capture frequency.
 * @param flowrate_raw  Original flow rate of flowmeter.uint: L/min
 * @param flowrate_filter  Filtered flow rate of flowmeter.uint: L/min
 * @param dosage_raw  Original dosage. uint: L
 * @param dosage_filter  Filtered dosage.uint: L 
 * @param status  flow meter status, see FLOWMETER_STATUS enum
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_flowmeter_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float period_us,float frequency,float flowrate_raw,float flowrate_filter,float dosage_raw,float dosage_filter,uint8_t status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_FLOWMETER_LEN];
    _mav_put_float(buf, 0, period_us);
    _mav_put_float(buf, 4, frequency);
    _mav_put_float(buf, 8, flowrate_raw);
    _mav_put_float(buf, 12, flowrate_filter);
    _mav_put_float(buf, 16, dosage_raw);
    _mav_put_float(buf, 20, dosage_filter);
    _mav_put_uint8_t(buf, 24, status);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_FLOWMETER_LEN);
#else
    mavlink_flowmeter_t packet;
    packet.period_us = period_us;
    packet.frequency = frequency;
    packet.flowrate_raw = flowrate_raw;
    packet.flowrate_filter = flowrate_filter;
    packet.dosage_raw = dosage_raw;
    packet.dosage_filter = dosage_filter;
    packet.status = status;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_FLOWMETER_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_FLOWMETER;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_FLOWMETER_MIN_LEN, MAVLINK_MSG_ID_FLOWMETER_LEN, MAVLINK_MSG_ID_FLOWMETER_CRC);
}

/**
 * @brief Encode a flowmeter struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param flowmeter C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_flowmeter_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_flowmeter_t* flowmeter)
{
    return mavlink_msg_flowmeter_pack(system_id, component_id, msg, flowmeter->period_us, flowmeter->frequency, flowmeter->flowrate_raw, flowmeter->flowrate_filter, flowmeter->dosage_raw, flowmeter->dosage_filter, flowmeter->status);
}

/**
 * @brief Encode a flowmeter struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param flowmeter C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_flowmeter_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_flowmeter_t* flowmeter)
{
    return mavlink_msg_flowmeter_pack_chan(system_id, component_id, chan, msg, flowmeter->period_us, flowmeter->frequency, flowmeter->flowrate_raw, flowmeter->flowrate_filter, flowmeter->dosage_raw, flowmeter->dosage_filter, flowmeter->status);
}

/**
 * @brief Send a flowmeter message
 * @param chan MAVLink channel to send the message
 *
 * @param period_us  PWM input capture period .
 * @param frequency  PWM input capture frequency.
 * @param flowrate_raw  Original flow rate of flowmeter.uint: L/min
 * @param flowrate_filter  Filtered flow rate of flowmeter.uint: L/min
 * @param dosage_raw  Original dosage. uint: L
 * @param dosage_filter  Filtered dosage.uint: L 
 * @param status  flow meter status, see FLOWMETER_STATUS enum
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_flowmeter_send(mavlink_channel_t chan, float period_us, float frequency, float flowrate_raw, float flowrate_filter, float dosage_raw, float dosage_filter, uint8_t status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_FLOWMETER_LEN];
    _mav_put_float(buf, 0, period_us);
    _mav_put_float(buf, 4, frequency);
    _mav_put_float(buf, 8, flowrate_raw);
    _mav_put_float(buf, 12, flowrate_filter);
    _mav_put_float(buf, 16, dosage_raw);
    _mav_put_float(buf, 20, dosage_filter);
    _mav_put_uint8_t(buf, 24, status);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FLOWMETER, buf, MAVLINK_MSG_ID_FLOWMETER_MIN_LEN, MAVLINK_MSG_ID_FLOWMETER_LEN, MAVLINK_MSG_ID_FLOWMETER_CRC);
#else
    mavlink_flowmeter_t packet;
    packet.period_us = period_us;
    packet.frequency = frequency;
    packet.flowrate_raw = flowrate_raw;
    packet.flowrate_filter = flowrate_filter;
    packet.dosage_raw = dosage_raw;
    packet.dosage_filter = dosage_filter;
    packet.status = status;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FLOWMETER, (const char *)&packet, MAVLINK_MSG_ID_FLOWMETER_MIN_LEN, MAVLINK_MSG_ID_FLOWMETER_LEN, MAVLINK_MSG_ID_FLOWMETER_CRC);
#endif
}

/**
 * @brief Send a flowmeter message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_flowmeter_send_struct(mavlink_channel_t chan, const mavlink_flowmeter_t* flowmeter)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_flowmeter_send(chan, flowmeter->period_us, flowmeter->frequency, flowmeter->flowrate_raw, flowmeter->flowrate_filter, flowmeter->dosage_raw, flowmeter->dosage_filter, flowmeter->status);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FLOWMETER, (const char *)flowmeter, MAVLINK_MSG_ID_FLOWMETER_MIN_LEN, MAVLINK_MSG_ID_FLOWMETER_LEN, MAVLINK_MSG_ID_FLOWMETER_CRC);
#endif
}

#if MAVLINK_MSG_ID_FLOWMETER_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_flowmeter_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float period_us, float frequency, float flowrate_raw, float flowrate_filter, float dosage_raw, float dosage_filter, uint8_t status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, period_us);
    _mav_put_float(buf, 4, frequency);
    _mav_put_float(buf, 8, flowrate_raw);
    _mav_put_float(buf, 12, flowrate_filter);
    _mav_put_float(buf, 16, dosage_raw);
    _mav_put_float(buf, 20, dosage_filter);
    _mav_put_uint8_t(buf, 24, status);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FLOWMETER, buf, MAVLINK_MSG_ID_FLOWMETER_MIN_LEN, MAVLINK_MSG_ID_FLOWMETER_LEN, MAVLINK_MSG_ID_FLOWMETER_CRC);
#else
    mavlink_flowmeter_t *packet = (mavlink_flowmeter_t *)msgbuf;
    packet->period_us = period_us;
    packet->frequency = frequency;
    packet->flowrate_raw = flowrate_raw;
    packet->flowrate_filter = flowrate_filter;
    packet->dosage_raw = dosage_raw;
    packet->dosage_filter = dosage_filter;
    packet->status = status;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FLOWMETER, (const char *)packet, MAVLINK_MSG_ID_FLOWMETER_MIN_LEN, MAVLINK_MSG_ID_FLOWMETER_LEN, MAVLINK_MSG_ID_FLOWMETER_CRC);
#endif
}
#endif

#endif

// MESSAGE FLOWMETER UNPACKING


/**
 * @brief Get field period_us from flowmeter message
 *
 * @return  PWM input capture period .
 */
static inline float mavlink_msg_flowmeter_get_period_us(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field frequency from flowmeter message
 *
 * @return  PWM input capture frequency.
 */
static inline float mavlink_msg_flowmeter_get_frequency(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field flowrate_raw from flowmeter message
 *
 * @return  Original flow rate of flowmeter.uint: L/min
 */
static inline float mavlink_msg_flowmeter_get_flowrate_raw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field flowrate_filter from flowmeter message
 *
 * @return  Filtered flow rate of flowmeter.uint: L/min
 */
static inline float mavlink_msg_flowmeter_get_flowrate_filter(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field dosage_raw from flowmeter message
 *
 * @return  Original dosage. uint: L
 */
static inline float mavlink_msg_flowmeter_get_dosage_raw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field dosage_filter from flowmeter message
 *
 * @return  Filtered dosage.uint: L 
 */
static inline float mavlink_msg_flowmeter_get_dosage_filter(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field status from flowmeter message
 *
 * @return  flow meter status, see FLOWMETER_STATUS enum
 */
static inline uint8_t mavlink_msg_flowmeter_get_status(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  24);
}

/**
 * @brief Decode a flowmeter message into a struct
 *
 * @param msg The message to decode
 * @param flowmeter C-struct to decode the message contents into
 */
static inline void mavlink_msg_flowmeter_decode(const mavlink_message_t* msg, mavlink_flowmeter_t* flowmeter)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    flowmeter->period_us = mavlink_msg_flowmeter_get_period_us(msg);
    flowmeter->frequency = mavlink_msg_flowmeter_get_frequency(msg);
    flowmeter->flowrate_raw = mavlink_msg_flowmeter_get_flowrate_raw(msg);
    flowmeter->flowrate_filter = mavlink_msg_flowmeter_get_flowrate_filter(msg);
    flowmeter->dosage_raw = mavlink_msg_flowmeter_get_dosage_raw(msg);
    flowmeter->dosage_filter = mavlink_msg_flowmeter_get_dosage_filter(msg);
    flowmeter->status = mavlink_msg_flowmeter_get_status(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_FLOWMETER_LEN? msg->len : MAVLINK_MSG_ID_FLOWMETER_LEN;
        memset(flowmeter, 0, MAVLINK_MSG_ID_FLOWMETER_LEN);
    memcpy(flowmeter, _MAV_PAYLOAD(msg), len);
#endif
}
