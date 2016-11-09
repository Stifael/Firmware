#pragma once
// MESSAGE AVOIDANCE_TRIPLET PACKING

#define MAVLINK_MSG_ID_AVOIDANCE_TRIPLET 235

MAVPACKED(
typedef struct __mavlink_avoidance_triplet_t {
 uint64_t time_usec; /*< Timestamp (micros since boot or Unix epoch)*/
 uint32_t gen_count; /*< Generation count for triplet. Increases monotonically and wraps once reached. This number is a copy of the last received planning triplet, which allows the system to update a planning triplet with avoidance data.*/
 float prev_x; /*< Local position X (NED) of previous position command in m*/
 float prev_y; /*< Local position Y (NED) of previous position command in m*/
 float prev_z; /*< Local position Z (NED) of previous position command in m*/
 float ctrl_x; /*< Local position X (NED) of control point in m*/
 float ctrl_y; /*< Local position Y (NED) of control point in m*/
 float ctrl_z; /*< Local position Z (NED) of control point in m*/
 float next_x; /*< Local position X (NED) of next position command in m*/
 float next_y; /*< Local position Y (NED) of next position command in m*/
 float next_z; /*< Local position Z (NED) of next position command in m*/
 float duration; /*< Desired time it takes to travel from prev to next in s*/
 float max_acc; /*< Maximum acceleration in m/s^2*/
 float acc_per_err; /*< Maximum acceleration in m/s^2 per meter error*/
 uint8_t target_system; /*< System ID*/
 uint8_t target_component; /*< Component ID*/
}) mavlink_avoidance_triplet_t;

#define MAVLINK_MSG_ID_AVOIDANCE_TRIPLET_LEN 62
#define MAVLINK_MSG_ID_AVOIDANCE_TRIPLET_MIN_LEN 62
#define MAVLINK_MSG_ID_235_LEN 62
#define MAVLINK_MSG_ID_235_MIN_LEN 62

#define MAVLINK_MSG_ID_AVOIDANCE_TRIPLET_CRC 63
#define MAVLINK_MSG_ID_235_CRC 63



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_AVOIDANCE_TRIPLET { \
    235, \
    "AVOIDANCE_TRIPLET", \
    16, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_avoidance_triplet_t, time_usec) }, \
         { "gen_count", NULL, MAVLINK_TYPE_UINT32_T, 0, 8, offsetof(mavlink_avoidance_triplet_t, gen_count) }, \
         { "prev_x", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_avoidance_triplet_t, prev_x) }, \
         { "prev_y", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_avoidance_triplet_t, prev_y) }, \
         { "prev_z", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_avoidance_triplet_t, prev_z) }, \
         { "ctrl_x", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_avoidance_triplet_t, ctrl_x) }, \
         { "ctrl_y", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_avoidance_triplet_t, ctrl_y) }, \
         { "ctrl_z", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_avoidance_triplet_t, ctrl_z) }, \
         { "next_x", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_avoidance_triplet_t, next_x) }, \
         { "next_y", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_avoidance_triplet_t, next_y) }, \
         { "next_z", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_avoidance_triplet_t, next_z) }, \
         { "duration", NULL, MAVLINK_TYPE_FLOAT, 0, 48, offsetof(mavlink_avoidance_triplet_t, duration) }, \
         { "max_acc", NULL, MAVLINK_TYPE_FLOAT, 0, 52, offsetof(mavlink_avoidance_triplet_t, max_acc) }, \
         { "acc_per_err", NULL, MAVLINK_TYPE_FLOAT, 0, 56, offsetof(mavlink_avoidance_triplet_t, acc_per_err) }, \
         { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 60, offsetof(mavlink_avoidance_triplet_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 61, offsetof(mavlink_avoidance_triplet_t, target_component) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_AVOIDANCE_TRIPLET { \
    "AVOIDANCE_TRIPLET", \
    16, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_avoidance_triplet_t, time_usec) }, \
         { "gen_count", NULL, MAVLINK_TYPE_UINT32_T, 0, 8, offsetof(mavlink_avoidance_triplet_t, gen_count) }, \
         { "prev_x", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_avoidance_triplet_t, prev_x) }, \
         { "prev_y", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_avoidance_triplet_t, prev_y) }, \
         { "prev_z", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_avoidance_triplet_t, prev_z) }, \
         { "ctrl_x", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_avoidance_triplet_t, ctrl_x) }, \
         { "ctrl_y", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_avoidance_triplet_t, ctrl_y) }, \
         { "ctrl_z", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_avoidance_triplet_t, ctrl_z) }, \
         { "next_x", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_avoidance_triplet_t, next_x) }, \
         { "next_y", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_avoidance_triplet_t, next_y) }, \
         { "next_z", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_avoidance_triplet_t, next_z) }, \
         { "duration", NULL, MAVLINK_TYPE_FLOAT, 0, 48, offsetof(mavlink_avoidance_triplet_t, duration) }, \
         { "max_acc", NULL, MAVLINK_TYPE_FLOAT, 0, 52, offsetof(mavlink_avoidance_triplet_t, max_acc) }, \
         { "acc_per_err", NULL, MAVLINK_TYPE_FLOAT, 0, 56, offsetof(mavlink_avoidance_triplet_t, acc_per_err) }, \
         { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 60, offsetof(mavlink_avoidance_triplet_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 61, offsetof(mavlink_avoidance_triplet_t, target_component) }, \
         } \
}
#endif

/**
 * @brief Pack a avoidance_triplet message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec Timestamp (micros since boot or Unix epoch)
 * @param gen_count Generation count for triplet. Increases monotonically and wraps once reached. This number is a copy of the last received planning triplet, which allows the system to update a planning triplet with avoidance data.
 * @param target_system System ID
 * @param target_component Component ID
 * @param prev_x Local position X (NED) of previous position command in m
 * @param prev_y Local position Y (NED) of previous position command in m
 * @param prev_z Local position Z (NED) of previous position command in m
 * @param ctrl_x Local position X (NED) of control point in m
 * @param ctrl_y Local position Y (NED) of control point in m
 * @param ctrl_z Local position Z (NED) of control point in m
 * @param next_x Local position X (NED) of next position command in m
 * @param next_y Local position Y (NED) of next position command in m
 * @param next_z Local position Z (NED) of next position command in m
 * @param duration Desired time it takes to travel from prev to next in s
 * @param max_acc Maximum acceleration in m/s^2
 * @param acc_per_err Maximum acceleration in m/s^2 per meter error
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_avoidance_triplet_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t time_usec, uint32_t gen_count, uint8_t target_system, uint8_t target_component, float prev_x, float prev_y, float prev_z, float ctrl_x, float ctrl_y, float ctrl_z, float next_x, float next_y, float next_z, float duration, float max_acc, float acc_per_err)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_AVOIDANCE_TRIPLET_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_uint32_t(buf, 8, gen_count);
    _mav_put_float(buf, 12, prev_x);
    _mav_put_float(buf, 16, prev_y);
    _mav_put_float(buf, 20, prev_z);
    _mav_put_float(buf, 24, ctrl_x);
    _mav_put_float(buf, 28, ctrl_y);
    _mav_put_float(buf, 32, ctrl_z);
    _mav_put_float(buf, 36, next_x);
    _mav_put_float(buf, 40, next_y);
    _mav_put_float(buf, 44, next_z);
    _mav_put_float(buf, 48, duration);
    _mav_put_float(buf, 52, max_acc);
    _mav_put_float(buf, 56, acc_per_err);
    _mav_put_uint8_t(buf, 60, target_system);
    _mav_put_uint8_t(buf, 61, target_component);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_AVOIDANCE_TRIPLET_LEN);
#else
    mavlink_avoidance_triplet_t packet;
    packet.time_usec = time_usec;
    packet.gen_count = gen_count;
    packet.prev_x = prev_x;
    packet.prev_y = prev_y;
    packet.prev_z = prev_z;
    packet.ctrl_x = ctrl_x;
    packet.ctrl_y = ctrl_y;
    packet.ctrl_z = ctrl_z;
    packet.next_x = next_x;
    packet.next_y = next_y;
    packet.next_z = next_z;
    packet.duration = duration;
    packet.max_acc = max_acc;
    packet.acc_per_err = acc_per_err;
    packet.target_system = target_system;
    packet.target_component = target_component;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_AVOIDANCE_TRIPLET_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_AVOIDANCE_TRIPLET;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_AVOIDANCE_TRIPLET_MIN_LEN, MAVLINK_MSG_ID_AVOIDANCE_TRIPLET_LEN, MAVLINK_MSG_ID_AVOIDANCE_TRIPLET_CRC);
}

/**
 * @brief Pack a avoidance_triplet message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_usec Timestamp (micros since boot or Unix epoch)
 * @param gen_count Generation count for triplet. Increases monotonically and wraps once reached. This number is a copy of the last received planning triplet, which allows the system to update a planning triplet with avoidance data.
 * @param target_system System ID
 * @param target_component Component ID
 * @param prev_x Local position X (NED) of previous position command in m
 * @param prev_y Local position Y (NED) of previous position command in m
 * @param prev_z Local position Z (NED) of previous position command in m
 * @param ctrl_x Local position X (NED) of control point in m
 * @param ctrl_y Local position Y (NED) of control point in m
 * @param ctrl_z Local position Z (NED) of control point in m
 * @param next_x Local position X (NED) of next position command in m
 * @param next_y Local position Y (NED) of next position command in m
 * @param next_z Local position Z (NED) of next position command in m
 * @param duration Desired time it takes to travel from prev to next in s
 * @param max_acc Maximum acceleration in m/s^2
 * @param acc_per_err Maximum acceleration in m/s^2 per meter error
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_avoidance_triplet_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t time_usec,uint32_t gen_count,uint8_t target_system,uint8_t target_component,float prev_x,float prev_y,float prev_z,float ctrl_x,float ctrl_y,float ctrl_z,float next_x,float next_y,float next_z,float duration,float max_acc,float acc_per_err)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_AVOIDANCE_TRIPLET_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_uint32_t(buf, 8, gen_count);
    _mav_put_float(buf, 12, prev_x);
    _mav_put_float(buf, 16, prev_y);
    _mav_put_float(buf, 20, prev_z);
    _mav_put_float(buf, 24, ctrl_x);
    _mav_put_float(buf, 28, ctrl_y);
    _mav_put_float(buf, 32, ctrl_z);
    _mav_put_float(buf, 36, next_x);
    _mav_put_float(buf, 40, next_y);
    _mav_put_float(buf, 44, next_z);
    _mav_put_float(buf, 48, duration);
    _mav_put_float(buf, 52, max_acc);
    _mav_put_float(buf, 56, acc_per_err);
    _mav_put_uint8_t(buf, 60, target_system);
    _mav_put_uint8_t(buf, 61, target_component);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_AVOIDANCE_TRIPLET_LEN);
#else
    mavlink_avoidance_triplet_t packet;
    packet.time_usec = time_usec;
    packet.gen_count = gen_count;
    packet.prev_x = prev_x;
    packet.prev_y = prev_y;
    packet.prev_z = prev_z;
    packet.ctrl_x = ctrl_x;
    packet.ctrl_y = ctrl_y;
    packet.ctrl_z = ctrl_z;
    packet.next_x = next_x;
    packet.next_y = next_y;
    packet.next_z = next_z;
    packet.duration = duration;
    packet.max_acc = max_acc;
    packet.acc_per_err = acc_per_err;
    packet.target_system = target_system;
    packet.target_component = target_component;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_AVOIDANCE_TRIPLET_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_AVOIDANCE_TRIPLET;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_AVOIDANCE_TRIPLET_MIN_LEN, MAVLINK_MSG_ID_AVOIDANCE_TRIPLET_LEN, MAVLINK_MSG_ID_AVOIDANCE_TRIPLET_CRC);
}

/**
 * @brief Encode a avoidance_triplet struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param avoidance_triplet C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_avoidance_triplet_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_avoidance_triplet_t* avoidance_triplet)
{
    return mavlink_msg_avoidance_triplet_pack(system_id, component_id, msg, avoidance_triplet->time_usec, avoidance_triplet->gen_count, avoidance_triplet->target_system, avoidance_triplet->target_component, avoidance_triplet->prev_x, avoidance_triplet->prev_y, avoidance_triplet->prev_z, avoidance_triplet->ctrl_x, avoidance_triplet->ctrl_y, avoidance_triplet->ctrl_z, avoidance_triplet->next_x, avoidance_triplet->next_y, avoidance_triplet->next_z, avoidance_triplet->duration, avoidance_triplet->max_acc, avoidance_triplet->acc_per_err);
}

/**
 * @brief Encode a avoidance_triplet struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param avoidance_triplet C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_avoidance_triplet_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_avoidance_triplet_t* avoidance_triplet)
{
    return mavlink_msg_avoidance_triplet_pack_chan(system_id, component_id, chan, msg, avoidance_triplet->time_usec, avoidance_triplet->gen_count, avoidance_triplet->target_system, avoidance_triplet->target_component, avoidance_triplet->prev_x, avoidance_triplet->prev_y, avoidance_triplet->prev_z, avoidance_triplet->ctrl_x, avoidance_triplet->ctrl_y, avoidance_triplet->ctrl_z, avoidance_triplet->next_x, avoidance_triplet->next_y, avoidance_triplet->next_z, avoidance_triplet->duration, avoidance_triplet->max_acc, avoidance_triplet->acc_per_err);
}

/**
 * @brief Send a avoidance_triplet message
 * @param chan MAVLink channel to send the message
 *
 * @param time_usec Timestamp (micros since boot or Unix epoch)
 * @param gen_count Generation count for triplet. Increases monotonically and wraps once reached. This number is a copy of the last received planning triplet, which allows the system to update a planning triplet with avoidance data.
 * @param target_system System ID
 * @param target_component Component ID
 * @param prev_x Local position X (NED) of previous position command in m
 * @param prev_y Local position Y (NED) of previous position command in m
 * @param prev_z Local position Z (NED) of previous position command in m
 * @param ctrl_x Local position X (NED) of control point in m
 * @param ctrl_y Local position Y (NED) of control point in m
 * @param ctrl_z Local position Z (NED) of control point in m
 * @param next_x Local position X (NED) of next position command in m
 * @param next_y Local position Y (NED) of next position command in m
 * @param next_z Local position Z (NED) of next position command in m
 * @param duration Desired time it takes to travel from prev to next in s
 * @param max_acc Maximum acceleration in m/s^2
 * @param acc_per_err Maximum acceleration in m/s^2 per meter error
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_avoidance_triplet_send(mavlink_channel_t chan, uint64_t time_usec, uint32_t gen_count, uint8_t target_system, uint8_t target_component, float prev_x, float prev_y, float prev_z, float ctrl_x, float ctrl_y, float ctrl_z, float next_x, float next_y, float next_z, float duration, float max_acc, float acc_per_err)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_AVOIDANCE_TRIPLET_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_uint32_t(buf, 8, gen_count);
    _mav_put_float(buf, 12, prev_x);
    _mav_put_float(buf, 16, prev_y);
    _mav_put_float(buf, 20, prev_z);
    _mav_put_float(buf, 24, ctrl_x);
    _mav_put_float(buf, 28, ctrl_y);
    _mav_put_float(buf, 32, ctrl_z);
    _mav_put_float(buf, 36, next_x);
    _mav_put_float(buf, 40, next_y);
    _mav_put_float(buf, 44, next_z);
    _mav_put_float(buf, 48, duration);
    _mav_put_float(buf, 52, max_acc);
    _mav_put_float(buf, 56, acc_per_err);
    _mav_put_uint8_t(buf, 60, target_system);
    _mav_put_uint8_t(buf, 61, target_component);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AVOIDANCE_TRIPLET, buf, MAVLINK_MSG_ID_AVOIDANCE_TRIPLET_MIN_LEN, MAVLINK_MSG_ID_AVOIDANCE_TRIPLET_LEN, MAVLINK_MSG_ID_AVOIDANCE_TRIPLET_CRC);
#else
    mavlink_avoidance_triplet_t packet;
    packet.time_usec = time_usec;
    packet.gen_count = gen_count;
    packet.prev_x = prev_x;
    packet.prev_y = prev_y;
    packet.prev_z = prev_z;
    packet.ctrl_x = ctrl_x;
    packet.ctrl_y = ctrl_y;
    packet.ctrl_z = ctrl_z;
    packet.next_x = next_x;
    packet.next_y = next_y;
    packet.next_z = next_z;
    packet.duration = duration;
    packet.max_acc = max_acc;
    packet.acc_per_err = acc_per_err;
    packet.target_system = target_system;
    packet.target_component = target_component;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AVOIDANCE_TRIPLET, (const char *)&packet, MAVLINK_MSG_ID_AVOIDANCE_TRIPLET_MIN_LEN, MAVLINK_MSG_ID_AVOIDANCE_TRIPLET_LEN, MAVLINK_MSG_ID_AVOIDANCE_TRIPLET_CRC);
#endif
}

/**
 * @brief Send a avoidance_triplet message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_avoidance_triplet_send_struct(mavlink_channel_t chan, const mavlink_avoidance_triplet_t* avoidance_triplet)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_avoidance_triplet_send(chan, avoidance_triplet->time_usec, avoidance_triplet->gen_count, avoidance_triplet->target_system, avoidance_triplet->target_component, avoidance_triplet->prev_x, avoidance_triplet->prev_y, avoidance_triplet->prev_z, avoidance_triplet->ctrl_x, avoidance_triplet->ctrl_y, avoidance_triplet->ctrl_z, avoidance_triplet->next_x, avoidance_triplet->next_y, avoidance_triplet->next_z, avoidance_triplet->duration, avoidance_triplet->max_acc, avoidance_triplet->acc_per_err);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AVOIDANCE_TRIPLET, (const char *)avoidance_triplet, MAVLINK_MSG_ID_AVOIDANCE_TRIPLET_MIN_LEN, MAVLINK_MSG_ID_AVOIDANCE_TRIPLET_LEN, MAVLINK_MSG_ID_AVOIDANCE_TRIPLET_CRC);
#endif
}

#if MAVLINK_MSG_ID_AVOIDANCE_TRIPLET_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_avoidance_triplet_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t time_usec, uint32_t gen_count, uint8_t target_system, uint8_t target_component, float prev_x, float prev_y, float prev_z, float ctrl_x, float ctrl_y, float ctrl_z, float next_x, float next_y, float next_z, float duration, float max_acc, float acc_per_err)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_uint32_t(buf, 8, gen_count);
    _mav_put_float(buf, 12, prev_x);
    _mav_put_float(buf, 16, prev_y);
    _mav_put_float(buf, 20, prev_z);
    _mav_put_float(buf, 24, ctrl_x);
    _mav_put_float(buf, 28, ctrl_y);
    _mav_put_float(buf, 32, ctrl_z);
    _mav_put_float(buf, 36, next_x);
    _mav_put_float(buf, 40, next_y);
    _mav_put_float(buf, 44, next_z);
    _mav_put_float(buf, 48, duration);
    _mav_put_float(buf, 52, max_acc);
    _mav_put_float(buf, 56, acc_per_err);
    _mav_put_uint8_t(buf, 60, target_system);
    _mav_put_uint8_t(buf, 61, target_component);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AVOIDANCE_TRIPLET, buf, MAVLINK_MSG_ID_AVOIDANCE_TRIPLET_MIN_LEN, MAVLINK_MSG_ID_AVOIDANCE_TRIPLET_LEN, MAVLINK_MSG_ID_AVOIDANCE_TRIPLET_CRC);
#else
    mavlink_avoidance_triplet_t *packet = (mavlink_avoidance_triplet_t *)msgbuf;
    packet->time_usec = time_usec;
    packet->gen_count = gen_count;
    packet->prev_x = prev_x;
    packet->prev_y = prev_y;
    packet->prev_z = prev_z;
    packet->ctrl_x = ctrl_x;
    packet->ctrl_y = ctrl_y;
    packet->ctrl_z = ctrl_z;
    packet->next_x = next_x;
    packet->next_y = next_y;
    packet->next_z = next_z;
    packet->duration = duration;
    packet->max_acc = max_acc;
    packet->acc_per_err = acc_per_err;
    packet->target_system = target_system;
    packet->target_component = target_component;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AVOIDANCE_TRIPLET, (const char *)packet, MAVLINK_MSG_ID_AVOIDANCE_TRIPLET_MIN_LEN, MAVLINK_MSG_ID_AVOIDANCE_TRIPLET_LEN, MAVLINK_MSG_ID_AVOIDANCE_TRIPLET_CRC);
#endif
}
#endif

#endif

// MESSAGE AVOIDANCE_TRIPLET UNPACKING


/**
 * @brief Get field time_usec from avoidance_triplet message
 *
 * @return Timestamp (micros since boot or Unix epoch)
 */
static inline uint64_t mavlink_msg_avoidance_triplet_get_time_usec(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field gen_count from avoidance_triplet message
 *
 * @return Generation count for triplet. Increases monotonically and wraps once reached. This number is a copy of the last received planning triplet, which allows the system to update a planning triplet with avoidance data.
 */
static inline uint32_t mavlink_msg_avoidance_triplet_get_gen_count(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  8);
}

/**
 * @brief Get field target_system from avoidance_triplet message
 *
 * @return System ID
 */
static inline uint8_t mavlink_msg_avoidance_triplet_get_target_system(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  60);
}

/**
 * @brief Get field target_component from avoidance_triplet message
 *
 * @return Component ID
 */
static inline uint8_t mavlink_msg_avoidance_triplet_get_target_component(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  61);
}

/**
 * @brief Get field prev_x from avoidance_triplet message
 *
 * @return Local position X (NED) of previous position command in m
 */
static inline float mavlink_msg_avoidance_triplet_get_prev_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field prev_y from avoidance_triplet message
 *
 * @return Local position Y (NED) of previous position command in m
 */
static inline float mavlink_msg_avoidance_triplet_get_prev_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field prev_z from avoidance_triplet message
 *
 * @return Local position Z (NED) of previous position command in m
 */
static inline float mavlink_msg_avoidance_triplet_get_prev_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field ctrl_x from avoidance_triplet message
 *
 * @return Local position X (NED) of control point in m
 */
static inline float mavlink_msg_avoidance_triplet_get_ctrl_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field ctrl_y from avoidance_triplet message
 *
 * @return Local position Y (NED) of control point in m
 */
static inline float mavlink_msg_avoidance_triplet_get_ctrl_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field ctrl_z from avoidance_triplet message
 *
 * @return Local position Z (NED) of control point in m
 */
static inline float mavlink_msg_avoidance_triplet_get_ctrl_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Get field next_x from avoidance_triplet message
 *
 * @return Local position X (NED) of next position command in m
 */
static inline float mavlink_msg_avoidance_triplet_get_next_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  36);
}

/**
 * @brief Get field next_y from avoidance_triplet message
 *
 * @return Local position Y (NED) of next position command in m
 */
static inline float mavlink_msg_avoidance_triplet_get_next_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  40);
}

/**
 * @brief Get field next_z from avoidance_triplet message
 *
 * @return Local position Z (NED) of next position command in m
 */
static inline float mavlink_msg_avoidance_triplet_get_next_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  44);
}

/**
 * @brief Get field duration from avoidance_triplet message
 *
 * @return Desired time it takes to travel from prev to next in s
 */
static inline float mavlink_msg_avoidance_triplet_get_duration(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  48);
}

/**
 * @brief Get field max_acc from avoidance_triplet message
 *
 * @return Maximum acceleration in m/s^2
 */
static inline float mavlink_msg_avoidance_triplet_get_max_acc(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  52);
}

/**
 * @brief Get field acc_per_err from avoidance_triplet message
 *
 * @return Maximum acceleration in m/s^2 per meter error
 */
static inline float mavlink_msg_avoidance_triplet_get_acc_per_err(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  56);
}

/**
 * @brief Decode a avoidance_triplet message into a struct
 *
 * @param msg The message to decode
 * @param avoidance_triplet C-struct to decode the message contents into
 */
static inline void mavlink_msg_avoidance_triplet_decode(const mavlink_message_t* msg, mavlink_avoidance_triplet_t* avoidance_triplet)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    avoidance_triplet->time_usec = mavlink_msg_avoidance_triplet_get_time_usec(msg);
    avoidance_triplet->gen_count = mavlink_msg_avoidance_triplet_get_gen_count(msg);
    avoidance_triplet->prev_x = mavlink_msg_avoidance_triplet_get_prev_x(msg);
    avoidance_triplet->prev_y = mavlink_msg_avoidance_triplet_get_prev_y(msg);
    avoidance_triplet->prev_z = mavlink_msg_avoidance_triplet_get_prev_z(msg);
    avoidance_triplet->ctrl_x = mavlink_msg_avoidance_triplet_get_ctrl_x(msg);
    avoidance_triplet->ctrl_y = mavlink_msg_avoidance_triplet_get_ctrl_y(msg);
    avoidance_triplet->ctrl_z = mavlink_msg_avoidance_triplet_get_ctrl_z(msg);
    avoidance_triplet->next_x = mavlink_msg_avoidance_triplet_get_next_x(msg);
    avoidance_triplet->next_y = mavlink_msg_avoidance_triplet_get_next_y(msg);
    avoidance_triplet->next_z = mavlink_msg_avoidance_triplet_get_next_z(msg);
    avoidance_triplet->duration = mavlink_msg_avoidance_triplet_get_duration(msg);
    avoidance_triplet->max_acc = mavlink_msg_avoidance_triplet_get_max_acc(msg);
    avoidance_triplet->acc_per_err = mavlink_msg_avoidance_triplet_get_acc_per_err(msg);
    avoidance_triplet->target_system = mavlink_msg_avoidance_triplet_get_target_system(msg);
    avoidance_triplet->target_component = mavlink_msg_avoidance_triplet_get_target_component(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_AVOIDANCE_TRIPLET_LEN? msg->len : MAVLINK_MSG_ID_AVOIDANCE_TRIPLET_LEN;
        memset(avoidance_triplet, 0, MAVLINK_MSG_ID_AVOIDANCE_TRIPLET_LEN);
    memcpy(avoidance_triplet, _MAV_PAYLOAD(msg), len);
#endif
}
