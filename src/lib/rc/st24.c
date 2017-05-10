/****************************************************************************
 *
 *   Copyright (c) 2014 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/*
 * @file st24.h
 *
 * RC protocol implementation for Yuneec ST24 transmitter.
 *
 * @author Lorenz Meier <lm@inf.ethz.ch>
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "st24.h"
#include "string.h"
#include <drivers/drv_hrt.h>

enum ST24_DECODE_STATE {
	ST24_DECODE_STATE_UNSYNCED = 0,
	ST24_DECODE_STATE_GOT_STX1,
	ST24_DECODE_STATE_GOT_STX2,
	ST24_DECODE_STATE_GOT_LEN,
	ST24_DECODE_STATE_GOT_TYPE,
	ST24_DECODE_STATE_GOT_DATA
};

const char *decode_states[] = {"UNSYNCED",
			       "GOT_STX1",
			       "GOT_STX2",
			       "GOT_LEN",
			       "GOT_TYPE",
			       "GOT_DATA"
			      };

/* define range mapping here, -+100% -> 1000..2000 */
#define ST24_RANGE_MIN 0.0f
#define ST24_RANGE_MAX 4096.0f

#define ST24_TARGET_MIN 1000.0f
#define ST24_TARGET_MAX 2000.0f

#define KILL_HOTKEY_TIME 1000000

/* pre-calculate the floating point stuff as far as possible at compile time */
#define ST24_SCALE_FACTOR ((ST24_TARGET_MAX - ST24_TARGET_MIN) / (ST24_RANGE_MAX - ST24_RANGE_MIN))
#define ST24_SCALE_OFFSET (ST24_TARGET_MIN - (ST24_SCALE_FACTOR * ST24_RANGE_MIN))

static enum ST24_DECODE_STATE _decode_state = ST24_DECODE_STATE_UNSYNCED;
static uint8_t _rxlen;

static uint16_t _last_channel_0 = 0; /* the last raw value channel 0 (throttle and button) had */
static hrt_abstime _kill_hotkey_start = 0; /* the time when the hotkey started to measure timeout */
static int _kill_hotkey_button_count = 0; /* how many times the button was pressed during the hotkey */
static bool _kill_state = false; /* the kill state in with we lockdown the motors until restart */

static ReceiverFcPacket _rxpacket;
static ReceiverFcPacket _txpacket;

uint8_t st24_common_crc8(uint8_t *ptr, uint8_t len)
{
	uint8_t i, crc ;
	crc = 0;

	while (len--) {
		for (i = 0x80; i != 0; i >>= 1) {
			if ((crc & 0x80) != 0) {
				crc <<= 1;
				crc ^= 0x07;

			} else {
				crc <<= 1;
			}

			if ((*ptr & i) != 0) {
				crc ^= 0x07;
			}
		}

		ptr++;
	}

	return (crc);
}


int st24_decode(uint8_t byte, uint8_t *rssi, uint8_t *lost_count, uint16_t *channel_count, uint16_t *channels,
		uint16_t max_chan_count)
{
	int ret = 1;

	switch (_decode_state) {
	case ST24_DECODE_STATE_UNSYNCED:
		if (byte == ST24_STX1) {
			_decode_state = ST24_DECODE_STATE_GOT_STX1;

		} else {
			ret = 3;
		}

		break;

	case ST24_DECODE_STATE_GOT_STX1:
		if (byte == ST24_STX2) {
			_decode_state = ST24_DECODE_STATE_GOT_STX2;

		} else {
			_decode_state = ST24_DECODE_STATE_UNSYNCED;
		}

		break;

	case ST24_DECODE_STATE_GOT_STX2:

		/* ensure no data overflow failure or hack is possible */
		if (((unsigned)byte <= sizeof(_rxpacket.length) + sizeof(_rxpacket.type) + sizeof(_rxpacket.st24_data))
		    && (unsigned)byte != 0) {
			_rxpacket.length = byte;
			_rxlen = 0;
			_decode_state = ST24_DECODE_STATE_GOT_LEN;

		} else {
			_decode_state = ST24_DECODE_STATE_UNSYNCED;
		}

		break;

	case ST24_DECODE_STATE_GOT_LEN:
		_rxpacket.type = byte;
		_rxlen++;
		_decode_state = ST24_DECODE_STATE_GOT_TYPE;
		break;

	case ST24_DECODE_STATE_GOT_TYPE:
		_rxpacket.st24_data[_rxlen - 1] = byte;
		_rxlen++;

		if (_rxlen == (_rxpacket.length - 1)) {
			_decode_state = ST24_DECODE_STATE_GOT_DATA;
		}

		break;

	case ST24_DECODE_STATE_GOT_DATA:
		_rxpacket.crc8 = byte;
		_rxlen++;

		if (st24_common_crc8((uint8_t *) & (_rxpacket.length), _rxlen) == _rxpacket.crc8) {

			ret = 0;

			/* decode the actual packet */

			switch (_rxpacket.type) {

			case ST24_PACKET_TYPE_CHANNELDATA12:
			case ST24_PACKET_TYPE_CHANNELDATA24: {
					ChannelData24 *d = (ChannelData24 *)&_rxpacket.st24_data;

					/* Scale from 0..255 to 100%. */
					*rssi = d->rssi * (100.0f / 255.0f);
					*lost_count = d->lost_count;

					/* this can lead to rounding of the strides */
					if (_rxpacket.type == ST24_PACKET_TYPE_CHANNELDATA12) {
						*channel_count = (max_chan_count < 12) ? max_chan_count : 12;

					} else if (_rxpacket.type == ST24_PACKET_TYPE_CHANNELDATA24) {
						*channel_count = (max_chan_count < 24) ? max_chan_count : 24;
					}

					/* decode channels always 3 bytes give 2 channels */
					unsigned stride_count = (*channel_count * 3) / 2;
					unsigned chan_index = 0;

					for (unsigned i = 0; i < stride_count; i += 3) {
						channels[chan_index] = ((uint16_t)d->channel[i] << 4);
						channels[chan_index] |= ((uint16_t)(0xF0 & d->channel[i + 1]) >> 4);
						chan_index++;

						channels[chan_index] = ((uint16_t)d->channel[i + 2]);
						channels[chan_index] |= (((uint16_t)(0x0F & d->channel[i + 1])) << 8);
						chan_index++;
					}

					const bool arm_button_pressed = channels[0] == 0;

					/* Kill hotkey: pressing the arm button three times with low throttle within 1.5s
					 * overrides a free channel with a virtual switch which has maximum value when kill was triggered
					 * need to be used in combination with RC_MAP_KILL_SW mapped to the channel */
					const bool first_time = _kill_hotkey_button_count == 0;
					const bool within_timeout = hrt_elapsed_time(&_kill_hotkey_start) < KILL_HOTKEY_TIME;
					const bool hotkey_complete = _kill_hotkey_button_count > 2;

					if (hotkey_complete) {
						_kill_state = !_kill_state;
					}

					if (_last_channel_0 < 1250 && (first_time || within_timeout) && !hotkey_complete) {
						if (_last_channel_0 != 0 && arm_button_pressed) {
							if (first_time) {
								_kill_hotkey_start = hrt_absolute_time();
							}

							_kill_hotkey_button_count++;
						}

					} else {
						_kill_hotkey_button_count = 0;
						_kill_hotkey_start = 0;
					}

					_last_channel_0 = channels[0];
					channels[ST16_VIRTUAL_KILL_SWITCH_CHANNEL] = _kill_state ? (uint16_t)ST24_RANGE_MAX : (uint16_t)ST24_RANGE_MIN;

					/* Handle the red arm/disarm button of the ST16 which is mapped to Channel 1 (Throttle) raw value 0 when pressed
					 * Override a free channel with a virtual switch which has maximum value when button is pressed
					 * needs to be used in combination with RC_MAP_ARM_SW mapped to the channel and parameter COM_ARM_SWISBTN enabled */
					if (arm_button_pressed) {
						channels[ST16_VIRTUAL_ARM_BUTTON_CHANNEL] = (uint16_t)ST24_RANGE_MAX;
						/* set a low throttle value when ST16 arm button pressed to descend */
						channels[0] = ST16_ARM_BUTTON_THROTTLE_VALUE_RAW;

					} else {
						channels[ST16_VIRTUAL_ARM_BUTTON_CHANNEL] = (uint16_t)ST24_RANGE_MIN;
					}

					/* convert values from 0-4096 serial value to 1000-2000 ppm encoding */
					for (unsigned i = 0; i < *channel_count; i++) {
						channels[i] = (uint16_t)(channels[i] * ST24_SCALE_FACTOR + ST24_SCALE_OFFSET + .5f);
					}
				}
				break;

			case ST24_PACKET_TYPE_TRANSMITTERGPSDATA: {

					// ReceiverFcPacket* d = (ReceiverFcPacket*)&_rxpacket.st24_data;
					/* we silently ignore this data for now, as it is unused */
					ret = 5;
				}
				break;

			default:
				ret = 2;
				break;
			}

		} else {
			/* decoding failed */
			ret = 4;
		}

		_decode_state = ST24_DECODE_STATE_UNSYNCED;
		break;
	}

	return ret;
}

ReceiverFcPacket *st24_encode(uint8_t type, const uint8_t *data, uint8_t bytecount)
{
	_txpacket.header1 = ST24_STX1;
	_txpacket.header2 = ST24_STX2;
	_txpacket.length = bytecount + 2; // 2 bytes more for length and type
	_txpacket.type = type;
	memcpy(_txpacket.st24_data, data, bytecount);
	_txpacket.st24_data[bytecount] = st24_common_crc8((uint8_t *) &_txpacket.length, _txpacket.length);
	return &_txpacket;
}

ReceiverFcPacket *st24_get_bind_packet(void)
{
	StBindCmd bind_cmd = {0, {'B', 'I', 'N', 'D'}};
	return st24_encode(ST24_PACKET_TYPE_BINDCMD, (uint8_t *) &bind_cmd, sizeof(StBindCmd));
}
