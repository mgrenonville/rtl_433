/** @file
    Template decoder for SX1211, tested with Frisquet Eco Radio System Visio.

    Copyright (C) 2023 Mathieu Grenonville

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
 */

/**
The device uses FSK encoding,
- 0 is encoded as 40 us pulse,
- 1 is encoded as 40 us pulse.
The device sends a transmission every 10 minutes.
A transmission starts with a preamble of 3 to 5 0xAA,

(describe the data and payload, e.g.)
Data layout:
    NN NN NN NN SS PP... CC CC

- N: 32 bit: Sync / network id
- S: 8-bit little-endian Payload size
- P: Size * 8-bit payload data
- C: 16-bit Checksum, CRC-16 poly=0x1021, initial=0x236b
*/

#include "decoder.h"

/*
 * Hypothetical template device
 *
 * Message is 68 bits long
 * Messages start with 0xAA
 * The message is repeated as 5 packets,
 * require at least 3 repeated packets.
 *
 */

#define SX1211_STARTBYTE       0xAA
#define SX1211_CRC_POLY        0x1021
#define SX1211_CRC_INIT        0x236b
#define SX1211_MAX_LENGHT_FIFO 64

static void print_row_bytes(char *row_bytes, uint8_t *source, int num)
{
    row_bytes[0] = '\0';
    // print byte-wide
    for (int col = 0; col < num; ++col) {
        sprintf(&row_bytes[2 * col], "%02x", source[col]);
    }
    // remove last nibble if needed
    row_bytes[2 * (num + 3)] = '\0';
}

static int sx1211_decode(r_device *decoder, bitbuffer_t *bitbuffer)
{
    data_t *data;
    int r;      // a row index
    uint8_t *b; // bits of a row

    uint8_t *network_id[4 * 2];
    uint8_t *payload[SX1211_MAX_LENGHT_FIFO * 2];
    uint8_t *raw_payload[(SX1211_MAX_LENGHT_FIFO + 10) * 2];
    int parity;
    uint16_t computed_crc;
    uint16_t message_crc;
    uint16_t sensor_id;
    uint8_t size, to_addr, from_addr;

    int16_t value;

    /*
     * Early debugging aid to see demodulated bits in buffer and
     * to determine if your limit settings are matched and firing
     * this decode callback.
     *
     * 1. Enable with -vvv (debug decoders)
     * 2. Delete this block when your decoder is working
     */
    decoder_log_bitbuffer(decoder, 1, __func__, bitbuffer, "");

    uint8_t const start_match[] = {SX1211_STARTBYTE, SX1211_STARTBYTE, SX1211_STARTBYTE}; // preamble only

    unsigned int start_pos = 0;
    decoder_log(decoder, 1, __func__, "Align on 4*0xAA before message in the first byte");

    for (int bit = 0; bit < 3 * 8 + 1; bit++) {

        unsigned int possible_start = bitbuffer_search(bitbuffer, 0, bit, start_match, sizeof(start_match) * 8);
        if (possible_start < 32) {
            start_pos = possible_start;
        }
        decoder_logf(decoder, 1, __func__, "Searching init pattern at %d, start_pos = %d", bit, start_pos, bitbuffer);
        decoder_log_bitrow(decoder, 1, __func__, bitbuffer->bb[0] + start_pos, 4 * 8, "Here");

        if (start_pos >= bitbuffer->bits_per_row[0]) {
            decoder_log(decoder, 1, __func__, "Not found");
            return DECODE_ABORT_EARLY;
        }
    }
    uint8_t msg_bytes = (bitbuffer->bits_per_row[0] - start_pos) / 8;

    uint8_t msg[SX1211_MAX_LENGHT_FIFO];
    bitbuffer_extract_bytes(bitbuffer, 0, start_pos + sizeof(start_match) * 8, msg, msg_bytes * 8);

    print_row_bytes(network_id, msg, 4);

    size      = msg[4];
    to_addr   = msg[5];
    from_addr = msg[6];
    if(size > SX1211_MAX_LENGHT_FIFO - 1) // FIFO is 64 bytes long, and one byte is used by size itself
    {
        return DECODE_ABORT_LENGTH;
    }

    print_row_bytes(payload, msg + 4, size + 1);

    print_row_bytes(raw_payload, msg, 4 + 1 + size + 2);

    message_crc  = (msg[4 + 1 + size] << 8) + (msg[4 + 1 + size + 1]);
    computed_crc = crc16(msg + 4, size + 1, SX1211_CRC_POLY, SX1211_CRC_INIT);

    data = data_make(
            "model", "", DATA_STRING, "sx1211",
            "network_id", "", DATA_STRING, network_id,
            "size", "", DATA_INT, size,
            "to", "", DATA_FORMAT, "%02x", DATA_INT, to_addr,
            "from", "", DATA_FORMAT, "%02x", DATA_INT, from_addr,
            "payload", "", DATA_STRING, payload,
            "crc", "", DATA_FORMAT, "%04x", DATA_INT, message_crc,
            "raw_payload", "", DATA_STRING, raw_payload,
            "mic", "", DATA_FORMAT, "%04x", DATA_INT, computed_crc,
            NULL);

    decoder_output_data(decoder, data);
    return 1;
};

/*
 * List of fields that may appear in the output
 *
 * Used to determine what fields will be output in what
 * order for this device when using -F csv.
 *
 */
static char const *output_fields[] = {
        "model",
        "network_id",
        "size",
        "to_address",
        "from_address",
        "payload",
        "crc",
        "raw_payload",
        "mic", // remove if not applicable
        NULL,
};

/*
 * r_device - registers device/callback. see rtl_433_devices.h
 *
 * Timings:
 *
 * short, long, and reset - specify pulse/period timings in [us].
 *     These timings will determine if the received pulses
 *     match, so your callback will fire after demodulation.
 *
 * Modulation:
 *
 * The function used to turn the received signal into bits.
 * See:
 * - pulse_slicer.h for descriptions
 * - r_device.h for the list of defined names
 *
 * This device is disabled and hidden, it can not be enabled.
 *
 * To enable your device, append it to the list in include/rtl_433_devices.h
 * and sort it into src/CMakeLists.txt or run ./maintainer_update.py
 *
 */
r_device const sx1211 = {
        .name        = "SX1211",
        .modulation  = FSK_PULSE_PCM,
        .short_width = 38,   // short gap is ~38 us
        .long_width  = 38,   // long gap is ~38 us
        .reset_limit = 4200, // a bit longer than packet gap
        .tolerance   = 3,    // a bit longer than packet gap
        .decode_fn   = &sx1211_decode,
        .disabled    = 0, // disabled and hidden, use 0 if there is a MIC, 1 otherwise
        .fields      = output_fields,
};
