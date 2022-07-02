/*
* JPEG2000 High Throughput block decoder
* Copyright (c) 2022 Caleb Etemesi<etemesicaleb@gmail.com>
*
* This file is part of FFmpeg.
*
* FFmpeg is free software; you can redistribute it and/or
* modify it under the terms of the GNU Lesser General Public
* License as published by the Free Software Foundation; either
* version 2.1 of the License, or (at your option) any later version.
*
* FFmpeg is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
* Lesser General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public
* License along with FFmpeg; if not, write to the Free Software
* Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */
#include <stdint.h>
#include "jpeg2000_htj2k.h"
#include <libavutil/attributes.h>
#include <libavutil/avassert.h>
#include <libavutil/common.h>
#include <libavutil/log.h>
#include <libavutil/mem.h>

#include "bytestream.h"

static uint32_t has_zero(uint32_t dword)
{
  // Borrowed from the famous stanford bithacks page
  // see https://graphics.stanford.edu/~seander/bithacks.html#ZeroInWord
  return ~((((dword & 0x7F7F7F7F) + 0x7F7F7F7F) | dword) | 0x7F7F7F7F);
}
static uint32_t has_byte(uint32_t dword, uint8_t byte)
{
  return has_zero(dword ^ (~0UL / 255 * (byte)));
}

static int jpeg2000_bitbuf_refill_backwards(StateVars *buffer, const uint8_t *array)
{
  uint64_t tmp = 0;
  int32_t position = buffer->pos;
  int new_bits = 32;

  // TODO: (cae), confirm if we need to swap in BE systems.
  if (buffer->bits_left > 32)
    return 0; // enough data, no need to pull in more bits

  if (position >= 3) {
    position -= 4;
    memcpy(&tmp, array + position + 1, 4);
    tmp = (uint64_t)av_bswap32((uint32_t)tmp);
  } else {
    new_bits = (position + 1) * 8;

    if (position >= 2)
      tmp = array[position - 2];
    if (position >= 1)
      tmp = tmp << 8 | array[position - 1];
    if (position >= 0)
      tmp = tmp << 8 | array[position];
    position = 0;
  }
  // check for stuff bytes (0xff)
  if (has_byte(tmp, 0xff)) {
    // borrowed from open_htj2k ht_block_decoding.cpp

    // TODO(cae): confirm this is working

    // Load the next byte to check for stuffing.
    tmp <<= 8;
    tmp |= (uint64_t) * (array + position);
    if ((tmp & 0x7FFF000000) > 0x7F8F000000) {
      tmp &= 0x7FFFFFFFFF;
      new_bits--;
    }
    if ((tmp & 0x007FFF0000) > 0x007F8F0000) {
      tmp = (tmp & 0x007FFFFFFF) + ((tmp & 0xFF00000000) >> 1);
      new_bits--;
    }
    if ((tmp & 0x00007FFF00) > 0x00007F8F00) {
      tmp = (tmp & 0x00007FFFFF) + ((tmp & 0xFFFF000000) >> 1);
      new_bits--;
    }
    if ((tmp & 0x0000007FFF) > 0x0000007F8F) {
      tmp = (tmp & 0x0000007FFF) + ((tmp & 0xFFFFFF0000) >> 1);
      new_bits--;
    }
    // remove temporary byte loaded.
    tmp >>= 8;
  }
  // Add bits to the MSB of the bit buffer
  buffer->bit_buf |= tmp << buffer->bits_left;
  buffer->bits_left += new_bits;
  buffer->pos = position;
  return 0;
}
static void jpeg2000_bitbuf_refill_bytewise(StateVars *buffer, const uint8_t *array, uint32_t length)
{
  while (buffer->bits_left < 32) {
    buffer->tmp = 0xFF;
    buffer->bits = (buffer->last == 0xFF) ? 7 : 8;
    if (buffer->pos < length) {
      buffer->tmp = array[buffer->pos];
      buffer->pos += 1;
      buffer->last = buffer->tmp;
    }
    buffer->bit_buf |= ((uint64_t)buffer->tmp) << buffer->bits_left;
    buffer->bits_left += buffer->bits;
  }
}
static int jpeg2000_bitbuf_refill_forwards(StateVars *buffer, const uint8_t *array, uint32_t length)
{
  uint64_t tmp = 0;
  int32_t position = buffer->pos;
  int new_bits = 32;

  uint32_t remaining = av_sat_sub32(length, buffer->pos);

  // TODO: (cae), confirm if we need to swap in BE systems.
  if (buffer->bits_left > 32)
    return 0; // enough data, no need to pull in more bits

  if (remaining >= 4) {
    memcpy(&tmp, array + position, 4);
    position += 4;
  } else if (remaining == 3) {
    memcpy(&tmp, array + position, 3);
    position += 3;
    new_bits -= 8;
  } else if (remaining == 2) {
    memcpy(&tmp, array + position, 2);
    position += 2;
    new_bits -= 16;
  } else if (remaining == 1) {
    memcpy(&tmp, array + position, 1);
    position += 1;
    new_bits -= 24;
  } else {
    av_assert0(0);
    return 1;
  }
  // check for stuff bytes (0xff)
  if (has_byte(tmp, 0xff)) {
    // borrowed from open_htj2k ht_block_decoding.cpp

    // TODO(cae): confirm this is working

    // Load the next byte to check for stuffing.
    tmp <<= 8;
    tmp |= (uint64_t) * (array + position);
    if ((tmp & 0x7FFF000000) > 0x7F8F000000) {
      tmp &= 0x7FFFFFFFFF;
      new_bits--;
    }
    if ((tmp & 0x007FFF0000) > 0x007F8F0000) {
      tmp = (tmp & 0x007FFFFFFF) + ((tmp & 0xFF00000000) >> 1);
      new_bits--;
    }
    if ((tmp & 0x00007FFF00) > 0x00007F8F00) {
      tmp = (tmp & 0x00007FFFFF) + ((tmp & 0xFFFF000000) >> 1);
      new_bits--;
    }
    if ((tmp & 0x0000007FFF) > 0x0000007F8F) {
      tmp = (tmp & 0x0000007FFF) + ((tmp & 0xFFFFFF0000) >> 1);
      new_bits--;
    }
    // remove temporary byte loaded.
    tmp >>= 8;
  }
  // Add bits to the MSB of the bit buffer
  buffer->bit_buf |= tmp << buffer->bits_left;
  buffer->bits_left += new_bits;
  buffer->pos = position;
  return 0;
};

/**
 * @brief Drops bits from lower bits in the bit buffer
 *
 * @param buf: Struct containing bit buffers
 * @param nbits: Number of bits to remove.
 * */
static av_always_inline void jpeg2000_bitbuf_drop_bits_lsb(StateVars *buf, uint8_t nbits)
{
  av_assert0(buf->bits_left >= nbits);
  buf->bit_buf >>= nbits;
  buf->bits_left -= nbits;
}

static av_always_inline uint64_t jpeg2000_bitbuf_get_bits_lsb(StateVars *bit_stream, uint8_t nbits, const uint8_t *buf)
{

  uint64_t bits;
  uint64_t mask = (1 << nbits) - 1;
  if (bit_stream->bits_left < nbits)
    jpeg2000_bitbuf_refill_backwards(bit_stream, buf);

  bits = bit_stream->bit_buf & mask;

  jpeg2000_bitbuf_drop_bits_lsb(bit_stream, nbits);
  return bits;
};

static av_always_inline uint64_t jpeg2000_bitbuf_get_bits_lsb_forward(StateVars *bit_stream, uint8_t nbits, const uint8_t *buf, uint32_t length)
{

  uint64_t bits;
  uint64_t mask = (1 << nbits) - 1;
  if (bit_stream->bits_left <= nbits)
    // TODO: (cae) this may fail I  guess if there are no more bits,add a check for it.
    jpeg2000_bitbuf_refill_bytewise(bit_stream, buf, length);

  bits = bit_stream->bit_buf & mask;
  jpeg2000_bitbuf_drop_bits_lsb(bit_stream, nbits);
  return bits;
};

static av_always_inline uint64_t jpeg2000_bitbuf_peek_bits_lsb(StateVars *stream, uint8_t nbits)
{
  uint64_t mask = (1 << nbits) - 1;

  return stream->bit_buf & mask;
}

