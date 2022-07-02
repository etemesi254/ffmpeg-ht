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


#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#define MAX(a, b) (((a) > (b)) ? (a) : (b))

/**
 * @brief Table 2 in clause 7.3.3
 * */
const static uint8_t MEL_E[13] = {0, 0, 0, 1, 1, 1, 2, 2, 2, 3, 3, 4, 5};

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

/*Initializers*/

static void jpeg2000_init_zero(StateVars *s)
{
  s->bits_left = 0;
  s->bit_buf = 0;
  s->tmp = 0;
  s->bits = 0;
  s->pos = 0;
  s->last = 0;
}
/*Initialize MEL bit stream*/
static void jpeg2000_init_mel(StateVars *s, uint32_t Pcup)
{
  jpeg2000_init_zero(s);
  s->pos = Pcup;
}

static void jpeg2000_init_vlc(StateVars *s, uint32_t Lcup, uint32_t Pcup, const uint8_t *Dcup)
{
  s->bits_left = 0;
  s->bit_buf = 0;
  s->pos = Lcup - 2 - Pcup;
  s->last = Dcup[Lcup - 2];
  s->tmp = (s->last) >> 4;
  s->bits = ((s->tmp & 7) < 7) ? 4 : 3;
}

static void jpeg2000_init_mag_ref(StateVars *s, uint32_t Lref)
{
  s->pos = Lref - 1;
  s->bits = 0;
  s->last = 0xFF;
  s->tmp = 0;
}

static void jpeg2000_init_mel_decoder(MelDecoderState *mel_state)
{
  mel_state->k = 0;
  mel_state->run = 0;
  mel_state->one = 0;
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

/* VLC decoding utilities */

static int jpeg2000_decode_ctx_vlc(Jpeg2000DecoderContext *s, StateVars *vlc_stream, const uint16_t *table, const uint8_t *Dcup, uint8_t *sig_pat, uint8_t *res_off, uint8_t *emb_pat_k, uint8_t *emb_pat_1, uint8_t pos, uint32_t Pcup, uint16_t context)
{
  uint32_t value;
  uint8_t len;
  int index;
  int code_word;

  jpeg2000_bitbuf_refill_backwards(vlc_stream, Dcup + Pcup);

  code_word = vlc_stream->bit_buf & 0x7f;
  index = code_word + (context << 7);

  // decode table has 1024 entries so ensure array access is in bounds
  av_assert0(index < 1024);

  value = table[index];
  len = (value & 0x000F) >> 1;

  res_off[pos] = (uint8_t)(value & 1);
  sig_pat[pos] = (uint8_t)((value & 0x00F0) >> 4);
  emb_pat_k[pos] = (uint8_t)((value & 0x0F00) >> 8);
  emb_pat_1[pos] = (uint8_t)((value & 0xF000) >> 12);
  jpeg2000_bitbuf_drop_bits_lsb(vlc_stream, len);
  return 0;
}


static av_always_inline uint8_t vlc_decode_u_prefix(StateVars *vlc_stream, const uint8_t *refill_array)
{
  uint8_t bits;
  if (vlc_stream->bits_left < 3)
    jpeg2000_bitbuf_refill_backwards(vlc_stream, refill_array);

  bits = jpeg2000_bitbuf_peek_bits_lsb(vlc_stream, 3);

  if (bits & 0b1) {
    jpeg2000_bitbuf_drop_bits_lsb(vlc_stream, 1);
    return 1;
  }
  if (bits & 0b10) {
    jpeg2000_bitbuf_drop_bits_lsb(vlc_stream, 2);
    return 2;
  }
  jpeg2000_bitbuf_drop_bits_lsb(vlc_stream, 3);

  if (bits & 0b100)
    return 3;
  else
    return 5;
}

static av_always_inline uint8_t vlc_decode_u_suffix(StateVars *vlc_stream, uint8_t suffix, const uint8_t *refill_array)
{
  uint8_t bits;
  if (suffix < 3)
    return 0;

  if (vlc_stream->bits_left < 5)
    jpeg2000_bitbuf_refill_backwards(vlc_stream, refill_array);

  bits = jpeg2000_bitbuf_peek_bits_lsb(vlc_stream, 5);

  if (suffix == 3) {
    jpeg2000_bitbuf_drop_bits_lsb(vlc_stream, 1);
    return bits & 1;
  }
  jpeg2000_bitbuf_drop_bits_lsb(vlc_stream, 5);

  return bits;
}

static av_always_inline uint8_t vlc_decode_u_extension(StateVars *vlc_stream, uint8_t suffix, const uint8_t *refill_array)
{
  uint8_t bits;
  if (suffix < 28)
    return 0;
  bits = jpeg2000_bitbuf_get_bits_lsb(vlc_stream, 4, refill_array);
  return bits;
}

static int32_t jpeg2000_decode_mag_sgn(StateVars *mag_sgn_stream, int32_t m_n, int32_t i_n, const uint8_t *buf, uint32_t length)
{
  int32_t val = 0;
  if (m_n > 0) {
    val = jpeg2000_bitbuf_get_bits_lsb_forward(mag_sgn_stream, m_n, buf, length);
    val += (i_n << m_n);
  }
  return val;
}

static av_always_inline void recover_mag_sgn(StateVars *mag_sgn, uint8_t pos, uint16_t q, int32_t m_n[2], int32_t known_1[2], const uint8_t emb_pat_1[2], int32_t v[2][4], const int32_t m[2][4], uint8_t *E, uint32_t *mu_n, const uint8_t *Dcup, uint32_t Pcup, uint32_t pLSB)
{
  for (int i = 0; i < 4; i++) {
    int32_t n = 4 * q + i;
    m_n[pos] = m[pos][i];
    known_1[pos] = (emb_pat_1[pos] >> i) & 1;
    v[pos][i] = jpeg2000_decode_mag_sgn(mag_sgn, m_n[pos], known_1[pos], Dcup, Pcup);
    if (m_n[pos] != 0) {
      E[n] = 32 - ff_clz(v[pos][i]);
      mu_n[n] = (v[pos][i] >> 1) + 1;
      mu_n[n] <<= pLSB;
      mu_n[n] |= ((uint32_t)(v[pos][i] & 1)) << 31; // sign bit.
    }
  }
}
/*MEL stream decoding procedure*/

static int jpeg2000_import_mel_bit(StateVars *mel_stream, const uint8_t *Dcup, uint32_t Lcup)
{
  // TODO (cae): Figure out how to use the other refill method here.
  if (mel_stream->bits == 0) {
    mel_stream->bits = (mel_stream->tmp == 0xFF) ? 7 : 8;
    if (mel_stream->pos < Lcup) {
      mel_stream->tmp = Dcup[mel_stream->pos];
      mel_stream->pos += 1;
    } else
      mel_stream->tmp = 0xFF;
  }
  mel_stream->bits -= 1;

  return (mel_stream->tmp >> mel_stream->bits) & 1;
}

static int jpeg2000_decode_mel_sym(MelDecoderState *mel_state, StateVars *mel_stream, const uint8_t *Dcup, uint32_t Lcup)
{

  if (mel_state->run == 0 && mel_state->one == 0) {
    uint8_t eval;
    uint8_t bit;

    eval = MEL_E[mel_state->k];
    bit = jpeg2000_import_mel_bit(mel_stream, Dcup, Lcup);
    if (bit == 1) {
      mel_state->run = 1 << eval;
      mel_state->k = MIN(12, mel_state->k + 1);
    } else {
      mel_state->run = 0;
      while (eval > 0) {
        bit = jpeg2000_import_mel_bit(mel_stream, Dcup, Lcup);
        mel_state->run = (2 * (mel_state->run)) + bit;
        eval -= 1;
      }
      mel_state->k = MAX(0, mel_state->k - 1);
      mel_state->one = 1;
    }
  }
  if (mel_state->run > 0) {
    mel_state->run -= 1;
    return 0;
  } else {
    mel_state->one = 0;
    return 1;
  }
}


static int jpeg2000_decode_ht_cleanup(Jpeg2000DecoderContext *s, Jpeg2000Cblk *cblk,Jpeg2000T1Context *t1, MelDecoderState *mel_state, StateVars *mel_stream, StateVars *vlc_stream, StateVars *mag_sgn_stream, const uint8_t *Dcup, uint32_t Lcup, uint32_t Pcup, uint8_t pLSB, int width, int height)
{

  return 1;

}
int decode_htj2k(Jpeg2000DecoderContext *s, Jpeg2000CodingStyle *codsty, Jpeg2000T1Context *t1, Jpeg2000Cblk *cblk, int width, int height, int bandpos, uint8_t roi_shift)
{
  uint8_t p0 = 0;    // Number of placeholder passes.
  uint32_t Lcup;     // Length of HT cleanup segment.
  uint32_t Lref = 0; // Length of Refinement segment.
  uint32_t Scup;     // HT cleanup segment suffix length.
  uint32_t Pcup;     // HT cleanup segment prefix length.

  uint8_t S_blk; // Bumber of skipped magnitude bitplanes;
  uint8_t pLSB;

  uint8_t *Dcup; // Byte of an HT cleanup segment.
  uint8_t *Dref; // Byte of an HT refinement segment.


  int z_blk; // Number of ht coding pass

  uint8_t empty_passes;

  StateVars mag_sgn;  // Magnitude and Sign
  StateVars mel;      // Adaptive run-length coding
  StateVars vlc;      // Variable Length coding
  StateVars sig_prop; // Significance propagation
  StateVars mag_ref;  // Magnitude and refinement.

  MelDecoderState mel_state;

  av_assert0(width <= 1024U && height <= 1024U);
  av_assert0(width*height <= 4096);
  memset(t1->data,0, t1->stride*height*sizeof(*t1->data));

  memset(t1->flags, 0, t1->stride * (height + 2) * sizeof(*t1->flags));
  int ret;

  if (cblk->npasses == 0) {
    return 0;
  }

  if (cblk->npasses > 3)
    // TODO:(cae) Add correct support for this
    // Currently use this as a dummy but should be fixed soon
    p0 = 0;
  else if (cblk->length == 0)
    p0 = 1;

  empty_passes = p0 * 3;
  z_blk = cblk->npasses - empty_passes;

  if (z_blk <= 0)
    // no passes within this set, continue
    return 0;

  Lcup = cblk->length;
  if (Lcup < 2) {
    av_log(s->avctx, AV_LOG_ERROR, "Cleanup pass length must be at least 2 bytes in length");
    return AVERROR_INVALIDDATA;
  }
  Dcup = cblk->data;
  // Dref comes after the refinement segment.
  Dref = cblk->data + Lcup;
  S_blk = p0 + cblk->zbp;

  pLSB = 30 - S_blk;

  Scup = (Dcup[Lcup - 1] << 4) | (Dcup[Lcup - 2] & 0x0F);

  if (Scup < 2 || Scup > Lcup || Scup > 4079) {
    av_log(s->avctx, AV_LOG_ERROR, "Cleanup pass suffix length is invalid %d", Scup);
    return AVERROR_INVALIDDATA;
  }

  Pcup = Lcup - Scup;

  // modDcup (shall be done before the creation of state_VLC instance)
  Dcup[Lcup - 1] = 0xFF;
  Dcup[Lcup - 2] |= 0x0F;

  jpeg2000_init_zero(&mag_sgn);
  jpeg2000_bitbuf_refill_forwards(&mag_sgn, Dcup, Pcup);

  jpeg2000_init_zero(&sig_prop);

  jpeg2000_init_mel(&mel, Pcup);

  jpeg2000_init_vlc(&vlc, Lcup, Pcup, Dcup);
  jpeg2000_bitbuf_refill_backwards(&vlc, Dcup + Pcup);
  jpeg2000_bitbuf_drop_bits_lsb(&vlc, 4);

  jpeg2000_init_mag_ref(&mag_ref, Lref);

  jpeg2000_init_mel_decoder(&mel_state);

  ret = jpeg2000_decode_ht_cleanup(s, cblk,t1, &mel_state, &mel, &vlc, &mag_sgn, Dcup, Lcup, Pcup, pLSB, width, height);

  return ret;


}
