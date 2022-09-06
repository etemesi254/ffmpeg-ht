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
#include "jpeg2000htdec.h"
#include <stdint.h>
#include <stdlib.h>

#include "bytestream.h"

#define J2K_Q1 0

#define J2K_Q2 1

#define HT_SHIFT_SIGMA 0
#define HT_SHIFT_SCAN 4
#define HT_SHIFT_REF 3
#define HT_SHIFT_REF_IND 2
/**
 * @brief Table 2 in clause 7.3.3
 * */
const static uint8_t MEL_E[13] = {0, 0, 0, 1, 1, 1, 2, 2, 2, 3, 3, 4, 5};

/**
 *  Given a precomputed c, checks whether n % d == 0
 **/
static av_always_inline uint32_t is_divisible(uint32_t n, uint64_t c)
{
    return n * c <= c - 1;
}

/* Initializers */

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

static void jpeg2000_init_mag_ref(StateVars *s, uint32_t Lref)
{
    s->pos = Lref - 2;
    s->bits = 0;
    s->last = 0xFF;
    s->tmp = 0;
    s->bits_left = 0;
    s->bit_buf = 0;
}

static void jpeg2000_init_mel_decoder(MelDecoderState *mel_state)
{
    mel_state->k = 0;
    mel_state->run = 0;
    mel_state->one = 0;
}
/**
 * Refill the buffer backwards in little Endian while skipping
 * over stuffing bits that appear in the position of any byte whose
 * LSBs are all 1's if the last consumed byte was larger than 0x8F
 */
static int jpeg2000_bitbuf_refill_backwards(StateVars *buffer,
                                            const uint8_t *array)
{
    uint64_t tmp = 0;
    int32_t position = buffer->pos;
    uint32_t new_bits = 32;
    uint32_t mask;

    if (buffer->bits_left > 32)
        return 0; // enough data, no need to pull in more bits

    /*
     * We are reading bits from end to start, and we need to handle them in LE.
     * therefore, if we read bytes ABCD, we need to convert
     * them to DCBA, but the bitstream is constructed in such a way that it is
     * BE when reading from back to front,so we need to swap bytes
     * but this doesn't work when position is less than 3,
     * we end up reading bits from the MEL byte-stream which is a recipe for sleepless nights.
     *
     * So the trick is to branchlessly read and mask
     * depending on whatever was the initial position,
     * the mask is simply either 32 bits, 24 bits ,8 bits or 0 bits.
     * depending on how many bytes are available, with this, we don't read past the end of the buffer,we mask
     * bits we already read ensuring we don't read twice.
     * and we can do it branchlessly without checking for positions.
     *
     * We need to watch out for negative shift values which are UB in C hence the
     * MAX declarative.
     * */

    position -= 4;
    mask = (UINT64_C(1) << (FFMIN(4, FFMAX(buffer->pos, 0))) * 8) - 1;
    memcpy(&tmp, array + 1 + position, 4);
    tmp = (uint64_t)av_be2ne32((uint32_t)tmp) & mask;

    // Branchlessly unstuff  bits

    // load temporary byte, which preceeds the position we
    // currently at, to ensure that we can also un-stuff if the
    // stuffed bit is the bottom most bits
    tmp <<= 8;
    tmp |= array[buffer->pos + 1];

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

    // Add bits to the MSB of the bit buffer
    buffer->bit_buf |= tmp << buffer->bits_left;
    buffer->bits_left += new_bits;
    buffer->pos = FFMAX(-1, position);
    return 0;
}

/* Refill  the stream with bytes */
static void jpeg2000_bitbuf_refill_bytewise(StateVars *buffer,
                                            const uint8_t *array,
                                            uint32_t length)
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

/**
 * @brief Drops bits from lower bits in the bit buffer
 *
 * @param buf: Struct containing bit buffers
 * @param nbits: Number of bits to remove.
 * */
static av_always_inline void jpeg2000_bitbuf_drop_bits_lsb(StateVars *buf,
                                                           uint8_t nbits)
{
    if (buf->bits_left < nbits) {
        av_log(NULL, AV_LOG_ERROR, "Invalid bit read of %d, bits in buffer are %d", nbits, buf->bits_left);
        av_assert0(0);
    }
    buf->bit_buf >>= nbits;
    buf->bits_left -= nbits;
}
/**
 * Get bits from the bit buffer reading them
 * from the least significant bits moving to the most significant bits.
 * in case there are fewer bits, refill from `buf` moving backwards.
 * */
static av_always_inline uint64_t jpeg2000_bitbuf_get_bits_lsb(
    StateVars *bit_stream, uint8_t nbits, const uint8_t *buf)
{
    uint64_t bits;
    uint64_t mask = (1ull << nbits) - 1;
    if (bit_stream->bits_left < nbits)
        jpeg2000_bitbuf_refill_backwards(bit_stream, buf);
    bits = bit_stream->bit_buf & mask;
    jpeg2000_bitbuf_drop_bits_lsb(bit_stream, nbits);
    return bits;
}
/**
 * Get bits from the bit buffer reading them
 * from the least significant bits moving to the most significant bits
 * in case there are fewer bits, refill from `buf` moving forward
 * */
static av_always_inline uint64_t jpeg2000_bitbuf_get_bits_lsb_forward(
    StateVars *bit_stream, uint8_t nbits, const uint8_t *buf, uint32_t length)
{
    uint64_t bits;
    uint64_t mask = (1ull << nbits) - 1;
    if (bit_stream->bits_left <= nbits)
        jpeg2000_bitbuf_refill_bytewise(bit_stream, buf, length);
    bits = bit_stream->bit_buf & mask;
    jpeg2000_bitbuf_drop_bits_lsb(bit_stream, nbits);
    return bits;
}
/**
 * Look ahead bit buffer without discarding bits
 * */
static av_always_inline uint64_t
jpeg2000_bitbuf_peek_bits_lsb(StateVars *stream, uint8_t nbits)
{
    uint64_t mask = (1ull << nbits) - 1;

    return stream->bit_buf & mask;
}

/* *
 * Variable Length Decoding Routines
 */

static void jpeg2000_init_vlc(StateVars *s, uint32_t Lcup, uint32_t Pcup, const uint8_t *Dcup)
{
    s->bits_left = 0;
    s->bit_buf = 0;
    s->pos = Lcup - 2 - Pcup;
    s->last = Dcup[Lcup - 2];
    s->tmp = (s->last) >> 4;
    s->bits = ((s->tmp & 7) < 7) ? 4 : 3;
    jpeg2000_bitbuf_refill_backwards(s, Dcup + Pcup);
    jpeg2000_bitbuf_drop_bits_lsb(s, 4);
}
/**
 * Decode prefix codes for VLC segment.
 */
static int jpeg2000_decode_ctx_vlc(Jpeg2000DecoderContext *s,
                                   StateVars *vlc_stream,
                                   const uint16_t *table,
                                   const uint8_t *Dcup,
                                   uint8_t *sig_pat,
                                   uint8_t *res_off,
                                   uint8_t *emb_pat_k,
                                   uint8_t *emb_pat_1,
                                   uint8_t pos,
                                   uint32_t Pcup,
                                   uint16_t context)
{
    // Described in clause 7.3.5
    uint32_t value;
    uint8_t len;
    uint64_t index;
    uint64_t code_word;

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
/**
 * Decode variable length u-vlc prefix
 * */
static av_always_inline uint8_t
vlc_decode_u_prefix(StateVars *vlc_stream, const uint8_t *refill_array)
{
    // clause 7.3.6
    // procedure : decodeUPrefix.

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
    return 5;
}

static av_always_inline uint8_t vlc_decode_u_suffix(
    StateVars *vlc_stream, uint8_t suffix, const uint8_t *refill_array)
{
    // clause 7.3.6
    // procedure: decodeUSuffix

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

static av_always_inline uint8_t vlc_decode_u_extension(
    StateVars *vlc_stream, uint8_t suffix, const uint8_t *refill_array)
{
    // clause 7.3.6
    // procedure decodeUExtension.
    if (suffix < 28)
        return 0;
    return jpeg2000_bitbuf_get_bits_lsb(vlc_stream, 4, refill_array);
}

/* Magnitude and Sign decode procedures*/

static int32_t jpeg2000_decode_mag_sgn(StateVars *mag_sgn_stream, int32_t m_n, int32_t i_n, const uint8_t *buf, uint32_t length)
{
    // clause 7.3.8
    // procedure: decodeMagSgnValue

    int32_t val = 0;
    if (m_n > 0) {
        val = jpeg2000_bitbuf_get_bits_lsb_forward(mag_sgn_stream, m_n, buf, length);
        val += (i_n << m_n);
    }
    return val;
}

static av_always_inline void
recover_mag_sgn(StateVars *mag_sgn, uint8_t pos, uint16_t q, int32_t m_n[2], int32_t known_1[2], const uint8_t emb_pat_1[2], int32_t v[2][4], int32_t m[2][4], uint8_t *E, uint32_t *mu_n, const uint8_t *Dcup, uint32_t Pcup, uint32_t pLSB)
{
    for (int i = 0; i < 4; i++) {
        int32_t n = 4 * q + i;
        m_n[pos] = m[pos][i];
        known_1[pos] = (emb_pat_1[pos] >> i) & 1;
        v[pos][i] = jpeg2000_decode_mag_sgn(mag_sgn, m_n[pos], known_1[pos], Dcup, Pcup);
        if (m_n[pos] != 0) {
            E[n] = 32 - ff_clz(v[pos][i] | 1);
            mu_n[n] = (v[pos][i] >> 1) + 1;
            mu_n[n] <<= pLSB;
            mu_n[n] |= ((uint32_t)(v[pos][i] & 1)) << 31; // sign bit.
        }
    }
}
/*MEL stream decoding procedure*/

static int jpeg2000_import_bit(StateVars *stream, const uint8_t *array, uint32_t length)
{
    if (stream->bits == 0) {
        stream->bits = (stream->tmp == 0xFF) ? 7 : 8;
        if (stream->pos < length) {
            stream->tmp = array[stream->pos];
            stream->pos += 1;
        } else
            stream->tmp = 0xFF;
    }
    stream->bits -= 1;
    return (stream->tmp >> stream->bits) & 1;
}

static int jpeg2000_decode_mel_sym(MelDecoderState *mel_state,
                                   StateVars *mel_stream,
                                   const uint8_t *Dcup,
                                   uint32_t Lcup)
{

    if (mel_state->run == 0 && mel_state->one == 0) {
        uint8_t eval;
        uint8_t bit;

        eval = MEL_E[mel_state->k];
        bit = jpeg2000_import_bit(mel_stream, Dcup, Lcup);
        if (bit == 1) {
            mel_state->run = 1 << eval;
            mel_state->k = FFMIN(12, mel_state->k + 1);
        } else {
            mel_state->run = 0;
            while (eval > 0) {
                bit = jpeg2000_import_bit(mel_stream, Dcup, Lcup);
                mel_state->run = (2 * (mel_state->run)) + bit;
                eval -= 1;
            }
            mel_state->k = FFMAX(0, mel_state->k - 1);
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

static av_always_inline int jpeg2000_import_magref_bit(StateVars *stream, const uint8_t *array, uint32_t length)
{
    return jpeg2000_bitbuf_get_bits_lsb(stream, 1, array);
}
/* Signal EMB decode */
static int
jpeg2000_decode_sig_emb(Jpeg2000DecoderContext *s, MelDecoderState *mel_state, StateVars *mel_stream, StateVars *vlc_stream, const uint16_t *vlc_table, const uint8_t *Dcup, uint8_t *sig_pat, uint8_t *res_off, uint8_t *emb_pat_k, uint8_t *emb_pat_1, uint8_t pos, uint16_t context, uint32_t Lcup, uint32_t Pcup)
{
    if (context == 0) {
        uint8_t sym;
        sym = jpeg2000_decode_mel_sym(mel_state, mel_stream, Dcup, Lcup);
        if (sym == 0) {
            sig_pat[pos] = 0;
            res_off[pos] = 0;
            emb_pat_k[pos] = 0;
            emb_pat_1[pos] = 0;
            return 0;
        }
    }
    return jpeg2000_decode_ctx_vlc(s, vlc_stream, vlc_table, Dcup, sig_pat,
                                   res_off, emb_pat_k, emb_pat_1, pos, Pcup,
                                   context);
}

static av_always_inline int jpeg2000_get_state(int x1, int x2, int width, int shift_by, const uint8_t *block_states)
{
    return (block_states[(x1 + 1) * (width + 2) + (x2 + 1)] >> shift_by) & 1;
}
static av_always_inline void jpeg2000_modify_state(int x1, int x2, int width, int value, uint8_t *block_states)
{
    block_states[(x1 + 1) * (width + 2) + (x2 + 1)] |= value;
}

static int jpeg2000_decode_ht_cleanup(
    Jpeg2000DecoderContext *s, Jpeg2000Cblk *cblk, Jpeg2000T1Context *t1, MelDecoderState *mel_state, StateVars *mel_stream, StateVars *vlc_stream, StateVars *mag_sgn_stream, const uint8_t *Dcup, uint32_t Lcup, uint32_t Pcup, uint8_t pLSB, int width, int height, int32_t *sample_buf, uint8_t *block_states)
{
    uint16_t q = 0; // Represents current quad position.
    uint16_t q1, q2;
    uint16_t context1, context2;
    uint16_t context = 0;

    uint8_t sig_pat[2] = {0};   // significance pattern
    uint8_t res_off[2] = {0};   // residual offset
    uint8_t emb_pat_k[2] = {0}; // Exponent Max Bound pattern K
    uint8_t emb_pat_1[2] = {0}; // Exponent Max Bound pattern 1.
    uint8_t gamma[2] = {0};

    uint8_t E_n[2] = {0};
    uint8_t E_ne[2] = {0};
    uint8_t E_nw[2] = {0};
    uint8_t E_nf[2] = {0};

    uint8_t max_e[2] = {0};

    uint8_t u_pfx[2] = {0};
    uint8_t u_sfx[2] = {0};
    uint8_t u_ext[2] = {0};

    int32_t u[2] = {0};
    int32_t U[2] = {0}; // Exponent bound (7.3.7)
    int32_t m_n[2] = {0};
    int32_t known_1[2] = {0};

    int32_t m[2][4] = {0};
    int32_t v[2][4] = {0};

    uint8_t kappa[2] = {1, 1};
    int ret = 0;

    int sp;

    uint64_t c;

    uint8_t *sigma;
    uint32_t *mu;

    const uint8_t *vlc_buf = Dcup + Pcup;
    // convert to raster-scan
    const uint16_t is_border_x = width % 2;
    const uint16_t is_border_y = height % 2;

    int j1, j2;

    const uint16_t quad_width = ff_jpeg2000_ceildivpow2(width, 1);
    const uint16_t quad_height = ff_jpeg2000_ceildivpow2(height, 1);

    size_t buf_size = 4 * quad_width * quad_height;

    uint8_t *sigma_n = av_calloc(buf_size, sizeof(uint8_t));
    uint8_t *E = av_calloc(buf_size, sizeof(uint8_t));
    uint32_t *mu_n = av_calloc(buf_size, sizeof(uint32_t));

    if (!sigma_n || !E || !mu_n) {
        ret = AVERROR(ENOMEM);
        goto free;
    }

    sigma = sigma_n;
    mu = mu_n;

    while (q < quad_width - 1) {
        q1 = q;
        q2 = q1 + 1;

        if ((ret = jpeg2000_decode_sig_emb(s, mel_state, mel_stream, vlc_stream,
                                           dec_CxtVLC_table0, Dcup, sig_pat, res_off,
                                           emb_pat_k, emb_pat_1, J2K_Q1, context, Lcup,
                                           Pcup))
            < 0)
            goto free;
        for (int i = 0; i < 4; i++)
            sigma_n[4 * q1 + i] = (sig_pat[J2K_Q1] >> i) & 1;

        // calculate context
        context = sigma_n[4 * q1];           // f
        context |= sigma_n[4 * q1 + 1];      // sf
        context += sigma_n[4 * q1 + 2] << 1; // w << 1
        context += sigma_n[4 * q1 + 3] << 2;

        if ((ret = jpeg2000_decode_sig_emb(s, mel_state, mel_stream, vlc_stream,
                                           dec_CxtVLC_table0, Dcup, sig_pat, res_off,
                                           emb_pat_k, emb_pat_1, J2K_Q2, context, Lcup,
                                           Pcup))
            < 0)
            goto free;

        for (int i = 0; i < 4; i++)
            sigma_n[4 * q2 + i] = (sig_pat[J2K_Q2] >> i) & 1;

        // calculate context for the next quad
        context = sigma_n[4 * q2];           // f
        context |= sigma_n[4 * q2 + 1];      // sf
        context += sigma_n[4 * q2 + 2] << 1; // w << 1
        context += sigma_n[4 * q2 + 3] << 2; // sw << 2

        u[0] = 0;
        u[1] = 0;

        jpeg2000_bitbuf_refill_backwards(vlc_stream, vlc_buf);

        if (res_off[J2K_Q1] == 1 && res_off[J2K_Q2] == 1) {

            if (jpeg2000_decode_mel_sym(mel_state, mel_stream, Dcup, Lcup) == 1) {

                u_pfx[J2K_Q1] = vlc_decode_u_prefix(vlc_stream, vlc_buf);
                u_pfx[J2K_Q2] = vlc_decode_u_prefix(vlc_stream, vlc_buf);

                u_sfx[J2K_Q1] = vlc_decode_u_suffix(vlc_stream, u_pfx[J2K_Q1], vlc_buf);
                u_sfx[J2K_Q2] = vlc_decode_u_suffix(vlc_stream, u_pfx[J2K_Q2], vlc_buf);

                u_ext[J2K_Q1] = vlc_decode_u_extension(vlc_stream, u_sfx[J2K_Q1], vlc_buf);
                u_ext[J2K_Q2] = vlc_decode_u_extension(vlc_stream, u_sfx[J2K_Q2], vlc_buf);

                u[J2K_Q1] = 2 + u_pfx[J2K_Q1] + u_sfx[J2K_Q1] + (u_ext[J2K_Q1] * 4);
                u[J2K_Q2] = 2 + u_pfx[J2K_Q2] + u_sfx[J2K_Q2] + (u_ext[J2K_Q2] * 4);

            } else {
                u_pfx[J2K_Q1] = vlc_decode_u_prefix(vlc_stream, vlc_buf);

                if (u_pfx[J2K_Q1] > 2) {
                    u[J2K_Q2] = jpeg2000_bitbuf_get_bits_lsb(vlc_stream, 1, vlc_buf) + 1;

                    u_sfx[J2K_Q1] = vlc_decode_u_suffix(vlc_stream, u_pfx[J2K_Q1], vlc_buf);

                    u_ext[J2K_Q1] = vlc_decode_u_extension(vlc_stream, u_sfx[J2K_Q1], vlc_buf);
                } else {
                    u_pfx[J2K_Q2] = vlc_decode_u_prefix(vlc_stream, vlc_buf);

                    u_sfx[J2K_Q1] = vlc_decode_u_suffix(vlc_stream, u_pfx[J2K_Q1], vlc_buf);
                    u_sfx[J2K_Q2] = vlc_decode_u_suffix(vlc_stream, u_pfx[J2K_Q2], vlc_buf);

                    u_ext[J2K_Q1] = vlc_decode_u_extension(vlc_stream, u_sfx[J2K_Q1], vlc_buf);
                    u_ext[J2K_Q2] = vlc_decode_u_extension(vlc_stream, u_sfx[J2K_Q2], vlc_buf);

                    u[J2K_Q2] = u_pfx[J2K_Q2] + u_sfx[J2K_Q2] + (u_ext[J2K_Q2] * 4);
                }
                // clause 7.3.6 (3)
                u[J2K_Q1] = u_pfx[J2K_Q1] + u_sfx[J2K_Q1] + (u_ext[J2K_Q1] * 4);
            }

        } else if (res_off[J2K_Q1] == 1 || res_off[J2K_Q2] == 1) {
            uint8_t pos = res_off[J2K_Q1] == 1 ? 0 : 1;

            u_pfx[pos] = vlc_decode_u_prefix(vlc_stream, vlc_buf);

            u_sfx[pos] = vlc_decode_u_suffix(vlc_stream, u_pfx[pos], vlc_buf);

            u_ext[pos] = vlc_decode_u_extension(vlc_stream, u_sfx[pos], vlc_buf);

            u[pos] = u_pfx[pos] + u_sfx[pos] + (u_ext[pos] * 4);
        }
        U[J2K_Q1] = kappa[J2K_Q1] + u[J2K_Q1];
        U[J2K_Q2] = kappa[J2K_Q2] + u[J2K_Q2];

        for (int i = 0; i < 4; i++) {
            m[J2K_Q1][i] = sigma_n[4 * q1 + i] * U[J2K_Q1] - ((emb_pat_k[J2K_Q1] >> i) & 1);
            m[J2K_Q2][i] = sigma_n[4 * q2 + i] * U[J2K_Q2] - ((emb_pat_k[J2K_Q2] >> i) & 1);
        }

        recover_mag_sgn(mag_sgn_stream, J2K_Q1, q1, m_n, known_1, emb_pat_1, v, m,
                        E, mu_n, Dcup, Pcup, pLSB);

        recover_mag_sgn(mag_sgn_stream, J2K_Q2, q2, m_n, known_1, emb_pat_1, v, m,
                        E, mu_n, Dcup, Pcup, pLSB);

        // prepare context for the next quad

        // move to the next quad pair
        q += 2;
    }
    if (quad_width % 2 == 1) { // If the quad width is an odd number
        q1 = q;

        if ((ret = jpeg2000_decode_sig_emb(s, mel_state, mel_stream, vlc_stream,
                                           dec_CxtVLC_table0, Dcup, sig_pat, res_off,
                                           emb_pat_k, emb_pat_1, J2K_Q1, context, Lcup,
                                           Pcup))
            < 0)
            goto free;

        for (int i = 0; i < 4; i++)
            sigma_n[4 * q1 + i] = (sig_pat[J2K_Q1] >> i) & 1;

        u[J2K_Q1] = 0;

        if (res_off[J2K_Q1] == 1) {
            u_pfx[J2K_Q1] = vlc_decode_u_prefix(vlc_stream, vlc_buf);
            u_sfx[J2K_Q1] = vlc_decode_u_suffix(vlc_stream, u_pfx[J2K_Q1], vlc_buf);
            u_ext[J2K_Q1] = vlc_decode_u_extension(vlc_stream, u_sfx[J2K_Q1], vlc_buf);
            u[J2K_Q1] = u_pfx[J2K_Q1] + u_sfx[J2K_Q1] + (u_ext[J2K_Q1] * 4);
        }

        U[J2K_Q1] = kappa[J2K_Q1] + u[J2K_Q1];

        for (int i = 0; i < 4; i++)
            m[J2K_Q1][i] = sigma_n[4 * q1 + i] * U[J2K_Q1] - ((emb_pat_k[J2K_Q1] >> i) & 1);

        recover_mag_sgn(mag_sgn_stream, J2K_Q1, q1, m_n, known_1, emb_pat_1, v, m,
                        E, mu_n, Dcup, Pcup, pLSB);

        q++; // move to next quad pair
    }
    // initial line pair end.

    /*
     * As an optimization, we can replace modulo operations with
     * checking if a number is divisible , since that's the only thing we need.
     * this is paired with is_divisible.
     * Credits to Daniel Lemire blog post: https://lemire.me/blog/2019/02/08/faster-remainders-when-the-divisor-is-a-constant-beating-compilers-and-libdivide/
     * It's UB on zero, but we can't have a quad being zero, the spec doesn't allow, so we error out early in case that's the case.
     * */

    c = 1 + UINT64_C(0xffffffffffffffff) / quad_width;

    for (int row = 1; row < quad_height; row++) {
        while ((q - (row * quad_width)) < quad_width - 1 && q < (quad_height * quad_width)) {
            q1 = q;
            q2 = q + 1;
            context1 = sigma_n[4 * (q1 - quad_width) + 1];
            context1 += sigma_n[4 * (q1 - quad_width) + 3] << 2; // ne

            if (!is_divisible(q1, c)) {
                context1 |= sigma_n[4 * (q1 - quad_width) - 1];               // nw
                context1 += (sigma_n[4 * q1 - 1] | sigma_n[4 * q1 - 2]) << 1; // sw| q
            }
            if (!is_divisible(q1 + 1, c))
                context1 |= sigma_n[4 * (q1 - quad_width) + 5] << 2;

            if ((ret = jpeg2000_decode_sig_emb(s, mel_state, mel_stream, vlc_stream,
                                               dec_CxtVLC_table1, Dcup, sig_pat, res_off,
                                               emb_pat_k, emb_pat_1, J2K_Q1, context1, Lcup,
                                               Pcup))
                < 0)
                goto free;

            for (int i = 0; i < 4; i++)
                sigma_n[4 * q1 + i] = (sig_pat[J2K_Q1] >> i) & 1;

            context2 = sigma_n[4 * (q2 - quad_width) + 1];
            context2 += sigma_n[4 * (q2 - quad_width) + 3] << 2;

            if (!is_divisible(q2, c)) {
                context2 |= sigma_n[4 * (q2 - quad_width) - 1];
                context2 += (sigma_n[4 * q2 - 1] | sigma_n[4 * q2 - 2]) << 1;
            }
            if (!is_divisible(q2 + 1, c))
                context2 |= sigma_n[4 * (q2 - quad_width) + 5] << 2;

            if ((ret = jpeg2000_decode_sig_emb(s, mel_state, mel_stream, vlc_stream,
                                               dec_CxtVLC_table1, Dcup, sig_pat, res_off,
                                               emb_pat_k, emb_pat_1, J2K_Q2, context2, Lcup,
                                               Pcup))
                < 0)
                goto free;

            for (int i = 0; i < 4; i++)
                sigma_n[4 * q2 + i] = (sig_pat[J2K_Q2] >> i) & 1;

            // fallback if res_off = [0,0]
            u[J2K_Q1] = 0;
            u[J2K_Q2] = 0;

            jpeg2000_bitbuf_refill_backwards(vlc_stream, vlc_buf);

            if (res_off[J2K_Q1] == 1 && res_off[J2K_Q2] == 1) {
                u_pfx[J2K_Q1] = vlc_decode_u_prefix(vlc_stream, vlc_buf);
                u_pfx[J2K_Q2] = vlc_decode_u_prefix(vlc_stream, vlc_buf);

                u_sfx[J2K_Q1] = vlc_decode_u_suffix(vlc_stream, u_pfx[J2K_Q1], vlc_buf);
                u_sfx[J2K_Q2] = vlc_decode_u_suffix(vlc_stream, u_pfx[J2K_Q2], vlc_buf);

                u_ext[J2K_Q1] = vlc_decode_u_extension(vlc_stream, u_sfx[J2K_Q1], vlc_buf);
                u_ext[J2K_Q2] = vlc_decode_u_extension(vlc_stream, u_sfx[J2K_Q2], vlc_buf);

                u[J2K_Q1] = u_pfx[J2K_Q1] + u_sfx[J2K_Q1] + (u_ext[J2K_Q1] << 2);
                u[J2K_Q2] = u_pfx[J2K_Q2] + u_sfx[J2K_Q2] + (u_ext[J2K_Q2] << 2);

            } else if (res_off[J2K_Q1] == 1 || res_off[J2K_Q2] == 1) {
                uint8_t pos = res_off[J2K_Q1] == 1 ? 0 : 1;

                u_pfx[pos] = vlc_decode_u_prefix(vlc_stream, vlc_buf);

                u_sfx[pos] = vlc_decode_u_suffix(vlc_stream, u_pfx[pos], vlc_buf);

                u_ext[pos] = vlc_decode_u_extension(vlc_stream, u_sfx[pos], vlc_buf);

                u[pos] = u_pfx[pos] + u_sfx[pos] + (u_ext[pos] << 2);
            }
            sp = sig_pat[J2K_Q1];

            gamma[J2K_Q1] = 1;

            if (sp == 0 || sp == 1 || sp == 2 || sp == 4 || sp == 8)
                gamma[J2K_Q1] = 0;

            sp = sig_pat[J2K_Q2];

            gamma[J2K_Q2] = 1;

            if (sp == 0 || sp == 1 || sp == 2 || sp == 4 || sp == 8)
                gamma[J2K_Q2] = 0;

            E_n[J2K_Q1] = E[4 * (q1 - quad_width) + 1];
            E_n[J2K_Q2] = E[4 * (q2 - quad_width) + 1];

            E_ne[J2K_Q1] = E[4 * (q1 - quad_width) + 3];
            E_ne[J2K_Q2] = E[4 * (q2 - quad_width) + 3];

            E_nw[J2K_Q1] = 0;
            E_nw[J2K_Q2] = 0;

            E_nf[J2K_Q1] = 0;
            E_nf[J2K_Q2] = 0;

            if (!is_divisible(q1, c))
                E_nw[J2K_Q1] = E[4 * (q1 - quad_width) - 1];
            if (!is_divisible(q2, c))
                E_nw[J2K_Q2] = E[4 * (q2 - quad_width) - 1];

            if (!is_divisible(q1 + 1, c))
                E_nf[J2K_Q1] = E[4 * (q1 - quad_width) + 5];
            if (!is_divisible(q2 + 1, c))
                E_nf[J2K_Q2] = E[4 * (q2 - quad_width) + 5];

            max_e[J2K_Q1] = FFMAX(E_nw[J2K_Q1], FFMAX3(E_n[J2K_Q1], E_ne[J2K_Q1], E_nf[J2K_Q1]));
            max_e[J2K_Q2] = FFMAX(E_nw[J2K_Q2], FFMAX3(E_n[J2K_Q2], E_ne[J2K_Q2], E_nf[J2K_Q2]));

            kappa[J2K_Q1] = FFMAX(1, gamma[J2K_Q1] * (max_e[J2K_Q1] - 1));
            kappa[J2K_Q2] = FFMAX(1, gamma[J2K_Q2] * (max_e[J2K_Q2] - 1));

            U[J2K_Q1] = kappa[J2K_Q1] + u[J2K_Q1];
            U[J2K_Q2] = kappa[J2K_Q2] + u[J2K_Q2];

            for (int i = 0; i < 4; i++) {
                m[J2K_Q1][i] = sigma_n[4 * q1 + i] * U[J2K_Q1] - ((emb_pat_k[J2K_Q1] >> i) & 1);
                m[J2K_Q2][i] = sigma_n[4 * q2 + i] * U[J2K_Q2] - ((emb_pat_k[J2K_Q2] >> i) & 1);
            }
            recover_mag_sgn(mag_sgn_stream, J2K_Q1, q1, m_n, known_1, emb_pat_1, v, m,
                            E, mu_n, Dcup, Pcup, pLSB);

            recover_mag_sgn(mag_sgn_stream, J2K_Q2, q2, m_n, known_1, emb_pat_1, v, m,
                            E, mu_n, Dcup, Pcup, pLSB);

            // move to the next quad pair
            q += 2;
        }
        if ((quad_width % 2) == 1) {
            q1 = q;
            // calculate context for current quad
            context1 = sigma_n[4 * (q1 - quad_width) + 1];         // n
            context1 += (sigma_n[4 * (q1 - quad_width) + 3] << 2); // ne

            if (!is_divisible(q1, c)) {
                context1 |= sigma_n[4 * (q1 - quad_width) - 1]; // nw
                context1 += (sigma_n[4 * q1 - 1] | sigma_n[4 * q1 - 2])
                    << 1; // (sw| w) << 1;
            }
            if (!is_divisible(q1 + 1, c))
                context1 |= sigma_n[4 * (q1 - quad_width) + 5] << 2;

            if ((ret = jpeg2000_decode_sig_emb(s, mel_state, mel_stream, vlc_stream,
                                               dec_CxtVLC_table1, Dcup, sig_pat, res_off,
                                               emb_pat_k, emb_pat_1, J2K_Q1, context1, Lcup,
                                               Pcup))
                < 0)
                goto free;

            for (int i = 0; i < 4; i++)
                sigma_n[4 * q1 + i] = (sig_pat[J2K_Q1] >> i) & 1;

            u[J2K_Q1] = 0;

            // Recover mag_sgn value
            if (res_off[J2K_Q1] == 1) {
                u_pfx[J2K_Q1] = vlc_decode_u_prefix(vlc_stream, vlc_buf);
                u_sfx[J2K_Q1] = vlc_decode_u_suffix(vlc_stream, u_pfx[J2K_Q1], vlc_buf);
                u_ext[J2K_Q1] = vlc_decode_u_extension(vlc_stream, u_sfx[J2K_Q1], vlc_buf);
                u[J2K_Q1] = u_pfx[J2K_Q1] + u_sfx[J2K_Q1] + (u_ext[J2K_Q1] << 2);
            }

            sp = sig_pat[J2K_Q1];

            gamma[J2K_Q1] = 1;

            if (sp == 0 || sp == 1 || sp == 2 || sp == 4 || sp == 8)
                gamma[J2K_Q1] = 0;

            E_n[J2K_Q1] = E[4 * (q1 - quad_width) + 1];

            E_ne[J2K_Q1] = E[4 * (q1 - quad_width) + 3];

            E_nw[J2K_Q1] = 0;

            E_nf[J2K_Q1] = 0;

            if (!is_divisible(q1, c))
                E_nw[J2K_Q1] = E[4 * (q1 - quad_width) - 1];

            if (!is_divisible(q1 + 1, c))
                E_nf[J2K_Q1] = E[4 * (q1 - quad_width) + 5];

            max_e[J2K_Q1] = FFMAX(E_nw[J2K_Q1], FFMAX3(E_n[J2K_Q1], E_ne[J2K_Q1], E_nf[J2K_Q1]));

            kappa[J2K_Q1] = FFMAX(1, gamma[J2K_Q1] * (max_e[J2K_Q1] - 1));

            U[J2K_Q1] = kappa[J2K_Q1] + u[J2K_Q1];

            for (int i = 0; i < 4; i++)
                m[J2K_Q1][i] = sigma_n[4 * q1 + i] * U[J2K_Q1] - ((emb_pat_k[J2K_Q1] >> i) & 1);

            recover_mag_sgn(mag_sgn_stream, J2K_Q1, q1, m_n, known_1, emb_pat_1, v, m,
                            E, mu_n, Dcup, Pcup, pLSB);
            // move to the next quad
            q += 1;
        }
    }
    // convert to raster-scan
    for (int y = 0; y < quad_height; y++) {
        for (int x = 0; x < quad_width; x++) {
            j1 = 2 * y;
            j2 = 2 * x;

            // set sample
            sample_buf[j2 + (j1 * width)] = (int32_t)*mu;
            jpeg2000_modify_state(j1, j2, width, *sigma, block_states);

            sigma += 1;
            mu += 1;

            if (y != quad_height - 1 || is_border_y == 0) {
                sample_buf[j2 + ((j1 + 1) * width)] = (int32_t)*mu;
                jpeg2000_modify_state(j1 + 1, j2, width, *sigma, block_states);
            }

            sigma += 1;
            mu += 1;

            if (x != quad_width - 1 || is_border_x == 0) {
                sample_buf[(j2 + 1) + (j1 * width)] = (int32_t)*mu;
                jpeg2000_modify_state(j1, j2 + 1, width, *sigma, block_states);
            }

            sigma += 1;
            mu += 1;

            if ((y != quad_height - 1 || is_border_y == 0) && (x != quad_width - 1 || is_border_x == 0)) {
                sample_buf[(j2 + 1) + (j1 + 1) * width] = (int32_t)*mu;
                jpeg2000_modify_state(j1 + 1, j2 + 1, width, *sigma, block_states);
            }
            sigma += 1;
            mu += 1;
        }
    }
    ret = 1;
free:
    av_freep(&sigma_n);
    av_freep(&E);
    av_freep(&mu_n);
    return ret;
}

static void jpeg2000_calc_mbr(uint8_t *mbr, const uint16_t i, const uint16_t j, const uint32_t mbr_info, uint8_t causal_cond, uint8_t *block_states, int width)
{

    int local_mbr = 0;
    local_mbr |= jpeg2000_get_state(i - 1, j - 1, width, HT_SHIFT_SIGMA, block_states);
    local_mbr |= jpeg2000_get_state(i - 1, j + 0, width, HT_SHIFT_SIGMA, block_states);
    local_mbr |= jpeg2000_get_state(i - 1, j + 1, width, HT_SHIFT_SIGMA, block_states);

    local_mbr |= jpeg2000_get_state(i + 0, j - 1, width, HT_SHIFT_SIGMA, block_states);
    local_mbr |= jpeg2000_get_state(i + 0, j + 1, width, HT_SHIFT_SIGMA, block_states);

    local_mbr |= jpeg2000_get_state(i + 1, j - 1, width, HT_SHIFT_SIGMA, block_states) * causal_cond;
    local_mbr |= jpeg2000_get_state(i + 1, j + 0, width, HT_SHIFT_SIGMA, block_states) * causal_cond;
    local_mbr |= jpeg2000_get_state(i + 1, j + 1, width, HT_SHIFT_SIGMA, block_states) * causal_cond;

    local_mbr |= jpeg2000_get_state(i - 1, j - 1, width, HT_SHIFT_REF, block_states) * jpeg2000_get_state(i - 1, j - 1, width, HT_SHIFT_SCAN, block_states);
    local_mbr |= jpeg2000_get_state(i - 1, j + 0, width, HT_SHIFT_REF, block_states) * jpeg2000_get_state(i - 1, j - 1, width, HT_SHIFT_SCAN, block_states);
    local_mbr |= jpeg2000_get_state(i - 1, j + 1, width, HT_SHIFT_REF, block_states) * jpeg2000_get_state(i - 1, j + 1, width, HT_SHIFT_SCAN, block_states);

    local_mbr |= jpeg2000_get_state(i + 0, j - 1, width, HT_SHIFT_REF, block_states) * jpeg2000_get_state(i + 0, j - 1, width, HT_SHIFT_SCAN, block_states);
    local_mbr |= jpeg2000_get_state(i + 0, j + 1, width, HT_SHIFT_REF, block_states) * jpeg2000_get_state(i + 0, j + 1, width, HT_SHIFT_SCAN, block_states);

    local_mbr |= jpeg2000_get_state(i + 1, j - 1, width, HT_SHIFT_REF, block_states) * jpeg2000_get_state(i + 1, j - 1, width, HT_SHIFT_SCAN, block_states) * causal_cond;
    local_mbr |= jpeg2000_get_state(i + 1, j + 0, width, HT_SHIFT_REF, block_states) * jpeg2000_get_state(i + 1, j + 0, width, HT_SHIFT_SCAN, block_states) * causal_cond;
    local_mbr |= jpeg2000_get_state(i + 1, j + 1, width, HT_SHIFT_REF, block_states) * jpeg2000_get_state(i + 1, j + 1, width, HT_SHIFT_SCAN, block_states) * causal_cond;

    *mbr |= local_mbr;
}
static void jpeg2000_process_stripes_block(StateVars *sig_prop, int i_s, int j_s, int width, int height, int stride, int pLSB, int32_t *sample_buf, uint8_t *block_states, uint8_t *magref_segment, uint32_t magref_length)
{
    int32_t *sp;
    uint8_t causal_cond = 0;
    uint8_t bit;
    uint8_t mbr;
    uint32_t mbr_info;

    for (int j = j_s; j < j_s + width; j++) {
        mbr_info = 0;
        for (int i = i_s; i < i_s + height; i++) {
            sp = &sample_buf[j + (i * (stride - 2))];
            mbr = 0;
            causal_cond = i != (i_s + height - 1);
            if (jpeg2000_get_state(i, j, stride - 2, HT_SHIFT_SIGMA, block_states) == 0)
                jpeg2000_calc_mbr(&mbr, i, j, mbr_info & 0x1EF, causal_cond, block_states, stride - 2);
            mbr_info >>= 3;
            if (mbr != 0) {
                jpeg2000_modify_state(i, j, stride - 2, 1 << HT_SHIFT_REF_IND, block_states);
                bit = jpeg2000_import_bit(sig_prop, magref_segment, magref_length);
                jpeg2000_modify_state(i, j, stride - 2, 1 << HT_SHIFT_REF, block_states);
                *sp |= bit << pLSB;
            }
            jpeg2000_modify_state(i, j, stride - 2, 1 << HT_SHIFT_SCAN, block_states);
        }
    }
    for (int j = 0; j < j_s + width; j++) {
        for (int i = 0; i < i_s + height; i++) {
            sp = &sample_buf[j + (i * (stride - 2))];
            if ((*sp & (1 << pLSB)) != 0) {
                *sp = (*sp & 0x7FFFFFFF) | (jpeg2000_import_bit(sig_prop, magref_segment, magref_length) << 31);
            }
        }
    }
}

static void jpeg2000_decode_sigprop(Jpeg2000Cblk *cblk, uint16_t width, uint16_t height, uint8_t *magref_segment, uint32_t magref_length, uint8_t pLSB, int32_t *sample_buf, uint8_t *block_states)
{
    // Described in clause 7.4
    // procedure: decodeSigPropMag

    StateVars sp_dec;
    const uint16_t num_v_stripe = height / 4;
    const uint16_t num_h_stripe = width / 4;
    int b_width = 4;
    int b_height = 4;
    int stride = width + 2;
    int last_width;
    uint16_t i = 0, j = 0;

    jpeg2000_init_zero(&sp_dec);

    for (int n1 = 0; n1 < num_v_stripe; n1++) {
        j = 0;
        for (int n2 = 0; n2 < num_h_stripe; n2++) {
            jpeg2000_process_stripes_block(&sp_dec, i, j, b_width, b_height, stride, pLSB, sample_buf, block_states, magref_segment, magref_length);
            j += 4;
        }
        last_width = width % 4;
        if (last_width)
            jpeg2000_process_stripes_block(&sp_dec, i, j, last_width, b_height, stride, pLSB, sample_buf, block_states, magref_segment, magref_length);
        i += 4;
    }
    // decode remaining height stripes
    b_height = height % 4;
    j = 0;
    for (int n2 = 0; n2 < num_h_stripe; n2++) {
        jpeg2000_process_stripes_block(&sp_dec, i, j, b_width, b_height, stride, pLSB, sample_buf, block_states, magref_segment, magref_length);
        j += 4;
    }
    last_width = width % 4;
    if (last_width)
        jpeg2000_process_stripes_block(&sp_dec, i, j, last_width, b_height, stride, pLSB, sample_buf, block_states, magref_segment, magref_length);
}

static int jpeg2000_decode_magref(Jpeg2000Cblk *cblk, uint16_t width, uint16_t block_height, uint8_t *magref_segment, uint32_t magref_length, uint8_t pLSB, int32_t *sample_buf, uint8_t *block_states)
{
    // Described in clause 7.5
    // procedure: decodeSigPropMag

    StateVars mag_ref = {0};
    const uint16_t num_v_stripe = block_height / 4;
    uint16_t height = 4;
    uint16_t i_start = 0;
    int32_t *sp;

    jpeg2000_init_mag_ref(&mag_ref, magref_length);

    for (int n1 = 0; n1 < num_v_stripe; n1++) {
        for (int j = 0; j < width; j++) {
            for (int i = i_start; i < i_start + height; i++) {
                // we move column wise, going from one quad to another
                // see figure 7.
                sp = &sample_buf[j + i * width];
                if (jpeg2000_get_state(i, j, width, HT_SHIFT_SIGMA, block_states) != 0) {
                    jpeg2000_modify_state(i, j, width, 1 << HT_SHIFT_REF_IND, block_states);
                    *sp |= jpeg2000_import_magref_bit(&mag_ref, magref_segment, magref_length) << pLSB;
                }
            }
        }
        i_start += 4;
    }
    height = block_height % 4;
    for (int j = 0; j < width; j++) {
        for (int i = i_start; i < i_start + height; i++) {
            sp = &sample_buf[j + i * width];
            if (jpeg2000_get_state(i, j, width, HT_SHIFT_SIGMA, block_states) != 0) {
                jpeg2000_modify_state(i, j, width, 1 << HT_SHIFT_REF_IND, block_states);
                *sp |= jpeg2000_import_magref_bit(&mag_ref, magref_segment, magref_length) << pLSB;
            }
        }
    }
    return 1;
}

int decode_htj2k(Jpeg2000DecoderContext *s, Jpeg2000CodingStyle *codsty, Jpeg2000T1Context *t1, Jpeg2000Cblk *cblk, int width, int height, int magp, uint8_t roi_shift)
{
    uint8_t p0 = 0; // Number of placeholder passes.
    uint32_t Lcup;  // Length of HT cleanup segment.
    uint32_t Lref;  // Length of Refinement segment.
    uint32_t Scup;  // HT cleanup segment suffix length.
    uint32_t Pcup;  // HT cleanup segment prefix length.

    uint8_t S_blk; // Number of skipped magnitude bitplanes;
    uint8_t pLSB;

    uint8_t *Dcup; // Byte of an HT cleanup segment.
    uint8_t *Dref; // Byte of an HT refinement segment.

    int z_blk; // Number of ht coding pass

    uint8_t empty_passes;

    StateVars mag_sgn;  // Magnitude and Sign
    StateVars mel;      // Adaptive run-length coding
    StateVars vlc;      // Variable Length coding
    StateVars sig_prop; // Significance propagation

    MelDecoderState mel_state;

    int ret;

    // Temporary buffers
    int32_t *sample_buf;
    uint8_t *block_states;

    // Post-processing
    int32_t n, val, sign;

    int32_t M_b = magp;
    av_assert0(width <= 1024U && height <= 1024U);
    av_assert0(width * height <= 4096);
    av_assert0(width * height > 0);

    memset(t1->data, 0, t1->stride * height * sizeof(*t1->data));
    memset(t1->flags, 0, t1->stride * (height + 2) * sizeof(*t1->flags));

    if (cblk->npasses == 0)
        return 0;

    if (cblk->npasses > 3)
        // Currently use this as a dummy but should be fixed soon
        p0 = 0;
    else if (cblk->length == 0)
        p0 = 1;

    empty_passes = p0 * 3;
    z_blk = cblk->npasses - empty_passes;

    if (z_blk <= 0)
        // no passes within this set, continue
        return 0;

    Lcup = cblk->pass_lengths[0];
    Lref = cblk->pass_lengths[1];

    if (Lcup < 2) {
        av_log(s->avctx, AV_LOG_ERROR,
               "Cleanup pass length must be at least 2 bytes in length\n");
        return AVERROR_INVALIDDATA;
    }
    Dcup = cblk->data;
    // Dref comes after the refinement segment.
    Dref = cblk->data + Lcup;
    S_blk = p0 + cblk->zbp;

    pLSB = 30 - S_blk;

    Scup = (Dcup[Lcup - 1] << 4) + (Dcup[Lcup - 2] & 0x0F);

    if (Scup < 2 || Scup > Lcup || Scup > 4079) {
        av_log(s->avctx, AV_LOG_ERROR, "Cleanup pass suffix length is invalid %d\n",
               Scup);
        ret = AVERROR_INVALIDDATA;
        goto free;
    }
    Pcup = Lcup - Scup;

    // modDcup shall be done before the creation of vlc instance.
    Dcup[Lcup - 1] = 0xFF;
    Dcup[Lcup - 2] |= 0x0F;
    // Magnitude and refinement
    jpeg2000_init_zero(&mag_sgn);
    jpeg2000_bitbuf_refill_bytewise(&mag_sgn, Dcup, Pcup);
    // Significance propagation
    jpeg2000_init_zero(&sig_prop);
    // Adaptive run length
    jpeg2000_init_mel(&mel, Pcup);
    // Variable Length coding.
    jpeg2000_init_vlc(&vlc, Lcup, Pcup, Dcup);

    jpeg2000_init_mel_decoder(&mel_state);

    sample_buf = av_calloc(width * height, sizeof(int32_t));
    block_states = av_calloc((width + 4) * (height + 4), sizeof(uint8_t));

    if (!sample_buf || !block_states) {
        ret = AVERROR(ENOMEM);
        goto free;
    }
    if ((ret = jpeg2000_decode_ht_cleanup(s, cblk, t1, &mel_state, &mel, &vlc, &mag_sgn, Dcup, Lcup, Pcup, pLSB, width, height, sample_buf, block_states)) < 0)
        goto free;

    if (cblk->npasses > 1)
        jpeg2000_decode_sigprop(cblk, width, height, Dref, Lref, pLSB - 1, sample_buf, block_states);

    if (cblk->npasses > 2)
        if ((ret = jpeg2000_decode_magref(cblk, width, height, Dref, Lref, pLSB - 1, sample_buf, block_states)) < 0)
            goto free;

    pLSB = 31 - M_b;
    // Reconstruct the values.
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            n = x + (y * t1->stride);
            val = sample_buf[x + (y * width)];
            sign = val & 0x80000000;
            val &= 0x7fffffff;
            // convert sign-magnitude to twos complement form
            if (sign)
                val = -val;
            val >>= (pLSB - 1);
            t1->data[n] = val;
        }
    }
free:
    av_freep(&sample_buf);
    av_freep(&block_states);
    return ret;
}
