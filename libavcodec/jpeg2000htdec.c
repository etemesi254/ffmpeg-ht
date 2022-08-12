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
#include "libavutil/attributes.h"
#include "libavutil/avassert.h"
#include "libavutil/common.h"
#include "libavutil/log.h"
#include "libavutil/mem.h"
#include <stdint.h>
#include <stdlib.h>

#include "bytestream.h"

#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#define MAX(a, b) (((a) > (b)) ? (a) : (b))

#define J2K_Q1 0

#define J2K_Q2 1

#define HT_SHIFT_SIGMA 0
#define HT_SHIFT_SCAN 4
#define HT_SHIFT_REF 3
#define HT_SHIFT_PI 2
#define HT_SHIFT_P 3

/**
 * @brief Table 2 in clause 7.3.3
 * */
const static uint8_t MEL_E[13] = {0, 0, 0, 1, 1, 1, 2, 2, 2, 3, 3, 4, 5};

static av_always_inline uint32_t has_zero(uint32_t dword)
{
    // Borrowed from the famous stanford bithacks page
    // see https://graphics.stanford.edu/~seander/bithacks.html#ZeroInWord
    return ~((((dword & 0x7F7F7F7F) + 0x7F7F7F7F) | dword) | 0x7F7F7F7F);
}
static av_always_inline uint32_t has_byte(uint32_t dword, uint8_t byte)
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

static int jpeg2000_bitbuf_refill_backwards(StateVars *buffer,
                                            const uint8_t *array)
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
        tmp |= (uint64_t) * (array + position + 1);
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
    av_assert0(buf->bits_left >= nbits);
    buf->bit_buf >>= nbits;
    buf->bits_left -= nbits;
}

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
};

static av_always_inline uint64_t jpeg2000_bitbuf_get_bits_lsb_forward(
    StateVars *bit_stream, uint8_t nbits, const uint8_t *buf, uint32_t length)
{

    uint64_t bits;
    uint64_t mask = (1ull << nbits) - 1;
    if (bit_stream->bits_left <= nbits)
        // TODO: (cae) this may fail I  guess if there are no more bits,add a check
        // for it.
        jpeg2000_bitbuf_refill_bytewise(bit_stream, buf, length);

    bits = bit_stream->bit_buf & mask;
    jpeg2000_bitbuf_drop_bits_lsb(bit_stream, nbits);
    return bits;
};

static av_always_inline uint64_t
jpeg2000_bitbuf_peek_bits_lsb(StateVars *stream, uint8_t nbits)
{
    uint64_t mask = (1ull << nbits) - 1;

    return stream->bit_buf & mask;
}

/* VLC decoding utilities */

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

static av_always_inline uint8_t
vlc_decode_u_prefix(StateVars *vlc_stream, const uint8_t *refill_array)
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

    return 5;
}

static av_always_inline uint8_t vlc_decode_u_suffix(
    StateVars *vlc_stream, uint8_t suffix, const uint8_t *refill_array)
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

static av_always_inline uint8_t vlc_decode_u_extension(
    StateVars *vlc_stream, uint8_t suffix, const uint8_t *refill_array)
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

static av_always_inline void
recover_mag_sgn(StateVars *mag_sgn, uint8_t pos, uint16_t q, int32_t m_n[2], int32_t known_1[2], uint8_t emb_pat_1[2], int32_t v[2][4], int32_t m[2][4], uint8_t *E, uint32_t *mu_n, const uint8_t *Dcup, uint32_t Pcup, uint32_t pLSB)
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
    // TODO (cae): Figure out how to use the other refill method here.
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
            mel_state->k = MIN(12, mel_state->k + 1);
        } else {
            mel_state->run = 0;
            while (eval > 0) {
                bit = jpeg2000_import_bit(mel_stream, Dcup, Lcup);
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

static av_always_inline uint32_t get_rem(uint32_t num, uint32_t divisor, uint32_t recp_freq, uint32_t recp_shift)
{

    uint32_t quotient = (((uint64_t)num * recp_freq) >> 32) >> recp_shift;

    uint32_t rem = (num - (quotient * divisor));

    return rem;
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

    uint32_t shift;
    uint32_t recp_freq;
    uint32_t recp_shift;

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

        //  if res_off = [0,0]
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

                u[J2K_Q1] = 2 + u_pfx[J2K_Q1] + u_sfx[J2K_Q1] + (u_ext[J2K_Q1] << 2);
                u[J2K_Q2] = 2 + u_pfx[J2K_Q2] + u_sfx[J2K_Q2] + (u_ext[J2K_Q2] << 2);

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

                    u[J2K_Q2] = u_pfx[J2K_Q2] + u_sfx[J2K_Q2] + (u_ext[J2K_Q2] << 2);
                }
                u[J2K_Q1] = u_pfx[J2K_Q1] + u_sfx[J2K_Q1] + (u_ext[J2K_Q1] << 2);
            }

        } else if (res_off[J2K_Q1] == 1 || res_off[J2K_Q2] == 1) {
            uint8_t pos;

            if (res_off[J2K_Q1] == 1)
                pos = 0;
            else
                pos = 1;

            u_pfx[pos] = vlc_decode_u_prefix(vlc_stream, vlc_buf);

            u_sfx[pos] = vlc_decode_u_suffix(vlc_stream, u_pfx[pos], vlc_buf);

            u_ext[pos] = vlc_decode_u_extension(vlc_stream, u_sfx[pos], vlc_buf);

            u[pos] = u_pfx[pos] + u_sfx[pos] + (u_ext[pos] << 2);
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
            u[J2K_Q1] = u_pfx[J2K_Q1] + u_sfx[J2K_Q1] + (u_ext[J2K_Q1] << 2);
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
     * As an optimization, we can save some cycles in the inner loop if
     * we replace modulo operations with multiplications by integer reciprocals.
     *
     * The technique is from the paper "Integer division using Reciprocals" by
     * Robert Alverson.
     *
     * The expensive division is done outside of the loop and the inner loop
     * implements modulus operation with multiplications and shifts.
     *
     * TODO:(cae) Daniel Lemire posted a faster way to calculate remainder, see
     * https://lemire.me/blog/2019/02/08/faster-remainders-when-the-divisor-is-a-constant-beating-compilers-and-libdivide/
     *
     * Consider it when optimizing this.
     */
    shift = 31 - ff_clz(quad_width);

    recp_freq = (uint32_t)(((1ull << (shift + 31)) + quad_width - 1) / quad_width);

    recp_shift = shift - 1;

    for (int row = 1; row < quad_height; row++) {
        while ((q - (row * quad_width)) < quad_width - 1 && q < (quad_height * quad_width)) {
            q1 = q;
            q2 = q + 1;
            context1 = sigma_n[4 * (q1 - quad_width) + 1];
            context1 += sigma_n[4 * (q1 - quad_width) + 3] << 2; // ne

            if (get_rem(q1, quad_width, recp_freq, recp_shift)) {
                context1 |= sigma_n[4 * (q1 - quad_width) - 1];               // nw
                context1 += (sigma_n[4 * q1 - 1] | sigma_n[4 * q1 - 2]) << 1; // sw| q
            }
            if (get_rem(q1 + 1, quad_width, recp_freq, recp_shift))
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

            if (get_rem(q2, quad_width, recp_freq, recp_shift)) {
                context2 |= sigma_n[4 * (q2 - quad_width) - 1];
                context2 += (sigma_n[4 * q2 - 1] | sigma_n[4 * q2 - 2]) << 1;
            }
            if (get_rem(q2 + 1, quad_width, recp_freq, recp_shift))
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
                uint8_t pos;

                if (res_off[J2K_Q1] == 1)
                    pos = 0;
                else
                    pos = 1;

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

            if (get_rem(q1, quad_width, recp_freq, recp_shift))
                E_nw[J2K_Q1] = E[4 * (q1 - quad_width) - 1];
            if (get_rem(q2, quad_width, recp_freq, recp_shift))
                E_nw[J2K_Q2] = E[4 * (q2 - quad_width) - 1];

            if (get_rem(q1 + 1, quad_width, recp_freq, recp_shift))
                E_nf[J2K_Q1] = E[4 * (q1 - quad_width) + 5];
            if (get_rem(q2 + 1, quad_width, recp_freq, recp_shift))
                E_nf[J2K_Q2] = E[4 * (q2 - quad_width) + 5];

            max_e[J2K_Q1] = MAX(E_nw[J2K_Q1], MAX(E_n[J2K_Q1], MAX(E_ne[J2K_Q1], E_nf[J2K_Q1])));
            max_e[J2K_Q2] = MAX(E_nw[J2K_Q2], MAX(E_n[J2K_Q2], MAX(E_ne[J2K_Q2], E_nf[J2K_Q2])));

            kappa[J2K_Q1] = MAX(1, gamma[J2K_Q1] * (max_e[J2K_Q1] - 1));
            kappa[J2K_Q2] = MAX(1, gamma[J2K_Q2] * (max_e[J2K_Q2] - 1));

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

            if (get_rem(q1, quad_width, recp_freq, recp_shift)) {
                context1 |= sigma_n[4 * (q1 - quad_width) - 1]; // nw
                context1 += (sigma_n[4 * q1 - 1] | sigma_n[4 * q1 - 2])
                    << 1; // (sw| w) << 1;
            }
            if (get_rem(q1 + 1, quad_width, recp_freq, recp_shift))
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

            if (get_rem(q1, quad_width, recp_freq, recp_shift))
                E_nw[J2K_Q1] = E[4 * (q1 - quad_width) - 1];

            if (get_rem(q1 + 1, quad_width, recp_freq, recp_shift))
                E_nf[J2K_Q1] = E[4 * (q1 - quad_width) + 5];

            max_e[J2K_Q1] = MAX(E_nw[J2K_Q1], MAX(E_n[J2K_Q1], MAX(E_ne[J2K_Q1], E_nf[J2K_Q1])));

            kappa[J2K_Q1] = MAX(1, gamma[J2K_Q1] * (max_e[J2K_Q1] - 1));

            U[J2K_Q1] = kappa[J2K_Q1] + u[J2K_Q1];

            for (int i = 0; i < 4; i++)
                m[J2K_Q1][i] = sigma_n[4 * q1 + i] * U[J2K_Q1] - ((emb_pat_k[J2K_Q1] >> i) & 1);

            recover_mag_sgn(mag_sgn_stream, J2K_Q1, q1, m_n, known_1, emb_pat_1, v, m,
                            E, mu_n, Dcup, Pcup, pLSB);
            // move to the next quad
            q++;
        }
        printf("%d\n", row);
    }
    // convert to raster-scan
    for (int y = 0; y < quad_height; y++) {
        for (int x = 0; x < quad_width; x++) {
            j1 = 2 * y;
            j2 = 2 * x;

            // set sample
            sample_buf[j2 + (j1 * width)] = *mu;
            // modify state
            block_states[(j1 + 1) * (width + 2) + (j2 + 1)] |= *sigma;

            sigma += 1;
            mu += 1;

            if (y != quad_height - 1 || is_border_y == 0) {
                sample_buf[j2 + ((j1 + 1) * width)] = *mu;
                block_states[(j1 + 2) * (width + 2) + (j2 + 1)] |= *sigma;
            }

            sigma += 1;
            mu += 1;

            if (x != quad_width - 1 || is_border_x == 0) {
                sample_buf[(j2 + 1) + (j1 * width)] = *mu;
                block_states[(j1 + 1) * (width + 2) + (j2 + 2)] |= *sigma;
            }

            sigma += 1;
            mu += 1;

            if ((y != quad_height - 1 || is_border_y == 0) && (x != quad_width - 1 || is_border_x == 0)) {
                sample_buf[(j2 + 1) + (j1 + 1) * width] = *mu;
                block_states[(j1 + 2) * (width + 2) + (j2 + 2)] |= *sigma;
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
static av_always_inline int jpeg2000_get_state(int x1, int x2, int width, int shift_by, uint8_t *block_states)
{
    return (block_states[(x1 + 1) * (width + 2) + (x2 + 1)] >> shift_by) & 1;
}
static void jpeg2000_calc_mbr(uint8_t *mbr, const uint16_t i, const uint16_t j, const uint32_t mbr_info, uint8_t causal_cond, uint8_t *block_states, int width)
{

    int local_mbr = 0;
    local_mbr |= jpeg2000_get_state(i - 1, j - 1, width, HT_SHIFT_SIGMA, block_states);
    local_mbr |= jpeg2000_get_state(i - 1, j, width, HT_SHIFT_SIGMA, block_states);
    local_mbr |= jpeg2000_get_state(i - 1, j + 1, width, HT_SHIFT_SIGMA, block_states);

    local_mbr |= jpeg2000_get_state(i, j - 1, width, HT_SHIFT_SIGMA, block_states);
    local_mbr |= jpeg2000_get_state(i, j + 1, width, HT_SHIFT_SIGMA, block_states);

    local_mbr |= jpeg2000_get_state(i + 1, j - 1, width, HT_SHIFT_SIGMA, block_states) * causal_cond;
    local_mbr |= jpeg2000_get_state(i + 1, j, width, HT_SHIFT_SIGMA, block_states) * causal_cond;
    local_mbr |= jpeg2000_get_state(i + 1, j + 1, width, HT_SHIFT_SIGMA, block_states) * causal_cond;

    local_mbr |= jpeg2000_get_state(i - 1, j - 1, width, HT_SHIFT_REF, block_states) * jpeg2000_get_state(i - 1, j - 1, width, HT_SHIFT_SCAN, block_states);
    local_mbr |= jpeg2000_get_state(i - 1, j, width, HT_SHIFT_REF, block_states) * jpeg2000_get_state(i, j - 1, width, HT_SHIFT_SCAN, block_states);
    local_mbr |= jpeg2000_get_state(i - 1, j + 1, width, HT_SHIFT_REF, block_states) * jpeg2000_get_state(i - 1, j + 1, width, HT_SHIFT_SCAN, block_states);

    local_mbr |= jpeg2000_get_state(i, j - 1, width, HT_SHIFT_REF, block_states) * jpeg2000_get_state(i, j - 1, width, HT_SHIFT_SCAN, block_states);
    local_mbr |= jpeg2000_get_state(i - 1, j + 1, width, HT_SHIFT_REF, block_states) * jpeg2000_get_state(i, j + 1, width, HT_SHIFT_SCAN, block_states);

    local_mbr |= jpeg2000_get_state(i + 1, j - 1, width, HT_SHIFT_REF, block_states) * jpeg2000_get_state(i - 1, j - 1, width, HT_SHIFT_SCAN, block_states) * causal_cond;
    local_mbr |= jpeg2000_get_state(i + 1, j, width, HT_SHIFT_REF, block_states) * jpeg2000_get_state(i, j - 1, width, HT_SHIFT_SCAN, block_states) * causal_cond;
    local_mbr |= jpeg2000_get_state(i + 1, j + 1, width, HT_SHIFT_REF, block_states) * jpeg2000_get_state(i - 1, j + 1, width, HT_SHIFT_SCAN, block_states) * causal_cond;

    *mbr |= local_mbr;
}
static int jpeg2000_process_stripes_block(StateVars *sig_prop, int i_s, int j_s, int width, int height, int stride, int pLSB, int32_t *sample_buf, uint8_t *block_states, uint8_t *magref_segment, uint32_t magref_length)
{
    int32_t *sp;
    uint8_t causal_cond = 0;
    uint8_t bit;
    uint8_t mbr;
    uint32_t mbr_info;

    for (int j = j_s; j < j_s + width; j++) {
        mbr_info = 0;
        for (int i = i_s; i < i_s + height; i++) {
            sp = &sample_buf[j + (i * width)];
            mbr = 0;
            causal_cond = i != (i_s + height - 1);
            if (block_states[(i + 1) * (width + 2) + (j + 1)] == 0)
                jpeg2000_calc_mbr(&mbr, i, j, mbr_info & 0x1EF, causal_cond, block_states, width);
            mbr_info >>= 3;
            if (mbr != 0) {
                block_states[(i + 1) * (width + 2) + j + 1] |= 1 << HT_SHIFT_PI;
                bit = jpeg2000_import_bit(sig_prop, magref_segment, magref_length);
                block_states[(i + 1) * (width + 2) + j + 1] |= bit << HT_SHIFT_REF;
                *sp |= bit << pLSB;
            }
            block_states[(i + 1) * (width + 2) + j + 1] |= 1 << HT_SHIFT_SCAN;
        }
    }
    for (int j = 0; j < j_s + width; j++) {
        for (int i = 0; i < i_s + height; i++) {
            sp = &sample_buf[j + i * width];
            if ((*sp & (1 << pLSB)) != 0) {
                *sp = (*sp & 0x7FFFFFFF) | (jpeg2000_import_bit(sig_prop, magref_segment, magref_length) << 31);
            }
        }
    }
    return 0;
}

static int jpeg2000_decode_sigprop(Jpeg2000Cblk *cblk, uint16_t width, uint16_t height, uint8_t *magref_segment, uint32_t magref_length, uint8_t pLSB, int32_t *sample_buf, uint8_t *block_states)
{
    StateVars sp_dec;
    const uint16_t num_v_stripe = height / 4;
    const uint16_t num_h_stripe = width / 4;
    int last_width;
    int ret;
    uint16_t i = 0, j = 0;

    jpeg2000_init_zero(&sp_dec);

    for (int n1 = 0; n1 < num_v_stripe; n1++) {
        j = 0;
        for (int n2 = 0; n2 < num_h_stripe; n2++) {
            if ((ret = jpeg2000_process_stripes_block(&sp_dec, i, j, width, height, width + 2, pLSB, sample_buf, block_states, magref_segment, magref_length)) < 0)
                return ret;
            j += 4;
        }
        last_width = width % 4;
        if (last_width) {
            if ((ret = jpeg2000_process_stripes_block(&sp_dec, i, j, width, height, width + 2, pLSB, sample_buf, block_states, magref_segment, magref_length)) < 0)
                return ret;
        }
        i += 4;
    }
    return 0;
}
int decode_htj2k(Jpeg2000DecoderContext *s, Jpeg2000CodingStyle *codsty, Jpeg2000T1Context *t1, Jpeg2000Cblk *cblk, int width, int height, int bandpos, uint8_t roi_shift)
{
    uint8_t p0 = 0;    // Number of placeholder passes.
    uint32_t Lcup;     // Length of HT cleanup segment.
    uint32_t Lref = 0; // Length of Refinement segment.
    uint32_t Scup;     // HT cleanup segment suffix length.
    uint32_t Pcup;     // HT cleanup segment prefix length.

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
    StateVars mag_ref;  // Magnitude and refinement.

    MelDecoderState mel_state;

    int ret;

    // Temporary buffers
    int32_t *sample_buf;
    uint8_t *block_states;

    // Post-processing
    int n, val, sign, r_val, N_b, offset;

    // TODO: Stop assuming
    int32_t M_b = 8;

    av_assert0(width <= 1024U && height <= 1024U);
    av_assert0(width * height <= 4096);
    memset(t1->data, 0, t1->stride * height * sizeof(*t1->data));

    memset(t1->flags, 0, t1->stride * (height + 2) * sizeof(*t1->flags));

    if (cblk->npasses == 0)
        return 0;

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

    Lcup = cblk->pass_lengths[0];
    if (Lcup < 2) {
        av_log(s->avctx, AV_LOG_ERROR,
               "Cleanup pass length must be at least 2 bytes in length\n");
        return AVERROR_INVALIDDATA;
    }
    Dcup = cblk->data;
    // Dref comes after the refinement segment.
    Dref = cblk->data + Lcup;
    S_blk = p0 + cblk->zbp;

    sample_buf = av_calloc(width * height, sizeof(int32_t));
    block_states = av_calloc((width + 2) * (height + 2), sizeof(uint8_t));

    if (!sample_buf || !block_states)
        return AVERROR(ENOMEM);

    pLSB = 30 - S_blk;

    Scup = (Dcup[Lcup - 1] << 4) + (Dcup[Lcup - 2] & 0x0F);

    if (Scup < 2 || Scup > Lcup || Scup > 4079) {
        av_log(s->avctx, AV_LOG_ERROR, "Cleanup pass suffix length is invalid %d\n",
               Scup);
        ret = AVERROR_INVALIDDATA;
        goto free;
    }
    Pcup = Lcup - Scup;

    // modDcup (shall be done before the creation of state_VLC instance)
    Dcup[Lcup - 1] = 0xFF;
    Dcup[Lcup - 2] |= 0x0F;
    jpeg2000_init_zero(&mag_sgn);
    jpeg2000_bitbuf_refill_bytewise(&mag_sgn, Dcup, Pcup);

    jpeg2000_init_zero(&sig_prop);

    jpeg2000_init_mel(&mel, Pcup);

    jpeg2000_init_vlc(&vlc, Lcup, Pcup, Dcup);
    jpeg2000_bitbuf_refill_backwards(&vlc, Dcup + Pcup);
    jpeg2000_bitbuf_drop_bits_lsb(&vlc, 4);

    jpeg2000_init_mag_ref(&mag_ref, Lref);

    jpeg2000_init_mel_decoder(&mel_state);

    if ((ret = jpeg2000_decode_ht_cleanup(s, cblk, t1, &mel_state, &mel, &vlc, &mag_sgn, Dcup, Lcup, Pcup, pLSB, width, height, sample_buf, block_states)) < 0)
        goto free;
    if (cblk->npasses > 1) {
        if ((ret = jpeg2000_decode_sigprop(cblk, width, height, Dref, Lref, pLSB + 1, sample_buf, block_states)) < 0)
            goto free;
    }

    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            n = x + (y * t1->stride);
            val = sample_buf[x + (y * width)];
            sign = val & 0x80000000;
            val &= 0x7fffffff;
            N_b = M_b;
            offset = M_b > N_b ? M_b - N_b : 0;
            r_val = 1 << (pLSB - 1 + offset);
            if (val != 0 && N_b < M_b)
                val |= r_val;
            // convert sign-magnitude to twos complement form
            if (sign)
                val = -val;
            t1->data[n] = val >> (pLSB - 1);
        }
    }

    av_freep(&sample_buf);
    av_freep(&block_states);
    return ret;
free:
    av_freep(&sample_buf);
    av_freep(&block_states);
    return ret;
}
