// Self-contained stand-in for <openssl/md5.h> (rebuilt repo, round 71).
//
// `System.cc` uses the classic OpenSSL MD5 API -- `MD5_CTX`, `MD5_Init`, `MD5_Update`, `MD5_Final`,
// `MD5_DIGEST_LENGTH` -- to checksum the vocabulary file, and upstream links `-lcrypto` for it.  Linking the
// conda libcrypto dragged a mismatched runtime into route_e.exe (the loader refused the process), and this
// checksum is not worth that: the digest only has to be *correct* and stable, so RFC 1321 is implemented
// here.  Same approach as the earlier workspace, which had no OpenSSL at all.
#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>

#define MD5_DIGEST_LENGTH 16
#define MD5_LONG uint32_t
#define MD5_CBLOCK 64

typedef struct MD5state_st {
  MD5_LONG A, B, C, D;
  MD5_LONG Nl, Nh;
  MD5_LONG data[MD5_CBLOCK / 4];
  unsigned int num;
} MD5_CTX;

namespace jo_md5 {

inline MD5_LONG rotl(MD5_LONG value, int shift) {
  return (value << shift) | (value >> (32 - shift));
}

inline void transform(MD5_LONG state[4], const unsigned char block[MD5_CBLOCK]) {
  static const MD5_LONG K[64] = {
      0xd76aa478, 0xe8c7b756, 0x242070db, 0xc1bdceee, 0xf57c0faf, 0x4787c62a, 0xa8304613, 0xfd469501,
      0x698098d8, 0x8b44f7af, 0xffff5bb1, 0x895cd7be, 0x6b901122, 0xfd987193, 0xa679438e, 0x49b40821,
      0xf61e2562, 0xc040b340, 0x265e5a51, 0xe9b6c7aa, 0xd62f105d, 0x02441453, 0xd8a1e681, 0xe7d3fbc8,
      0x21e1cde6, 0xc33707d6, 0xf4d50d87, 0x455a14ed, 0xa9e3e905, 0xfcefa3f8, 0x676f02d9, 0x8d2a4c8a,
      0xfffa3942, 0x8771f681, 0x6d9d6122, 0xfde5380c, 0xa4beea44, 0x4bdecfa9, 0xf6bb4b60, 0xbebfbc70,
      0x289b7ec6, 0xeaa127fa, 0xd4ef3085, 0x04881d05, 0xd9d4d039, 0xe6db99e5, 0x1fa27cf8, 0xc4ac5665,
      0xf4292244, 0x432aff97, 0xab9423a7, 0xfc93a039, 0x655b59c3, 0x8f0ccc92, 0xffeff47d, 0x85845dd1,
      0x6fa87e4f, 0xfe2ce6e0, 0xa3014314, 0x4e0811a1, 0xf7537e82, 0xbd3af235, 0x2ad7d2bb, 0xeb86d391};
  static const int S[64] = {7, 12, 17, 22, 7, 12, 17, 22, 7, 12, 17, 22, 7, 12, 17, 22,
                            5, 9,  14, 20, 5, 9,  14, 20, 5, 9,  14, 20, 5, 9,  14, 20,
                            4, 11, 16, 23, 4, 11, 16, 23, 4, 11, 16, 23, 4, 11, 16, 23,
                            6, 10, 15, 21, 6, 10, 15, 21, 6, 10, 15, 21, 6, 10, 15, 21};
  MD5_LONG M[16];
  for (int i = 0; i < 16; ++i) {
    M[i] = static_cast<MD5_LONG>(block[i * 4]) | (static_cast<MD5_LONG>(block[i * 4 + 1]) << 8) |
           (static_cast<MD5_LONG>(block[i * 4 + 2]) << 16) | (static_cast<MD5_LONG>(block[i * 4 + 3]) << 24);
  }
  MD5_LONG a = state[0], b = state[1], c = state[2], d = state[3];
  for (int i = 0; i < 64; ++i) {
    MD5_LONG f;
    int g;
    if (i < 16) {
      f = (b & c) | (~b & d);
      g = i;
    } else if (i < 32) {
      f = (d & b) | (~d & c);
      g = (5 * i + 1) % 16;
    } else if (i < 48) {
      f = b ^ c ^ d;
      g = (3 * i + 5) % 16;
    } else {
      f = c ^ (b | ~d);
      g = (7 * i) % 16;
    }
    f += a + K[i] + M[g];
    a = d;
    d = c;
    c = b;
    b += rotl(f, S[i]);
  }
  state[0] += a;
  state[1] += b;
  state[2] += c;
  state[3] += d;
}

inline void init(MD5_CTX* context) {
  context->A = 0x67452301;
  context->B = 0xefcdab89;
  context->C = 0x98badcfe;
  context->D = 0x10325476;
  context->Nl = context->Nh = 0;
  context->num = 0;
}

inline void update(MD5_CTX* context, const void* data, std::size_t length) {
  const unsigned char* input = static_cast<const unsigned char*>(data);
  std::size_t index = context->num % 4u;
  const std::size_t part = MD5_CBLOCK - index;
  context->num += static_cast<unsigned int>(length);
  if (index != 0 && length >= part) {
    std::memcpy(&context->data[index / 4], input, part);
    transform(&context->A, reinterpret_cast<const unsigned char*>(context->data));
    input += part;
    length -= part;
    index = 0;
  }
  while (length >= MD5_CBLOCK) {
    transform(&context->A, input);
    input += MD5_CBLOCK;
    length -= MD5_CBLOCK;
  }
  std::memcpy(&context->data[index / 4], input, length);
}

inline void final(unsigned char digest[MD5_DIGEST_LENGTH], MD5_CTX* context) {
  static const unsigned char padding[MD5_CBLOCK] = {0x80};
  unsigned char bits[8];
  const std::uint64_t bit_count = static_cast<std::uint64_t>(context->num) * 8u;
  for (int i = 0; i < 8; ++i) {
    bits[i] = static_cast<unsigned char>((bit_count >> (8 * i)) & 0xffu);
  }
  const unsigned int index = context->num % MD5_CBLOCK;
  update(context, padding, (index < 56) ? (56 - index) : (120 - index));
  update(context, bits, 8);
  const MD5_LONG registers[4] = {context->A, context->B, context->C, context->D};
  for (int r = 0; r < 4; ++r) {
    for (int j = 0; j < 4; ++j) {
      digest[r * 4 + j] = static_cast<unsigned char>((registers[r] >> (8 * j)) & 0xffu);
    }
  }
}

}  // namespace jo_md5

inline void MD5_Init(MD5_CTX* context) { jo_md5::init(context); }
inline void MD5_Update(MD5_CTX* context, const void* data, std::size_t length) {
  jo_md5::update(context, data, length);
}
inline void MD5_Final(unsigned char digest[MD5_DIGEST_LENGTH], MD5_CTX* context) {
  jo_md5::final(digest, context);
}
