/**
 * @file bit_reader.hpp
 * @brief Bit-level manipulation for binary protocol decoding.
 */

#ifndef RTK_ENGINE_BIT_READER_HPP
#define RTK_ENGINE_BIT_READER_HPP

#include <cstdint>
#include <cstddef>

namespace rtk {

/**
 * @brief Helper to read arbitrary bit-length fields from a byte array.
 */
class BitReader {
public:
    /**
     * @param data Pointer to the start of the bitstream.
     * @param len Length of the data in bytes.
     */
    BitReader(const uint8_t* data, size_t len) : data_(data), len_(len), bit_pos_(0) {}

    /**
     * @brief Read an unsigned integer of specified bit length.
     * @param bits Number of bits to read (1-32).
     * @return uint32_t Decoded value.
     */
    uint32_t read(int bits) {
        if (bits <= 0 || bits > 32) return 0;
        uint32_t val = 0;
        for (int i = 0; i < bits; ++i) {
            size_t byte_idx = bit_pos_ / 8;
            int bit_idx = 7 - (bit_pos_ % 8);
            if (byte_idx < len_) {
                if (data_[byte_idx] & (1 << bit_idx)) {
                    val |= (1 << (bits - 1 - i));
                }
            }
            bit_pos_++;
        }
        return val;
    }

    /**
     * @brief Read a signed integer (two's complement) of specified bit length.
     * @param bits Number of bits to read (1-32).
     * @return int32_t Decoded signed value.
     */
    int32_t readSigned(int bits) {
        uint32_t uval = read(bits);
        if (uval & (1 << (bits - 1))) {
            return static_cast<int32_t>(uval | (~0U << bits));
        }
        return static_cast<int32_t>(uval);
    }

    /** @brief Skip a specified number of bits. */
    void skip(int bits) { bit_pos_ += bits; }

private:
    const uint8_t* data_;
    size_t len_;
    size_t bit_pos_;
};

/**
 * @brief Helper to read up to 64 bits with sign extension.
 */
static int64_t readSigned64(BitReader& br, int bits) {
    uint64_t val = 0;
    for (int i = 0; i < bits; ++i) {
        val = (val << 1) | br.read(1);
    }
    if (val & (1ULL << (bits - 1))) {
        return static_cast<int64_t>(val | (~0ULL << bits));
    }
    return static_cast<int64_t>(val);
}

} // namespace rtk

#endif // RTK_ENGINE_BIT_READER_HPP
