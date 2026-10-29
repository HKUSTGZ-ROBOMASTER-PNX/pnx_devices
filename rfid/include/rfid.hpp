#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

// Read-only subset of the UART reader protocol used by pnx_template_F4.
namespace rfid
{
using data = std::array<uint8_t, 16>;
using request = std::array<uint8_t, 8>;

request read_block(uint8_t address, uint8_t block) noexcept;

enum class decode_result : uint8_t
{
    none,
    success,
    failed
};

// Single caller; feed every byte, including split/concatenated UART frames.
// Distinguishes successful data and an explicit read failure after checksum/address checks.
// Failure and unrelated frames never overwrite output. reset() discards partial data.
class decoder
{
public:
    explicit decoder(uint8_t address) noexcept : address_(address)
    {
    }

    decode_result push(uint8_t byte, data& output) noexcept;
    void reset() noexcept;

private:
    std::array<uint8_t, 22> bytes_{};
    std::size_t size_ = 0;
    uint8_t address_;
};
} // namespace rfid
