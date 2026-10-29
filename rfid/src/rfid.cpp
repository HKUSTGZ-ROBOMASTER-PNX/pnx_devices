#include "rfid.hpp"
#include <algorithm>

namespace rfid
{
namespace
{
uint8_t checksum(const uint8_t* bytes, std::size_t size) noexcept
{
    uint8_t value = 0;
    for (std::size_t i = 0; i < size; ++i)
    {
        value ^= bytes[i];
    }
    return static_cast<uint8_t>(~value);
}
} // namespace

request read_block(uint8_t address, uint8_t block) noexcept
{
    // Byte 5 disables the reader's own indication; the application owns the buzzer.
    request bytes{0x01, 0x08, 0xA3, address, block, 0x00, 0x00, 0x00};
    bytes.back() = checksum(bytes.data(), bytes.size() - 1);
    return bytes;
}

void decoder::reset() noexcept
{
    size_ = 0;
}

decode_result decoder::push(uint8_t byte, data& output) noexcept
{
    bytes_[size_++] = byte;
    while (size_ != 0)
    {
        if (bytes_[0] == 0x01 && (size_ < 2 || bytes_[1] == 8 || bytes_[1] == bytes_.size()) &&
            (size_ < 3 || bytes_[2] == 0xA3) && (size_ < 4 || bytes_[3] == address_) &&
            (size_ < 5 || bytes_[4] == (bytes_[1] == 8 ? 0x01 : 0x00)))
        {
            if (size_ < 2 || size_ < bytes_[1])
            {
                return decode_result::none;
            }
            if (checksum(bytes_.data(), size_ - 1) == bytes_[size_ - 1])
            {
                const bool success = size_ == bytes_.size();
                if (success)
                {
                    std::copy_n(bytes_.begin() + 5, output.size(), output.begin());
                }
                reset();
                return success ? decode_result::success : decode_result::failed;
            }
        }
        // Seek the next header after noise, wrong address or corrupt frame.
        --size_;
        std::move(bytes_.begin() + 1, bytes_.begin() + 1 + size_, bytes_.begin());
    }
    return decode_result::none;
}
} // namespace rfid
