#include "remoter.hpp"
#include <atomic>

namespace remotes
{
namespace
{
std::atomic<bool> permitted{true};
}

void set_enabled(bool enabled) noexcept
{
    permitted.store(enabled, std::memory_order_relaxed);
}

bool enabled() noexcept
{
    return permitted.load(std::memory_order_relaxed);
}
} // namespace remotes
