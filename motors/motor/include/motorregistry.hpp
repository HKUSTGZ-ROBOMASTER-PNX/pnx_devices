#pragma once

#include "motor.hpp"
#include "motorhandler.hpp"

namespace motors::detail
{

// Returns the single protocol handler owned by the device layer.
// The returned handler has static lifetime; nullptr means unsupported protocol.
motorhandler* handler_for(protocol value) noexcept;

} // namespace motors::detail
