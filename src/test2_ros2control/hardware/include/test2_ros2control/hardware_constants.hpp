#ifndef HARDWARE_CONSTANTS__TYPES__HARDWARE_CONSTANTS_TYPE_VALUES_HPP_
#define HARDWARE_CONSTANTS__TYPES__HARDWARE_CONSTANTS_TYPE_VALUES_HPP_

namespace hardware_constants
{
    /// Constant defining position interface
    constexpr char HW_IF_POSITION[] = "position";
    /// Constant defining velocity interface
    constexpr char HW_IF_VELOCITY[] = "velocity";
    /// Constant defining acceleration interface
    constexpr char HW_IF_ACCELERATION[] = "acceleration";
    /// Constant defining effort interface
    constexpr char HW_IF_EFFORT[] = "effort";
    /// Number of hardware interfaces
    constexpr int NUM_STATE = 1;
    /// Number of commands
    constexpr int NUM_COMMANDS = 1;
} // namespace hardware_constants

#endif // HARDWARE_CONSTANTS__TYPES__HARDWARE_CONSTANTS_TYPE_VALUES_HPP_
