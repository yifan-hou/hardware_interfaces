#ifndef _TYPES_HEADER_
#define _TYPES_HEADER_

#include <string>

/**
 * Safety mode. Determines what to do when the commanded pose is out of the safety region.
 *  SAFETY_MODE_NONE: Do not perform safety checking.
 *  SAFETY_MODE_TRUNCATE: Truncate the commanded pose to be within safe range.
 *  SAFETY_MODE_STOP: Throw an error and stop.
 */
enum class RobotSafetyMode {
  NONE,
  SAFETY_MODE_NONE,
  SAFETY_MODE_TRUNCATE,
  SAFETY_MODE_STOP
};

enum class RobotOperationMode {
  NONE,
  OPERATION_MODE_CARTESIAN,
  OPERATION_MODE_JOINT
};

/**
 * Force sensing mode: specifies what sensor does the wrench loop use.
 */
enum class ForceSensingMode {
  NONE,
  FORCE_MODE_ATI,
  FORCE_MODE_ROBOTIQ,
  FORCE_MODE_COINFT
};

enum class CameraSelection { NONE, GOPRO, REALSENSE };

enum class RandomType {
  NONE,
  CONSTANT,
  UNIFORM,
  GAUSSIAN,
};

template <class Enum>
const char* to_string(const Enum e);

// Get the enum for the provided string.
// return NONE if no match is found.
template <class EnumType>
EnumType string_to_enum(const std::string& string);

#endif  // _TYPES_HEADER_