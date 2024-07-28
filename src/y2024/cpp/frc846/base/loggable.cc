#include "frc846/base/loggable.h"

#include <sstream>

namespace frc846::base {

std::string Loggable::Join(std::string p, std::string n) { return p + "/" + n; }

unsigned int Loggable::GetWarnCount() { return warn_count_; }

unsigned int Loggable::GetErrorCount() { return error_count_; }

unsigned int Loggable::warn_count_ = 0;
unsigned int Loggable::error_count_ = 0;

}  // namespace frc846