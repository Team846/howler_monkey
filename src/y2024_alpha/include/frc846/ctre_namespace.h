#ifndef FRC846_CTRE_NAMESPACE_H_
#define FRC846_CTRE_NAMESPACE_H_

#include <ctre/phoenix6/TalonFX.hpp>

// Gathers all commonly used CTRE namespaces into a single `ctre` namespace.
#define FRC846_CTRE_NAMESPACE()                      \
  namespace ctre {                                   \
  using namespace ctre::phoenix6::controls;          \
  using namespace ctre::phoenix6::configs;           \
  using namespace ctre::phoenix6::signals;           \
  using namespace ctre::phoenix6::hardware;          \
  using namespace ctre::phoenix6;                    \
  }
#endif  // FRC846_CTRE_NAMESPACE_H_