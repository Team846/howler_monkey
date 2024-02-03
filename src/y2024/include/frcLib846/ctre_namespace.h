#ifndef frcLib846_CTRE_NAMESPACE_H_
#define frcLib846_CTRE_NAMESPACE_H_

// Gathers all commonly used CTRE namespaces into a single `ctre` namespace.
#define frcLib846_CTRE_NAMESPACE()                     \
  namespace ctre {                                  \
  using namespace ctre::phoenix::motorcontrol::can; \
  using namespace ctre::phoenix::motorcontrol;      \
  using namespace ctre::phoenix::sensors;           \
  using namespace ctre::phoenix;                    \
  }
#endif  // frcLib846_CTRE_NAMESPACE_H_