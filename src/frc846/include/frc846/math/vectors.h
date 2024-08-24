// #pragma once

// #include <units/angle.h>
// #include <units/constants.h>
// #include <units/length.h>
// #include <units/math.h>

// #include <cmath>

// namespace frc846::math {

// template <typename T, int N = 2>
// class VectorND {
//   static_assert(N > 0,
//                 "VectorND can not be created with a dimension less than 1");
//   static_assert(units::traits::is_unit_t<T>(),
//                 "VectorND can only be created with unit types");

//  public:
//   // Constructs an N-dimensional vector given displacement of vector in each
//   // dimension
//   VectorND(const T (&arr)[N]) : dims{} {
//     for (int i = 0; i < N; i++) {
//       dims[i] = arr[i];
//     }
//   }

//   // Constructs an 2-dimensional vector from polar arguments (r, theta)
//   //   template <int Dimensionality = N,
//   //             typename = std::enable_if<Dimensionality == 2>>
//   //   VectorND(units::degree_t theta, T r) {}

//  private:
//   T dims[N];
// };

// };  // namespace frc846::math