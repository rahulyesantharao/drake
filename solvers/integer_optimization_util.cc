#include "drake/solvers/integer_optimization_util.h"

#include "drake/solvers/binding.h"
#include "drake/solvers/create_constraint.h"

namespace drake {
namespace solvers {
Binding<LinearConstraint> CreateLogicalAndConstraint(
    const symbolic::Expression& b1, const symbolic::Expression& b2,
    const symbolic::Expression& b1_and_b2) {
  return internal::BindingDynamicCast<LinearConstraint>(
      // clang-format off
      internal::ParseConstraint(b1_and_b2 >= b1 + b2 - 1 &&
                                    b1_and_b2 <= b1 &&
                                    b1_and_b2 <= b2 &&
                                    0 <= b1_and_b2 &&
                                    b1_and_b2 <= 1));
      // clang-format on
}

Binding<LinearConstraint> CreateLogicalOrConstraint(
    const symbolic::Expression& b1, const symbolic::Expression& b2,
    const symbolic::Expression& b1_or_b2) {
  return internal::BindingDynamicCast<LinearConstraint>(
      // clang-format off
      internal::ParseConstraint(b1_or_b2 <= b1 + b2 &&
                                    b1_or_b2 >= b1 &&
                                    b1_or_b2 >= b2 &&
                                    0 <= b1_or_b2 &&
                                    b1_or_b2 <= 1));
      // clang-format on
}

Binding<LinearConstraint> CreateLogicalXorConstraint(
    const symbolic::Expression& b1, const symbolic::Expression& b2,
    const symbolic::Expression& b1_xor_b2) {
  return internal::BindingDynamicCast<LinearConstraint>(
      // clang-format off
      internal::ParseConstraint(b1_xor_b2 <= b1 + b2 &&
                                    b1_xor_b2 >= b1 - b2 &&
                                    b1_xor_b2 >= b2 - b1 &&
                                    b1_xor_b2 <= 2 - b1 - b2 &&
                                    0 <= b1_xor_b2 &&
                                    b1_xor_b2 <= 1));
      // clang-format on
}
}  // namespace solvers
}  // namespace drake
