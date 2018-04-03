#pragma once

#include <memory>
#include <vector>

#include <Eigen/Core>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/kinematics_cache.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/solvers/gurobi_solver.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace manipulation {

struct QuasistaticSystemOptions {
  double period_sec{0.01};
  std::vector<bool> is_contact_2d{};
  double mu{1};  // coefficent of friction for all contacts
  double kBigM{1};
  bool is_analytic{false};
  bool is_using_kinetic_energy_minimizing_QP{false};
};

/// Given the current configuration of a discrete, quasistatic, rigid, multibody
/// system, QuasistaticSystem computes the configuration of the system's
/// configuration of the next time step.
/// For a system with the following manipulator equation:
/// H(q) q_ddot + C(q, dq)q_dot = B tau + G(q),
/// the system being quasistatic means that the LHS of the manipulatator
/// equation is always 0, i.e. the equations of motion of the system becomes
/// B tau + G(q) = 0
///
/// q = [qu; qa]: configuration of the system.
/// qu ∈ R^{nu}: unactuated DOFs.
/// qa ∈ R^{na}: actuated DOFs.
/// v = [vu; va]: generalized velocity of the system, which can be different
/// from q_dot if rotation is represented using quaternions.
///
/// nu: number of unactuated states.
/// na: number of actuated states.
/// nq = nu + na.
/// nc: number of contacts.
/// nf ∈ R^{nc}. nf[i]: number of tangent vectors spanning the tangent space at
/// contact i.
/// nd = ∑nf[i]: total number of tangent vectors.
///
/// φ ∈ R^{nc}: signed distance function for all contacts.
/// λn ∈ R^{nc}: normal contact forces.
/// λf ∈ R^{nd}: friction components along each of the tangent vectors.
/// Wn = [∂φ/∂qu]^T ∈ R^{nv_u × nc}. Wn*λn is the generalized forces
/// corresponding to unactuated velocities generated by λn.
/// Wf ∈ R^{nv_u × nd}. Wf*λf is the generalized forces corresponding to
/// unactuated velocities generated by λf.
/// Jn = ∂φ/∂q ∈ R^{nc × nq} : partial derivative (Jacobian) of φ w.r.t q.
/// Jf ∈ R^{nd × nq} : partial derivative of relative sliding velocities w.r.t
/// q_dot.
///
/// γ ∈ R^{nc}: slack variables for all contacts.
template <class Scalar>
class QuasistaticSystem : public systems::LeafSystem<Scalar> {
 public:
  // idx_base: base body (has a floating joint to the world) of
  // the unactuated rigid body mechanism.
  QuasistaticSystem(int idx_base, const std::vector<int>& idx_unactuated_bodies,
                    const std::vector<int>& fixed_base_velocities,
                    const std::vector<int>& fixed_base_positions,
                    const QuasistaticSystemOptions& options);

  const systems::OutputPort<Scalar>& state_output() const;
  const systems::OutputPort<Scalar>& decision_variables_output() const;
  double get_period_sec() const { return period_sec_; }
  void set_analytic(bool is_analytic) { is_analytic_ = is_analytic; }

 protected:
  void Initialize();
  void DoCalcDiscreteVariableUpdates(
      const systems::Context<Scalar>& context,
      const std::vector<const systems::DiscreteUpdateEvent<Scalar>*>&,
      systems::DiscreteValues<Scalar>* discrete_state) const override;
  VectorX<Scalar> GetQuasistaticSystemStatesFromRigidBodyTreePositions(
      const KinematicsCache<double>& cache) const;
  VectorX<Scalar> GetRigidBodyTreePositionsFromQuasistaticSystemStates(
      const Eigen::Ref<const VectorX<Scalar>>& q_quasistatic_system) const;

  // As half of the vectors that span the tangent spaces are the negation of
  // the other half, half of the columns of Jf are also the negation of the
  // other half. Jf_half is one of the two halfs of J.
  void CalcJf(const Eigen::Ref<const Eigen::MatrixXd>& Jf_half,
              Eigen::MatrixXd* const Jf_ptr) const;
  void CalcWnWfJnJfPhi(const KinematicsCache<double>& cache,
                       Eigen::MatrixXd* const Wn_ptr,
                       Eigen::MatrixXd* const Wf_ptr,
                       Eigen::MatrixXd* const Jn_ptr,
                       Eigen::MatrixXd* const Jf_ptr,
                       Eigen::VectorXd* const phi_ptr) const;
  void DoCalcWnWfJnJfPhi(const KinematicsCache<double>& cache,
                         Eigen::MatrixXd* const Wn_ptr,
                         Eigen::MatrixXd* const Wf_ptr,
                         Eigen::MatrixXd* const Jn_ptr,
                         Eigen::MatrixXd* const Jf_ptr,
                         Eigen::VectorXd* const phi_ptr) const;

  double CalcBigM(double max_impulse, double max_delta_q, double max_gamma,
                  const Eigen::Ref<const Eigen::MatrixXd>& Jn,
                  const Eigen::Ref<const Eigen::MatrixXd>& Jf,
                  const Eigen::Ref<const Eigen::VectorXd>& phi,
                  const Eigen::Ref<const Eigen::VectorXd>& qa_dot_d) const;
  virtual void DoCalcWnWfJnJfPhiAnalytic(const KinematicsCache<double>& cache,
                                         Eigen::MatrixXd* const Wn_ptr,
                                         Eigen::MatrixXd* const Wf_ptr,
                                         Eigen::MatrixXd* const Jn_ptr,
                                         Eigen::MatrixXd* const Jf_ptr,
                                         Eigen::VectorXd* const phi_ptr) const;
  // E ∈ R^{nd × nq}: a matrix with entries that are either 0 or 1. E'*λn
  // gives an vector whose i-th entry is the sum of all friction
  // force components at contact i.
  Eigen::MatrixXd CalcE() const;

  // returns the generalized forces corresponding to unactuated velocities
  // generated by gravity.
  Eigen::VectorXd CalcExternalGeneralizedForce(
      KinematicsCache<double>* const cache) const;

  void CopyStateOut(const systems::Context<Scalar>& context,
                    systems::BasicVector<Scalar>* output) const {
    output->SetFromVector(GetRigidBodyTreePositionsFromQuasistaticSystemStates(
        context.get_discrete_state(0).CopyToVector().head(n1_)));
  }

  void CopyDecisionVariablesOut(const systems::Context<Scalar>& context,
                                systems::BasicVector<Scalar>* output) const {
    output->SetFromVector(
        context.get_discrete_state(0).CopyToVector().segment(n1_, n_));
  }

  const double period_sec_;
  // indices of unactuated bodies into the RigidBodyTree. The force balance of
  // these bodies are constraints in the MIQP.
  std::vector<int> idx_unactuated_bodies_;
  // index (into rigidbodytree) of the base link of the unactuated mechanism.
  int idx_base_;
  // DoFs of the floating joint of the base body of the unactuated rigid body
  // mechanisms that are fixed.
  const std::vector<int> fixed_base_positions_;
  // When base rotation is parametrized by a quaternion, fixing components of
  // the quaternion doesn't make much sense. Instead, we fix the rotations
  // about axes defined by components of the base body's angular velocity.
  std::vector<int> fixed_base_velocities_;

  // Each entry indicates if a contact is 2-dimensional, i.e. friction force
  // spanned only by 2 vectors.
  std::vector<bool> is_contact_2d_;

  const double mu_;
  // Gives users the option to specify a big M.
  // Currently this value is overwritten by a value calculated with interval
  // arithmetic based on bounds of all decision variables.
  const double kBigM_;
  // true if analytic expressions are available for Jn and Jf.
  bool is_analytic_;
  const bool is_using_kinetic_energy_minimizing_QP_;

  int nu_{0};           // number of underactuated DOFs, dim(qu).
  int n_vu_{0};         // number of underactuated velocities.
  int na_{0};           // number of actuated DOFs (inputs), dim(qa)
  int nc_;              // number of contacts.
  int nq_tree_;         // number of positions in RBT
  Eigen::VectorXi nf_;  // nubmer of vectors spanning each friction cone

  // indices of actuated states in increasing order.
  std::vector<int> idx_qa_;
  // columns of "contactJacobian" corresponding to qa.
  std::vector<int> idx_qa_in_q_;

  // indices of unactuated states of RigidBodyTree in increasing order
  // also the columns of "contactJacobian" corresponding to qu.
  std::vector<int> idx_qu_;
  std::vector<int> idx_qu_in_q_;

  std::vector<int> idx_vu_;
  bool is_base_quaternion_;

  // indices of all states in increasing order.
  // columns of "contactJacobian" that are used in the MIQP.
  std::vector<int> idx_q_;

  int n1_;  // = nu_ + na_  dimension of discrete state q = [qu; qa]
  int nd_;  // defined in QuasistaticSystem::UpdateNs()
  int n2_;  // defined in QuasistaticSystem::UpdateNs()
  int n_;   // number of decision variables in the MIQP

  // rigid body tree for model
  std::unique_ptr<RigidBodyTree<double>> tree_ =
      std::make_unique<RigidBodyTree<double>>();

 private:
  void UpdateNs() {
    // creates nf_ from is_contact_2d_
    nf_.resize(nc_);
    for (int i = 0; i < nc_; i++) {
      if (is_contact_2d_[i]) {
        nf_(i) = 2;
      } else {
        nf_(i) = 4;
      }
    }
    nu_ = idx_qu_.size();
    n_vu_ = idx_vu_.size();
    na_ = idx_qa_.size();
    n1_ = nu_ + na_;
    nd_ = nf_.sum();
    n2_ = nc_ * 2 + nd_;
    n_ = n1_ + 2 * n2_;
  }

  // After solving the MIQP, there could exist multiple feasible
  // motions (delta_q) that satisfy the force balance and contact/friction
  // constraints.
  // By solving a QP that minimizes the kinetic energy of the systemn, this
  // function finds the delta_q that results in a motion consistent with the
  // MIQP solution, and at the same time minimizes the KE of the system.
  void MinimizeKineticEnergy(
      Eigen::VectorXd* const delta_q_value_ptr,
      const Eigen::Ref<const Eigen::MatrixXd>& Jn,
      const Eigen::Ref<const Eigen::MatrixXd>& Jf,
      const Eigen::Ref<const Eigen::VectorXd>& z_n_value,
      const Eigen::Ref<const Eigen::VectorXd>& z_f_value,
      const Eigen::Ref<const Eigen::VectorXd>& z_gamma_value,
      const Eigen::Ref<const Eigen::VectorXd>& phi,
      KinematicsCache<double>* const cache) const;

  // This function calculates the system configuration at the next time step
  // given the system configuration at the current time step.
  // U: a diagonal matrix whose entries are the coefficient of friction for
  // each contact.
  // qa_dot_d: prescribed (commanded) change of the actuated DOFs over the
  // next time step.
  void StepForward(const Eigen::Ref<const Eigen::MatrixXd>& Wn,
                   const Eigen::Ref<const Eigen::MatrixXd>& Wf,
                   const Eigen::Ref<const Eigen::MatrixXd>& Jn,
                   const Eigen::Ref<const Eigen::MatrixXd>& Jf,
                   const Eigen::Ref<const Eigen::MatrixXd>& U,
                   const Eigen::Ref<const Eigen::MatrixXd>& E,
                   const Eigen::Ref<const Eigen::VectorXd>& phi,
                   const Eigen::Ref<const Eigen::VectorXd>& f,
                   const Eigen::Ref<const Eigen::VectorXd>& qa_dot_d) const;

  // initial guesses
  mutable Eigen::VectorXd lambda_n_start_;
  mutable Eigen::VectorXd lambda_f_start_;
  mutable Eigen::VectorXd gamma_start_;
  mutable Eigen::VectorXd z_n_start_;
  mutable Eigen::VectorXd z_f_start_;
  mutable Eigen::VectorXd z_gamma_start_;

  // "hypothetical penetration"
  mutable Eigen::VectorXd phi_bar_;

  // MIQP program
  std::unique_ptr<solvers::MathematicalProgram> prog_;
  solvers::GurobiSolver solver_;

  solvers::VectorXDecisionVariable delta_q_;
  solvers::VectorXDecisionVariable lambda_n_;
  solvers::VectorXDecisionVariable lambda_f_;
  solvers::VectorXDecisionVariable gamma_;
  solvers::VectorXDecisionVariable z_n_;
  solvers::VectorXDecisionVariable z_f_;
  solvers::VectorXDecisionVariable z_gamma_;

  solvers::BoundingBoxConstraint* bounds_delta_q_{nullptr};
  solvers::BoundingBoxConstraint* bounds_gamma_{nullptr};
  solvers::BoundingBoxConstraint* bounds_lambda_n_{nullptr};
  solvers::BoundingBoxConstraint* bounds_lambda_f_{nullptr};
  solvers::LinearEqualityConstraint* force_balance_{nullptr};
  solvers::LinearConstraint* non_penetration_{nullptr};
  solvers::LinearConstraint* coulomb_friction1_{nullptr};
  solvers::LinearConstraint* coulomb_friction2_{nullptr};
  solvers::LinearConstraint* non_penetration_complementary_{nullptr};
  solvers::LinearConstraint* coulomb_friction1_complementary_{nullptr};
  solvers::LinearConstraint* coulomb_friction2_complementary_{nullptr};
  solvers::LinearConstraint* decision_variables_complementary_{nullptr};
  solvers::QuadraticCost* objective_{nullptr};
};

}  // namespace manipulation
}  // namespace drake
