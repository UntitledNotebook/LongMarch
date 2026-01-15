#pragma once

#include <vector>

#include "contradium/rbd/rbd_body.h"
#include "contradium/rbd/rbd_joint.h"


namespace contradium::rbd {

/** \brief Contains all information about the rigid body model
 *
 * This class contains all information required to perform the forward
 * dynamics calculation. The variables in this class are also used for
 * storage of temporary values. It is designed for use of the Articulated
 * Rigid Body Algorithm (which is implemented in ForwardDynamics()) and
 * follows the numbering as described in Featherstone's book.
 *
 * Please note that body 0 is the root body and the moving bodies start at
 * index 1. This numbering scheme is very beneficial in terms of
 * readability of the code as the resulting code is very similar to the
 * pseudo-code in the RBDA book.
 */
struct Model {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Model();

  // =========================================================================
  // Structural information
  // =========================================================================

  /// \brief The id of the parent body
  std::vector<unsigned int> lambda;

  /** \brief Number of degrees of freedom of the model
   *
   * This value contains the number of entries in the generalized state (q)
   * velocity (qdot), acceleration (qddot), and force (tau) vector.
   */
  unsigned int dof_count; /// Should be equal to mBodies.size() - 1

  /// \brief Id of the previously added body (for AppendBody)
  unsigned int previously_added_body_id;

  /// \brief The cartesian vector of gravity
  Vector3 gravity;

  // =========================================================================
  // State information
  // =========================================================================

  /// \brief The spatial velocity of the bodies
  std::vector<SpatialVector> v;

  /// \brief The spatial acceleration of the bodies
  std::vector<SpatialVector> a;

  // =========================================================================
  // Joints
  // =========================================================================

  /// \brief All joints
  std::vector<Joint> mJoints;

  /// \brief The joint axis for joint i (motion subspace S)
  std::vector<SpatialVector> S;

  std::vector<SpatialVector> v_J; // v_{Ji} = S_i * \dot{q}_i
  std::vector<SpatialVector> c_J; // c_J = \circ{S}_i * \dot{q}_i
  std::vector<SpatialTransform> X_T; // X_{T}(i) = {}^{\lambda(i),i} X_{\lambda(i)}

  // =========================================================================
  // Dynamics variables P. 96 (RNEA) P. 107 (CRBA) P. 132 (ABA)
  // =========================================================================

  std::vector<SpatialVector> c;  // c_{i}=c_{\mathrm{J}i}+v_{i}\times v_{\mathrm{J}i}
  std::vector<SpatialRigidBodyInertia> I; // at link origin, not COM

  // ABA
  std::vector<SpatialMatrix> IA;
  std::vector<SpatialVector> pA;
  std::vector<SpatialVector> U;
  Eigen::VectorXd d;
  Eigen::VectorXd u;

  /// RNEA
  std::vector<SpatialVector> f;

  // CRBA
  std::vector<SpatialRigidBodyInertia> Ic;

  // =========================================================================
  // Bodies
  // =========================================================================

  std::vector<SpatialTransform> X_lambda; // {}^i X_{\lambda(i)}
  std::vector<SpatialTransform> X_base; // ^{i}X_0
  std::vector<Body> mBodies; // 0 is the base, moving parts start with 1, do not contain fixed
  std::vector<FixedBody> mFixedBodies;
  unsigned int fixed_body_discriminator;

  // =========================================================================
  // Methods
  // =========================================================================

  /** \brief Connects a given body to the model
   *
   * \param parent_id   id of the parent body
   * \param joint_frame the transformation from the parent frame to the origin
   *                    of the joint frame
   * \param joint       specification for the joint
   * \param body        specification of the body itself
   *
   * \returns id of the added body
   */
  unsigned int AddBody(unsigned int parent_id,
                       const SpatialTransform &joint_frame,
                       const Joint &joint,
                       const Body &body);

  /** \brief Adds a Body to the model such that the previously added Body
   * is the Parent.
   */
  unsigned int AppendBody(const SpatialTransform &joint_frame, const Joint &joint, const Body &body);
};

} // namespace contradium::rbd

