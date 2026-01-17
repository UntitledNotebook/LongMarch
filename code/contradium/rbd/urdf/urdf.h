//
// Created by f3f3xo on 12/25/25.
//

#ifndef LONGMARCH_URDF_H
#define LONGMARCH_URDF_H

#include "urdfModel.h"
#include "../rbd_model.h"
#include "../rbd_body.h"
#include "../rbd_joint.h"
namespace urdf {

/**
 * @brief Convert a URDF model to a contradium::rbd::Model
 * @param urdf_model The parsed URDF model
 * @return A contradium::rbd::Model constructed from the URDF
 */
contradium::rbd::Model toRBDModel(const std::shared_ptr<urdf::Model>& urdf_model);

}  // namespace urdf

#endif  // LONGMARCH_URDF_H
