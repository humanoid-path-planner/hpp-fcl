/*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2020, INRIA
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Open Source Robotics Foundation nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*/

#include <hpp/fcl/octree.h>

namespace hpp
{
namespace fcl
{
  OcTreePtr_t makeOctree
  (const Eigen::Matrix<FCL_REAL,Eigen::Dynamic,3>& point_cloud,
   const FCL_REAL & resolution)
  {
    typedef Eigen::Matrix<FCL_REAL,Eigen::Dynamic,3> InputType;
    typedef InputType::ConstRowXpr RowType;
    
    boost::shared_ptr<octomap::OcTree> octree(new octomap::OcTree(resolution));
    for(Eigen::DenseIndex row_id = 0; row_id < point_cloud.rows(); ++row_id)
    {
      RowType row = point_cloud.row(row_id);
      octomap::point3d p(static_cast<float>(row[0]),static_cast<float>(row[1]),static_cast<float>(row[2]));
      octree->updateNode(p, true);
    }
    octree->updateInnerOccupancy();
    
    return OcTreePtr_t (new OcTree(octree));
  }
}
}
