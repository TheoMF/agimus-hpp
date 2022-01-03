// Copyright 2022, CNRS, Airbus SAS

// Author: Florent Lamiraux

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:

// 1. Redistributions of source code must retain the above copyright
// notice, this list of conditions and the following disclaimer.

// 2. Redistributions in binary form must reproduce the above
// copyright notice, this list of conditions and the following
// disclaimer in the documentation and/or other materials provided
// with the distribution.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
// INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
// HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
// STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
// OF THE POSSIBILITY OF SUCH DAMAGE.

#ifndef AGIMUS_HPP_FWD_HH
#define AGIMUS_HPP_FWD_HH

#include <hpp/manipulation/fwd.hh>

namespace hpp{
namespace agimus{
  
  typedef pinocchio::DevicePtr_t DevicePtr_t;
  typedef pinocchio::Frame Frame;
  typedef pinocchio::FrameIndex FrameIndex;
  typedef pinocchio::JointIndex JointIndex;
  typedef pinocchio::JointPtr_t JointPtr_t;
  typedef pinocchio::size_type size_type;
  typedef pinocchio::Transform3f Transform3f;
  typedef pinocchio::value_type value_type;
  typedef pinocchio::vector_t vector_t;
  typedef manipulation::ProblemSolverPtr_t ProblemSolverPtr_t;
  HPP_PREDEF_CLASS(PointCloud);
  typedef shared_ptr<PointCloud> PointCloudPtr_t;
  typedef Eigen::Matrix<value_type, Eigen::Dynamic, 3> PointMatrix_t;
} // namespace agimus
} // namespace hpp
#endif // AGIMUS_HPP_FWD_HH
