// Copyright 2021, 2022, CNRS, Airbus SAS

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

#ifndef HPP_AGIMUS_POINT_CLOUD_HH
#define HPP_AGIMUS_POINT_CLOUD_HH

#include <boost/thread/mutex.hpp>

#include <ros/node_handle.h>
#include <sensor_msgs/PointCloud2.h>

#include <hpp/util/pointer.hh>
#include <hpp/agimus/fwd.hh>

namespace hpp {
  namespace agimus {
    class PointCloud
    {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      typedef hpp::fcl::OcTreePtr_t OcTreePtr_t;
      static PointCloudPtr_t create (const ProblemSolverPtr_t& ps)
      {
	PointCloudPtr_t ptr (new PointCloud(ps));
	ptr->init(ptr);
	return ptr;
      }
      /// Initialize a ROS node with given name
      bool initializeRosNode (const std::string& name, bool anonymous);

      void shutdownRos ();
      /// Build an octree from a point cloud read on a ROS topic
      /// \param octreeFrame frame to which the octree is attached.
      /// \param topic name of the topic. Topic should be of type
      ///        sensor_msgs/PointCloud2
      /// \param sensorFrame name of the frame in which the point cloud is
      ///        expressed.
      /// \param resolution resolution of the Octree built from the point cloud.
      /// \param configuration configuration of the system. Used to compute the
      ///        pose of the joint holding the octree with respect to the
      ///        sensor frame.
      /// \param timeOut time after which the function returns error if no data
      ///        has been published (in seconds).
      bool getPointCloud(const std::string& octreeFrame,
			 const std::string& topic,
			 const std::string& sensorFrame,
			 value_type resolution, const vector_t& configuration,
			 value_type timeOut);
      /// Callback to the point cloud topic
      void pointCloudCb(const sensor_msgs::PointCloud2ConstPtr& data);
      /// Shut down ROS
      ~PointCloud();
    private:
      /// Constructor
      PointCloud(const ProblemSolverPtr_t& ps);
      void init (const PointCloudWkPtr_t)
      {}
      void attachOctreeToRobot
      (const OcTreePtr_t& octree, const std::string& octreeFrame,
       const std::string& sensorFrame, const vector_t& configuration);
      bool displayOctree(const OcTreePtr_t& octree,
			 const std::string& octreeFrame,
			 const Transform3f& jMoctree);
      ProblemSolverPtr_t problemSolver_;
      bool waitingForData_;
      boost::mutex mutex_;
      ros::NodeHandle* handle_;
      PointMatrix_t points_;

    }; // class PointCloud
  } // namespace agimus
} // namespace hpp

#endif // HPP_AGIMUS_POINT_CLOUD_HH
