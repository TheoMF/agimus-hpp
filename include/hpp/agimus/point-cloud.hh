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
#include <hpp/pinocchio/fwd.hh>

namespace hpp {
  namespace agimus {
    typedef pinocchio::DevicePtr_t DevicePtr_t;
    typedef pinocchio::size_type size_type;
    typedef pinocchio::value_type value_type;
    HPP_PREDEF_CLASS(PointCloud);
    typedef shared_ptr<PointCloud> PointCloudPtr_t;

    class PointCloud
    {
    public:
      static PointCloudPtr_t create (const DevicePtr_t device)
      {
	PointCloudPtr_t ptr (new PointCloud(device));
	ptr->init(ptr);
	return ptr;
      }
      /// Initialize a ROS node with given name
      bool initializeRosNode (const std::string& name, bool anonymous);

      void shutdownRos ();
      /// Get point cloud from ROS topic
      /// \param topic name of the topic,
      /// \param timeOut time after which the function returns error if no data
      ///        has been published (in seconds).
      /// \param topic name of the topic, should be of type
      ///        sensor_msgs/PointCloud2
      /// \param joint joint to which the point cloud is attached.
      ///        point cloud has been read.
      bool getPointCloud(const std::string& joint, const std::string& topic,
			 value_type timeOut);
      /// Callback to the point cloud topic
      void pointCloudCb(const sensor_msgs::PointCloud2ConstPtr& data);
      /// Shut down ROS
      ~PointCloud();
    private:
      /// Constructor
      PointCloud(const DevicePtr_t& device): device_ (device),
					     waitingForData_(false),
					     handle_(0x0)
      {}
      void init (const PointCloudWkPtr_t)
      {}
      DevicePtr_t device_;
      bool waitingForData_;
      boost::mutex mutex_;
      ros::NodeHandle* handle_;
    }; // class PointCloud
  } // namespace agimus
} // namespace hpp

#endif // HPP_AGIMUS_POINT_CLOUD_HH
