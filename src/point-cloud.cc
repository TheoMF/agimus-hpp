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

#include <hpp/agimus/point-cloud.hh>

#include <ros/node_handle.h>
#include <hpp/pinocchio/device.hh>

namespace hpp {
  namespace agimus {

    PointCloud::~PointCloud()
    {
      shutdownRos();
    }

    bool PointCloud::initializeRosNode (const std::string& name, bool anonymous)
    {
      if (!ros::isInitialized()) {
        // Call ros init
        int option = ros::init_options::NoSigintHandler |
	  (anonymous ? ros::init_options::AnonymousName : 0);
        int argc = 0;
        ros::init (argc, NULL, name, option);
      }
      bool ret = false;
      if (!handle_) {
        handle_ = new ros::NodeHandle();
        ret = true;
      }
      return ret;
    }

    void PointCloud::shutdownRos ()
    {
      if (!handle_) return;
      boost::mutex::scoped_lock lock(mutex_);
      if (handle_) delete handle_;
      handle_ = NULL;
    }

    bool PointCloud::getPointCloud
    (const std::string& /*joint*/, const std::string& topic, value_type timeOut)
    {
      if (!handle_)
        throw std::logic_error ("Initialize ROS first");
      // create subscriber to topic
      waitingForData_ = false;
      ros::Subscriber subscriber = handle_->subscribe
	(topic, 1, &PointCloud::pointCloudCb, this);

      waitingForData_ = true;
      value_type begin = ros::Time::now().toSec();
      while (waitingForData_) {
	value_type now = ros::Time::now().toSec();
	if (now - begin >= timeOut) {
	  break;
	}
	ros::spinOnce();
	ros::Duration(1e-2).sleep();
      }
      if (waitingForData_) {
	// timeout reached
	waitingForData_ = false;
	return false;
      }
      return true;
    }

    void PointCloud::pointCloudCb(const sensor_msgs::PointCloud2ConstPtr& data)
    {
      std::cout << "PointCloud::pointCloudCb" << std::endl;
      if (!waitingForData_) return;
      waitingForData_ = false;
      std::cout << "PointCloud2" << std::endl;
      std::cout << "  size:" << data->data.size() << std::endl;
      for (auto& field : data->fields) {
	std::cout << "  == field =====================" << std::endl;
	std::cout << "    name: " << field.name << std::endl;
	std::cout << "    offset: " << field.offset << std::endl;
	std::cout << "    datatype: " << field.datatype << std::endl;
	std::cout << "    count: " << field.count << std::endl;
      }
    }
  } // namespace agimus
} // namespace hpp
