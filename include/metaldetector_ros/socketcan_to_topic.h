/*
 * Copyright (c) 2016, Ivor Wanders
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef METALDETECTOR_ROS_SOCKETCAN_TO_TOPIC_H
#define METALDETECTOR_ROS_SOCKETCAN_TO_TOPIC_H

#include <socketcan_interface/socketcan.h>
#include <can_msgs/Frame.h>
#include <ros/ros.h>
#include "yaml-cpp/yaml.h"

namespace metaldetector_ros
{

  class CanPublisher {
    public:
      ros::Publisher pub;
      int32_t pre_offset;
      float offset;
      float scale;
      CanPublisher();
  };

  CanPublisher::CanPublisher(void) {
    pre_offset = 0;
    offset = 0.0;
    scale = 1.0;
  }

  class SocketCANToTopic
  {
    public:
      SocketCANToTopic(ros::NodeHandle* nh, ros::NodeHandle* nh_param, boost::shared_ptr<can::DriverInterface> driver);
      void setup();

    private:
      ros::Publisher can_topic_;
      boost::shared_ptr<can::DriverInterface> driver_;

      can::CommInterface::FrameListener::Ptr frame_listener_;
      can::StateInterface::StateListener::Ptr state_listener_;

      std::map<std::string, CanPublisher*> pub_map_;
      YAML::Node receive_config;

      void frameCallback(const can::Frame& f);
      void stateCallback(const can::State & s);
  };

  void convertSocketCANToMessage(const can::Frame& f, can_msgs::Frame& m)
  {
    m.id = f.id;
    m.dlc = f.dlc;
    m.is_error = f.is_error;
    m.is_rtr = f.is_rtr;
    m.is_extended = f.is_extended;

    for (int i = 0; i < 8; i++)  // always copy all data, regardless of dlc.
    {
      m.data[i] = f.data[i];
    }
  };

};


#endif
