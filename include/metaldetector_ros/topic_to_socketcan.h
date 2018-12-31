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

#ifndef METALDETECTOR_ROS_TOPIC_TO_SOCKETCAN_H
#define METALDETECTOR_ROS_TOPIC_TO_SOCKETCAN_H

#include <socketcan_interface/socketcan.h>
#include <ros/ros.h>
#include <metaldetector_ros/StampedFloat.h>
#include "yaml-cpp/yaml.h"
#include <string>
#include <sstream>
#include <iostream>
#include "ros/package.h"

namespace metaldetector_ros
{

  class CanSubscriber {
    public:
      ros::Subscriber sub;
      int32_t pre_offset;
      float offset;
      float scale;
      CanSubscriber();
  };

  CanSubscriber::CanSubscriber(void) {
    pre_offset = 0;
    offset = 0.0;
    scale = 1.0;
  }

  class TopicToSocketCAN
  {
    public:
      TopicToSocketCAN(ros::NodeHandle* nh, ros::NodeHandle* nh_param, boost::shared_ptr<can::DriverInterface> driver);
      void setup();

    private:
      ros::Subscriber can_topic_;
      boost::shared_ptr<can::DriverInterface> driver_;
      YAML::Node send_config;
      std::map<std::string, CanSubscriber*> sub_map_;

      can::StateInterface::StateListener::Ptr state_listener_;

      void canSendCallback(const StampedFloat::ConstPtr& message, int device,
          int reg);
      void stateCallback(const can::State & s);
  };

};


#endif
