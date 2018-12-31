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

#include <metaldetector_ros/socketcan_to_topic.h>
#include <socketcan_interface/string.h>
#include <metaldetector_ros/StampedFloat.h>
#include <string>
#include <sstream>
#include <iostream>
#include "ros/package.h"

namespace metaldetector_ros
{

  SocketCANToTopic::SocketCANToTopic(ros::NodeHandle* nh, ros::NodeHandle* nh_param,
      boost::shared_ptr<can::DriverInterface> driver)
  {
    driver_ = driver;

    receive_config = YAML::LoadFile(ros::package::getPath("metaldetector_ros") + "/config/receive_config.yaml");
    if (receive_config.IsNull()) {
      std::cout << "No receive_config file found!";
    } else {
      for (YAML::const_iterator it = receive_config.begin();
          it != receive_config.end(); ++it) {
        YAML::Node reg_conf = it->second;

        for (YAML::const_iterator reg_it = reg_conf.begin();
            reg_it != reg_conf.end(); ++reg_it) {
          YAML::Node reg_config = reg_it->second;
          std::cout << "Found receive command set for: "
            << reg_config["topic"].as<std::string>() << std::endl;

          std::ostringstream id;
          id << it->first.as<std::string>() << "." << reg_it->first.as<std::string>();

          ros::Publisher pub = nh->advertise <StampedFloat>(reg_config["topic"].as<std::string>(), 10);

          CanPublisher* cp = new CanPublisher();
          cp->pub = pub;

          if (reg_config["pre-offset"]) {
            cp->pre_offset = reg_config["pre-offset"].as<int32_t>();
          }

          if (reg_config["scale"]) {
            cp->scale = reg_config["scale"].as<float>();
          }

          if (reg_config["offset"]) {
            cp->offset = reg_config["offset"].as<float>();
          }

          pub_map_[id.str()] = cp;


        }
      }
    }



  };

  void SocketCANToTopic::setup()
  {
    // register handler for frames and state changes.
    frame_listener_ = driver_->createMsgListener(
        can::CommInterface::FrameDelegate(this, &SocketCANToTopic::frameCallback));

    state_listener_ = driver_->createStateListener(
        can::StateInterface::StateDelegate(this, &SocketCANToTopic::stateCallback));
  };

  void SocketCANToTopic::frameCallback(const can::Frame& f)
  {
    // ROS_DEBUG("Message came in: %s", can::tostring(f, true).c_str());
    can::Frame frame = f;  // copy the frame first, cannot call isValid() on const.
    if (!frame.isValid())
    {
      ROS_ERROR("Invalid frame from SocketCAN: id: %#04x, length: %d, is_extended: %d, is_error: %d, is_rtr: %d",
          f.id, f.dlc, f.is_extended, f.is_error, f.is_rtr);
      return;
    }
    else
    {
      if (f.is_error)
      {
        // can::tostring cannot be used for dlc > 8 frames. It causes an crash
        // due to usage of boost::array for the data array. The should always work.
        ROS_WARN("Received frame is error: %s", can::tostring(f, true).c_str());
      }
    }


    int reg = f.data[0] & 0x7F;

    uint32_t data = f.data[1] << 24
      | f.data[2] << 16
      | f.data[3] << 8
      | f.data[4];

    int32_t pre_data;
    std::ostringstream id;
    id << f.id << "." << reg;


    StampedFloat msg;
    msg.stamp = ros::Time::now();
    msg.data = data;


    CanPublisher* cp = pub_map_[id.str()];

    if (cp != NULL){
      pre_data = data + cp->pre_offset;
      msg.data = pre_data * cp->scale + cp->offset;
      cp->pub.publish(msg);

    } else {
      std::cout << "unable to find recv-conf: "<< id.str() <<"\n";
    }



  };


  void SocketCANToTopic::stateCallback(const can::State & s)
  {
    std::string err;
    driver_->translateError(s.internal_error, err);
    if (!s.internal_error)
    {
      ROS_INFO("State: %s, asio: %s", err.c_str(), s.error_code.message().c_str());
    }
    else
    {
      ROS_ERROR("Error: %s, asio: %s", err.c_str(), s.error_code.message().c_str());
    }
  };
};
