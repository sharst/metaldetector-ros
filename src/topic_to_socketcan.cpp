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

#include <metaldetector_ros/topic_to_socketcan.h>
#include <socketcan_interface/string.h>


namespace metaldetector_ros
{
  TopicToSocketCAN::TopicToSocketCAN(ros::NodeHandle* nh, ros::NodeHandle* nh_param,
      boost::shared_ptr<can::DriverInterface> driver)
  {
    driver_ = driver;
    // Register callback for topics to be sent
    send_config = YAML::LoadFile(ros::package::getPath("metaldetector_ros") + "/config/send_config.yaml");
    if (send_config.IsNull()) {
      std::cout << "No file send_config.yaml found.";
    } else {
      for (YAML::const_iterator it = send_config.begin();
          it != send_config.end(); ++it) {
        YAML::Node reg_conf = it->second;

        for (YAML::const_iterator reg_it = reg_conf.begin();
            reg_it != reg_conf.end(); ++reg_it) {
          YAML::Node reg_config = reg_it->second;
          std::cout << "Found send command set for: "
            << reg_config["topic"].as<std::string>() << std::endl;
          std::ostringstream id;
          id << it->first.as<std::string>() << "." << reg_it->first.as<std::string>();

          int reg = reg_it->first.as<int>();
          int dev = it->first.as<int>();
          ros::Subscriber sub =
            nh->subscribe<StampedFloat>(reg_config["topic"].as<std::string>(), 10, boost::bind(&TopicToSocketCAN::canSendCallback, this, _1, dev, reg));

          CanSubscriber* cs = new CanSubscriber();
          cs->sub = sub;

          if (reg_config["pre-offset"]) {
            cs->pre_offset = reg_config["pre-offset"].as<int32_t>();
          }

          if (reg_config["scale"]) {
            cs->scale = reg_config["scale"].as<float>();
          }

          if (reg_config["offset"]) {
            cs->offset = reg_config["offset"].as<float>();
          }

          sub_map_[id.str()] = cs;
        }
      }
    }
  };

  void TopicToSocketCAN::setup()
  {
    state_listener_ = driver_->createStateListener(
        can::StateInterface::StateDelegate(this, &TopicToSocketCAN::stateCallback));
  };

  void TopicToSocketCAN::canSendCallback(
      const StampedFloat::ConstPtr& message, int device,
      int reg) {
    int data = message->data;
    int32_t pre_data;

    std::ostringstream id;
    id << device << "." << reg;

    CanSubscriber* cs = sub_map_[id.str()];

    if (cs != NULL){
      pre_data = data - cs->offset;
      data = pre_data / cs->scale - cs->pre_offset;

      can::Frame f;  // socketcan type
      f.id = device;
      f.dlc = 5;
      f.is_error = 0;
      f.is_rtr = 0;
      f.is_extended = 1;
      f.data[0] = (1 << 7) | reg;  // Write (1) to register
      f.data[1] = (data >> 24) & 0xFF;
      f.data[2] = (data >> 16) & 0xFF;
      f.data[3] = (data >> 8) & 0xFF;
      f.data[4] = data & 0xFF;
      f.data[5] = 0;
      f.data[6] = 0;
      f.data[7] = 0;

      if (!f.isValid())  // check if the id and flags are appropriate.
      {
        // ROS_WARN("Refusing to send invalid frame: %s.", can::tostring(f, true).c_str());
        // can::tostring cannot be used for dlc > 8 frames. It causes an crash
        // due to usage of boost::array for the data array. The should always work.
        ROS_ERROR("Invalid frame from topic: id: %#04x, length: %d, is_extended: %d", f.id, f.dlc, f.is_extended);
        return;
      }

      bool res = driver_->send(f);
      if (!res)
      {
        ROS_ERROR("Failed to send message: %s.", can::tostring(f, true).c_str());
      } 

    }else{
      std::cout << "unable to find send-conf: "<< id.str() << "\n";
    }
  }

  void TopicToSocketCAN::stateCallback(const can::State & s)
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
