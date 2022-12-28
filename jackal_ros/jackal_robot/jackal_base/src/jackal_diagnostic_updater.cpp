/**
 *
 *  \file
 *  \brief      Diagnostic updating class for Jackal
 *  \author     Mike Purvis <mpurvis@clearpathrobotics.com>
 *  \copyright  Copyright (c) 2014, Clearpath Robotics, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Clearpath Robotics, Inc. nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL CLEARPATH ROBOTICS, INC. BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Please send comments, questions, or patches to code@clearpathrobotics.com
 *
 */

#include <string>
#include <sys/types.h>

#include <boost/algorithm/string/predicate.hpp>
#include <diagnostic_updater/update_functions.h>

#if defined(_WIN32)
#include <cmath>
#include <memory>

#include <winsock2.h>
#include <iphlpapi.h>

// link with iphlpapi.lib
#pragma comment(lib, "iphlpapi.lib")
#else
#include <ifaddrs.h>
#endif

#include "jackal_base/jackal_diagnostic_updater.h"

#if defined(_WIN32) && defined(ERROR)
#undef ERROR
#endif

namespace jackal_base
{

JackalDiagnosticUpdater::JackalDiagnosticUpdater()
{
  setHardwareID("unknown");

  add("General", this, &JackalDiagnosticUpdater::generalDiagnostics);
  add("Battery", this, &JackalDiagnosticUpdater::batteryDiagnostics);
  add("User voltage supplies", this, &JackalDiagnosticUpdater::voltageDiagnostics);
  add("Current consumption", this, &JackalDiagnosticUpdater::currentDiagnostics);
  add("Power consumption", this, &JackalDiagnosticUpdater::powerDiagnostics);

  // The arrival of this message runs the update() method and triggers the above callbacks.
  status_sub_ = nh_.subscribe("status", 5, &JackalDiagnosticUpdater::statusCallback, this);

  // These message frequencies are reported on separately.
  ros::param::param("~expected_imu_frequency", expected_imu_frequency_, 50.0);
  imu_diagnostic_ = new diagnostic_updater::TopicDiagnostic("/imu/data_raw", *this,
      diagnostic_updater::FrequencyStatusParam(&expected_imu_frequency_, &expected_imu_frequency_, 0.15),
      diagnostic_updater::TimeStampStatusParam(-1, 1.0));
  imu_sub_ = nh_.subscribe("/imu/data_raw", 5, &JackalDiagnosticUpdater::imuCallback, this);

  ros::param::param("~expected_navsat_frequency", expected_navsat_frequency_, 10.0);
  ros::param::param<std::string>("~navsat_frequency_sentence", navsat_frequency_sentence_, "$GPRMC");
  navsat_diagnostic_ = new diagnostic_updater::TopicDiagnostic("/navsat/nmea_sentence", *this,
      diagnostic_updater::FrequencyStatusParam(&expected_navsat_frequency_, &expected_navsat_frequency_, 0.1),
      diagnostic_updater::TimeStampStatusParam(-1, 1.0));
  navsat_sub_ = nh_.subscribe("/navsat/nmea_sentence", 5, &JackalDiagnosticUpdater::navsatCallback, this);

  // Publish whether the wireless interface has an IP address every second.
  ros::param::param<std::string>("~wireless_interface", wireless_interface_, "wlp2s0");
  ROS_INFO_STREAM("Checking for wireless connectivity on interface: " << wireless_interface_);
  wifi_connected_pub_ = nh_.advertise<std_msgs::Bool>("wifi_connected", 1);
  wireless_monitor_timer_ = nh_.createTimer(ros::Duration(1.0),
      &JackalDiagnosticUpdater::wirelessMonitorCallback, this);
}

void JackalDiagnosticUpdater::generalDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  stat.add("MCU uptime (s)", last_status_->mcu_uptime.toSec());
  stat.addf("Motor drivers energized", "%s", last_status_->drivers_active ? "true" : "false");

  if (last_status_->driver_external_stop_present)
  {
    if (last_status_->driver_external_stop_stopped)
    {
      stat.add("External stop", "Present, asserting stop");
    }
    else
    {
      stat.add("External stop", "Present, not asserting stop");
    }
  }
  else
  {
    stat.add("External stop", "Absent");
  }

  if (last_status_->driver_external_stop_stopped)
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "External stop device asserting motor stop.");
  }
  else if (!last_status_->drivers_active)
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Motor drivers not energized.");
  }
  else
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "System OK.");
  }
}

void JackalDiagnosticUpdater::batteryDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  stat.add("Battery Voltage (V)", last_status_->measured_battery);

  if (last_status_->measured_battery > 30.0)
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Battery overvoltage.");
  }
  else if (last_status_->measured_battery < 1.0)
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Battery voltage not detected, check BATT fuse.");
  }
  else if (last_status_->measured_battery < 20.0)
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Battery critically under voltage.");
  }
  else if (last_status_->measured_battery < 24.0)
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Battery low voltage.");
  }
  else
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Battery OK.");
  }
}

void JackalDiagnosticUpdater::voltageDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  stat.add("12V Supply (V)", last_status_->measured_12v);
  stat.add("5V Supply (V)", last_status_->measured_5v);

  if (last_status_->measured_12v > 12.5 || last_status_->measured_5v > 5.5)
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR,
        "User supply overvoltage. Accessories may be damaged.");
  }
  else if (last_status_->measured_12v < 1.0 || last_status_->measured_5v < 1.0)
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "User supplies absent. Check tray fuses.");
  }
  else if (last_status_->measured_12v < 11.0 || last_status_->measured_5v < 4.0)
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Voltage supplies undervoltage. Check loading levels.");
  }
  else
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "User supplies OK.");
  }
}

void JackalDiagnosticUpdater::currentDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  stat.add("Drive current (A)", last_status_->drive_current);
  stat.add("User current (A)", last_status_->user_current);
  stat.add("Computer current (A)", last_status_->computer_current);
  stat.add("Total current (A)", last_status_->total_current);

  if (last_status_->total_current > 32.0)
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Current draw critical.");
  }
  else if (last_status_->total_current > 20.0)
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Current draw warning.");
  }
  else if (last_status_->total_current > 10.0)
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Current draw requires monitoring.");
  }
  else
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Current draw nominal.");
  }
}

void JackalDiagnosticUpdater::powerDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  stat.add("Total power consumption (Wh)", last_status_->total_power_consumed);

  if (last_status_->total_power_consumed > 260.0)
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Power consumed exceeds capacity of standard battery.");
  }
  else if (last_status_->total_power_consumed > 220.0)
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Power consumed approaches capacity of standard battery.");
  }
  else
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Battery OK.");
  }
}

void JackalDiagnosticUpdater::statusCallback(const jackal_msgs::Status::ConstPtr& status)
{
  // Fresh data from the MCU, publish a diagnostic update.
  last_status_ = status;
  setHardwareID(last_status_->hardware_id);
  update();
}

void JackalDiagnosticUpdater::imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
  imu_diagnostic_->tick(msg->header.stamp);
}

void JackalDiagnosticUpdater::navsatCallback(const nmea_msgs::Sentence::ConstPtr& msg)
{
  if (boost::starts_with(msg->sentence, navsat_frequency_sentence_))
  {
    navsat_diagnostic_->tick(msg->header.stamp);
  }
}

void JackalDiagnosticUpdater::wirelessMonitorCallback(const ros::TimerEvent& te)
{
  std_msgs::Bool wifi_connected_msg;
  wifi_connected_msg.data = false;

#if defined(_WIN32)
  // Get system structure of interface IP addresses.
  ULONG flags = GAA_FLAG_INCLUDE_PREFIX;

  // check for both IPv4 and IPv6
  ULONG family = AF_UNSPEC;

  // pre-allocate 15 KB of buffer is sufficient for most computers
  int length = static_cast<int>(std::ceil(15e3 / sizeof(IP_ADAPTER_ADDRESSES)));
  auto addresses = std::make_unique<IP_ADAPTER_ADDRESSES[]>(length);
  while (true)
  {
    ULONG bufferLength = sizeof(IP_ADAPTER_ADDRESSES) * length;
    auto result = ::GetAdaptersAddresses(family, flags, nullptr, addresses.get(), &bufferLength);
    if (result == ERROR_BUFFER_OVERFLOW)
    {
      length = static_cast<int>(std::ceil(bufferLength / sizeof(IP_ADAPTER_ADDRESSES)));
      addresses = std::make_unique<IP_ADAPTER_ADDRESSES[]>(length);
      continue;
    }
    else if (result == ERROR_SUCCESS)
    {
      break;
    }
    else
    {
      // GetAdaptersAddresses failed
      wifi_connected_pub_.publish(wifi_connected_msg);
      return;
    }
  }

  // Iterate structure looking for the wireless interface.
  auto address = addresses.get();
  while (address)
  {
    // https://docs.microsoft.com/en-us/windows/win32/api/iptypes/ns-iptypes-_ip_adapter_addresses_lh
    // use IF_TYPE_IEEE80211 for 802.11 wireless network
    // win32 code path does not consume the wireless_interface_ variable
    if (address->IfType == IF_TYPE_IEEE80211 && address->OperStatus == IF_OPER_STATUS::IfOperStatusUp)
    {
      wifi_connected_msg.data = true;
      break;
    }

    address = address->Next;
  }
#else
  // Get system structure of interface IP addresses.
  struct ifaddrs* ifa_head;
  if (getifaddrs(&ifa_head) != 0)
  {
    ROS_WARN("System call getifaddrs returned error code. Unable to detect network interfaces.");
    return;
  }

  // Iterate structure looking for the wireless interface.
  struct ifaddrs* ifa_current = ifa_head;
  while (ifa_current != NULL)
  {
    if (strcmp(ifa_current->ifa_name, wireless_interface_.c_str()) == 0)
    {
      int family = ifa_current->ifa_addr->sa_family;
      if (family == AF_INET || family == AF_INET6)
      {
        wifi_connected_msg.data = true;
        break;
      }
    }

    ifa_current = ifa_current->ifa_next;
  }

  // Free structure, publish result message.
  freeifaddrs(ifa_head);
#endif

  wifi_connected_pub_.publish(wifi_connected_msg);
}

}  // namespace jackal_base
