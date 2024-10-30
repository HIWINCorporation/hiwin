/*
 * Copyright 2024, HIWIN Technologies Corp. (trimming)
 *
 * Copyright 2019, FZI Forschungszentrum Informatik (refactor)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <csignal>

#include <ros/ros.h>
#include <controller_manager/controller_manager.h>

#include <hiwin_driver/hardware_interface.h>

std::unique_ptr<hiwin_driver::HardwareInterface> g_hw_interface;

void signalHandler(int signum)
{
  std::cout << "Interrupt signal (" << signum << ") received.\n";

  g_hw_interface.reset();

  exit(signum);
}

int main(int argc, char** argv)
{
  // Set up ROS.
  ros::init(argc, argv, "hiwin_hardware_interface");
  ros::AsyncSpinner spinner(2);
  spinner.start();

  ros::NodeHandle nh;
  ros::NodeHandle nh_priv("~");

  // register signal SIGINT and signal handler
  signal(SIGINT, signalHandler);

  // Set up timers
  ros::Time timestamp;
  ros::Duration period;
  auto stopwatch_last = std::chrono::steady_clock::now();
  auto stopwatch_now = stopwatch_last;

  g_hw_interface.reset(new hiwin_driver::HardwareInterface);
  if (!g_hw_interface->init(nh, nh_priv))
  {
    ROS_ERROR_STREAM("Could not correctly initialize robot. Exiting");
    exit(1);
  }
  ROS_DEBUG_STREAM("initialized hw interface");
  controller_manager::ControllerManager cm(g_hw_interface.get(), nh);

  // Get current time and elapsed time since last read
  timestamp = ros::Time::now();
  stopwatch_now = std::chrono::steady_clock::now();
  period.fromSec(std::chrono::duration_cast<std::chrono::duration<double>>(stopwatch_now - stopwatch_last).count());
  stopwatch_last = stopwatch_now;

  // Run as fast as possible
  while (ros::ok())
  {
    // Receive current state from robot
    g_hw_interface->read(timestamp, period);

    // Get current time and elapsed time since last read
    timestamp = ros::Time::now();
    stopwatch_now = std::chrono::steady_clock::now();
    period.fromSec(std::chrono::duration_cast<std::chrono::duration<double>>(stopwatch_now - stopwatch_last).count());
    stopwatch_last = stopwatch_now;

    cm.update(timestamp, period, g_hw_interface->shouldResetControllers());

    g_hw_interface->write(timestamp, period);
  }

  spinner.stop();
  ROS_INFO_STREAM_NAMED("hardware_interface", "Shutting down.");
  return 0;
}