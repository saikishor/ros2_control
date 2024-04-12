// Copyright 2024 PAL Robotics S.L.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef CONTROLLER_INTERFACE__ASYNC_FUNCTION_HANDLER_HPP_
#define CONTROLLER_INTERFACE__ASYNC_FUNCTION_HANDLER_HPP_

#include <atomic>
#include <condition_variable>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace ros2_control
{
template <typename T>
class AsyncFunctionHandler
{
public:
  AsyncFunctionHandler() = default;

  ~AsyncFunctionHandler()
  {
    async_update_stop_ = true;
    if (thread_.joinable())
    {
      async_update_condition_.notify_one();
      thread_.join();
    }
  }

  template <typename T>
  void init(
    std::function<const rclcpp_lifecycle::State &()> get_state_function,
    std::function<T(rclcpp::Time, rclcpp::Duration)> async_function)
  {
    if (thread_.joinable())
    {
      throw std::runtime_error("AsyncFunctionHandler already initialized!");
    }
    get_state_function_ = get_state_function;
    async_function_ = async_function;
  }

  template <typename T>
  T trigger_async_update(const rclcpp::Time & time, const rclcpp::Duration & period)
  {
    initialize_async_update_thread(get_state_function_, async_function_);
    std::unique_lock<std::mutex> lock(async_mtx_, std::try_to_lock);
    if (!async_update_ready_ && lock.owns_lock())
    {
      async_update_ready_ = true;
      current_update_time_ = time;
      current_update_period_ = period;
      async_update_condition_.notify_one();
    }
    return async_update_return_;
  }

  void wait_for_update_to_finish() const
  {
    std::unique_lock<std::mutex> lock(async_mtx_);
    async_update_condition_.wait(lock, [this] { return !async_update_ready_; });
  }

  bool is_initialized() const
  {
    return thread_.joinable() && get_state_function_ != nullptr && async_function_ != nullptr;
  }

private:
  template <typename T>
  void initialize_async_update_thread(
    std::function<const rclcpp_lifecycle::State &()> get_state_function,
    std::function<T(rclcpp::Time, rclcpp::Duration)> async_function)
  {
    // * If the thread is not joinable, create a new thread and call the update function and wait
    // for it to be triggered again and repeat the same process.
    // * If the thread is joinable, check if the current update cycle is finished. If so, then
    // trigger a new update cycle.
    // * The controller managr is responsible for triggering the update cycle and maintaining the
    // controller's update rate and should be acting as a scheduler.
    if (get_state_function_ == nullptr || async_function_ == nullptr)
    {
      throw std::runtime_error("AsyncFunctionHandler not initialized properly!");
    }
    if (!thread_.joinable())
    {
      auto update_fn = [this, get_state_function, async_function]() -> void
      {
        async_update_stop_ = false;
        async_update_ready_ = false;
        std::unique_lock<std::mutex> lock(async_mtx_);
        // \note There might be an concurrency issue with the get_state() call here. This mightn't
        // be critical here as the state of the controller is not expected to change during the
        // update cycle
        while (get_state_function().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE &&
               !async_update_stop_)
        {
          async_update_condition_.wait(
            lock, [this] { return async_update_ready_ || async_update_stop_; });
          if (async_update_stop_)
          {
            lock.unlock();
            break;
          }
          async_update_return_ = async_function(current_update_time_, current_update_period_);
          async_update_ready_ = false;
          lock.unlock();
          async_update_condition_.notify_one();
        }
      };
      thread_ = std::thread(update_fn);
    }
  }

  rclcpp::Time current_update_time_;
  rclcpp::Duration current_update_period_{0, 0};

  std::function<const rclcpp_lifecycle::State &()> get_state_function_;
  std::function<T(rclcpp::Time, rclcpp::Duration)> async_function_;

  // Async related variables
  std::thread thread_;
  std::atomic_bool async_update_stop_{false};
  std::atomic_bool async_update_ready_{false};
  std::atomic<T> async_update_return_;
  std::condition_variable async_update_condition_;
  std::mutex async_mtx_;
};
}  // namespace ros2_control

#endif  // CONTROLLER_INTERFACE__ASYNC_FUNCTION_HANDLER_HPP_