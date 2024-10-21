//
// The MIT License (MIT)
//
// Copyright (c) 2022 Livox. All rights reserved.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

#ifndef LIVOX_ROS_DRIVER_LIVOX_LIDAR_CALLBACK_H_
#define LIVOX_ROS_DRIVER_LIVOX_LIDAR_CALLBACK_H_

#include <functional>
#include "../lds.h"
#include "../lds_lidar.h"
#include "../comm/comm.h"

#include "livox_lidar_api.h"
#include "livox_lidar_def.h"

namespace livox_ros {

class LivoxLidarCallback {
 public:
  static void LidarInfoChangeCallback(const uint32_t handle,
                                      const LivoxLidarInfo* info,
                                      void* client_data);
  static void WorkModeChangedCallback(livox_status status,
                                      uint32_t handle,
                                      LivoxLidarAsyncControlResponse *response,
                                      void *client_data);
  static void SetDataTypeCallback(livox_status status, uint32_t handle,
                                  LivoxLidarAsyncControlResponse *response,
                                  void *client_data);
  static void SetPatternModeCallback(livox_status status, uint32_t handle,
                                     LivoxLidarAsyncControlResponse *response,
                                     void *client_data);
  static void SetBlindSpotCallback(livox_status status, uint32_t handle,
                                   LivoxLidarAsyncControlResponse *response,
                                   void *client_data);
  static void SetDualEmitCallback(livox_status status, uint32_t handle,
                                  LivoxLidarAsyncControlResponse *response,
                                  void *client_data);
  static void SetAttitudeCallback(livox_status status, uint32_t handle,
                                  LivoxLidarAsyncControlResponse *response,
                                  void *client_data);
  static void EnableLivoxLidarImuDataCallback(livox_status status, uint32_t handle,
                                  LivoxLidarAsyncControlResponse *response,
                                  void *client_data);
  static void SetFovCfg0Callback(livox_status status, uint32_t handle,
                                  LivoxLidarAsyncControlResponse *response,
                                  void *client_data);
  static void SetFovCfg1Callback(livox_status status, uint32_t handle,
                                  LivoxLidarAsyncControlResponse *response,
                                  void *client_data);
  static void EnableLivoxLidarFovCallback(livox_status status, uint32_t handle,
                                  LivoxLidarAsyncControlResponse *response,
                                  void *client_data);

  static void LivoxLidarPushMsgCallback(const uint32_t handle, const uint8_t dev_type, const char* info, void* client_data);

  static void QueryInternalInfoCallback(livox_status status, uint32_t handle,
                                  LivoxLidarDiagInternalInfoResponse *response,
                                  void *client_data);
  using LIVOX_CB = std::function<void(livox_status status,
                                                 uint32_t handle,
                                                 LivoxLidarAsyncControlResponse *response,
                                                 void *client_data)>;
  static LIVOX_CB defaultCallBack(std::string func_name);
 private:
  static LidarDevice* GetLidarDevice(const uint32_t handle, void* client_data);
};

} // namespace livox_ros

#endif  // LIVOX_ROS_DRIVER_LIVOX_LIDAR_CALLBACK_H_
