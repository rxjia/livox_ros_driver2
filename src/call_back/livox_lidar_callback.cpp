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

#include "livox_lidar_callback.h"

#include <arpa/inet.h>

#include <iostream>
#include <string>
#include <thread>

#include "parse_internal_info.h"

#define LIVOX_INFO_STREAM(ip_prefix, ...) std::cout << (ip_prefix) << __VA_ARGS__ << std::endl;

#define LIVOX_WARN_STREAM(ip_prefix, ...) std::cout << (ip_prefix) << __VA_ARGS__ << std::endl;

#define LIVOX_CB_LOG_FAILED()                                       \
  LIVOX_WARN_STREAM(                                                \
    ip_prefix, func_name << " failed"                               \
                         << ", return code: " << response->ret_code \
                         << ", error key: " << response->error_key);

namespace livox_ros
{

void LivoxLidarCallback::LidarInfoChangeCallback(
  const uint32_t handle, const LivoxLidarInfo * info, void * client_data)
{
  std::string ip_prefix = IpNumToStringPrefix(handle);

  if (client_data == nullptr) {
    LIVOX_WARN_STREAM(ip_prefix, "lidar info change callback failed, client data is nullptr");
    return;
  }
  LdsLidar * lds_lidar = static_cast<LdsLidar *>(client_data);

  LidarDevice * lidar_device = GetLidarDevice(handle, client_data);

  if (lidar_device == nullptr) {
    LIVOX_WARN_STREAM(ip_prefix, "found lidar not defined in the user-defined config");
    // add lidar device
    uint8_t index = 0;
    int8_t ret = lds_lidar->cache_index_.GetFreeIndex(kLivoxLidarType, handle, index);
    if (ret != 0) {
      LIVOX_WARN_STREAM(ip_prefix, "failed to add lidar device");
      return;
    }
    LidarDevice * p_lidar = &(lds_lidar->lidars_[index]);
    p_lidar->lidar_type = kLivoxLidarType;
  } else {
    // set the lidar according to the user-defined config
    const UserLivoxLidarConfig & config = lidar_device->livox_config;

    // lock for modify the lidar device set_bits
    {
      std::lock_guard<std::mutex> lock(lds_lidar->config_mutex_);
      if (config.pcl_data_type != -1) {
        lidar_device->livox_config.set_bits |= kConfigDataType;
        SetLivoxLidarPclDataType(
          handle, static_cast<LivoxLidarPointDataType>(config.pcl_data_type),
          LivoxLidarCallback::SetDataTypeCallback, lds_lidar);
        LIVOX_INFO_STREAM(
          ip_prefix, "set pcl data type: " << static_cast<int32_t>(config.pcl_data_type));
      }
      if (config.pattern_mode != -1) {
        lidar_device->livox_config.set_bits |= kConfigScanPattern;
        SetLivoxLidarScanPattern(
          handle, static_cast<LivoxLidarScanPattern>(config.pattern_mode),
          LivoxLidarCallback::SetPatternModeCallback, lds_lidar);
        LIVOX_INFO_STREAM(
          ip_prefix, "set scan pattern: " << static_cast<int32_t>(config.pattern_mode));
      }
      if (config.blind_spot_set != -1) {
        lidar_device->livox_config.set_bits |= kConfigBlindSpot;
        SetLivoxLidarBlindSpot(
          handle, config.blind_spot_set, LivoxLidarCallback::SetBlindSpotCallback, lds_lidar);

        LIVOX_INFO_STREAM(ip_prefix, "set blind spot distance: " << config.blind_spot_set);
      }
      if (config.dual_emit_en != -1) {
        lidar_device->livox_config.set_bits |= kConfigDualEmit;
        SetLivoxLidarDualEmit(
          handle, (config.dual_emit_en == 0 ? false : true),
          LivoxLidarCallback::SetDualEmitCallback, lds_lidar);
        LIVOX_INFO_STREAM(
          ip_prefix, "set dual emit mode: " << static_cast<int32_t>(config.dual_emit_en));
      }
      if ((config.fov_cfg_en != 255) && (config.fov_cfg_en & (0x01 << 0))) {
        FovCfg fov_cfg0 = config.fov_cfg0;
        SetLivoxLidarFovCfg0(
          config.handle, &fov_cfg0, LivoxLidarCallback::SetFovCfg0Callback, lds_lidar);
        LIVOX_INFO_STREAM(
          ip_prefix, "set fov_cfg0"
                       << ", yaw_start: " << fov_cfg0.yaw_start << ", yaw_stop: "
                       << fov_cfg0.yaw_stop << ", pitch_start: " << fov_cfg0.pitch_start
                       << ", pitch_stop: " << fov_cfg0.pitch_stop);
      }
      if ((config.fov_cfg_en != 255) && (config.fov_cfg_en & (0x01 << 1))) {
        FovCfg fov_cfg1 = config.fov_cfg1;
        SetLivoxLidarFovCfg1(
          config.handle, &fov_cfg1, LivoxLidarCallback::SetFovCfg1Callback, lds_lidar);
        LIVOX_INFO_STREAM(
          ip_prefix, "set fov_cfg1"
                       << ", yaw_start: " << fov_cfg1.yaw_start << ", yaw_stop: "
                       << fov_cfg1.yaw_stop << ", pitch_start: " << fov_cfg1.pitch_start
                       << ", pitch_stop: " << fov_cfg1.pitch_stop);
      }
      if ((config.fov_cfg_en != 255) && (config.fov_cfg_en != 255)) {
        EnableLivoxLidarFov(
          config.handle, config.fov_cfg_en, LivoxLidarCallback::EnableLivoxLidarFovCallback,
          lds_lidar);
        // EnableLivoxLidarFov(config.handle, config.fov_cfg_en,
        LIVOX_INFO_STREAM(ip_prefix, "set fov_cfg_en: " << int(config.fov_cfg_en));
      }

      // QueryLivoxLidarInternalInfo(handle,
      // LivoxLidarCallback::QueryInternalInfoCallback, lds_lidar);

    }  // free lock for set_bits

    // set extrinsic params into lidar
    LivoxLidarInstallAttitude attitude{config.extrinsic_param.roll, config.extrinsic_param.pitch,
                                       config.extrinsic_param.yaw,  config.extrinsic_param.x,
                                       config.extrinsic_param.y,    config.extrinsic_param.z};
    SetLivoxLidarInstallAttitude(
      config.handle, &attitude, LivoxLidarCallback::SetAttitudeCallback, lds_lidar);
  }
  LIVOX_INFO_STREAM(ip_prefix, "begin to change work mode to 'Normal'");
  SetLivoxLidarWorkMode(handle, kLivoxLidarNormal, WorkModeChangedCallback, nullptr);
  EnableLivoxLidarImuData(handle, LivoxLidarCallback::EnableLivoxLidarImuDataCallback, lds_lidar);
  return;
}

LivoxLidarCallback::LIVOX_CB LivoxLidarCallback::defaultCallBack(std::string func_name)
{
  auto cb = [func_name](
              livox_status status, uint32_t handle, LivoxLidarAsyncControlResponse * response,
              void * client_data) {
    std::string ip_prefix = IpNumToStringPrefix(handle);

    if (response == nullptr) {
      LIVOX_WARN_STREAM(ip_prefix, func_name << " failed to get response");
      return;
    }

    if (status == kLivoxLidarStatusSuccess) {
      LIVOX_INFO_STREAM(ip_prefix, func_name << " success.");
    } else if (status == kLivoxLidarStatusTimeout) {
      LIVOX_INFO_STREAM(ip_prefix, func_name << " timeout!");
    } else {
      LIVOX_WARN_STREAM(
        ip_prefix, func_name << " failed"
                             << ", return code: " << response->ret_code
                             << ", error key: " << response->error_key);
    }
  };

  return cb;
}

void LivoxLidarCallback::SetFovCfg0Callback(
  livox_status status, uint32_t handle, LivoxLidarAsyncControlResponse * response,
  void * client_data)
{
  return defaultCallBack("SetFovCfg0")(status, handle, response, client_data);
}

void LivoxLidarCallback::SetFovCfg1Callback(
  livox_status status, uint32_t handle, LivoxLidarAsyncControlResponse * response,
  void * client_data)
{
  return defaultCallBack("SetFovCfg1")(status, handle, response, client_data);
}

void LivoxLidarCallback::EnableLivoxLidarFovCallback(
  livox_status status, uint32_t handle, LivoxLidarAsyncControlResponse * response,
  void * client_data)
{
  return defaultCallBack("EnableLivoxLidarFov")(status, handle, response, client_data);
}

void LivoxLidarCallback::WorkModeChangedCallback(
  livox_status status, uint32_t handle, LivoxLidarAsyncControlResponse * response,
  void * client_data)
{
  std::string func_name = "change work mode";
  std::string ip_prefix = IpNumToStringPrefix(handle);
  if (status != kLivoxLidarStatusSuccess) {
    LIVOX_WARN_STREAM(ip_prefix, func_name << "failed, try again...");
    std::this_thread::sleep_for(std::chrono::seconds(1));
    SetLivoxLidarWorkMode(handle, kLivoxLidarNormal, WorkModeChangedCallback, nullptr);
    return;
  }
  LIVOX_INFO_STREAM(ip_prefix, func_name << " success.");
  return;
}

void LivoxLidarCallback::SetDataTypeCallback(
  livox_status status, uint32_t handle, LivoxLidarAsyncControlResponse * response,
  void * client_data)
{
  std::string func_name = "SetDataType";
  std::string ip_prefix = IpNumToStringPrefix(handle);
  LidarDevice * lidar_device = GetLidarDevice(handle, client_data);
  if (lidar_device == nullptr) {
    LIVOX_WARN_STREAM(ip_prefix, "failed to set data type since no lidar device found");
    return;
  }
  LdsLidar * lds_lidar = static_cast<LdsLidar *>(client_data);

  if (status == kLivoxLidarStatusSuccess) {
    std::lock_guard<std::mutex> lock(lds_lidar->config_mutex_);
    lidar_device->livox_config.set_bits &= ~((uint32_t)(kConfigDataType));
    if (!lidar_device->livox_config.set_bits) {
      lidar_device->connect_state = kConnectStateSampling;
    }
    LIVOX_INFO_STREAM(
      ip_prefix, "successfully set data type"
                   << ", set_bit: " << lidar_device->livox_config.set_bits);
  } else if (status == kLivoxLidarStatusTimeout) {
    const UserLivoxLidarConfig & config = lidar_device->livox_config;
    SetLivoxLidarPclDataType(
      handle, static_cast<LivoxLidarPointDataType>(config.pcl_data_type),
      LivoxLidarCallback::SetDataTypeCallback, client_data);
    LIVOX_WARN_STREAM(ip_prefix, "set data type timeout, try again...");
  } else {
    LIVOX_CB_LOG_FAILED();
  }
  return;
}

void LivoxLidarCallback::SetPatternModeCallback(
  livox_status status, uint32_t handle, LivoxLidarAsyncControlResponse * response,
  void * client_data)
{
  std::string func_name = "SetPatternMode";
  std::string ip_prefix = IpNumToStringPrefix(handle);
  LidarDevice * lidar_device = GetLidarDevice(handle, client_data);
  if (lidar_device == nullptr) {
    LIVOX_WARN_STREAM(ip_prefix, "failed to set pattern mode since no lidar device found");
    return;
  }
  LdsLidar * lds_lidar = static_cast<LdsLidar *>(client_data);

  if (status == kLivoxLidarStatusSuccess) {
    std::lock_guard<std::mutex> lock(lds_lidar->config_mutex_);
    lidar_device->livox_config.set_bits &= ~((uint32_t)(kConfigScanPattern));
    if (!lidar_device->livox_config.set_bits) {
      lidar_device->connect_state = kConnectStateSampling;
    }
    LIVOX_INFO_STREAM(
      ip_prefix, "successfully set pattern mode"
                   << ", set_bit: " << lidar_device->livox_config.set_bits);
  } else if (status == kLivoxLidarStatusTimeout) {
    const UserLivoxLidarConfig & config = lidar_device->livox_config;
    SetLivoxLidarScanPattern(
      handle, static_cast<LivoxLidarScanPattern>(config.pattern_mode),
      LivoxLidarCallback::SetPatternModeCallback, client_data);
    LIVOX_WARN_STREAM(ip_prefix, "set pattern mode timeout, try again...");
  } else {
    LIVOX_CB_LOG_FAILED();
  }
  return;
}

void LivoxLidarCallback::SetBlindSpotCallback(
  livox_status status, uint32_t handle, LivoxLidarAsyncControlResponse * response,
  void * client_data)
{
  std::string func_name = "SetBlindSpot";
  std::string ip_prefix = IpNumToStringPrefix(handle);
  LidarDevice * lidar_device = GetLidarDevice(handle, client_data);
  if (lidar_device == nullptr) {
    LIVOX_WARN_STREAM(ip_prefix, "failed to set blind spot since no lidar device found");
    return;
  }
  LdsLidar * lds_lidar = static_cast<LdsLidar *>(client_data);

  if (status == kLivoxLidarStatusSuccess) {
    std::lock_guard<std::mutex> lock(lds_lidar->config_mutex_);
    lidar_device->livox_config.set_bits &= ~((uint32_t)(kConfigBlindSpot));
    if (!lidar_device->livox_config.set_bits) {
      lidar_device->connect_state = kConnectStateSampling;
    }
    LIVOX_INFO_STREAM(
      ip_prefix, "successfully set blind spot"
                   << ", set_bit: " << lidar_device->livox_config.set_bits);
  } else if (status == kLivoxLidarStatusTimeout) {
    LIVOX_WARN_STREAM(ip_prefix, "set blind spot timeout, try again...");

    const UserLivoxLidarConfig & config = lidar_device->livox_config;
    SetLivoxLidarBlindSpot(
      handle, config.blind_spot_set, LivoxLidarCallback::SetBlindSpotCallback, client_data);
  } else {
    LIVOX_CB_LOG_FAILED();
  }
  return;
}

void LivoxLidarCallback::SetDualEmitCallback(
  livox_status status, uint32_t handle, LivoxLidarAsyncControlResponse * response,
  void * client_data)
{
  std::string func_name = "SetDualEmit";
  std::string ip_prefix = IpNumToStringPrefix(handle);
  LidarDevice * lidar_device = GetLidarDevice(handle, client_data);
  if (lidar_device == nullptr) {
    LIVOX_WARN_STREAM(ip_prefix, "failed to set dual emit mode since no lidar device found");
    return;
  }

  LdsLidar * lds_lidar = static_cast<LdsLidar *>(client_data);
  if (status == kLivoxLidarStatusSuccess) {
    std::lock_guard<std::mutex> lock(lds_lidar->config_mutex_);
    lidar_device->livox_config.set_bits &= ~((uint32_t)(kConfigDualEmit));
    if (!lidar_device->livox_config.set_bits) {
      lidar_device->connect_state = kConnectStateSampling;
    }
    LIVOX_INFO_STREAM(
      ip_prefix, "successfully set dual emit mode"
                   << ", set_bit: " << lidar_device->livox_config.set_bits);
  } else if (status == kLivoxLidarStatusTimeout) {
    const UserLivoxLidarConfig & config = lidar_device->livox_config;
    SetLivoxLidarDualEmit(
      handle, config.dual_emit_en, LivoxLidarCallback::SetDualEmitCallback, client_data);
    LIVOX_WARN_STREAM(ip_prefix, "set dual emit mode timeout, try again...");
  } else {
    LIVOX_CB_LOG_FAILED();
  }
  return;
}

void LivoxLidarCallback::SetAttitudeCallback(
  livox_status status, uint32_t handle, LivoxLidarAsyncControlResponse * response,
  void * client_data)
{
  std::string func_name = "SetAttitude";
  std::string ip_prefix = IpNumToStringPrefix(handle);
  LidarDevice * lidar_device = GetLidarDevice(handle, client_data);
  if (lidar_device == nullptr) {
    LIVOX_WARN_STREAM(ip_prefix, func_name << " failed since no lidar device found");
    return;
  }
  LdsLidar * lds_lidar = static_cast<LdsLidar *>(client_data);
  if (status == kLivoxLidarStatusSuccess) {
    LIVOX_INFO_STREAM(ip_prefix, func_name << "success.");
  } else if (status == kLivoxLidarStatusTimeout) {
    LIVOX_WARN_STREAM(ip_prefix, func_name << " timeout, try again...");
    const UserLivoxLidarConfig & config = lidar_device->livox_config;
    LivoxLidarInstallAttitude attitude{config.extrinsic_param.roll, config.extrinsic_param.pitch,
                                       config.extrinsic_param.yaw,  config.extrinsic_param.x,
                                       config.extrinsic_param.y,    config.extrinsic_param.z};
    SetLivoxLidarInstallAttitude(
      config.handle, &attitude, LivoxLidarCallback::SetAttitudeCallback, lds_lidar);
  } else {
    LIVOX_CB_LOG_FAILED();
  }
}

void LivoxLidarCallback::EnableLivoxLidarImuDataCallback(
  livox_status status, uint32_t handle, LivoxLidarAsyncControlResponse * response,
  void * client_data)
{
  std::string func_name = "EnableLivoxLidarImu";
  std::string ip_prefix = IpNumToStringPrefix(handle);
  LidarDevice * lidar_device = GetLidarDevice(handle, client_data);

  if (lidar_device == nullptr) {
    LIVOX_WARN_STREAM(ip_prefix, func_name << " failed since no lidar device found.");
    return;
  }
  LdsLidar * lds_lidar = static_cast<LdsLidar *>(client_data);

  if (response == nullptr) {
    LIVOX_WARN_STREAM(
      ip_prefix, func_name << " failed to get response since no lidar IMU sensor found.");
    return;
  }

  if (status == kLivoxLidarStatusSuccess) {
    LIVOX_INFO_STREAM(ip_prefix, func_name << " success.");
  } else if (status == kLivoxLidarStatusTimeout) {
    LIVOX_INFO_STREAM(ip_prefix, func_name << " timeout, try again...");
    EnableLivoxLidarImuData(handle, LivoxLidarCallback::EnableLivoxLidarImuDataCallback, lds_lidar);
  } else {
    LIVOX_CB_LOG_FAILED();
  }
}

LidarDevice * LivoxLidarCallback::GetLidarDevice(const uint32_t handle, void * client_data)
{
  std::string ip_prefix = IpNumToStringPrefix(handle);

  if (client_data == nullptr) {
    LIVOX_WARN_STREAM(ip_prefix, "failed to get lidar device, client data is nullptr");
    return nullptr;
  }

  LdsLidar * lds_lidar = static_cast<LdsLidar *>(client_data);
  uint8_t index = 0;
  int8_t ret = lds_lidar->cache_index_.GetIndex(kLivoxLidarType, handle, index);
  if (ret != 0) {
    return nullptr;
  }

  return &(lds_lidar->lidars_[index]);
}

void LivoxLidarCallback::LivoxLidarPushMsgCallback(
  const uint32_t handle, const uint8_t dev_type, const char * info, void * client_data)
{
  std::string ip_prefix = IpNumToStringPrefix(handle);
  LIVOX_INFO_STREAM(ip_prefix, "push msg info: " << std::endl << info)
  return;
}

void LivoxLidarCallback::QueryInternalInfoCallback(
  livox_status status, uint32_t handle, LivoxLidarDiagInternalInfoResponse * response,
  void * client_data)
{
  std::string func_name = "QueryInternalInfo";
  std::string ip_prefix = IpNumToStringPrefix(handle);

  if (status != kLivoxLidarStatusSuccess) {
    LIVOX_WARN_STREAM(ip_prefix, "Query lidar internal info failed.");
    QueryLivoxLidarInternalInfo(handle, QueryInternalInfoCallback, nullptr);
    return;
  }

  if (response == nullptr) {
    return;
  }

  std::string info_str;
  livox_parser::ParseLidarStateInfo::Parse(*response, info_str);
  LIVOX_INFO_STREAM(ip_prefix, "QueryInternalInfo: " << info_str);
}

}  // namespace livox_ros
