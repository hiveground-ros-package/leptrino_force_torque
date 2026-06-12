/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2013, Imai Laboratory, Keio University.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *      * Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *      * Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *      * Neither the name of the Imai Laboratory, nor the name of its
 *      contributors may be used to endorse or promote products derived from
 *      this software without specific prior written permission.
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
 *
 * Author: Mahisorn Wongphati
 * Notice: Modified & copied from Leptrino CD example source code
 * Notice: Modified for ROS2 by Vineet
 */

// =============================================================================
//  CFS_Sample 本体部
//
//          Filename: main.c
//
// =============================================================================
//    Ver 1.0.0   2022/06/01
// =============================================================================
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <chrono>
#include <cstdint>
#include <iomanip>
#include <sstream>
#include <string>

#include <leptrino/pCommon.h>
#include <leptrino/rs_comm.h>
#include <leptrino/pComResInternal.h>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"

// =============================================================================
//  マクロ定義 Defining macros
// =============================================================================
#define PRG_VER  "Ver 1.0.0"

// =============================================================================
//  構造体定義 Defining structs
// =============================================================================
typedef struct ST_SystemInfo
{
  int com_ok;
} SystemInfo;

// =============================================================================
//  プロトタイプ宣言 Declaring prototypes
// =============================================================================
void App_Init(void);
void App_Close(rclcpp::Logger logger);
ULONG SendData(UCHAR *pucInput, USHORT usSize);
void GetProductInfo(rclcpp::Logger logger);
void GetLimit(rclcpp::Logger logger);
void SerialStart(rclcpp::Logger logger);
void SerialStop(rclcpp::Logger logger);
void DrainReceiveQueue(void);
bool ReceiveResponse(const rclcpp::Node::SharedPtr& node, UCHAR expected_cmd, int min_len,
                     UCHAR *buffer, int *length);

// =============================================================================
//  モジュール変数定義 Defining module variables
// =============================================================================
SystemInfo gSys;
UCHAR CommRcvBuff[256];
UCHAR CommSendBuff[1024];
UCHAR SendBuff[512];
double conversion_factor[FN_Num];

std::string g_com_port;
int g_rate;

namespace
{
constexpr int kResponseHeaderSize = 4;
constexpr int kProductInfoResponseSize = 0x20;
constexpr int kLimitResponseSize = 0x1c;
constexpr int kForceDataResponseSize = 0x14;
constexpr int kResponseTimeoutMs = 1000;

std::string FixedAsciiString(const UCHAR *data, size_t size)
{
  size_t len = 0;
  while (len < size && data[len] != '\0')
  {
    ++len;
  }
  while (len > 0 && data[len - 1] == ' ')
  {
    --len;
  }
  return std::string(reinterpret_cast<const char *>(data), len);
}

float ReadLittleEndianFloat(const UCHAR *data)
{
  static_assert(sizeof(float) == sizeof(uint32_t), "float must be 32-bit");
  const uint32_t raw = static_cast<uint32_t>(data[0]) |
                       (static_cast<uint32_t>(data[1]) << 8) |
                       (static_cast<uint32_t>(data[2]) << 16) |
                       (static_cast<uint32_t>(data[3]) << 24);
  float value;
  memcpy(&value, &raw, sizeof(value));
  return value;
}

SSHORT ReadLittleEndianInt16(const UCHAR *data)
{
  const uint16_t raw = static_cast<uint16_t>(data[0]) |
                       (static_cast<uint16_t>(data[1]) << 8);
  return static_cast<SSHORT>(static_cast<int16_t>(raw));
}

std::string FrameToHex(const UCHAR *data, int length)
{
  std::ostringstream stream;
  stream << std::hex << std::setfill('0');
  for (int i = 0; i < length; ++i)
  {
    if (i > 0)
    {
      stream << ' ';
    }
    stream << std::setw(2) << static_cast<unsigned int>(data[i]);
  }
  return stream.str();
}
}  // namespace

#define TEST_TIME 0

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("leptrino");

  node->declare_parameter<std::string>("com_port", "/dev/ttyUSB0");
  node->declare_parameter<int>("rate", 1200);
  node->declare_parameter<std::string>("frame_id", "leptrino");

  node->get_parameter("com_port", g_com_port);
  node->get_parameter("rate", g_rate);
  rclcpp::Rate rate(g_rate);

  std::string frame_id;
  node->get_parameter("frame_id", frame_id);

  int rt = 0;
  int response_length = 0;

  App_Init();

  if (gSys.com_ok == NG)
  {
    RCLCPP_ERROR(node->get_logger(), "%s open failed\n", g_com_port.c_str());
    rclcpp::shutdown();
    return 1;
  }

  // Stop and drain first in case the sensor was left in continuous-output mode.
  SerialStop(node->get_logger());
  DrainReceiveQueue();

  // 製品情報取得
  GetProductInfo(node->get_logger());
  if (!ReceiveResponse(node, CMD_GET_INF, kProductInfoResponseSize, CommRcvBuff, &response_length))
  {
    App_Close(node->get_logger());
    rclcpp::shutdown();
    return 1;
  }
  RCLCPP_INFO(node->get_logger(), "Version: %s",
              FixedAsciiString(&CommRcvBuff[4 + P_NAME_SIZE + SERIAL_SIZE], F_VER_SIZE).c_str());
  RCLCPP_INFO(node->get_logger(), "SerialNo: %s",
              FixedAsciiString(&CommRcvBuff[4 + P_NAME_SIZE], SERIAL_SIZE).c_str());
  RCLCPP_INFO(node->get_logger(), "Type: %s",
              FixedAsciiString(&CommRcvBuff[4], P_NAME_SIZE).c_str());

  GetLimit(node->get_logger());
  if (!ReceiveResponse(node, CMD_GET_LIMIT, kLimitResponseSize, CommRcvBuff, &response_length))
  {
    App_Close(node->get_logger());
    rclcpp::shutdown();
    return 1;
  }
  for (int i = 0; i < FN_Num; i++)
  {
    const float limit = ReadLittleEndianFloat(&CommRcvBuff[4 + i * sizeof(float)]);
    RCLCPP_INFO(node->get_logger(), "\tLimit[%d]: %f", i, limit);
    conversion_factor[i] = limit * 1e-4;
  }

  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr force_torque_pub = node->create_publisher<geometry_msgs::msg::WrenchStamped>("force_torque", 1);

  usleep(10000);

  // 連続送信開始
  SerialStart(node->get_logger());

#if TEST_TIME
  double dt_sum = 0;
  int dt_count = 0;
  rclcpp::Time start_time;
#endif

  while (rclcpp::ok())
  {
    Comm_Rcv();
    if (Comm_CheckRcv() != 0)
    { //受信データ有

#if TEST_TIME
      dt_count++;
      dt_sum += (node->now() - start_time).toSec();
      if (dt_sum >= 1.0)
      {
        ROS_INFO("Time test: read %d in %6.3f sec: %6.3f kHz", dt_count, dt_sum, (dt_count / dt_sum) * 0.001);
        dt_count = 0;
        dt_sum = 0.0;
      }
      start_time = node->now();
#endif

      memset(CommRcvBuff, 0, sizeof(CommRcvBuff));
      rt = Comm_GetRcvData(CommRcvBuff);
      if (rt > 0)
      {
        if (rt < kForceDataResponseSize || CommRcvBuff[2] != CMD_DATA_START || CommRcvBuff[3] != RES_ERR_OK)
        {
          RCLCPP_DEBUG(node->get_logger(), "Ignore frame: %s", FrameToHex(CommRcvBuff, rt).c_str());
          continue;
        }

        SSHORT force[FN_Num];
        for (int i = 0; i < FN_Num; ++i)
        {
          force[i] = ReadLittleEndianInt16(&CommRcvBuff[4 + i * sizeof(SSHORT)]);
        }

        auto& clk = *node->get_clock();
        RCLCPP_DEBUG_THROTTLE(node->get_logger(),
                              clk,
                              0.1,
                              "%d,%d,%d,%d,%d,%d",
                              force[0], force[1], force[2], force[3], force[4], force[5]
                             );

        auto msg = geometry_msgs::msg::WrenchStamped();
        msg.header.stamp = node->now();
        msg.header.frame_id = frame_id;
        msg.wrench.force.x = force[0] * conversion_factor[0];
        msg.wrench.force.y = force[1] * conversion_factor[1];
        msg.wrench.force.z = force[2] * conversion_factor[2];
        msg.wrench.torque.x = force[3] * conversion_factor[3];
        msg.wrench.torque.y = force[4] * conversion_factor[4];
        msg.wrench.torque.z = force[5] * conversion_factor[5];
        force_torque_pub->publish(msg);
      }
    }
    else
    {
      rate.sleep();
    }

  } //while

  SerialStop(node->get_logger());
  App_Close(node->get_logger());
  return 0;
}

// ----------------------------------------------------------------------------------
//  アプリケーション初期化
// ----------------------------------------------------------------------------------
//  引数  : none
//  戻り値  : none
// ----------------------------------------------------------------------------------
void App_Init(void)
{
  int rt;

  //Commポート初期化
  gSys.com_ok = NG;
  rt = Comm_Open(g_com_port.c_str());
  if (rt == OK)
  {
    Comm_Setup(460800, PAR_NON, BIT_LEN_8, 0, 0, CHR_ETX);
    gSys.com_ok = OK;
  }

}

// ----------------------------------------------------------------------------------
//  アプリケーション終了処理
// ----------------------------------------------------------------------------------
//  引数  : none
//  戻り値  : none
// ----------------------------------------------------------------------------------
void App_Close(rclcpp::Logger logger)
{
  RCLCPP_DEBUG(logger, "Application close\n");

  if (gSys.com_ok == OK)
  {
    Comm_Close();
  }
}

void DrainReceiveQueue(void)
{
  UCHAR discard[256];

  for (int i = 0; i < 20; ++i)
  {
    Comm_Rcv();
    while (Comm_CheckRcv() != 0)
    {
      memset(discard, 0, sizeof(discard));
      Comm_GetRcvData(discard);
    }
    usleep(10000);
  }
}

bool ReceiveResponse(const rclcpp::Node::SharedPtr& node, UCHAR expected_cmd, int min_len,
                     UCHAR *buffer, int *length)
{
  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::milliseconds(kResponseTimeoutMs);

  while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline)
  {
    Comm_Rcv();
    while (Comm_CheckRcv() != 0)
    {
      memset(buffer, 0, sizeof(CommRcvBuff));
      const int frame_len = Comm_GetRcvData(buffer);
      if (frame_len <= 0)
      {
        continue;
      }

      RCLCPP_DEBUG(node->get_logger(), "RX frame: %s", FrameToHex(buffer, frame_len).c_str());

      if (frame_len < kResponseHeaderSize)
      {
        RCLCPP_WARN(node->get_logger(), "Ignore short response: %s",
                    FrameToHex(buffer, frame_len).c_str());
        continue;
      }
      if (buffer[2] != expected_cmd)
      {
        RCLCPP_DEBUG(node->get_logger(),
                     "Ignore response command 0x%02X while waiting for 0x%02X",
                     static_cast<unsigned int>(buffer[2]),
                     static_cast<unsigned int>(expected_cmd));
        continue;
      }
      if (buffer[3] != RES_ERR_OK)
      {
        RCLCPP_ERROR(node->get_logger(), "Response 0x%02X returned error 0x%02X",
                     static_cast<unsigned int>(expected_cmd),
                     static_cast<unsigned int>(buffer[3]));
        return false;
      }
      if (frame_len < min_len)
      {
        RCLCPP_ERROR(node->get_logger(),
                     "Response 0x%02X is too short: %d bytes, expected at least %d",
                     static_cast<unsigned int>(expected_cmd), frame_len, min_len);
        return false;
      }

      *length = frame_len;
      return true;
    }

    usleep(1000);
  }

  RCLCPP_ERROR(node->get_logger(), "Timed out waiting for response 0x%02X",
               static_cast<unsigned int>(expected_cmd));
  return false;
}

/*********************************************************************************
 * Function Name  : HST_SendResp
 * Description    : データを整形して送信する
 * Input          : pucInput 送信データ
 *                : 送信データサイズ
 * Output         :
 * Return         :
 *********************************************************************************/
ULONG SendData(UCHAR *pucInput, USHORT usSize)
{
  USHORT usCnt;
  UCHAR ucWork;
  UCHAR ucBCC = 0;
  UCHAR *pucWrite = &CommSendBuff[0];
  USHORT usRealSize;

  // データ整形
  *pucWrite = CHR_DLE; // DLE
  pucWrite++;
  *pucWrite = CHR_STX; // STX
  pucWrite++;
  usRealSize = 2;

  for (usCnt = 0; usCnt < usSize; usCnt++)
  {
    ucWork = pucInput[usCnt];
    if (ucWork == CHR_DLE)
    { // データが0x10ならば0x10を付加
      *pucWrite = CHR_DLE; // DLE付加
      pucWrite++; // 書き込み先
      usRealSize++; // 実サイズ
      // BCCは計算しない!
    }
    *pucWrite = ucWork; // データ
    ucBCC ^= ucWork; // BCC
    pucWrite++; // 書き込み先
    usRealSize++; // 実サイズ
  }

  *pucWrite = CHR_DLE; // DLE
  pucWrite++;
  *pucWrite = CHR_ETX; // ETX
  ucBCC ^= CHR_ETX; // BCC計算
  pucWrite++;
  *pucWrite = ucBCC; // BCC付加
  usRealSize += 3;

  Comm_SendData(&CommSendBuff[0], usRealSize);

  return OK;
}

void GetProductInfo(rclcpp::Logger logger)
{
  USHORT len;

  RCLCPP_INFO(logger, "Get sensor information");
  len = 0x04; // データ長
  SendBuff[0] = len; // レングス
  SendBuff[1] = 0xFF; // センサNo.
  SendBuff[2] = CMD_GET_INF; // コマンド種別
  SendBuff[3] = 0; // 予備

  SendData(SendBuff, len);
}

void GetLimit(rclcpp::Logger logger)
{
  USHORT len;

  RCLCPP_INFO(logger, "Get sensor limit");
  len = 0x04;
  SendBuff[0] = len; // レングス length
  SendBuff[1] = 0xFF; // センサNo. Sensor no.
  SendBuff[2] = CMD_GET_LIMIT; // コマンド種別 Command type
  SendBuff[3] = 0; // 予備 reserve

  SendData(SendBuff, len);
}

void SerialStart(rclcpp::Logger logger)
{
  USHORT len;

  RCLCPP_INFO(logger, "Start sensor");
  len = 0x04; // データ長
  SendBuff[0] = len; // レングス
  SendBuff[1] = 0xFF; // センサNo.
  SendBuff[2] = CMD_DATA_START; // コマンド種別
  SendBuff[3] = 0; // 予備

  SendData(SendBuff, len);
}

void SerialStop(rclcpp::Logger logger)
{
  USHORT len;

  RCLCPP_INFO(logger, "Stop sensor\n");
  len = 0x04; // データ長
  SendBuff[0] = len; // レングス
  SendBuff[1] = 0xFF; // センサNo.
  SendBuff[2] = CMD_DATA_STOP; // コマンド種別
  SendBuff[3] = 0; // 予備

  SendData(SendBuff, len);
}
