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
 */

// =============================================================================
//	CFS_Sample 本体部
//
//					Filename: main.c
//
// =============================================================================
//		Ver 1.0.0		2012/11/01
// =============================================================================
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>

#include <leptrino/pCommon.h>
#include <leptrino/rs_comm.h>
#include <leptrino/pComResInternal.h>

#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>

// =============================================================================
//	マクロ定義
// =============================================================================
#define PRG_VER	 "Ver 1.0.0"

// =============================================================================
//	構造体定義
// =============================================================================
typedef struct ST_SystemInfo
{
  int com_ok;
} SystemInfo;

// =============================================================================
//	プロトタイプ宣言
// =============================================================================
void App_Init(void);
void App_Close(void);
ULONG SendData(UCHAR *pucInput, USHORT usSize);
void GetProductInfo(void);
void GetLimit(void);
void SerialStart(void);
void SerialStop(void);

// =============================================================================
//	モジュール変数定義
// =============================================================================
SystemInfo gSys;
UCHAR CommRcvBuff[256];
UCHAR CommSendBuff[1024];
UCHAR SendBuff[512];
double conversion_factor[FN_Num];

std::string g_com_port;
int g_rate;

#define TEST_TIME 0

int main(int argc, char** argv)
{
  ros::init(argc, argv, "leptrino");

  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  if (!nh_private.getParam("com_port", g_com_port))
  {
    ROS_WARN("Port is not defined, trying /dev/ttyUSB0");
    g_com_port = "/dev/ttyUSB0";
  }

  if (!nh_private.getParam("rate", g_rate))
  {
    ROS_WARN("Rate is not defined, using maximum 1.2 kHz");
    g_rate = 1200;
  }
  ros::Rate rate(g_rate);

  std::string frame_id = "leptrino";
  nh_private.getParam("frame_id", frame_id);

  int i, l = 0, rt = 0;
  ST_RES_HEAD *stCmdHead;
  ST_R_DATA_GET_F *stForce;
  ST_R_GET_INF *stGetInfo;
  ST_R_LEP_GET_LIMIT* stGetLimit;

  App_Init();

  if (gSys.com_ok == NG)
  {
    ROS_ERROR("%s open failed\n", g_com_port.c_str());
    exit(0);
  }

  // 製品情報取得
  GetProductInfo();
  while (ros::ok())
  {
    Comm_Rcv();
    if (Comm_CheckRcv() != 0)
    { //受信データ有
      CommRcvBuff[0] = 0;

      rt = Comm_GetRcvData(CommRcvBuff);
      if (rt > 0)
      {
        stGetInfo = (ST_R_GET_INF *)CommRcvBuff;
        stGetInfo->scFVer[F_VER_SIZE] = 0;
        ROS_INFO("Version: %s", stGetInfo->scFVer);
        stGetInfo->scSerial[SERIAL_SIZE] = 0;
        ROS_INFO("SerialNo: %s", stGetInfo->scSerial);
        stGetInfo->scPName[P_NAME_SIZE] = 0;
        ROS_INFO("Type: %s", stGetInfo->scPName);
        break;
      }
    }
    else
    {
      rate.sleep();
    }
  }

  GetLimit();
  while (ros::ok())
  {
    Comm_Rcv();
    if (Comm_CheckRcv() != 0)
    { //受信データ有
      CommRcvBuff[0] = 0;

      rt = Comm_GetRcvData(CommRcvBuff);
      if (rt > 0)
      {
        stGetLimit = (ST_R_LEP_GET_LIMIT *)CommRcvBuff;
        for (int i = 0; i < FN_Num; i++)
        {
          ROS_INFO("\tLimit[%d]: %f", i, stGetLimit->fLimit[i]);
          conversion_factor[i] = stGetLimit->fLimit[i] * 1e-4;
        }
        break;
      }
    }
    else
    {
      rate.sleep();
    }
  }

  ros::Publisher force_torque_pub = nh_private.advertise<geometry_msgs::WrenchStamped>("force_torque", 1);

  usleep(10000);

  // 連続送信開始
  SerialStart();

#if TEST_TIME
  double dt_sum = 0;
  int dt_count = 0;
  ros::Time start_time;
#endif

  while (ros::ok())
  {
    Comm_Rcv();
    if (Comm_CheckRcv() != 0)
    { //受信データ有

#if TEST_TIME
      dt_count++;
      dt_sum += (ros::Time::now() - start_time).toSec();
      if (dt_sum >= 1.0)
      {
        ROS_INFO("Time test: read %d in %6.3f sec: %6.3f kHz", dt_count, dt_sum, (dt_count / dt_sum) * 0.001);
        dt_count = 0;
        dt_sum = 0.0;
      }
      start_time = ros::Time::now();
#endif

      memset(CommRcvBuff, 0, sizeof(CommRcvBuff));
      rt = Comm_GetRcvData(CommRcvBuff);
      if (rt > 0)
      {
        stForce = (ST_R_DATA_GET_F *)CommRcvBuff;
        ROS_DEBUG_THROTTLE(0.1, "%d,%d,%d,%d,%d,%d", stForce->ssForce[0], stForce->ssForce[1], stForce->ssForce[2],
                           stForce->ssForce[3], stForce->ssForce[4], stForce->ssForce[5]);

        geometry_msgs::WrenchStampedPtr msg(new geometry_msgs::WrenchStamped);
        msg->header.stamp = ros::Time::now();
        msg->header.frame_id = frame_id;
        msg->wrench.force.x = stForce->ssForce[0] * conversion_factor[0];
        msg->wrench.force.y = stForce->ssForce[1] * conversion_factor[1];
        msg->wrench.force.z = stForce->ssForce[2] * conversion_factor[2];
        msg->wrench.torque.x = stForce->ssForce[3] * conversion_factor[3];
        msg->wrench.torque.y = stForce->ssForce[4] * conversion_factor[4];
        msg->wrench.torque.z = stForce->ssForce[5] * conversion_factor[5];
        force_torque_pub.publish(msg);
      }
    }
    else
    {
      rate.sleep();
    }

    ros::spinOnce();
  } //while

  SerialStop();
  App_Close();
  return 0;
}

// ----------------------------------------------------------------------------------
//	アプリケーション初期化
// ----------------------------------------------------------------------------------
//	引数	: non
//	戻り値	: non
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
//	アプリケーション終了処理
// ----------------------------------------------------------------------------------
//	引数	: non
//	戻り値	: non
// ----------------------------------------------------------------------------------
void App_Close(void)
{
  printf("Application close\n");

  if (gSys.com_ok == OK)
  {
    Comm_Close();
  }
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

void GetProductInfo(void)
{
  USHORT len;

  ROS_INFO("Get sensor information");
  len = 0x04; // データ長
  SendBuff[0] = len; // レングス
  SendBuff[1] = 0xFF; // センサNo.
  SendBuff[2] = CMD_GET_INF; // コマンド種別
  SendBuff[3] = 0; // 予備

  SendData(SendBuff, len);
}

void GetLimit(void)
{
  USHORT len;

  ROS_INFO("Get sensor limit");
  len = 0x04;
  SendBuff[0] = len; // レングス
  SendBuff[1] = 0xFF; // センサNo.
  SendBuff[2] = CMD_GET_LIMIT; // コマンド種別
  SendBuff[3] = 0; // 予備

  SendData(SendBuff, len);
}

void SerialStart(void)
{
  USHORT len;

  ROS_INFO("Start sensor");
  len = 0x04; // データ長
  SendBuff[0] = len; // レングス
  SendBuff[1] = 0xFF; // センサNo.
  SendBuff[2] = CMD_DATA_START; // コマンド種別
  SendBuff[3] = 0; // 予備

  SendData(SendBuff, len);
}

void SerialStop(void)
{
  USHORT len;

  printf("Stop sensor\n");
  len = 0x04; // データ長
  SendBuff[0] = len; // レングス
  SendBuff[1] = 0xFF; // センサNo.
  SendBuff[2] = CMD_DATA_STOP; // コマンド種別
  SendBuff[3] = 0; // 予備

  SendData(SendBuff, len);
}
