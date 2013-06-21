// =============================================================================
//	RS232C 用モジュール
//
//					Filename: rs_comm.h
//
// =============================================================================
//		Ver 1.0.0		2012/11/01
// =============================================================================
#include "pCommon.h"

// =============================================================================
//	マクロ定義
// =============================================================================
#ifndef _RSCOMM_H
#define _RSCOMM_H

#define PAR_NON 		0
#define PAR_ODD 		1
#define PAR_EVEN 		2

#define BIT_LEN_7		7
#define BIT_LEN_8		8

#define CHR_STX		0x02
#define CHR_ETX		0x03
#define CHR_EOT		0x04
#define CHR_ENQ		0x05
#define CHR_ACK		0x06
#define CHR_LF		0x0A
#define CHR_CR		0x0D
#define CHR_DLE		0x10
#define CHR_NAK		0x15
#define CHR_SUB		0x1A

int Comm_Open(const char * dev); //デバイスオープン
void Comm_Close(void); //デバイスクローズ
void Comm_Setup(long baud, int parity, int bitlen, int rts, int dtr, char code); //ポート設定
int Comm_SendData(UCHAR *buff, int l);
int Comm_CheckRcv(void); //受信有無確認
int Comm_GetRcvData(UCHAR *buff); //受信データ取得
void Comm_Rcv(void);

#endif
