/********************************************************************************/
/* File Name			: pComResInternal.h										*/
/* Description			: コマレス処理内部定義									*/
/********************************************************************************/
#ifndef _PCOMRESINTERNAL_H
#define _PCOMRESINTERNAL_H

#include "pCommon.h"

/********************************************************************************/
/*	定数定義 																	*/
/********************************************************************************/
/* コマンドコード */
#define CMD_GET_INF		0x2A			/* 製品情報確認 					*/
#define CMD_GET_LIMIT		0x2B			/* 定格値確認 						*/
#define CMD_DATA_START		0x32			/* データ送信開始 					*/
#define CMD_DATA_STOP		0x33			/* データ送信停止 					*/
#define CMD_SET_DF		0xA6			/* デジタルフィルタ設定 			*/
#define CMD_GET_DF		0xB6			/* デジタルフィルタ確認 			*/


/* レスポンス結果 */
#define RES_ERR_OK		0x00			/* 正常終了 						*/
#define RES_ERR_LEN		0x01			/* 電文長異常 						*/
#define RES_ERR_UNDEF		0x02			/* 未定義コマンド 					*/
#define RES_ERR_VAL		0x03			/* 設定値異常 						*/
#define RES_ERR_STATUS		0x04			/* 状態異常 						*/


/********************************************************************************/
/*	構造体定義 																	*/
/********************************************************************************/
/*** コマンド ***/
/* コマンドヘッダー */
typedef struct tagCmdHead {
	UCHAR	ucLen;							/* レングス 						*/
	UCHAR	ucTermNo;						/* 端末No. 							*/
	UCHAR	ucCmd;							/* コマンド種別 					*/
	UCHAR	ucRsv;							/* 予備 							*/
} ST_CMD_HEAD;

/* デジタルフィルタ設定 */
typedef struct tagCLepSetDf {
	ST_CMD_HEAD	stHead;						/* ヘッダ 							*/
	UCHAR		ucDF;						/* デジタルフィルタ有効無効			*/
	UCHAR		ucRsv[3];					/* 予備 							*/
} ST_C_LEP_SET_DF;


/*** レスポンス ***/
/* レスポンスヘッダー */
typedef struct tagResHead {
	UCHAR	ucLen;							/* レングス 						*/
	UCHAR	ucTermNo;						/* 端末No. 							*/
	UCHAR	ucCmd;							/* コマンド種別 					*/
	UCHAR	ucResult;						/* 結果 							*/
} ST_RES_HEAD;

/* 製品情報確認 */
typedef struct tagRGetInf {
	ST_RES_HEAD	stHead;						/* ヘッダ 							*/
	SCHAR		scPName[P_NAME_SIZE];				/* 製品型式 						*/
	SCHAR		scSerial[SERIAL_SIZE];				/* シリアルNo.						*/
	SCHAR		scFVer[F_VER_SIZE];				/* ファームバージョン 				*/
	SCHAR		scFreq[FREQ_SIZE];				/* 出力レート						*/
} ST_R_GET_INF;

/* データ取得 */
typedef struct tagRDataGetF {
	ST_RES_HEAD	stHead;						/* ヘッダ 							*/
	SSHORT		ssForce[FN_Num];				/* 力データ 						*/
	SSHORT		ssTemp;						/* 温度データ 						*/
	UCHAR		ucStatus;					/* ステータス 						*/
	UCHAR		ucRsv;						/* 予備 							*/
} ST_R_DATA_GET_F;

/* 定格値確認 */
typedef struct tagRLepGetLimit {
	ST_RES_HEAD	stHead;						/* ヘッダ 							*/
	float		fLimit[FN_Num];					/* 定格 							*/
} ST_R_LEP_GET_LIMIT;

/* デジタルフィルタ確認 */
typedef struct tagRLepGetDf {
	ST_RES_HEAD	stHead;						/* ヘッダ 							*/
	UCHAR		ucDF;						/* デジタルフィルタ有効無効			*/
	UCHAR		ucRsv[3];					/* 予備 							*/
} ST_R_LEP_GET_DF;

/********************************************************************************/
/*	外部公開関数定義 															*/
/********************************************************************************/



#endif
/************************* (C) COPYRIGHT 2010 Leptrino **************************/
