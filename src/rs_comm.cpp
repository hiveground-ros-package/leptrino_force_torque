// =============================================================================
//	RS232C 用モジュール
//
//					Filename: rs_comm.c
//
// =============================================================================
//		Ver 1.0.0		2012/11/01
// =============================================================================

//シリアル通信用
#include <sys/ioctl.h>
#include <fcntl.h>
#include <termios.h>
#include <string.h>
#include <unistd.h>
#include <stdio.h>
#include <leptrino/rs_comm.h>

#define MAX_BUFF		10
#define MAX_LENGTH	255

#define STS_IDLE		0
#define STS_WAIT_STX	1
#define STS_DATA		2
#define STS_WAIT_ETX	3
#define STS_WAIT_BCC	4

static int Comm_RcvF = 0; //受信データ有フラグ
static int p_rd = 0, p_wr = 0; //受信リングバッファ読出し、書込みポインタ
static int fd = 0; //
static int rcv_n = 0; //受信文字数

static UCHAR delim; //受信データデリミタ
static UCHAR rcv_buff[MAX_BUFF][MAX_LENGTH]; //受信リングバッファ
static UCHAR stmp[MAX_LENGTH]; //

struct termios tio; //ポート設定構造体

// ----------------------------------------------------------------------------------
//	デバイスオープン
// ----------------------------------------------------------------------------------
//	引数	: dev .. シリアルポート
//	戻り値	: 正常時:0   エラー時:-1
// ----------------------------------------------------------------------------------
int Comm_Open(const char * dev)
{
  //既にオープンしているときは一度閉じる
  if (fd != 0)
    Comm_Close();
  //ポートオープン
  fd = open(dev, O_RDWR | O_NDELAY | O_NOCTTY);
  if (fd < 0)
    return NG;
  //デリミタ
  delim = 0;

  return OK;
}

// ----------------------------------------------------------------------------------
//	デバイスクローズ
// ----------------------------------------------------------------------------------
//	引数	: non
//	戻り値	: non
// ----------------------------------------------------------------------------------
void Comm_Close()
{
  if (fd > 0)
  {
    close(fd);
  }
  fd = 0;

  return;
}

// ----------------------------------------------------------------------------------
//	ポート設定
// ----------------------------------------------------------------------------------
//	引数	: boud   .. ボーレート 9600 19200 ....
//			: parity .. パリティー 
//			: bitlen .. ビット長
//			: rts    .. RTS制御
//			: dtr    .. DTR制御
//	戻り値	: non
// ----------------------------------------------------------------------------------
void Comm_Setup(long baud, int parity, int bitlen, int rts, int dtr, char code)
{
  long brate;
  long cflg;

  switch (baud)
  {
    case 2400:
      brate = B2400;
      break;
    case 4800:
      brate = B4800;
      break;
    case 9600:
      brate = B9600;
      break;
    case 19200:
      brate = B19200;
      break;
    case 38400:
      brate = B38400;
      break;
    case 57600:
      brate = B57600;
      break;
    case 115200:
      brate = B115200;
      break;
    case 230400:
      brate = B230400;
      break;
    case 460800:
      brate = B460800;
      break;
    default:
      brate = B9600;
      break;
  }
  //パリティ
  switch (parity)
  {
    case PAR_NON:
      cflg = 0;
      break;
    case PAR_ODD:
      cflg = PARENB | PARODD;
      break;
    default:
      cflg = PARENB;
      break;
  }
  //データ長
  switch (bitlen)
  {
    case 7:
      cflg |= CS7;
      break;
    default:
      cflg |= CS8;
      break;
  }
  //DTR
  switch (dtr)
  {
    case 1:
      cflg &= ~CLOCAL;
      break;
    default:
      cflg |= CLOCAL;
      break;
  }
  //RTS CTS
  switch (rts)
  {
    case 0:
      cflg &= ~CRTSCTS;
      break;
    default:
      cflg |= CRTSCTS;
      break;
  }

  //ポート設定フラグ
  tio.c_cflag = cflg | CREAD;
  tio.c_lflag = 0;
  tio.c_iflag = 0;
  tio.c_oflag = 0;
  tio.c_cc[VTIME] = 0;
  tio.c_cc[VMIN] = 0;

  cfsetspeed(&tio, brate);
  tcflush(fd, TCIFLUSH); //バッファの消去
  tcsetattr(fd, TCSANOW, &tio); //属性の設定

  delim = code; //デリミタコード
  return;
}

// ----------------------------------------------------------------------------------
//	文字列送信
// ----------------------------------------------------------------------------------
//	引数	: buff .. 文字列バッファ
//			: l    .. 送信文字数
//	戻り値	: 1:OK -1:NG
// ----------------------------------------------------------------------------------
int Comm_SendData(UCHAR *buff, int l)
{
  if (fd <= 0)
    return -1;

  int count = write(fd, buff, l);
  if(count != l)
    return NG;

  return OK;
}

// ----------------------------------------------------------------------------------
//	受信データ取得
// ----------------------------------------------------------------------------------
//	引数	: buff .. 文字列バッファ
//	戻り値	: 受信文字数
// ----------------------------------------------------------------------------------
int Comm_GetRcvData(UCHAR *buff)
{
  int l = rcv_buff[p_rd][0];

  if (p_wr == p_rd)
    return 0;

  memcpy(buff, &rcv_buff[p_rd][0], l);
  p_rd++;
  if (p_rd >= MAX_BUFF)
    p_rd = 0;

  l = strlen((char*)buff);

  return l;
}

// ----------------------------------------------------------------------------------
//	受信有無確認
// ----------------------------------------------------------------------------------
//	引数	: non 
//	戻り値	: 0:なし 0以外：あり
// ----------------------------------------------------------------------------------
int Comm_CheckRcv()
{
  return p_wr - p_rd;
}

// ----------------------------------------------------------------------------------
//	受信監視スレッド
// ----------------------------------------------------------------------------------
//	引数	: pParam .. 
//	戻り値	: non
// ----------------------------------------------------------------------------------
unsigned char rbuff[MAX_LENGTH];
unsigned char ucBCC;
void Comm_Rcv(void)
{
  int rt = 0;
  unsigned char ch;
  static int RcvSts = 0;

  while (1)
  {
    rt = read(fd, rbuff, 1);
    //受信データあり
    if (rt > 0)
    {
      rbuff[rt] = 0;
      ch = rbuff[0];

      switch (RcvSts)
      {
        case STS_IDLE:
          ucBCC = 0; /* BCC */
          rcv_n = 0;
          if (ch == CHR_DLE)
            RcvSts = STS_WAIT_STX;
          break;
        case STS_WAIT_STX:
          if (ch == CHR_STX)
          { /* STXがあれば次はデータ */
            RcvSts = STS_DATA;
          }
          else
          { /* STXでなければ元に戻る */
            RcvSts = STS_IDLE;
          }
          break;
        case STS_DATA:
          if (ch == CHR_DLE)
          { /* DLEがあれば次はETX */
            RcvSts = STS_WAIT_ETX;
          }
          else
          { /* 受信データ保存 */
            stmp[rcv_n] = ch;
            ucBCC ^= ch; /* BCC */
            rcv_n++;
          }
          break;
        case STS_WAIT_ETX:
          if (ch == CHR_DLE)
          { /* DLEならばデータである */
            stmp[rcv_n] = ch;
            ucBCC ^= ch; /* BCC */
            rcv_n++;
            RcvSts = STS_DATA;
          }
          else if (ch == CHR_ETX)
          { /* ETXなら次はBCC */
            RcvSts = STS_WAIT_BCC;
            ucBCC ^= ch; /* BCC */
          }
          else if (ch == CHR_STX)
          { /* STXならリセット */
            ucBCC = 0; /* BCC */
            rcv_n = 0;
            RcvSts = STS_DATA;
          }
          else
          {
            ucBCC = 0; /* BCC */
            rcv_n = 0;
            RcvSts = STS_IDLE;
          }
          break;
        case STS_WAIT_BCC:
          if (ucBCC == ch)
          { /* BCC一致 */
            //作成された文字列をリングバッファへコピー
            memcpy(rcv_buff[p_wr], stmp, rcv_n);
            p_wr++;
            if (p_wr >= MAX_BUFF)
              p_wr = 0;
          }
          /* 次のデータ受信に備える */
          ucBCC = 0; /* BCC */
          rcv_n = 0;
          RcvSts = STS_IDLE;
          break;
        default:
          RcvSts = STS_IDLE;
          break;
      }

      if (rcv_n > MAX_LENGTH)
      {
        ucBCC = 0;
        rcv_n = 0;
        RcvSts = STS_IDLE;
      }
    }
    else
    {
      break;
    }

    //受信完了フラグ
    if (p_rd != p_wr)
    {
      Comm_RcvF = 1;
    }
    else
    {
      Comm_RcvF = 0;
    }
  }
}
