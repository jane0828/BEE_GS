#pragma once
#ifndef _MIMAN_COMS_H_
#define _MIMAN_COMS_H_

#include "miman_config.h"
#include "miman_orbital.h"

typedef struct{
	uint16_t Identifier;
	uint16_t PacketType;
	uint32_t Length;
	uint8_t Data[];

}__attribute__((packed)) packetsign;

typedef struct{
	uint16_t target;
	uint16_t filestatus;
	uint32_t filenum;
	uint32_t offset;
	uint32_t step;
}__attribute__((packed)) dlreqdata;

typedef struct{
	int32_t filenum;
	uint32_t file[];
}__attribute__((packed)) filelist;

// typedef enum {
//     GS_FTP_INFO_FILE  = 0,   /**< File size and checksum */
//     GS_FTP_INFO_CRC = 1,         /**< CRC of remote and local file */
//     GS_FTP_INFO_COMPLETED = 2,   /**< Completed and total chunks */
//     GS_FTP_INFO_PROGRESS = 3,    /**< Current chunk, total_chunks and chunk_size */
// } gs_ftp_info_type_t;

typedef struct {
    FILE       * fp;
    FILE       * fp_map;
    csp_conn_t * conn;
    uint32_t   timeout;
    char       file_name[GS_FTP_PATH_LENGTH];
    uint32_t   file_size;
    uint32_t   chunks;
    int        chunk_size;
    uint32_t   checksum;
    ftp_status_element_t last_status[GS_FTP_STATUS_CHUNKS];
    uint32_t   last_entries;
    gs_ftp_info_callback_t info_callback;
    void       * info_data;
} gs_ftp_state_t;

typedef struct {
    gs_ftp_backend_type_t backend;
    const char * path;
    uint32_t addr;
    uint32_t size;
} gs_ftp_url_t;


// Changed by JHKim 25.01.29 06:12 (LA,US)
typedef struct {

	//FM
    uint8 Callsign[6];
    uint8 CurrentMode;
    uint8 CurrentSubmode;
    uint8 PrevioudMode;
    uint8 PreviousSubmode;
    uint8 CurrentModeFlag;
    uint8 PreviousModeFlag;
    uint32 ApplicationRunStatus;
    uint32 SatelliteTime;

	//UANT
	uint16 DeployState_UANT;

	//UTRX
	uint32 rxfreq;
	uint32 txfreq;
	int16 LastRssi;
    uint32 TotRxBytes;
    uint8 StatusConfiguration;

	//EPS - P60 Dock
	uint8 out_en_dock[7]; //01458910
    int16 temp_dock[2];
    uint32 bootcause;
    uint32 bootcnt;
    uint32 uptime;
    uint16 resetcause;
    uint8 batt_mode;
    uint8 heater_on;
    uint16 latchup_dock[7]; //01458910
    uint16 vbat_v;
    int16 batt_v;
    int16 batt_temp[2];
    uint8 device_status[8];
    uint32 wdt_cnt_gnd;
    uint32 wdt_gnd_left;
    int16 batt_chrg;
    int16 batt_dischrg;

	//EPS - PDU
    int16 vbat;
    uint8 out_en_pdu[6]; //034758
    uint16 latchup_pdu[6];
    uint16 out_voltage[6]; // 034758

	//EPS - ACU
    int16 c_in[4]; //0123
    uint16 v_in[4]; //0123

	//ADCS
	uint8 RWL0_PowerState;
	uint8 RWL1_PowerState;
	uint8 RWL2_PowerState;
	uint8 MAG0_PowerState;
    uint8 GYRO0_PowerState;
	uint8 FSS0_PowerState;
	uint8 HSS0_PowerState;
	uint8 Control_Mode;
	uint16 Mag_Control_Timeout;
	float GYRO_Calib_rate_X;
	float GYRO_Calib_rate_Y;
	float GYRO_Calib_rate_Z;
	
	//STX
	uint8 Status;
	int16 Board_Temperature;
	int16 Battery_Current;
	int16 Battery_Voltage;

	//PAYC
	int16 temp_PAYC;
	uint16 icore;

	//PAYR
	uint8 DeployStatus_PAYR;

	//PAYS
	uint8 PAYS_State;
	uint8 PAYS_Sign;
	uint8 PAYS_Temp;

}__attribute__((packed)) Beacon;

#define a sizeof(Beacon);

typedef struct {
    uint8 Callsign[6];
    uint8 CurrentMode;
    uint8 CurrentSubmode;
    uint8 PrevioudMode;
    uint8 PreviousSubmode;
    uint8 CurrentModeFlag;
    uint8 PreviousModeFlag;
    uint32 ApplicationRunStatus;
    uint32 SatelliteTime;
    uint16 RebootCount;
    uint8 RebootCause;

}__attribute__((packed)) FM_HK_;

typedef struct {
	uint16 DeployState_UANT;
}__attribute__((packed)) UANT_;

typedef struct {
	uint32 rxfreq;
	uint32 txfreq;
	int16 LastRssi;
    uint32 TotRxBytes;
    uint8 StatusConfiguration;
}__attribute__((packed)) UTRX_;

typedef struct {
	//EPS - P60 Dock
	uint8 out_en_dock[7]; //01458910
    int16 temp_dock[2];
    uint32 bootcause;
    uint32 bootcnt;
    uint32 uptime;
    uint16 resetcause;
    uint8 batt_mode;
    uint8 heater_on;
    uint16 latchup_dock[7]; //01458910
    uint16 vbat_v;
    int16 batt_v;
    int16 batt_temp[2];
    uint8 device_status[8];
    uint32 wdt_cnt_gnd;
    uint32 wdt_gnd_left;
    int16 batt_chrg;
    int16 batt_dischrg;
	//EPS - PDU
    int16 vbat;
    uint8 out_en_pdu[6]; //034758
    uint16 latchup_pdu[6];
    uint16 out_voltage[6]; // 034758
	//EPS - ACU
    int16 c_in[4]; //0123
    uint16 v_in[4]; //0123
}__attribute__((packed)) EPS_;

typedef struct {
	uint8 RWL0_PowerState;
	uint8 RWL1_PowerState;
	uint8 RWL2_PowerState;
	uint8 MAG0_PowerState;
	uint8 FSS0_PowerState;
	uint8 HSS0_PowerState;
	uint8 Control_Mode;
	uint16 Mag_Control_Timeout;
	float GYRO_Calib_rate_X;
	float GYRO_Calib_rate_Y;
	float GYRO_Calib_rate_Z;
}__attribute__((packed)) ADCS_;

typedef struct {
	uint8 Status;
	int16 Board_Temperature;
	int16 Battery_Current;
	int16 Battery_Voltage;
}__attribute__((packed)) STX_;

typedef struct {
	int16 temp_PAYC;
	uint16 icore;
}__attribute__((packed)) PAYC_;

typedef struct {
	uint8 DeployStatus_PAYR;
}__attribute__((packed)) PAYR_;

typedef struct {
	uint8 PAYS_State;
	uint8 PAYS_Sign;
	uint8 PAYS_Temp;
}__attribute__((packed)) PAYS_;


// typedef struct {
// 	CCSDS_Header_ CCSDS_Header;
// 	FM_HK_ FM;
// 	EPS_ EPS;
// 	TCS_ TCS;
// 	RWA_ RWA;
// 	MTQ_ MTQ;
// 	SNSR_ SNSR;
// 	UTRX_ UTRX;
// 	STX_ STX;
// 	PAY_ PAY;
// }__attribute__((packed)) HK;

// typedef struct {
// 	FM_HK_ FM;
// 	ADCS_ ADCS;
// }__attribute__((packed)) AOD;

typedef struct {
	uint32_t ExTime;
	uint32_t ExWindow;
	uint16_t EntryID;
	uint16_t GroupID;
	uint8_t cmd[];
}__attribute__((packed)) Book;

void * TRxController(void *);
void * SignalTest(void*);
void now_rx_bytes_update();
void set_rx_bytes(uint32_t nowbytes);
uint32_t get_rx_bytes();
uint32_t * get_rx_bytes_address();
void buf_allclear();
void CalculateChecksum(CommandHeader_t* Cmd);
int32_t GenerateCmdMsg(CommandHeader_t* Cmd, uint16_t MsgId, uint8_t FcnCode, uint32_t ArgLen);
csp_socket_t *  DL_sock_initialize();
int BeaconSaver(Beacon * bec);
void * task_downlink_onorbit(void * socketinfo);
void * task_uplink_onorbit(void * sign_);

int PacketHandler(csp_packet_t *packet, int type, int NowCursor);
packetsign * PingInit(FSWTle * FSWTleinfo);
csp_packet_t * PacketEncoder(packetsign * sign, bool freeer = true);
packetsign * PacketDecoder(csp_packet_t * packet);


class CmdGenerator_GS {
private:
    void SetHeaderWord(uint8_t* Word16, uint16_t Value, uint16_t Mask);
    void SetHeaderByte(uint8_t* Byte, uint8_t Value, uint8_t Mask);
    void GetHeaderWord(const uint8_t* Word16, uint16_t& Value, uint16_t Mask);
    uint32_t ComputeCheckSum(void);

public:
	CFE_MSG_CommandHeader* CmdHeader;
	bool Scheduled = false;
	bool Checksum = true;

    CmdGenerator_GS(void);
    ~CmdGenerator_GS(void);

    int GenerateCmdHeader(uint32_t MsgId, uint16_t FncCode, uint32_t Size, void* Data);
    void CopyCmdHeaderToBuffer(uint8_t* Buffer);

    void InitHeader(void);
    void SetHeader(const CFE_MSG_CommandHeader* Header);
    const CFE_MSG_CommandHeader* GetHeader(void) const;

    int SetHasSecondaryHeader(bool HasSec);
    int SetMsgId(uint16_t MsgId);
    int SetSize(uint16_t Size);
    int SetSegmentationFlag(uint16_t SegFlag);
    int SetFncCode(uint16_t FncCode);

    bool HasSecondaryHeader(void) const;
    uint16_t GetSize(void);
    uint16_t GetFncCode(void) const;

    int GenerateChecksum(void);
	int Scheduling(uint32_t ExecutionTime, uint32_t ExecutionWindow, uint32_t EntryID, uint16_t GroupID);
	packetsign * GenerateCMDPacket(void);
};

void * Direct_Shell(void * data);

#endif _MIMAN_COMS_H_