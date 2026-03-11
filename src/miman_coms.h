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



// BEE-1000 RPT structure
typedef struct {
    bool valid;
    uint16_t CCMessage_ID;
    uint16_t CCCount;
    uint16_t CCLength;
    uint8_t CCTime_code[6];
    uint16_t msg_id;
    uint8_t cc;
    uint8_t ret_type;
    int32_t ret_code;
    uint16_t ret_val_size;
    std::vector<uint8_t> payload;
} ReportPacket_t;

extern ReportPacket_t g_last_report;


// BEE-1000 Beacon
typedef struct {

	// CCSDS Header
    uint8 CCSDS_MID[2];
    uint8 CCSDS_Seq[2];
    uint8 CCSDS_Len[2];
    uint8 CCSDS_TimeCode[6];

	// FSW - RPT - shoud be deprecated
	// uint16 RPT_BootCount;
    // uint32 RPT_ScTimeSec;
    // uint32 RPT_ScTimeSubsec;
    // uint32 RPT_Sequence;
    // uint8 RPT_ResetCause;

    /* FSW - RPT - Revised (Kweon Hyeokjin) */
    uint8_t RPT_CmdCounter;
    uint8_t RPT_ErrCounter;
    uint8_t RPT_ReportCnt;
    uint8_t RPT_CriticalCnt;
    uint16_t RPT_BootCount;
    uint32_t RPT_ScTimeSec;
    uint32_t RPT_ScTimeSubsec;
    uint8_t RPT_Sequence_LSB;
    /*--------------End of BEE RPT Revision (Kweon Hyeokjin)--------------*/

	// COMS - STX
    // all param
    uint8_t  STX_symbol_rate;
    uint8_t  STX_transmit_power;
    uint8_t  STX_modcod;
    uint8_t  STX_roll_off;
    uint8_t  STX_pilot_signal;
    uint8_t  STX_fec_frame_size;
    uint16_t STX_pretransmission_delay;
    float    STX_center_frequency;
    // modulation interface -> cpu temp 빠짐
    uint8_t STX_modulator_interface_type;  
    uint8_t STX_lvds_io_type;    
	uint8 STX_SystemState;
    uint8 STX_StatusFlag;
    // float STX_CpuTemp;

	// COMS - UANT
    uint8 UANT1_Chan0;
    uint8 UANT1_Chan1;
    uint8 UANT1_BackupActive;
    uint8 UANT2_Chan0;
    uint8 UANT2_Chan1;
    uint8 UANT2_BackupActive;

	// COMS - UTRX
    uint8 UTRX_ActiveConf;
    uint16 UTRX_BootCount;
    uint32 UTRX_BootCause;
    int16 UTRX_BoardTemp;

    // PCDU - P60 Dock
    int16 P60D_Cout[9];
    uint16 P60D_Vout[9];
    uint16 P60D_OutEn;
    uint32 P60D_BootCause;
    uint32 P60D_BootCount;
    uint8 P60D_BattMode;
    uint8 P60D_HeaterOn;
    uint16 P60D_VbatV;
    int16 P60D_VccC;
    uint16 P60D_BattV;
    int16 P60D_BattTemp[2];
    uint32 P60D_WdtCanLeft;
    int16 P60D_BattChrg;
    int16 P60D_BattDischrg;

    // PCDU - P60 PDU
    int16  P60P_Cout[9];
    uint16 P60P_Vout[9];
    int16  P60P_Vcc;
    uint8  P60P_ConvEn;
    uint16 P60P_OutEn;

    // PCDU - P60 ACU
    int16  P60A_Cin[6];
    uint16 P60A_Vin[6];

    // ADCS
       /** Combined Power State
     *  | 7 |  6 |  5 |  4 |  3 |  2 |  1 |  0 |
     *  +--------------------------------------+
     *  |Rsv|RWL0|RWL1|RWL2|MAG0|GYRO|FSS0|HSS0|
     *  +--------------------------------------+
     */
    uint8 ADCS_PowerState; // ID 183

    uint8 ADCS_ControlMode; // ID 185

    float ADCS_GYR0CalibratedRateXComponent;
    float ADCS_GYR0CalibratedRateYComponent;
    float ADCS_GYR0CalibratedRateZComponent; // ID 207, 12bytes

}__attribute__((packed)) Beacon;

#define a sizeof(Beacon);

struct GETFILEINFO {
    // ===== TLM Header =====

    uint8 CCSDS_MID[2];
    uint8 CCSDS_Seq[2];
    uint8 CCSDS_Len[2];
    uint8 CCSDS_TimeCode[6];
    uint8 padding[4];

    // ===== Payload =====
    uint8 FileStatus;
    uint8 CRC_Computed;
    uint8 Spare[2];
    uint32 CRC;
    uint32 FileSize;
    uint32 LastModifiedTime;
    uint32 Mode;
    char Filename[64];

};

#define a sizeof(GETFILEINFO);


struct Report {
    // ===== CCSDS Header =====
    uint16 CCSDS_MsgId;
    uint16 CCSDS_Seq;
    uint16 CCSDS_Len;
    uint8 CCSDS_TimeCode[6];
    uint32 CCSDS_Padding;


    // ===== Report Body =====
    uint16 ReflectedMID;
    uint8  ReflectedCC;
    uint8  RetType;
    int32  RetCode;
    uint16 RetValSize;
    uint8  RetVal[126];

};

#define a sizeof(Report)

typedef enum
{
    REPORT_KIND_NONE = 0,

    // ADCS Sunpointing RPT
    REPORT_KIND_ADCS_LOG_MASK,
    REPORT_KIND_ADCS_UNSOLICIT_TLM_SETUP_TLM,

    REPORT_KIND_UANT_GET_STATUS_TLM,

    REPORT_KIND_SC_GENERIC,
    REPORT_KIND_SC_ATS_STATUS,
    REPORT_KIND_SC_RTS_STATUS
} ReportKind_t;

typedef struct
{
    uint8_t bytes[126];
} RptGenericPayload_t;

typedef struct
{
    uint16_t AtsId;
    uint16_t State;
} RptAtsStatusPayload_t;

typedef struct
{
    uint16_t RtsId;
    uint16_t State;
} RptRtsStatusPayload_t;

typedef struct __attribute__((__packed__)) gs_gssb_ant6_release_status_t {
    /**
       Burn state of the first channel (Burning = 1, Idle = 0)
     */
    uint8_t channel_0_state;
    /**
       Release status of the first channel (Released = 1, Not released = 0)
     */
    uint8_t channel_0_status;
    /**
       Burn time left of the first channel [s]
     */
    uint8_t channel_0_burn_time_left;
    /**
       Counter of have many burns there has been attempted
     */
    uint8_t channel_0_burn_tries;
    /**
       Burn state of the second channel (Burning = 1, Idle = 0)
     */
    uint8_t channel_1_state;
    /**
       Release status of the second channel (Released = 1, Not released = 0)
     */
    uint8_t channel_1_status;
    /**
       Burn time left of the second channel [s]
     */
    uint8_t channel_1_burn_time_left;
    /**
       Counter of have many burns there has been attempted
     */
    uint8_t channel_1_burn_tries;
} gs_gssb_ant6_release_status_t;


typedef struct
{
    bool     valid;
    uint16_t CCMessage_ID;
    uint16_t CCCount;
    uint16_t CCLength;
    uint8_t  CCTime_code[6];

    uint16_t msg_id;
    uint8_t  cc;
    uint8_t  ret_type;
    int32_t  ret_code;
    uint16_t ret_val_size;

    ReportKind_t kind;

    union
    {
        RptGenericPayload_t    generic;
        RptAtsStatusPayload_t  ats_status;
        RptRtsStatusPayload_t  rts_status;




        ADCS_TlmLogInclMaskTlm_Payload_t       adcs_logmask;
        ADCS_UnsolicitTlmMsgSetupTlm_Payload_t adcs_unsolicited_tlm_tlm;

        gs_gssb_ant6_release_status_t          uant_getstatus;
        


    } u;
} ReportView_t;

static ReportView_t g_report_view;


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
uint16_t get_boot_count();
uint16_t * get_boot_count_address();
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