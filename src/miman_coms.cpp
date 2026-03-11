/* Customs */
#include "miman_config.h"
#include "miman_csp.h"
#include "miman_coms.h"
#include "miman_imgui.h"
#include "miman_config.h"
#include "miman_orbital.h"
#include "miman_radial.h"
#include "miman_ftp.h"
#include <netinet/in.h>

#define _CRT_SECURE_NO_WARNINGS

extern FILE *log_ptr;
extern bool raw_data;

//int signallen = 8;
Beacon* beacon = (Beacon *)malloc(MIM_LEN_BEACON);
Report* report = (Report *)malloc(BEE_LEN_REPORT);
GETFILEINFO* getfileinfo = (GETFILEINFO *)malloc(BEE_LEN_GETFILEINFO);
CFE_EVS_LongEventTlm_Payload_t *cFS_Event = (CFE_EVS_LongEventTlm_Payload_t *)calloc(1, sizeof(CFE_EVS_LongEventTlm_Payload_t));
int NowFTP = 0;

char HKbuf[202];
int HKbufCursor = 0;
char AODbuf[131];
int AODbufCufsor = 0;
CmdGenerator_GS * SatCMD[256];
extern StateCheckUnit State;
extern pthread_t p_thread[16];
extern pthread_mutex_t conn_lock;

// HK *NowHK;
// AOD *NowAOD;

extern Console console;
extern Setup * setup;

pthread_t LinkTrhead;

int BeaconCounter;
int ReportCounter;
int PingCounter;
uint32_t remote_total_rx_bytes = 0;
uint16_t remote_boot_count = 0;

void * TRxController(void *)
{
    csp_socket_t * DLsocket = DL_sock_initialize();
    while(State.TRx_mode)
    {
        if(State.downlink_mode)
        {
            printf("Create downlink onorbit...\n");
            pthread_create(&LinkTrhead, NULL, task_downlink_onorbit, DLsocket);
            pthread_join(LinkTrhead, NULL);
            State.uplink_mode = true;
        }
        else
            continue;
    }
}

void buf_allclear()
{
    memset(HKbuf, 0, sizeof(HKbuf));
    memset(AODbuf, 0, sizeof(HKbuf));
    HKbufCursor = 0;
    //AODBufCursor = 0;
}

void * SignalTest(void*)
{
    State.Signaltest = true;
    while(State.Signaltest)
    {
        if(csp_ping(19, 100, 1, 0))
            continue;
    }
}

void now_rx_bytes_update()
{
    // if(beacon->UXTotRXByte != remote_total_rx_bytes)
    //     remote_total_rx_bytes = beacon->UXTotRXByte;
}

void set_rx_bytes(uint32_t nowbytes)
{
    remote_total_rx_bytes = nowbytes;
}

uint32_t get_rx_bytes()
{
    return remote_total_rx_bytes;
}

uint16_t get_boot_count()
{
    return remote_boot_count;
}

uint32_t * get_rx_bytes_address()
{
    return &remote_total_rx_bytes;
}

uint16_t * get_boot_count_address()
{
    return &remote_boot_count;
}

CmdGenerator_GS::CmdGenerator_GS(void) {
    uint8_t* pool = new uint8_t[MIM_DEFAULT_DATALEN];
    if (!pool) {
        console.AddLog("Generator init error: alloc failed");
        return;
    }
    memset(pool, 0, MIM_DEFAULT_DATALEN);
    this->CmdHeader = (CFE_MSG_CommandHeader*) pool;
}

CmdGenerator_GS::~CmdGenerator_GS(void) {
    uint8_t* pool = (uint8_t*) this->CmdHeader;
    delete[] CmdHeader;
}

int CmdGenerator_GS::GenerateCmdHeader(uint32_t MsgId, uint16_t FncCode, uint32_t Size, void* Data) {
    memset(this->CmdHeader->Msg.Byte, 0, Size);

    if (this->SetHasSecondaryHeader(true) != 0) {
        return -1;
    }
    if (this->SetSegmentationFlag(CFE_MSG_SEGFLG_UNSEG) != 0) {
        return -1;
    }
    if (this->SetMsgId(MsgId) != 0) {
        return -1;
    }
    if (this->SetSize(Size) != 0) {
        return -1;
    }
    if (this->SetFncCode(FncCode) != 0) {
        return -1;
    }
    if (this->GenerateChecksum() != 0) {
        return -1;
    }
    if (Data && (Size - sizeof(CFE_MSG_CommandHeader)) > 0)
        memcpy(this->CmdHeader->Msg.Byte + sizeof(CFE_MSG_CommandHeader), Data, Size - sizeof(CFE_MSG_CommandHeader));
    return 0;
}

void CmdGenerator_GS::CopyCmdHeaderToBuffer(uint8_t* Buffer) {
    memcpy(Buffer, this->CmdHeader->Msg.Byte, this->GetSize());
}

int CmdGenerator_GS::SetFncCode(uint16_t FncCode) {
    if (!this->HasSecondaryHeader()) {
        return -1;
    }
    this->CmdHeader->Sec.FunctionCode = FncCode;
    return 0;
}


void CmdGenerator_GS::SetHeaderWord(uint8_t* Word16, uint16_t Value, uint16_t Mask) {
    Word16[0] = (Word16[0] & ~(Mask >> 8)) | ((Value & Mask) >> 8);
    Word16[1] = ((Word16[1] & ~Mask) | (Value & Mask)) & 0xFF;
}


void CmdGenerator_GS::GetHeaderWord(const uint8_t* ByteField, uint16_t& Value, uint16_t Mask) {
    Value = (ByteField[0] << 8 | ByteField[1]) & Mask;
}

void CmdGenerator_GS::SetHeaderByte(uint8_t* Byte, uint8_t Value, uint8_t Mask) {
    Byte[0] = (Byte[0] & ~Mask) | (Value & Mask);
}

int CmdGenerator_GS::SetHasSecondaryHeader(bool HasSec) {
    SetHeaderByte(&this->CmdHeader->Msg.CCSDS.Pri.StreamId[0], HasSec, 0x08);
    return 0;
}

bool CmdGenerator_GS::HasSecondaryHeader(void) const {
    return (this->CmdHeader->Msg.CCSDS.Pri.StreamId[0] & (CFE_MSG_SHDR_MASK >> 8)) != 0;
}


int CmdGenerator_GS::SetSize(uint16_t Size) {
    if (Size < CFE_MSG_SIZE_OFFSET || Size > (0xFFFF + CFE_MSG_SIZE_OFFSET)) {
        return -1;
    }
    Size -= CFE_MSG_SIZE_OFFSET;
    SetHeaderWord(this->CmdHeader->Msg.CCSDS.Pri.Length, Size, 0xFFFF);
    return 0;
}

uint16_t CmdGenerator_GS::GetSize(void) {
    uint16_t RetVal;
    GetHeaderWord(this->CmdHeader->Msg.CCSDS.Pri.Length, RetVal, 0xFFFF);
    return RetVal + CFE_MSG_SIZE_OFFSET;
}

uint16_t CmdGenerator_GS::GetFncCode(void) const
{
    return this->CmdHeader->Sec.FunctionCode;
}

int CmdGenerator_GS::SetMsgId(uint16_t MsgId) {
    SetHeaderWord(this->CmdHeader->Msg.CCSDS.Pri.StreamId, MsgId, 0xFFFF);
    return 0;
}

int CmdGenerator_GS::SetSegmentationFlag(uint16_t SegFlag) {
    SetHeaderWord(this->CmdHeader->Msg.CCSDS.Pri.Sequence, SegFlag, CFE_MSG_SEGFLG_MASK);
    return 0;
}


int CmdGenerator_GS::GenerateChecksum(void) {
    CFE_MSG_CommandHeader* Cmd = this->CmdHeader;
    Cmd->Sec.Checksum = 0;
    Cmd->Sec.Checksum = this->ComputeCheckSum();
    return 0;
}

uint32_t CmdGenerator_GS::ComputeCheckSum(void) {
    uint16_t Len = this->GetSize();
    const uint8_t* BytePtr = this->CmdHeader->Msg.Byte;
    uint32_t chksum  = 0xFF;

    while (Len--) {
        chksum ^= *(BytePtr++);
    }
    return chksum;
}


csp_socket_t * DL_sock_initialize()
{
	csp_socket_t * sock = csp_socket(0);
	if(!csp_bind(sock, 13)) {
        console.AddLog("[OK]##TM Port 13 bind success.");
    }
    if(!csp_bind(sock, 31)) {
        console.AddLog("[OK]##BCN Port 31 bind success.");
    }
    if(!csp_bind(sock, 25)) {
        console.AddLog("[OK]##RPT Port 25 bind success.");
    }
    // while(true) {
    //     if (csp_bind(sock, 23) == 0) { // Add for HVD_TMTC_TEST
    //     console.AddLog("[OK]Bind Success.");
    //     break;
    //     };
    // }
	csp_listen(sock, 10);
    //Fail preventation would be needed!
    return sock;
}

// void * PacketDebugMsg(csp_packet_t * packet)
// {
//     char DebugMsg[1024];
//     (char *)packet->data
//     for(int i = 0; i < packet->length; i++)
//     {
//         sprintf(DebugMsg + i, "%")
//     }
// }
static void DecodeReport(uint16_t CCMessage_ID,
                         uint16_t CCCount,
                         uint16_t CCLength,
                         const uint8_t CCTime_code[6],
                         uint16_t msg_id,
                         uint8_t cc,
                         uint8_t ret_type,
                         int32_t ret_code,
                         uint16_t ret_val_size,
                         const std::vector<uint8_t> &payload)
{
    memset(&g_report_view, 0, sizeof(g_report_view));

    g_report_view.valid        = true;
    g_report_view.CCMessage_ID = CCMessage_ID;
    g_report_view.CCCount      = CCCount;
    g_report_view.CCLength     = CCLength;

    memcpy(g_report_view.CCTime_code, CCTime_code, 6);

    g_report_view.msg_id       = msg_id;
    g_report_view.cc           = cc;
    g_report_view.ret_type     = ret_type;
    g_report_view.ret_code     = ret_code;
    g_report_view.ret_val_size = ret_val_size;

    const uint8_t *p = payload.data();

    switch (CCMessage_ID)
    {
    case 0x0825:  
        switch (msg_id)
        {
        case ADCS_CMD_ID:     
            switch (cc)
            {
            case ADCS_GET_TLM_LOG_INCLMASK_CC:  
                g_report_view.kind = REPORT_KIND_ADCS_LOG_MASK;

                if (ret_val_size >= sizeof(ADCS_TlmLogInclMaskTlm_Payload_t))
                {
                    memcpy(&g_report_view.u.adcs_logmask,
                           p,
                           sizeof(ADCS_TlmLogInclMaskTlm_Payload_t));
                }
                else
                {
                    memset(&g_report_view.u.adcs_logmask, 0,
                           sizeof(ADCS_TlmLogInclMaskTlm_Payload_t));
                    memcpy(&g_report_view.u.adcs_logmask, p, ret_val_size);
                }
                break;

            case ADCS_SET_UNSOLICIT_TLM_MSG_SETUP_CC:
            {
                g_report_view.kind = REPORT_KIND_ADCS_UNSOLICIT_TLM_SETUP_TLM;
                size_t copy_sz = ret_val_size;
                if (copy_sz > sizeof(ADCS_UnsolicitTlmMsgSetupTlm_Payload_t))
                    copy_sz = sizeof(ADCS_UnsolicitTlmMsgSetupTlm_Payload_t);
                memset(&g_report_view.u.adcs_unsolicited_tlm_tlm, 0,
                       sizeof(ADCS_UnsolicitTlmMsgSetupTlm_Payload_t));
                memcpy(&g_report_view.u.adcs_unsolicited_tlm_tlm, p, copy_sz);
            }



            default:
                g_report_view.kind = REPORT_KIND_SC_GENERIC;
                memcpy(g_report_view.u.generic.bytes, p, ret_val_size);
                break;
            }
            break;

        default:
            g_report_view.kind = REPORT_KIND_SC_GENERIC;
            memcpy(g_report_view.u.generic.bytes, p, ret_val_size);
            break;
        }
        break;

    default:
        g_report_view.kind = REPORT_KIND_SC_GENERIC;
        memcpy(g_report_view.u.generic.bytes, p, ret_val_size);
        break;
    }
}



int BeaconSaver(Beacon* bec)
{
    if (!bec) return -1;

    BeaconCounter++;

    char filename[128];
    time_t tmtime = time(0);
    struct tm* local = localtime(&tmtime);

    sprintf(filename,
            "../data/beacon/Beacon--%04d-%02d-%02d-%02d-%02d-%02d--.txt",
            local->tm_year + 1900, local->tm_mon + 1, local->tm_mday,
            local->tm_hour, local->tm_min, local->tm_sec);

    FILE* fp = fopen(filename, "w");
    if (!fp) return -2;

    fprintf(fp, "================= BEACON SAVE =================\n");

    // ---------------------------------------------------
    //  CCSDS HEADER
    // ---------------------------------------------------
    fprintf(fp, "\n[CCSDS HEADER]\n");
    fprintf(fp, "MID        : %02X %02X\n", bec->CCSDS_MID[0], bec->CCSDS_MID[1]);
    fprintf(fp, "SEQ        : %02X %02X\n", bec->CCSDS_Seq[0], bec->CCSDS_Seq[1]);
    fprintf(fp, "LEN        : %02X %02X\n", bec->CCSDS_Len[0], bec->CCSDS_Len[1]);
    fprintf(fp, "TimeCode   : %02X %02X %02X %02X %02X %02X\n",
            bec->CCSDS_TimeCode[0], bec->CCSDS_TimeCode[1], bec->CCSDS_TimeCode[2],
            bec->CCSDS_TimeCode[3], bec->CCSDS_TimeCode[4], bec->CCSDS_TimeCode[5]);

    // // ---------------------------------------------------
    // //  RPT (FSW) - Should be deprecated
    // // ---------------------------------------------------
    // fprintf(fp, "\n[FSW - RPT]\n");
    // fprintf(fp, "BootCount     : %" PRIu16 "\n", bec->RPT_BootCount);
    // fprintf(fp, "SC Time Sec   : %" PRIu32 "\n", bec->RPT_ScTimeSec);
    // fprintf(fp, "SC Time Sub   : %" PRIu32 "\n", bec->RPT_ScTimeSubsec);
    // fprintf(fp, "Sequence      : %" PRIu32 "\n", bec->RPT_Sequence);
    // fprintf(fp, "Reset Cause   : %" PRIu8  "\n", bec->RPT_ResetCause);
    /********************************************************************/
    /*               BEE RPT Revision (Kweon Hyeokjin)                  */
    /********************************************************************/
    fprintf(fp, "\n[FSW - RPT]\n");
    fprintf(fp, "Cmd Counter   : %" PRIu8 "\n", bec->RPT_CmdCounter);
    fprintf(fp, "Err Counter   : %" PRIu8 "\n", bec->RPT_ErrCounter);
    fprintf(fp, "Report Q Cnt  : %" PRIu8 "\n", bec->RPT_ReportCnt);
    fprintf(fp, "Critical Q Cnt: %" PRIu8 "\n", bec->RPT_CriticalCnt);
    fprintf(fp, "Boot Count    : %" PRIu16 "\n", bec->RPT_BootCount);
    fprintf(fp, "SC Time Sec   : %" PRIu32 "\n", bec->RPT_ScTimeSec);
    fprintf(fp, "SC Time Sub   : %" PRIu32 "\n", bec->RPT_ScTimeSubsec);
    fprintf(fp, "Sequence(LSB) : %" PRIu8  "\n", bec->RPT_Sequence_LSB);
    /*--------------End of BEE RPT Revision (Kweon Hyeokjin)--------------*/

    // ---------------------------------------------------
    // STX (S-band)
    // ---------------------------------------------------
    fprintf(fp, "\n[COMS - STX]\n");
    fprintf(fp, "Symbol Rate        : %" PRIu8 "\n", bec->STX_symbol_rate);
    fprintf(fp, "Tx Power           : %" PRIu8 "\n", bec->STX_transmit_power);
    fprintf(fp, "MODCOD             : %" PRIu8 "\n", bec->STX_modcod);
    fprintf(fp, "Roll-off           : %" PRIu8 "\n", bec->STX_roll_off);
    fprintf(fp, "Pilot Signal       : %" PRIu8 "\n", bec->STX_pilot_signal);
    fprintf(fp, "FEC Frame Size     : %" PRIu8 "\n", bec->STX_fec_frame_size);
    fprintf(fp, "Pre-Tx Delay       : %" PRIu16 "\n", bec->STX_pretransmission_delay);
    fprintf(fp, "Center Frequency   : %f\n", bec->STX_center_frequency);
    fprintf(fp, "Mod Interface Type : %" PRIu8 "\n", bec->STX_modulator_interface_type);
    fprintf(fp, "LVDS IO Type       : %" PRIu8 "\n", bec->STX_lvds_io_type);
    fprintf(fp, "System State       : %" PRIu8 "\n", bec->STX_SystemState);
    fprintf(fp, "Status Flag        : %" PRIu8 "\n", bec->STX_StatusFlag);
    // fprintf(fp, "CPU Temp           : %f\n", bec->STX_CpuTemp);

    // ---------------------------------------------------
    // UANT (UHF Antenna)
    // ---------------------------------------------------
    fprintf(fp, "\n[COMS - UANT]\n");
    fprintf(fp, "UANT1 (0/1/BK) : %" PRIu8 " %" PRIu8 " (BK:%" PRIu8 ")\n",
            bec->UANT1_Chan0, bec->UANT1_Chan1, bec->UANT1_BackupActive);
    fprintf(fp, "UANT2 (0/1/BK) : %" PRIu8 " %" PRIu8 " (BK:%" PRIu8 ")\n",
            bec->UANT2_Chan0, bec->UANT2_Chan1, bec->UANT2_BackupActive);

    // ---------------------------------------------------
    // UTRX
    // ---------------------------------------------------
    fprintf(fp, "\n[COMS - UTRX]\n");
    fprintf(fp, "UTRX ActiveConf : %" PRIu8 "\n", bec->UTRX_ActiveConf);
    fprintf(fp, "UTRX BootCount  : %" PRIu16 "\n", bec->UTRX_BootCount);
    fprintf(fp, "UTRX BootCause  : %" PRIu32 "\n", bec->UTRX_BootCause);
    fprintf(fp, "UTRX Temp       : %" PRId16 "\n", bec->UTRX_BoardTemp);

    // ---------------------------------------------------
    // P60 Dock
    // ---------------------------------------------------
    fprintf(fp, "\n[PCDU - P60 DOCK]\n");
    fprintf(fp, "Cout[0..8]    : ");
    for (int i = 0; i < 9; i++) fprintf(fp, "%d ", bec->P60D_Cout[i]);
    fprintf(fp, "\n");

    fprintf(fp, "Vout[0..8]    : ");
    for (int i = 0; i < 9; i++) fprintf(fp, "%u ", bec->P60D_Vout[i]);
    fprintf(fp, "\n");

    fprintf(fp, "OutEn         : 0x%04X\n", bec->P60D_OutEn);
    fprintf(fp, "BootCause     : %" PRIu32 "\n", bec->P60D_BootCause);
    fprintf(fp, "BootCount     : %" PRIu32 "\n", bec->P60D_BootCount);
    fprintf(fp, "BattMode      : %" PRIu8  "\n", bec->P60D_BattMode);
    fprintf(fp, "HeaterOn      : %" PRIu8  "\n", bec->P60D_HeaterOn);
    fprintf(fp, "VBAT          : %" PRIu16 "\n", bec->P60D_VbatV);
    fprintf(fp, "VCC Current   : %" PRIi16 "\n", bec->P60D_VccC);
    fprintf(fp, "BattV         : %" PRIu16 "\n", bec->P60D_BattV);
    fprintf(fp, "BattTemp      : %d %d\n", bec->P60D_BattTemp[0], bec->P60D_BattTemp[1]);
    fprintf(fp, "WDT CAN Left  : %" PRIu32 "\n", bec->P60D_WdtCanLeft);
    fprintf(fp, "Batt Chg Curr : %" PRId16 "\n", bec->P60D_BattChrg);
    fprintf(fp, "Batt Dis Curr : %" PRId16 "\n", bec->P60D_BattDischrg);

    // ---------------------------------------------------
    // P60 PDU
    // ---------------------------------------------------
    fprintf(fp, "\n[PCDU - P60 PDU]\n");
    fprintf(fp, "Cout : ");
    for (int i = 0; i < 9; i++) fprintf(fp, "%d ", bec->P60P_Cout[i]);
    fprintf(fp, "\n");

    fprintf(fp, "Vout : ");
    for (int i = 0; i < 9; i++) fprintf(fp, "%u ", bec->P60P_Vout[i]);
    fprintf(fp, "\n");

    fprintf(fp, "Vcc     : %d\n", bec->P60P_Vcc);
    fprintf(fp, "ConvEn  : %" PRIu8 "\n", bec->P60P_ConvEn);
    fprintf(fp, "OutEn   : 0x%04X\n", bec->P60P_OutEn);

    // ---------------------------------------------------
    // P60 ACU
    // ---------------------------------------------------
    fprintf(fp, "\n[PCDU - P60 ACU]\n");
    fprintf(fp, "Cin : ");
    for (int i = 0; i < 6; i++) fprintf(fp, "%d ", bec->P60A_Cin[i]);
    fprintf(fp, "\n");

    fprintf(fp, "Vin : ");
    for (int i = 0; i < 6; i++) fprintf(fp, "%u ", bec->P60A_Vin[i]);
    fprintf(fp, "\n");

    // ---------------------------------------------------
    // ADCS
    // ---------------------------------------------------
    fprintf(fp, "\n[ADCS]\n");
    fprintf(fp, "PowerState       : 0x%02X\n", bec->ADCS_PowerState);
    fprintf(fp, "ControlMode      : %" PRIu8 "\n", bec->ADCS_ControlMode);
    fprintf(fp, "GYR0 Calib X     : %f\n", bec->ADCS_GYR0CalibratedRateXComponent);
    fprintf(fp, "GYR0 Calib Y     : %f\n", bec->ADCS_GYR0CalibratedRateYComponent);
    fprintf(fp, "GYR0 Calib Z     : %f\n", bec->ADCS_GYR0CalibratedRateZComponent);

    // ---------------------------------------------------
    // BINARY DUMP
    // ---------------------------------------------------
    fprintf(fp, "\n[BINARY DATA]\n");
    for (size_t i = 0; i < sizeof(*bec); i++)
        fprintf(fp, "%02X ", ((unsigned char*)bec)[i]);
    fprintf(fp, "\n");

    fclose(fp);
    return 0;
}
int ReportSaver(Report* rpt)
{
    if (!rpt) return -1;

    ReportCounter++;

    char filename[128];
    time_t tmtime = time(0);
    struct tm* local = localtime(&tmtime);

    sprintf(filename,
            "../data/report_parsed/Report--%04d-%02d-%02d-%02d-%02d-%02d--.txt",
            local->tm_year + 1900, local->tm_mon + 1, local->tm_mday,
            local->tm_hour, local->tm_min, local->tm_sec);

    FILE* fp = fopen(filename, "w");
    if (!fp) return -2;

    fprintf(fp, "================= REPORT SAVE =================\n");

    fprintf(fp, "\n[CCSDS HEADER]\n");
    fprintf(fp, "MsgId      : 0x%04X\n", (unsigned int)rpt->CCSDS_MsgId);
    fprintf(fp, "Seq        : 0x%04X\n", (unsigned int)rpt->CCSDS_Seq);
    fprintf(fp, "Len        : 0x%04X\n", (unsigned int)rpt->CCSDS_Len);
    fprintf(fp, "TimeCode   : %02X %02X %02X %02X %02X %02X\n",
            rpt->CCSDS_TimeCode[0], rpt->CCSDS_TimeCode[1],
            rpt->CCSDS_TimeCode[2], rpt->CCSDS_TimeCode[3],
            rpt->CCSDS_TimeCode[4], rpt->CCSDS_TimeCode[5]);
    fprintf(fp, "Padding    : 0x%08X\n", (unsigned int)rpt->CCSDS_Padding);

    fprintf(fp, "\n[REPORT BODY]\n");
    fprintf(fp, "Reflected MID  : 0x%04X\n", (unsigned int)rpt->ReflectedMID);
    fprintf(fp, "Reflected CC   : 0x%02X\n", (unsigned int)rpt->ReflectedCC);
    fprintf(fp, "RetType        : 0x%02X\n", (unsigned int)rpt->RetType);
    fprintf(fp, "RetCode        : %" PRId32 "\n", (int32_t)rpt->RetCode);
    fprintf(fp, "RetValSize     : %" PRIu16 "\n", (uint16_t)rpt->RetValSize);

    fprintf(fp, "\n[RETURN VALUE]\n");
    uint16_t dump_size = rpt->RetValSize;
    if (dump_size > sizeof(rpt->RetVal))
        dump_size = sizeof(rpt->RetVal);

    for (uint16_t i = 0; i < dump_size; i++) {
        fprintf(fp, "%02X ", rpt->RetVal[i]);
        if ((i + 1) % 16 == 0)
            fprintf(fp, "\n");
    }
    if (dump_size % 16 != 0)
        fprintf(fp, "\n");

    fprintf(fp, "\n[BINARY DATA]\n");
    for (size_t i = 0; i < sizeof(*rpt); i++)
        fprintf(fp, "%02X ", ((unsigned char*)rpt)[i]);
    fprintf(fp, "\n");

    fclose(fp);
    return 0;
}
void * task_downlink_onorbit(void * socketinfo) 
{
    csp_socket_t * sock = (csp_socket_t *)socketinfo;

    // csp_bind(sock, CSP_PING);
	// csp_bind(sock, 12);
    // csp_bind(sock, 31);
	// csp_listen(sock, 10);
    csp_packet_t * packet = NULL;
    packetsign * confirm = (packetsign *)malloc(MIM_LEN_PACKET);
    csp_conn_t * conn;

    //////////////////////////////////////////////////////////////////////////////
    bool image_packet_received = false;

    std::string filename = "/home/miman/Downloads/FTP_TEST.jpg";
    std::vector <uint8_t> image_packet_data(14956);
    std::vector <int> received_index;
    int index;
    int count;
    std::ofstream fout;

    std::map <uint16_t, std::map<uint16_t, std::map<uint8_t, std::vector<uint8_t>>>> rpt_map;
    /////////////////////////////////////////////////////////////////////////



    //Need to copy pointer
    //This function must be on p_thread[3]
    float seconds = 0.0f;
	while (State.downlink_mode) {
        //printf("Downlink ongoing...\n");
		if ((conn = csp_accept(sock, setup->default_timeout)) == NULL)
        {
            // printf("Running...but no comming...%f\n", seconds);
            seconds += 0.5;
            continue;
        }
            
        //console.AddLog("Someone Comming...");
		while ((packet = csp_read(conn, setup->default_timeout)) != NULL) {
			switch(csp_conn_dport(conn)) {
                //For RPT : Port 25
                case 25 : {
                    std::vector <uint8_t> rpt_data;
                    size_t packet_length = packet->length;
                    rpt_data.resize(packet_length);
                    memcpy(rpt_data.data(), packet->data, packet_length);

                    if (packet_length < 26){
                        printf("invalid packet length : %d \n", (int)packet_length);
                        break;
                    }
                    printf("25 port repotty\n");
                    size_t ccsds_length = 16;

                    uint16_t CCMessage_ID = (rpt_data[0] << 8) | rpt_data[1];
                    uint8_t CCFlag = rpt_data[2] >> 6;
                    uint16_t CCCount = ((rpt_data[2] & 0x3F) << 8) | rpt_data[3];
                    uint16_t CCLength = rpt_data[4] << 8 | rpt_data[5];
                    std::vector <uint8_t> CCTime_code(rpt_data.begin()+6, rpt_data.begin()+12);

                    uint16_t msg_id = (rpt_data[17] <<8) | rpt_data[16];
                    uint8_t cc = rpt_data[18];
                    uint8_t ret_type = rpt_data[19];
                    int32_t ret_code = (rpt_data[23] << 24) | (rpt_data[22] << 16) | (rpt_data[21] << 8) | rpt_data[20];
                    uint16_t ret_val_size = (rpt_data[25] << 8) | rpt_data[24];

                    rpt_map[CCMessage_ID][msg_id][cc].insert(rpt_map[CCMessage_ID][msg_id][cc].end(), rpt_data.begin()+26, rpt_data.end());

                    if ((CCFlag == 0x01) || (CCFlag == 0x03)){
                        g_last_report.valid = true;
                        g_last_report.CCMessage_ID = CCMessage_ID;
                        g_last_report.CCCount = CCCount;
                        g_last_report.CCLength = CCLength;

                        for (int i=0;i<6;i++)
                            g_last_report.CCTime_code[i] = CCTime_code[i];

                        g_last_report.msg_id = msg_id;
                        g_last_report.cc = cc;
                        g_last_report.ret_type = ret_type;
                        g_last_report.ret_code = ret_code;
                        g_last_report.ret_val_size = ret_val_size;

                        g_last_report.payload = rpt_map[CCMessage_ID][msg_id][cc];
                    

                        printf("--------------------report-------------------");
                        printf("CCMessage_ID : 0x%04x \n", CCMessage_ID);
                        printf("CCCount : 0x%04x \n", CCCount);
                        printf("CCLength : 0x%04x \n", CCLength);
                        printf("CCTime_code : 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x \n", CCTime_code[0], CCTime_code[1], CCTime_code[2], CCTime_code[3], CCTime_code[4], CCTime_code[5]);
                        printf("msg_id : 0x%04x \n", msg_id);
                        printf("cc : 0x%02x \n", cc);
                        printf("ret_type : 0x%02x \n", ret_type);
                        printf("ret code : 0x%08X\n", ret_code);
                        printf("ret_val_size : 0x%04x \n", ret_val_size);
                        
                        for (size_t i = 0; i < rpt_map[CCMessage_ID][msg_id][cc].size(); i++){
                            printf("0x%02x ", rpt_map[CCMessage_ID][msg_id][cc][i]);
                        }
                        printf("\n");
                        rpt_map[CCMessage_ID][msg_id][cc].clear();

                        switch (CCMessage_ID){
                            case 0x0825 :
                                break;
                            
                            default :
                                break;
                        }
                    }

                    printf("Binary (%d B) : \n", (int)packet_length);

                    for (size_t i = 0; i < packet_length; i++){
                        printf("%02x ", rpt_data[i]);
                        if ((i+1) %10 == 0){
                            printf("\n");
                        }
                    }

                    break;






                    // test
                    if (packet->length == BEE_LEN_REPORT) {
                        char rptpktfilename[128];
                        time_t tmtime = time(0);
                        struct tm *local = localtime(&tmtime);

                        sprintf(rptpktfilename,
                                "../data/report/rpt_fromp13--%04d-%02d-%02d-%02d-%02d-%02d--",
                                local->tm_year + 1900,
                                local->tm_mon + 1,
                                local->tm_mday,
                                local->tm_hour,
                                local->tm_min,
                                local->tm_sec);

                        console.AddLog("Received Report from port : %d.", csp_conn_dport(conn));

                        FILE *rpt_fp = fopen(rptpktfilename, "wb");
                        printf("Report Length: %u", packet->length);

                        for (int i = 0; i < packet->length; i++) {
                            if (!(i % 10) && i != 0) {
                                printf("\n");
                                if (rpt_fp)
                                    fprintf(rpt_fp, "\n");
                            }
                            printf("0x%x ", packet->data[i]);
                            if (rpt_fp)
                                fprintf(rpt_fp, "%02hhx\t", packet->data[i]);
                        }

                        memset(report, 0, sizeof(*report));
                        memcpy(report, packet->data, BEE_LEN_REPORT);
                        ReportSaver(report);


                    std::vector <uint8_t> rpt_data;
                    size_t packet_length = packet->length;
                    rpt_data.resize(packet_length);
                    memcpy(rpt_data.data(), packet->data, packet_length);

                        if (packet_length < 26){
                            printf("invalid packet length : %d \n", (int)packet_length);
                            break;
                        }

                    size_t ccsds_length = 16;

                    uint16_t CCMessage_ID = (rpt_data[0] << 8) | rpt_data[1];
                    uint8_t CCFlag = rpt_data[2] >> 6;
                    uint16_t CCCount = ((rpt_data[2] & 0x3F) << 8) | rpt_data[3];
                    uint16_t CCLength = rpt_data[4] << 8 | rpt_data[5];
                    std::vector <uint8_t> CCTime_code(rpt_data.begin()+6, rpt_data.begin()+12);

                    uint16_t msg_id = (rpt_data[16] <<8) | rpt_data[17];
                    uint8_t cc = rpt_data[18];
                    uint8_t ret_type = rpt_data[19];
                    int32_t ret_code = (rpt_data[20] << 24) | (rpt_data[21] << 16) | (rpt_data[22] << 8) | rpt_data[23];
                    uint16_t ret_val_size = (rpt_data[24] << 8) | rpt_data[25];

                    rpt_map[CCMessage_ID][msg_id][cc].insert(rpt_map[CCMessage_ID][msg_id][cc].end(), rpt_data.begin()+26, rpt_data.end());

                        if ((CCFlag == 0x01) || (CCFlag == 0x03))
                        {
                            std::vector<uint8_t> full_payload = rpt_map[CCMessage_ID][msg_id][cc];

                            DecodeReport(CCMessage_ID,
                                        CCCount,
                                        CCLength,
                                        CCTime_code.data(),
                                        msg_id,
                                        cc,
                                        ret_type,
                                        ret_code,
                                        ret_val_size,
                                        full_payload);

                            rpt_map[CCMessage_ID][msg_id][cc].clear();
                        }


                            if (rpt_fp != NULL)
                                fclose(rpt_fp);
                    }




















                }

                // For TMTC Test: Port 23
                case 23: {
                    char tmtcfilename[128];
                    time_t tmtime = time(0);
                    struct tm * local = localtime(&tmtime);
                    sprintf(tmtcfilename, "../data/tmtc/tmtc_test--%04d-%02d-%02d-%02d-%02d-%02d--", local->tm_year+1900, local->tm_mon+1, local->tm_mday,local->tm_hour, local->tm_min, local->tm_sec);
                    
                    uint16_t PacketLength = packet->length;
                    memcpy(confirm, packet->data, PacketLength);
                    console.AddLog("TMTC Test Downlink requested.");
                    FILE * TMTC_fp;
                    TMTC_fp = fopen(tmtcfilename,"wb");
                    for (int i=0; i<PacketLength; i++) {
                        fprintf(TMTC_fp, "Data %d: %u\n",i,packet->data[i]);
                    }
                    if (packet != NULL)
                    {
                        csp_buffer_free(packet);
                        packet = NULL;
                    }
                    if (conn != NULL)
                    {
                        csp_close(conn);
                        conn = NULL;
                    }
                    if(TMTC_fp != NULL)
                    {
                        fclose(TMTC_fp);
                    }
                    break;
                    
                }


                case 13: {

                    // BEE Beacon을 받았어
                    if (packet->length == MIM_LEN_BEACON) {
                        char bcnpktfilename[128];
                        time_t tmtime = time(0);
                        struct tm *local = localtime(&tmtime);

                        sprintf(bcnpktfilename,
                                "../data/bcnpkt/bcnpktp13--%04d-%02d-%02d-%02d-%02d-%02d--",
                                local->tm_year + 1900,
                                local->tm_mon + 1,
                                local->tm_mday,
                                local->tm_hour,
                                local->tm_min,
                                local->tm_sec);

                        console.AddLog("Received Beacon from port : %d.", csp_conn_dport(conn));

                        FILE *bcn_fp = fopen(bcnpktfilename, "wb");
                        printf("\nBeacon Length: %u", packet->length);

                        for (int i = 0; i < packet->length; i++) {
                            if (!(i % 10) && i != 0) {
                                printf("\n");
                                if (bcn_fp)
                                    fprintf(bcn_fp, "\n");
                            }
                            printf("0x%x ", packet->data[i]);
                            if (bcn_fp)
                                fprintf(bcn_fp, "%02hhx\t", packet->data[i]);
                        }

                        memset(beacon, 0, sizeof(*beacon));
                        memcpy(beacon, packet->data, MIM_LEN_BEACON);
                        BeaconSaver(beacon);

                        if (bcn_fp != NULL)
                            fclose(bcn_fp);
                    }

                    // BEE RPT를 받았어
                    else if (packet->length == BEE_LEN_REPORT) {
                        char rptpktfilename[128];
                        time_t tmtime = time(0);
                        struct tm *local = localtime(&tmtime);

                        sprintf(rptpktfilename,
                                "../data/report/rpt_fromp13--%04d-%02d-%02d-%02d-%02d-%02d--",
                                local->tm_year + 1900,
                                local->tm_mon + 1,
                                local->tm_mday,
                                local->tm_hour,
                                local->tm_min,
                                local->tm_sec);

                        console.AddLog("Received Report from port : %d.", csp_conn_dport(conn));

                        FILE *rpt_fp = fopen(rptpktfilename, "wb");
                        printf("Report Length: %u", packet->length);

                        for (int i = 0; i < packet->length; i++) {
                            if (!(i % 10) && i != 0) {
                                printf("\n");
                                if (rpt_fp)
                                    fprintf(rpt_fp, "\n");
                            }
                            printf("0x%x ", packet->data[i]);
                            if (rpt_fp)
                                fprintf(rpt_fp, "%02hhx\t", packet->data[i]);
                        }

                        memset(report, 0, sizeof(*report));
                        memcpy(report, packet->data, BEE_LEN_REPORT);
                        ReportSaver(report);


                    std::vector <uint8_t> rpt_data;
                    size_t packet_length = packet->length;
                    rpt_data.resize(packet_length);
                    memcpy(rpt_data.data(), packet->data, packet_length);

                        if (packet_length < 26){
                            printf("invalid packet length : %d \n", (int)packet_length);
                            break;
                        }

                    size_t ccsds_length = 16;

                    uint16_t CCMessage_ID = (rpt_data[0] << 8) | rpt_data[1];
                    uint8_t CCFlag = rpt_data[2] >> 6;
                    uint16_t CCCount = ((rpt_data[2] & 0x3F) << 8) | rpt_data[3];
                    uint16_t CCLength = rpt_data[4] << 8 | rpt_data[5];
                    std::vector <uint8_t> CCTime_code(rpt_data.begin()+6, rpt_data.begin()+12);

                    uint16_t msg_id = (rpt_data[16] <<8) | rpt_data[17];
                    uint8_t cc = rpt_data[18];
                    uint8_t ret_type = rpt_data[19];
                    int32_t ret_code = (rpt_data[20] << 24) | (rpt_data[21] << 16) | (rpt_data[22] << 8) | rpt_data[23];
                    uint16_t ret_val_size = (rpt_data[24] << 8) | rpt_data[25];

                    rpt_map[CCMessage_ID][msg_id][cc].insert(rpt_map[CCMessage_ID][msg_id][cc].end(), rpt_data.begin()+26, rpt_data.end());

                        if ((CCFlag == 0x01) || (CCFlag == 0x03))
                        {
                            std::vector<uint8_t> full_payload = rpt_map[CCMessage_ID][msg_id][cc];

                            DecodeReport(CCMessage_ID,
                                        CCCount,
                                        CCLength,
                                        CCTime_code.data(),
                                        msg_id,
                                        cc,
                                        ret_type,
                                        ret_code,
                                        ret_val_size,
                                        full_payload);

                            rpt_map[CCMessage_ID][msg_id][cc].clear();
                        }


                            if (rpt_fp != NULL)
                                fclose(rpt_fp);
                    }


                    // GETFILEINFO를 받았어
                    else if (packet->length == BEE_LEN_GETFILEINFO) {
                        char getfileinfofilename[128];
                        time_t tmtime = time(0);
                        struct tm *local = localtime(&tmtime);

                        sprintf(getfileinfofilename,
                                "../data/response/GETFILEINFO--%04d-%02d-%02d-%02d-%02d-%02d--",
                                local->tm_year + 1900,
                                local->tm_mon + 1,
                                local->tm_mday,
                                local->tm_hour,
                                local->tm_min,
                                local->tm_sec);

                        console.AddLog("Received GETFILEINFO Response from port : %d.", csp_conn_dport(conn));

                        FILE *GETFILEINFO_fp = fopen(getfileinfofilename, "wb");
                        printf("Received GETFILEINFO response Length: %u", packet->length);

                        for (int i = 0; i < packet->length; i++) {
                            if (!(i % 10) && i != 0) {
                                printf("\n");
                                if (GETFILEINFO_fp)
                                    fprintf(GETFILEINFO_fp, "\n");
                            }
                            printf("0x%x ", packet->data[i]);
                            if (GETFILEINFO_fp)
                                fprintf(GETFILEINFO_fp, "%02hhx\t", packet->data[i]);
                        }

                        memset(getfileinfo, 0, sizeof(*getfileinfo));
                        memcpy(getfileinfo, packet->data, BEE_LEN_GETFILEINFO);
                    

                        if (GETFILEINFO_fp != NULL)
                            fclose(GETFILEINFO_fp);
                    }



                    // 길이 field는 안맞지만 뭐가 들어오긴 했음.
                    else {
                        console.AddLog("Received Something but brocken.");

                        char unknownfilename[128];
                        time_t tmtime = time(0);
                        struct tm *local = localtime(&tmtime);

                        sprintf(unknownfilename,
                                "../data/unknown/unknown_fromport13--%04d-%02d-%02d-%02d-%02d-%02d--",
                                local->tm_year + 1900,
                                local->tm_mon + 1,
                                local->tm_mday,
                                local->tm_hour,
                                local->tm_min,
                                local->tm_sec);

                        FILE *unk_fp = fopen(unknownfilename, "wb");

                        printf("Unknown Packet Length: %u\n", packet->length);
                        printf("===== UNKNOWN PACKET DUMP =====\n");

                        for (int i = 0; i < packet->length; i++) {

                            // 터미널 출력
                            if (!(i % 10) && i != 0)
                                printf("\n");
                            printf("0x%02X ", packet->data[i]);

                            // 파일 출력
                            if (unk_fp) {
                                if (!(i % 10) && i != 0)
                                    fprintf(unk_fp, "\n");
                                fprintf(unk_fp, "%02hhx\t", packet->data[i]);
                            }
                        }

                        printf("\n===============================\n");

                        if (unk_fp)
                            fclose(unk_fp);
                    }


                    if (packet != NULL)
                        csp_buffer_free(packet);
                    if (conn != NULL)
                        csp_close(conn);

                    break;
                }


                //Case 13 : TM Packet Downlink -> 13번으로 비콘 들어온다고 함. 일단 혹시 모르니까 31번도 비콘 받을 수 있게 설정
                case 31: {

                    char bcnpktfilename[128];
                    time_t tmtime = time(0);
                    struct tm * local = localtime(&tmtime);
                    sprintf(bcnpktfilename, "../data/bcnpkt/bcnpktp31--%04d-%02d-%02d-%02d-%02d-%02d--", local->tm_year+1900, local->tm_mon+1, local->tm_mday,local->tm_hour, local->tm_min, local->tm_sec);
                    console.AddLog("Received Beacon from port : %d.", csp_conn_dport(conn));
                    FILE * bcn_fp;
                    bcn_fp = fopen(bcnpktfilename, "wb");
                    printf("Beacon Length: %u",packet->length);
                    for(int i=0; i< packet->length; i++) {
                        if(!(i%10) && i!=0) {
                            printf("\n");
                            fprintf(bcn_fp, "\n");
                        }
                        printf("0x%x ",packet->data[i]);
                        fprintf(bcn_fp, "%02hhx\t",packet->data[i]);
                    }
                    memset(beacon, 0, sizeof(Beacon));
                    if(packet->length == MIM_LEN_BEACON)
                    {
                        console.AddLog("Received Beacon from port : %d.", csp_conn_dport(conn));
                        if(State.Debugmode)
                        {
                            // printf("Beacon Binary : \n");
                            // for(int i = 0 ; i < packet->length; i++)
                            // {
                            //     printf("%x\t", packet->data[i]);
                            // }
                            // printf("\n");
                        }
                        memcpy(beacon, packet->data, MIM_LEN_BEACON);		
                        //BeaconSaver(beacon);
                    }
                    else
                    {
                        console.AddLog("Received Beacon but brocken.");
                    }
					

                    /* Clean up */
                    if (packet != NULL)
                        csp_buffer_free(packet);
                    if (conn != NULL)
                        csp_close(conn);
                    if(bcn_fp != NULL)
                    {
                        fclose(bcn_fp);
                    }
                    break;

                    // /////////////////////////////////////////////////////////////////////////
                    
                    // if (image_packet_received == false && data[0] == 0x08 && data[1] == 0x78){
                    //     // 
                    //     count = 0;

                    //     image_packet_received = true;
                        
                    //     printf("image packet received start \n");

                    // }

                    // if (image_packet_received && data[0] == 0x08 && data[1] == 0x78){
                    //     // 

                    //     index = ((int)((data[2] & 0x3F)<<8) | (int)data[3]);    //extract index of packet
                        
                    //     //received_index.push_back(index);
                        
                    //     printf("\nindex: %d\n", index);
                        
                    //     memcpy(&image_packet_data[128 * index], &data[16], PacketLength-16);


                    //     if ((data[2] >> 6) == 0x02 || count >= 3072 || (data[2] >> 6) == 0x03){    //need better trigger that say end of transmit specially in resend case 
                    //         //extract error packet index
                    //         printf("image packet received finish \n");


                    //         fout.open(filename, std::ios::out | std::ios::binary);
                    //         if (fout.is_open()){
                    //             printf("file open success \n");
                                
                    //             fout.write(reinterpret_cast<char*>(image_packet_data.data()), image_packet_data.size() * sizeof(uint8_t));    //save data to reuse it

                    //             printf("write %d bytes \n", image_packet_data.size());
                    //             fout.close();
                    //         }

                    //         image_packet_received = false;    //reset progress
                    //     }
                    // }

                    // if (image_packet_received){
                    //     count += 1;
                    // }
                    



                    // /////////////////////////////////////////////////////////////////////////////////////

                    // // for(uint8_t i=0; i < packet->length; i++) {
                    // //     printf("0x%02X\t", data[i]);
                    // //     if (i%10 == 9) printf("\n");
                    // // }
                    // // break;
                    





                    // printf("DL Length: %u\n",packet->length);
                    // if(log_ptr == NULL) {
                    //     printf("Invalid file pointer.\n");
                    //     continue;
                    // }
                    // fprintf(log_ptr, "|| Downlink || Packet Length: %u\n",packet->length);
                    // // Parsing & write header
                    // HYVRID_TelemetryHeader_t hdr = {0,};
                    // if(packet->length == 24 || packet->length <52) { // Reply to CMD (Set command) -> Only header
                    //     fprintf(log_ptr, "Reply to CMD_MID.\n");
                    //     memcpy(&hdr, packet->data, sizeof(HYVRID_TelemetryHeader_t));
                    // }
                    // else { // Reply to OIF (Get command)
                    //     fprintf(log_ptr, "Reply to OIF_MID.\n");
                    //     memcpy(&hdr, packet->data + OIF_TLM_HDR_OFFSET, sizeof(HYVRID_TelemetryHeader_t));
                    // }
                    // fprintf(log_ptr, "[ CCSDS Header ]\n");
                    // fprintf(log_ptr, "Stream ID: %x\n", htons(hdr.Tlmhdr.pri.stream));
                    // fprintf(log_ptr, "Sequence: %x\n", htons(hdr.Tlmhdr.pri.sequence));
                    // fprintf(log_ptr, "Length: %x\n", htons(hdr.Tlmhdr.pri.length));
                    // fprintf(log_ptr, "Time Stamp: %x\t%x\t%x\t%x\t%x\t%x\n\n",
                    //         hdr.Tlmhdr.sec.Time[0],hdr.Tlmhdr.sec.Time[1],hdr.Tlmhdr.sec.Time[2],
                    //         hdr.Tlmhdr.sec.Time[3],hdr.Tlmhdr.sec.Time[4],hdr.Tlmhdr.sec.Time[5]);

                    // fprintf(log_ptr, "[ HYVRID Execution Report ]\n");
                    // fprintf(log_ptr, "RetCodeType: %x\n", hdr.Report.RetCodeType);
                    // fprintf(log_ptr, "RetCode: %x\n", hdr.Report.RetCode);
                    // fprintf(log_ptr, "MsgId: %x\t", hdr.Report.MsgId);
                    // WriteSystemName(hdr.Report.MsgId);
                    // fprintf(log_ptr, "CC: %x\n", hdr.Report.CommandCode);
                    // fprintf(log_ptr, "DataSize (Exclude header): %x\n", hdr.Report.DataSize);
                    // fprintf(log_ptr, "Used State: %x\n\n", hdr.Report.UsedState);
                    // fprintf(log_ptr, "Output Data.\n");
                    
                    // const int offset = OIF_TLM_HDR_OFFSET + sizeof(HYVRID_TelemetryHeader_t);
                    // for(int i = offset; i< packet->length; i++) {
                    //     if(!((i-offset) % 10) && (i - offset) != 0) {
                    //         printf("\n");
                    //         fprintf(log_ptr, "\n");
                    //     }
                    //     printf("0x%x ",packet->data[i]);
                    //     fprintf(log_ptr, "%02hhx\t",packet->data[i]);
                    // }
                    // fprintf(log_ptr, "\n");
                        
                    // if(raw_data) {
                    //     fprintf(log_ptr, "[ Downlink Raw Data ]\n");
                    //     for(int i = 0; i< packet->length; i++) {
                    //         if(!(i % 10) && i != 0) {
                    //             printf("\n");
                    //             fprintf(log_ptr, "\n");
                    //         }
                    //     printf("0x%x ",packet->data[i]);
                    //     fprintf(log_ptr, "%02hhx\t",packet->data[i]);
                    //     }
                    // } fprintf(log_ptr, "\n");
                    // /* Clean up */
                    // if (packet != NULL)
                    // {
                    //     csp_buffer_free(packet);
                    //     packet = NULL;
                    // }
                    // if (conn != NULL)
                    // {
                    //     csp_close(conn);
                    //     conn = NULL;
                    // }
                    // // if(log_ptr != NULL)
                    // // {
                    // //     fclose(DL_fp);
                    // // }
                    // printf("Report DL done.\n");
                    break;
                }
                
                //Case 31 : Beacon  -- port 바뀐 것 같음. 일단 13번으로 비콘 세팅, 만약 아니면 다시 바꿔야함
				case 12: {
                    char bcnpktfilename[128];
                    time_t tmtime = time(0);
                    struct tm * local = localtime(&tmtime);
                    sprintf(bcnpktfilename, "../data/bcnpkt/bcnpktp13--%04d-%02d-%02d-%02d-%02d-%02d--", local->tm_year+1900, local->tm_mon+1, local->tm_mday,local->tm_hour, local->tm_min, local->tm_sec);
                    console.AddLog("Received Beacon from port : %d.", csp_conn_dport(conn));
                    FILE * bcn_fp;
                    bcn_fp = fopen(bcnpktfilename, "wb");
                    printf("Beacon Length: %u",packet->length);
                    for(int i=0; i< packet->length; i++) {
                        if(!(i%10) && i!=0) {
                            printf("\n");
                            fprintf(bcn_fp, "\n");
                        }
                        printf("0x%x ",packet->data[i]);
                        fprintf(bcn_fp, "%02hhx\t",packet->data[i]);
                    }
                    memset(beacon, 0, sizeof(Beacon));
                    if(packet->length == MIM_LEN_BEACON)
                    {
                        console.AddLog("Received Beacon from port : %d.", csp_conn_dport(conn));
                        if(State.Debugmode)
                        {
                            // printf("Beacon Binary : \n");
                            // for(int i = 0 ; i < packet->length; i++)
                            // {
                            //     printf("%x\t", packet->data[i]);
                            // }
                            // printf("\n");
                        }
                        memcpy(beacon, packet->data, MIM_LEN_BEACON);		
                        //BeaconSaver(beacon);
                    }
                    else
                    {
                        console.AddLog("Received Beacon but brocken.");
                    }
					

                    /* Clean up */
                    if (packet != NULL)
                        csp_buffer_free(packet);
                    if (conn != NULL)
                        csp_close(conn);
                    if(bcn_fp != NULL)
                    {
                        fclose(bcn_fp);
                    }
                    break;
                   
				}
                case 27: /* EVS port */
                    console.AddLog("FSW Event received throuth port 27.");
                    if (packet->length == sizeof(CFE_MSG_TelemetryHeader) + sizeof(CFE_EVS_LongEventTlm_Payload_t)) {
                        memcpy(cFS_Event, (packet->data) + sizeof(CFE_MSG_TelemetryHeader), sizeof(CFE_EVS_LongEventTlm_Payload_t));
                    }
                    /* Clean up */
                    if (packet != NULL)
                        csp_buffer_free(packet);
                    if (conn != NULL)
                        csp_close(conn);
                    break;
                    


				default: {
                    if (csp_conn_dport(conn) == 1)
                        console.AddLog("Ping Received through port 1.");
                    else
                    {
                        console.AddLog("Packet Received on unknown port %d", csp_conn_dport(conn));
                        console.AddLog("Packet length is %d", packet->length);
                        char unknownfilename[128];
                        time_t tmtime = time(0);
                        struct tm * local = localtime(&tmtime);
                        sprintf(unknownfilename, "../data/unknown/unknown--%04d-%02d-%02d-%02d-%02d-%02d--", local->tm_year+1900, local->tm_mon+1, local->tm_mday,local->tm_hour, local->tm_min, local->tm_sec);
                        
                        uint16_t PacketLength = packet->length;
                        memcpy(confirm, packet->data, PacketLength);
                        console.AddLog("TMTC Test Downlink requested.");
                        FILE * TMTC_fp;
                        TMTC_fp = fopen(unknownfilename,"wb");
                        for (int i=0; i<PacketLength; i++) {
                            fprintf(TMTC_fp, "Data %d: %u\n",i,packet->data[i]);
                        }
                        if (packet != NULL)
                        {
                            csp_buffer_free(packet);
                            packet = NULL;
                        }
                        if (conn != NULL)
                        {
                            csp_close(conn);
                            conn = NULL;
                        }
                        if(TMTC_fp != NULL)
                        {
                            fclose(TMTC_fp);
                        }
                        break;
                    }
					csp_service_handler(conn, packet);
                    /* Clean up */
                    if (packet != NULL)
                        csp_buffer_free(packet);
                    if (conn != NULL)
                        csp_close(conn);
                    break;
				}
			}
		}
        RSSI_Monitoring();
    }
    printf("Downlink thread dead.\n");
    packet = NULL;
    conn = NULL;
    free(confirm);
    if (confirm != NULL)
        confirm = NULL;
}
void * task_uplink_onorbit(void * sign_)
{

    State.uplink_mode = true;
    //State.downlink_mode = false;
    pthread_mutex_lock(&conn_lock);
    //while(!State.RotatorReadReady)
    //    continue;
    bool dlstate = true;


    //This funcion must be on p_thread[4]
    if((dlstate))
    {
        State.downlink_mode = false;
    }
        
    while(!State.uplink_mode)
        continue;
    uint32_t start, time = 0;
    start = csp_get_ms();
    packetsign * sign;
    csp_packet_t * packet;
    csp_conn_t* txconn;
    console.AddLog("[DEBUG]## Start Uplink Task.");

    sign = (packetsign *) sign_;
    // for(int i =0; i < sign->Length; i++)
    //     printf("PacketByte : %d", sign->Data[i]);
    //packet = PacketEncoder(sign);

    

    uint16_t Ptype = sign->PacketType;
    uint32_t Plen = sign->Length;
    uint16_t filetype;
    uint16_t filestatus;

    if(Ptype == MIM_PT_DLREQ)
    {
        filetype = ((dlreqdata *)(sign->Data))->target;
        filestatus = ((dlreqdata *)(sign->Data))->filestatus;
    }
    
    
    if(State.Debugmode)
    {
        for(int i = 0 ; i < sign->Length; i++)
            printf("%x\t", sign->Data[i]);
        printf("\n");
    }
    uint16_t tx_length = (uint16_t)(Plen + MIM_HAND_DATAFDSTART + 4);
    packet = PacketEncoder(sign);
    console.AddLog("[DEBUG]##Send Packet of Ptype : %u. Check Terminal Log.", Ptype);
    

    switch(Ptype){
        //Ptype 1 : Ping
        case MIM_PT_PING : {
            csp_packet_t * confirm_ = (csp_packet_t *)csp_buffer_get(MIM_LEN_PACKET);
            
            while(State.uplink_mode)
            {
                if ((txconn = csp_connect(CSP_PRIO_HIGH, setup->obc_node, TX_PORT, MIM_DEFAULT_TIMEOUT, 0)) == NULL)
                    continue;
                else
                    break;
            }
            while (State.uplink_mode && txconn != NULL)
            {
                if(csp_send(txconn, packet, setup->default_timeout))
                {
                    packet = NULL;
                    break;
                }
                else
                    continue;
            }
            if(txconn == NULL)
            {
                console.AddLog("[ERROR]##Connection Buffer Busy. Skip this command.");
                if(confirm_!=NULL)
                {
                    csp_buffer_free(confirm_);
                    confirm_ = NULL;
                }
                break;
            }
            if((confirm_ = csp_read(txconn, rx_delay_ms(Plen, setup->ax100_node))) != NULL && State.uplink_mode)
            {
                packetsign * confirm = PacketDecoder(confirm_);
                if(State.Debugmode)
                {
                    for(int i = 0; i < confirm->Length; i++)
                        printf("%d\t", confirm->Data[i]);
                    printf("\n");
                }
                

                uint16_t * retcode = (uint16_t *)&confirm->Data[0];
                uint16_t * pingcount = (uint16_t *)&confirm->Data[2];

                if(*retcode == 0)
                {
                    console.AddLog("[OK]##[OBC]Ping Success. retcode : %u, pingcount : %u", *retcode, *pingcount);
                    PingCounter ++;
                }
                else
                    console.AddLog("[ERROR]##[OBC]Ping Received but brocken. retcode : %u, pingcount : %u", *retcode, *pingcount);
                if(confirm != NULL)
                {
                    free(confirm);
                    confirm = NULL;
                }
            }
            else
                console.AddLog("[ERROR]##[OBC]Ping Failed.");
            
            if(confirm_!=NULL)
            {
                csp_buffer_free(confirm_);
                confirm_ = NULL;
            }
            break;
        }
        //Ptype 2 : Uplink Signal
        case MIM_PT_SIGNAL : {
            if(State.Debugmode)
            {
                for(int i = 0; i < packet->length; i++)
                {
                    printf("%x", packet[i]);
                }
                printf("\n");
            }
            
            while(State.uplink_mode)
            {
                if ((txconn = csp_connect(CSP_PRIO_HIGH, setup->obc_node, TX_PORT, MIM_DEFAULT_TIMEOUT, 0)) == NULL)
                    continue;
                else
                    break;
            }
            while (State.uplink_mode && txconn != NULL)
            {
                if(csp_send(txconn, packet, setup->default_timeout))
                {
                    packet = NULL;
                    break;
                }
                else
                    continue;
            }
            if(txconn == NULL)
            {
                console.AddLog("[ERROR]##Connection Buffer Busy. Skip this command.");
                break;
            }
            console.AddLog("Uplink Signal TX Done.");
            //Ping Timeout : 10 sec
            break;
        }
        case MIM_PT_DLREQ : {
            csp_packet_t * confirm_ = (csp_packet_t *)csp_buffer_get(MIM_LEN_PACKET);
            while(State.uplink_mode)
            {
                if ((txconn = csp_connect(CSP_PRIO_HIGH, setup->obc_node, TX_PORT, MIM_DEFAULT_TIMEOUT, 0)) == NULL)
                    continue;
                else
                    break;
            }
            while (State.uplink_mode && txconn != NULL)
            {
                if(csp_send(txconn, packet, setup->default_timeout))
                {
                    packet = NULL;
                    break;
                }
                else
                    continue;
            }
            if(txconn == NULL)
            {
                console.AddLog("[ERROR]##Connection Buffer Busy. Skip this command.");
                if(confirm_!=NULL)
                {
                    csp_buffer_free(confirm_);
                    confirm_ = NULL;
                }
                break;
            }
            console.AddLog("Request FTP filelist.");
            if((confirm_ = csp_read(txconn, rx_delay_ms(Plen, setup->ax100_node))) != NULL && State.uplink_mode)
            {
                filelist * confirm = (filelist *)malloc(confirm_->length);
                packetsign* confirmlist =  PacketDecoder(confirm_);
                memcpy(confirm, confirmlist->Data, confirmlist->Length);
                if(confirm->filenum > 0)
                {
                    for(int i = 0; i < 64; i++)
                    {
                        memset(&State.ftplistup[i], 0, sizeof(ftpinfo));
                    }
                    console.AddLog("[OK]##Received File list.");
                    for(int i = 0; i < 64; i++)
                    {
                        
                        if(i < confirm->filenum)
                        {
                            console.AddLog("[DEBUG]##Make Lists : %d\tFileName: %u", i, confirm->file[i]);
                            // memcpy(&State.ftplistup[i], filelisthandler(confirm, filetype, filestatus, i), sizeof(ftpinfo));
                        }
                        else
                            continue;
                    }
                }
                else if(confirm->filenum == 0)
                    console.AddLog("[ERROR]##No more files.");
                else   
                    console.AddLog("[ERROR]##IO Error.");
                if(confirm!=NULL)
                {
                    free(confirm);
                    confirm = NULL;
                    sign = NULL;
                }
                    
            }
            else
            {
                console.AddLog("[ERROR]##Cannot Receive File List.");
            }
            //Ping Timeout : 10 sec
            if(confirm_!=NULL)
            {
                csp_buffer_free(confirm_);
                confirm_ = NULL;
            }
            break;
        }
        case MIM_PT_CMD : { //according to Ptype
            csp_packet_t * confirm_ = (csp_packet_t *)csp_buffer_get(MIM_LEN_PACKET);
            while(State.uplink_mode)
            {
                if ((txconn = csp_connect(CSP_PRIO_HIGH, setup->obc_node, TX_PORT, MIM_DEFAULT_TIMEOUT, 0)) == NULL)
                    continue;
                else
                    break;
            }
            while (State.uplink_mode && txconn != NULL)
            {
                if(csp_send(txconn, packet, setup->default_timeout))
                {
                    packet = NULL;
                    break;
                }
                else
                    continue;
            }
            if(txconn == NULL)
            {
                console.AddLog("[ERROR]##Connection Buffer Busy. Skip this command.");
                if(confirm_!=NULL)
                {
                    csp_buffer_free(confirm_);
                    confirm_ = NULL;
                }
                break;
            }
            if((confirm_ = csp_read(txconn, rx_delay_ms(Plen, setup->ax100_node))) != NULL)
            {
                packetsign* confirmlist =  PacketDecoder(confirm_);
                uint8_t * retcode = (uint8_t *)&confirmlist->Data[0];
                console.AddLog("[OK]Received Command Reply. Retcode %d CMD Count %d", *retcode, State.CMDCount);
                State.CMDCount ++;
                if(confirmlist!=NULL)
                {
                    free(confirmlist);
                    confirmlist = NULL;
                    sign = NULL;
                } 
                if(confirm_!=NULL)
                {
                    csp_buffer_free(confirm_);
                    confirm_ = NULL;
                }
            }
            else
                console.AddLog("[ERROR]##No Command Reply.");
            break;
        }
        case MIM_PT_TMTC_TEST : { // == 10. Add for TMTC Test
            csp_packet_t * confirm_ = (csp_packet_t *)csp_buffer_get(MIM_LEN_PACKET);
            while(State.uplink_mode)
            {
                if(State.Scheduled)
                {
                    if ((txconn = csp_connect(CSP_PRIO_HIGH, 3, 13, MIM_DEFAULT_TIMEOUT, 0)) == NULL) {
                    /*!!!!!!!!!!!Revise setup->obc_node!!!!!!!!!!*/ //-> Change to 28.
                    /*!!!!!!!!!!!!Need to revise Port!!!!!!!!!!!*/
                        continue;
                    }
                    else
                        break;
                }
                else
                {
                    if ((txconn = csp_connect(CSP_PRIO_HIGH, 3, 13, MIM_DEFAULT_TIMEOUT, 0)) == NULL) {
                    /*!!!!!!!!!!!Revise setup->obc_node!!!!!!!!!!*/ //-> Change to 28.
                    /*!!!!!!!!!!!!Need to revise Port!!!!!!!!!!!*/
                        continue;
                    }
                    else
                        break;
                }

            }
            while (State.uplink_mode && txconn != NULL)
            {
                console.AddLog("[OK]CMD Packet Header: 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x",
                    packet->data[0],packet->data[1],packet->data[2],packet->data[3],packet->data[4],packet->data[5],packet->data[6],packet->data[7]);
                fprintf(log_ptr, "|| Uplink || Length: %d\n",packet->length);
                    for(int i=0; i<packet->length; i++) {
                        if(!(i%10) && i !=0) {
                            fprintf(log_ptr, "\n");
                        }
                        fprintf(log_ptr, "%02hhx\t",packet->data[i]);
                    } fprintf(log_ptr,"\n\n");
                if(csp_send(txconn, packet, setup->default_timeout)) // Success. then,
                {   
                    packet = NULL; // discard packet and,
                    break; // End process.
                }
                else // Fail. then,
                    continue; //Go to loof, and try again.
            }
            if(txconn == NULL)
            {
                console.AddLog("[ERROR]##Connection Buffer Busy. Skip this command.");
                if(confirm_!=NULL)
                {
                    csp_buffer_free(confirm_);
                    confirm_ = NULL;
                }
                break;
            }
            // if((confirm_ = csp_read(txconn, rx_delay_ms(Plen, setup->ax100_node))) != NULL)
            // {
            //     packetsign* confirmlist =  PacketDecoder(confirm_);
            //     uint8_t * retcode = (uint8_t *)&confirmlist->Data[0];
            //     console.AddLog("[OK]Received Command Reply. Retcode %d CMD Count %d", *retcode, State.CMDCount);
            //     State.CMDCount ++;
            //     if(confirmlist!=NULL)
            //     {
            //         free(confirmlist);
            //         confirmlist = NULL;
            //         sign = NULL;
            //     } 
            //     if(confirm_!=NULL)
            //     {
            //         csp_buffer_free(confirm_);
            //         confirm_ = NULL;
            //     }
            // }
            // else
            //     console.AddLog("[ERROR]##No Command Reply.");
            break;
        }
        case MIM_PT_STCMD : {
            csp_packet_t * confirm_ = (csp_packet_t *)csp_buffer_get(MIM_LEN_PACKET);
            while(State.uplink_mode)
            {
                if ((txconn = csp_connect(CSP_PRIO_HIGH, setup->obc_node, TX_PORT, MIM_DEFAULT_TIMEOUT, 0)) == NULL)
                    continue;
                else
                    break;
            }
            while (State.uplink_mode && txconn != NULL)
            {
                if(csp_send(txconn, packet, setup->default_timeout))
                {
                    packet = NULL;
                    break;
                }
                else
                    continue;
            }
            if(txconn == NULL)
            {
                console.AddLog("[ERROR]##Connection Buffer Busy. Skip this command.");
                if(confirm_!=NULL)
                {
                    csp_buffer_free(confirm_);
                    confirm_ = NULL;
                }
                break;
            }
            if((confirm_ = csp_read(txconn, rx_delay_ms(Plen, setup->ax100_node))) != NULL && State.uplink_mode)
            {
                if(State.uplink_mode)
                {
                    packetsign* confirmlist =  PacketDecoder(confirm_);
                    uint8_t * retcode = (uint8_t *)&confirmlist->Data[0];
                    console.AddLog("[OK]##Get Command Reply. Retcode : %d", *retcode);
                    if(confirmlist!=NULL)
                    {
                        free(confirmlist);
                        confirmlist = NULL;
                        sign = NULL;
                    } 
                    if(confirm_!=NULL)
                    {
                        csp_buffer_free(confirm_);
                        confirm_ = NULL;
                    }
                }
            }
            else
                console.AddLog("[ERROR]##Cannot Receive Command Reply.");
            break;
        }
        case MIM_PT_NCCMD : {
            csp_packet_t * confirm_ = (csp_packet_t *)csp_buffer_get(MIM_LEN_PACKET);
            packetsign* confirmlist =  PacketDecoder(confirm_);
            while(State.uplink_mode)
            {
                if ((txconn = csp_connect(CSP_PRIO_HIGH, setup->obc_node, TX_PORT, MIM_DEFAULT_TIMEOUT, 0)) == NULL)
                    continue;
                else
                    break;
            }
            while (State.uplink_mode && txconn != NULL)
            {
                if(csp_send(txconn, packet, setup->default_timeout))
                {
                    packet = NULL;
                    break;
                }
                else
                    continue;
            }
            if(txconn == NULL)
            {
                console.AddLog("[ERROR]##Connection Buffer Busy. Skip this command.");
                if(confirm_!=NULL)
                {
                    csp_buffer_free(confirm_);
                    confirm_ = NULL;
                }
                break;
            }
            if((confirm_ = csp_read(txconn,rx_delay_ms(Plen, setup->ax100_node))) != NULL)
            {
                if(State.uplink_mode)
                {
                    console.AddLog("[OK]##Get Command Reply. No Checksum.");
                    if(confirmlist!=NULL)
                    {
                        free(confirmlist);
                        confirmlist = NULL;
                        sign = NULL;
                    } 
                    if(confirm_!=NULL)
                    {
                        csp_buffer_free(confirm_);
                        confirm_ = NULL;
                    }
                }
            }
            else
                console.AddLog("[ERROR]##Cannot Receive Command Reply.");
            break;
        }
        default : {
            console.AddLog("[ERROR]##Undefied type of uplink packetsign. Type : %"PRIu16, Ptype);
            break;
        }
    }
    
    if(txconn != NULL)
    {
        csp_close(txconn);
        txconn = NULL;
    }
    
    if(packet!=NULL)
    {
        csp_buffer_free(packet);
        packet = NULL;
    }
    time = csp_get_ms() - start;
    console.AddLog("[DEBUG]##Communication Time : %u", time);

    State.uplink_mode = false;
    if(dlstate)
    {
        State.downlink_mode = dlstate;
    }
    State.downlink_mode = true;
    pthread_mutex_unlock(&conn_lock);
}

int PacketHandler(csp_packet_t *packet, int type, int NowCursor)
{
    while(1) {
        switch(type) {
            case 0: {
                type = packet->data[NowCursor];
                NowCursor ++;
                return PacketHandler(packet, type, NowCursor);
            }
            case 1: {
                // Declare NowHK as a pointer to HK struct
                // HK *NowHK; 
                for (HKbufCursor; HKbufCursor < sizeof(HKbuf); HKbufCursor ++)
                {
                    HKbuf[HKbufCursor] = packet->data[NowCursor];
                    NowCursor ++;
                    if (NowCursor > MIM_LEN_PACKET)
                    {
                        //Read All Packet
                        //Need to go next packet
                        NowCursor = 0;
                        return 0;
                    }
                }
                // NowHK = (HK *)HKbuf;
                buf_allclear();
                return NowCursor;
            }
            case 3: {
                // for (AODbufCursor; AODbufCursor < sizeof(AODbuf); AODbufCursor ++)
                // {
                //     AODbuf[AODbufCursor] = packet->data[NowCursor];
                //     NowCursor ++;
                //     if (NowCursor > MIM_LEN_PACKET)
                //     {
                //         //Read All Packet
                //         //Need to go next packet
                //         NowCursor = 0;
                //         return 0;
                //     }
                // }
                // NowAOD = (AOD *)HKbuf;
                // buf_allclear();
                // return NowCursor;
            }
            default :{
                
            }
        }
    }
}

packetsign * PingInit(FSWTle * FSWTleinfo)
{
    packetsign * sign;
    sign = (packetsign *)malloc(sizeof(FSWTle) + MIM_HAND_DATAFDSTART);
    sign->Identifier = MIM_ID;
    sign->PacketType = MIM_PT_PING;
    sign->Length = sizeof(FSWTle);
    // printf("Start. Length : %u, Second : %u\n", sign->Length, FSWTleinfo->sec);
    memcpy(sign->Data, (void *)FSWTleinfo, sizeof(FSWTle));

    if(FSWTleinfo != NULL)
    {
        // free(FSWTleinfo);
        FSWTleinfo = NULL;
    }
    

    return sign;
}

csp_packet_t * PacketEncoder(packetsign * sign,bool freeer)
{
    if(sign == NULL)
    {
        console.AddLog("[DEBUG]##NULL POINTER GIVEN TO PACKETENCODER");
        return NULL;
    }
    uint32_t len = sign->Length;
    csp_packet_t *packet = (csp_packet_t *)csp_buffer_get(len);    
    packet->length = len;
    printf("packet len: %u\n", packet->length);
    memcpy(packet->data, sign->Data, len);
    // uint32_t packetsignlen = sign->Length +4; // Data size + Identifier(uint16) + PacketType(uint16)
    // csp_packet_t * packet = (csp_packet_t *)csp_buffer_get(packetsignlen);
    // packet->length = packetsignlen;
    // console.AddLog("[DEBUG]##Encoding Pakcets...Lnegth : %u", packetsignlen);
    // //Copy Header
    // memcpy((packet->data), sign, 4);
    // memcpy((packet->data) + 4, sign->Data, packetsignlen-4 ); // copy data
    for (int i =0; i < packet->length; i++) {
        printf("0x%x ", packet->data[i]);
    }printf("\n");
    if(freeer)
        sign = NULL;
    return packet;
}

packetsign * PacketDecoder(csp_packet_t * packet)
{
    uint32_t packetsignlen = packet->length +4;
    packetsign * sign = (packetsign *)malloc(packetsignlen);
    memcpy(sign, (packet->data), 4);
    sign->Length = packet->length-4;
    memcpy(sign->Data, packet->data + 4, sign->Length);
    return sign;
}

int CmdGenerator_GS::Scheduling(uint32_t ExecutionTime, uint32_t ExecutionWindow, uint32_t EntryID, uint16_t GroupID)
{
    uint16_t CMDlen = this->GetSize() + 12;
    Book * container = (Book *)malloc(CMDlen);
    memset(container, 0, CMDlen);
    container->ExTime = ExecutionTime;
    container->ExWindow = ExecutionWindow;
    container->EntryID = EntryID;
    container->GroupID = GroupID;
    this->CopyCmdHeaderToBuffer(container->cmd);
    this->GenerateCmdHeader(TS_CMD_MID, TS_INSERT_SCHEDULE_ENTRY_CC, sizeof(CFE_MSG_CommandHeader) + CMDlen, (void *)container);
    
}

packetsign * CmdGenerator_GS::GenerateCMDPacket(void)
{
    packetsign * ResultPacket = (packetsign * )malloc(this->GetSize() + MIM_HAND_DATAFDSTART);
    ResultPacket->Identifier = MIM_ID;
    if(this->Scheduled)
        ResultPacket->PacketType = MIM_PT_STCMD;
    else if(!this->Checksum)
        ResultPacket->PacketType = MIM_PT_NCCMD;
    else
        ResultPacket->PacketType = MIM_PT_CMD;
    ResultPacket->Length = this->GetSize();
    console.AddLog("[DEBUG]##Packetsign Length : %u", ResultPacket->Length);
    memcpy(ResultPacket->Data, this->CmdHeader, ResultPacket->Length);
    return ResultPacket;
}

void * Direct_Shell(void * data)
{
    pthread_mutex_lock(&conn_lock);
    while(!State.RotatorReadReady)
        continue;
    bool dlstate = true;


    //This funcion must be on p_thread[4]
    if((dlstate))
    {
        State.downlink_mode = false;
    }
        
    while(!State.uplink_mode)
        continue;

    cmd_packet_t * ResultCMD = (cmd_packet_t *)data;
    reply_packet_t * ResultReply = new reply_packet_t;
    csp_conn_t * txconn;
    csp_packet_t * packet = (csp_packet_t *)csp_buffer_get(sizeof(cmd_packet_t));
    csp_packet_t * confirm = (csp_packet_t *)csp_buffer_get(sizeof(reply_packet_t));

    uint32_t start, time = 0;
    start = csp_get_ms();

    memcpy(packet->data, ResultCMD, sizeof(cmd_packet_t));
    packet->length = sizeof(cmd_packet_t);

    if(State.Debugmode)
    {
        printf("Direct Shell Data : ");
        for(int i = 0; i < sizeof(cmd_packet_t); i++)
            printf("%u\t", ((uint8_t *)packet->data)[i]);
        printf("\n");
    }


    while(State.uplink_mode)
    {
        if ((txconn = csp_connect(CSP_PRIO_HIGH, setup->obc_node, 27, MIM_DEFAULT_TIMEOUT, 0)) == NULL)
            continue;
        else
            break;
    }
    while (State.uplink_mode && txconn != NULL)
    {
        if(csp_send(txconn, packet, setup->default_timeout *2))
        {
            packet = NULL;
            break;
        }
        else
            continue;
    }
    if(txconn == NULL)
    {
        console.AddLog("[ERROR]##Connection Buffer Busy. Skip this command.");
        if(confirm!=NULL)
        {
            csp_buffer_free(confirm);
            confirm = NULL;
        }
    }
    if((confirm = csp_read(txconn, 2 *rx_delay_ms(sizeof(MIM_LEN_PACKET), setup->ax100_node))) != NULL && State.uplink_mode)
    {
        if(State.uplink_mode)
        {
            memcpy(ResultReply, confirm->data, confirm->length);
            console.AddLog("[OK]## Received Shell Reply. Type : %"PRIu8", Result : %"PRIu8", Retcode : %d", ResultReply->type, ResultReply->result, ResultReply->retcode);
            if(confirm!=NULL)
            {
                csp_buffer_free(confirm);
                confirm = NULL;
            }
        }
    }
    else
        console.AddLog("[ERROR]##Cannot Receive Shell Reply.");
    
    if(txconn != NULL)
    {
        csp_close(txconn);
        txconn = NULL;
    }
    
    if(packet!=NULL)
    {
        csp_buffer_free(packet);
        packet = NULL;
    }
    time = csp_get_ms() - start;
    console.AddLog("[DEBUG]##Communication Time : %u", time);

    State.uplink_mode = false;
    if(dlstate)
    {
        State.downlink_mode = dlstate;
    }
    pthread_mutex_unlock(&conn_lock);
}