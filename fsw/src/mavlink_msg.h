    
#ifndef MAVLINK_MSG_H
#define MAVLINK_MSG_H

/************************************************************************
** Pragmas
*************************************************************************/

/************************************************************************
** Includes
*************************************************************************/
#include "cfe.h"

#ifdef __cplusplus
extern "C" {
#endif

/************************************************************************
** Local Defines
*************************************************************************/

/************************************************************************
** MAVLINK Command Codes
*************************************************************************/

/** \mavlinkcmd Noop 
**  
**  \par Description
**       Implements the Noop command that demonstrates the MAVLINK task is alive
**
**  \mavlinkcmdmnemonic \MAVLINK_NOOP
**
**  \par Command Structure
**       #MAVLINK_NoArgCmd_t
**
**  \par Command Verification
**       Successful execution of this command may be verified with
**       the following telemetry:
**       - \b \c \MAVLINK_CMDACPTCNT - command counter will increment
**       - The #MAVLINK_CMD_INF_EID informational event message will be 
**         generated when the command is received
** 
**  \par Error Conditions
**       This command may fail for the following reason(s):
**       - Command packet length not as expected
** 
**  \par Evidence of failure may be found in the following telemetry: 
**       - \b \c \MAVLINK_CMDRJCTCNT - command error counter will increment
**       - Error specific event message #MAVLINK_MSGID_ERR_EID
**
**  \par Criticality
**       None
**
**  \sa #MAVLINK_RESET_CC
*/
#define MAVLINK_NOOP_CC                 (0)

/** \mavlinkcmd Reset Counters
**  
**  \par Description
**       Resets the mavlink housekeeping counters
**
**  \mavlinkcmdmnemonic \MAVLINK_TLMRST
**
**  \par Command Structure
**       #MAVLINK_NoArgCmd_t
**
**  \par Command Verification
**       Successful execution of this command may be verified with
**       the following telemetry:
**       - \b \c \MAVLINK_CMDACTPCNT       - command counter will be cleared
**       - \b \c \MAVLINK_CMDRJCTCNT       - command error counter will be cleared
**       - The #MAVLINK_CMD_INF_EID debug event message will be 
**         generated when the command is executed
** 
**  \par Error Conditions
**       This command may fail for the following reason(s):
**       - Command packet length not as expected
** 
**  \par Evidence of failure may be found in the following telemetry: 
**       - \b \c \MAVLINK_CMDRJCTCNT - command error counter will increment
**       - Error specific event message #MAVLINK_MSGID_ERR_EID
**
**  \par Criticality
**       None
**
**  \sa #MAVLINK_NOOP_CC
*/
#define MAVLINK_RESET_CC                (1)

#define MAVLINK_HEARTBEAT_CC                (2)

/************************************************************************
** Local Structure Declarations
*************************************************************************/

/** 
**  \brief No Arguments Command
**  For command details see #MAVLINK_NOOP_CC, #MAVLINK_RESET_CC
**  Also see #MAVLINK_SEND_HK_MID
*/
typedef struct
{
    uint8  ucCmdHeader[CFE_SB_CMD_HDR_SIZE];
} MAVLINK_NoArgCmd_t;

/** 
**  \brief TODO Elaborate this struct
**  Boilerplate example of application-specific incoming data
*/
typedef struct
{
    uint8   TlmHeader[CFE_SB_TLM_HDR_SIZE];
    uint32  counter;

    /* TODO:  Add input data to this application here, such as raw data read from I/O
    **        devices.
    **        Option: for data that is already defined by another app, include
    **        that app's message header above.
    */

} MAVLINK_InData_t;

/** 
**  \brief TODO Elaborate this struct
**  Boilerplate example of application-specific outgoing data
*/
typedef struct
{
    uint8   ucTlmHeader[CFE_SB_TLM_HDR_SIZE];
    uint32  uiCounter;
} MAVLINK_OutData_t;

/** 
**  \brief MAVLINK application housekeeping data
*/
typedef struct
{
    /** \brief cFE SB Tlm Msg Hdr */
    uint8              TlmHeader[CFE_SB_TLM_HDR_SIZE];

    /** \mavlinktlmmnemonic \MAVLINK_CMDACPTCNT
        \brief Count of accepted commands */
    uint8              usCmdCnt;   

    /** \mavlinktlmmnemonic \MAVLINK_CMDRJCTCNT
        \brief Count of failed commands */
    uint8              usCmdErrCnt; 

} MAVLINK_HkTlm_t;

/**
**  \brief Mavlink version 1 packet
*/
typedef struct
{
	uint8 magic;               ///< protocol magic marker
	uint8 len;                 ///< Length of payload
	uint8 seq;                 ///< Sequence of packet
	uint8 sysid;               ///< ID of message sender system/aircraft
	uint8 compid;              ///< ID of the message sender component
	uint8 msgid;               ///< ID of message in payload
	uint8 payload[255];    ///< A maximum of 255 payload bytes
	uint16 checksum;           ///< X.25 CRC

} MAVLINK_MavPktV1_t;

/**
**  \brief Mavlink version 2 packet
*/
typedef struct
{
	uint8 magic;              ///< protocol magic marker
	uint8 len;                ///< Length of payload
	uint8 incompat_flags;     ///< flags that must be understood
	uint8 compat_flags;       ///< flags that can be ignored if not understood
	uint8 seq;                ///< Sequence of packet
	uint8 sysid;              ///< ID of message sender system/aircraft
	uint8 compid;             ///< ID of the message sender component
	//uint8 msgid 0:7;          ///< first 8 bits of the ID of the message
	//uint8 msgid 8:15;         ///< middle 8 bits of the ID of the message
	//uint8 msgid 16:23;        ///< last 8 bits of the ID of the message
	uint8 target_sysid;       ///< Optional field for point-to-point messages, used for payload else
	uint8 target_compid;      ///< Optional field for point-to-point messages, used for payload else
	uint8 payload[253];   ///< A maximum of 253 payload bytes
	uint16 checksum;          ///< X.25 CRC
	uint8 signature[13];      ///< Signature which allows ensuring that the link is tamper-proof

} MAVLINK_MavPktV2_t;

typedef struct
{
	uint8 type;              ///< protocol magic marker
	uint8 autopilot;                ///< Length of payload
	uint8 base_mode;     ///< flags that must be understood
	uint32 custom_mode;       ///< flags that can be ignored if not understood
	uint8 system_status;       
	uint8 mavlink_version;       
} Heartbeat_t;


#ifdef __cplusplus
}
#endif

#endif /* MAVLINK_MSG_H */

/************************/
/*  End of File Comment */
/************************/
