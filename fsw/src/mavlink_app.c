
/************************************************************************
** Pragmas
*************************************************************************/

/************************************************************************
** Includes
*************************************************************************/
#include <string.h>

#include "cfe.h"

#include "mavlink_version.h"
#include "mavlink_app.h"
#include "mavlink_msg.h"

/************************************************************************
** Local Defines
*************************************************************************/

/************************************************************************
** Local Structure Declarations
*************************************************************************/

/************************************************************************
** External Global Variables
*************************************************************************/

/************************************************************************
** Global Variables
*************************************************************************/
MAVLINK_AppData_t  MAVLINK_AppData;

/************************************************************************
** Local Variables
*************************************************************************/

/************************************************************************
** Local Function Definitions
*************************************************************************/

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* Initialize event tables                                         */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

int32 MAVLINK_InitEvent()
{
    int32  iStatus=CFE_SUCCESS;
    int32  ind = 0;

    /* Initialize the event filter table.
     * Note: 0 is the CFE_EVS_NO_FILTER mask and event 0 is reserved (not used) */
    memset((void*)MAVLINK_AppData.EventTbl, 0x00, sizeof(MAVLINK_AppData.EventTbl));

    /* TODO: Choose the events you want to filter.  CFE_EVS_MAX_EVENT_FILTERS
     * limits the number of filters per app.  An explicit CFE_EVS_NO_FILTER 
     * (the default) has been provided as an example. */
    MAVLINK_AppData.EventTbl[  ind].EventID = MAVLINK_RESERVED_EID;
    MAVLINK_AppData.EventTbl[ind++].Mask    = CFE_EVS_NO_FILTER;

    MAVLINK_AppData.EventTbl[  ind].EventID = MAVLINK_INF_EID;
    MAVLINK_AppData.EventTbl[ind++].Mask    = CFE_EVS_NO_FILTER;

    MAVLINK_AppData.EventTbl[  ind].EventID = MAVLINK_PARAM_TABLE_ERR_EID;
    MAVLINK_AppData.EventTbl[ind++].Mask    = CFE_EVS_NO_FILTER;

    MAVLINK_AppData.EventTbl[  ind].EventID = MAVLINK_CDS_ERR_EID;
    MAVLINK_AppData.EventTbl[ind++].Mask    = CFE_EVS_NO_FILTER;

    MAVLINK_AppData.EventTbl[  ind].EventID = MAVLINK_PIPE_ERR_EID;
    MAVLINK_AppData.EventTbl[ind++].Mask    = CFE_EVS_NO_FILTER;

    MAVLINK_AppData.EventTbl[  ind].EventID = MAVLINK_MSGID_ERR_EID;
    MAVLINK_AppData.EventTbl[ind++].Mask    = CFE_EVS_NO_FILTER;

    MAVLINK_AppData.EventTbl[  ind].EventID = MAVLINK_MSGLEN_ERR_EID;
    MAVLINK_AppData.EventTbl[ind++].Mask    = CFE_EVS_NO_FILTER;

    /* Register the table with CFE */
    iStatus = CFE_EVS_Register(MAVLINK_AppData.EventTbl,
                               MAVLINK_EVT_CNT, CFE_EVS_BINARY_FILTER);
    if (iStatus != CFE_SUCCESS)
    {
        (void) CFE_ES_WriteToSysLog("MAVLINK - Failed to register with EVS (0x%08X)\n", (unsigned int)iStatus);
    }

    return (iStatus);
}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* Initialize Message Pipes                                        */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

int32 MAVLINK_InitPipe()
{
    int32  iStatus=CFE_SUCCESS;

    /* Init schedule pipe and subscribe to wakeup messages */
    iStatus = CFE_SB_CreatePipe(&MAVLINK_AppData.SchPipeId,
                                 MAVLINK_SCH_PIPE_DEPTH,
                                 MAVLINK_SCH_PIPE_NAME);
    if (iStatus == CFE_SUCCESS)
    {
        iStatus = CFE_SB_SubscribeEx(MAVLINK_WAKEUP_MID, MAVLINK_AppData.SchPipeId, CFE_SB_Default_Qos, MAVLINK_SCH_PIPE_WAKEUP_RESERVED);

        if (iStatus != CFE_SUCCESS)
        {
            (void) CFE_EVS_SendEvent(MAVLINK_INIT_ERR_EID, CFE_EVS_ERROR,
                                     "Sch Pipe failed to subscribe to MAVLINK_WAKEUP_MID. (0x%08X)",
                                     (unsigned int)iStatus);
            goto MAVLINK_InitPipe_Exit_Tag;
        }

        iStatus = CFE_SB_SubscribeEx(MAVLINK_SEND_HK_MID, MAVLINK_AppData.SchPipeId, CFE_SB_Default_Qos, MAVLINK_SCH_PIPE_SEND_HK_RESERVED);

        if (iStatus != CFE_SUCCESS)
        {
            (void) CFE_EVS_SendEvent(MAVLINK_INIT_ERR_EID, CFE_EVS_ERROR,
                                     "CMD Pipe failed to subscribe to MAVLINK_SEND_HK_MID. (0x%08X)",
                                     (unsigned int)iStatus);
            goto MAVLINK_InitPipe_Exit_Tag;
        }

    }
    else
    {
        (void) CFE_EVS_SendEvent(MAVLINK_INIT_ERR_EID, CFE_EVS_ERROR,
                                 "Failed to create SCH pipe (0x%08X)",
                                 (unsigned int)iStatus);
        goto MAVLINK_InitPipe_Exit_Tag;
    }

    /* Init command pipe and subscribe to command messages */
    iStatus = CFE_SB_CreatePipe(&MAVLINK_AppData.CmdPipeId,
                                 MAVLINK_CMD_PIPE_DEPTH,
                                 MAVLINK_CMD_PIPE_NAME);
    if (iStatus == CFE_SUCCESS)
    {
        /* Subscribe to command messages */
        iStatus = CFE_SB_Subscribe(MAVLINK_CMD_MID, MAVLINK_AppData.CmdPipeId);

        if (iStatus != CFE_SUCCESS)
        {
            (void) CFE_EVS_SendEvent(MAVLINK_INIT_ERR_EID, CFE_EVS_ERROR,
                                     "CMD Pipe failed to subscribe to MAVLINK_CMD_MID. (0x%08X)",
                                     (unsigned int)iStatus);
            goto MAVLINK_InitPipe_Exit_Tag;
        }
    }
    else
    {
        (void) CFE_EVS_SendEvent(MAVLINK_INIT_ERR_EID, CFE_EVS_ERROR,
                                 "Failed to create CMD pipe (0x%08X)",
                                 (unsigned int)iStatus);
        goto MAVLINK_InitPipe_Exit_Tag;
    }

    /* Init data pipe and subscribe to messages on the data pipe */
    iStatus = CFE_SB_CreatePipe(&MAVLINK_AppData.DataPipeId,
                                 MAVLINK_DATA_PIPE_DEPTH,
                                 MAVLINK_DATA_PIPE_NAME);
    if (iStatus == CFE_SUCCESS)
    {
        /* TODO:  Add CFE_SB_Subscribe() calls for other apps' output data here.
        **
        ** Examples:
        **     CFE_SB_Subscribe(GNCEXEC_OUT_DATA_MID, MAVLINK_AppData.DataPipeId);
        */
    }
    else
    {
        (void) CFE_EVS_SendEvent(MAVLINK_INIT_ERR_EID, CFE_EVS_ERROR,
                                 "Failed to create Data pipe (0x%08X)",
                                 (unsigned int)iStatus);
        goto MAVLINK_InitPipe_Exit_Tag;
    }

MAVLINK_InitPipe_Exit_Tag:
    return (iStatus);
}
    

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* Initialize Global Variables                                     */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

int32 MAVLINK_InitData()
{
    int32  iStatus=CFE_SUCCESS;

    /* Init input data */
    memset((void*)&MAVLINK_AppData.InData, 0x00, sizeof(MAVLINK_AppData.InData));

    /* Init output data */
    memset((void*)&MAVLINK_AppData.OutData, 0x00, sizeof(MAVLINK_AppData.OutData));
    CFE_SB_InitMsg(&MAVLINK_AppData.OutData,
                   MAVLINK_OUT_DATA_MID, sizeof(MAVLINK_AppData.OutData), TRUE);

    /* Init housekeeping packet */
    memset((void*)&MAVLINK_AppData.HkTlm, 0x00, sizeof(MAVLINK_AppData.HkTlm));
    CFE_SB_InitMsg(&MAVLINK_AppData.HkTlm,
                   MAVLINK_HK_TLM_MID, sizeof(MAVLINK_AppData.HkTlm), TRUE);

    return (iStatus);
}




/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* MAVLINK initialization                                              */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

int32 MAVLINK_InitApp()
{
    int32  iStatus   = CFE_SUCCESS;
    int8   hasEvents = 0;

    iStatus = MAVLINK_InitEvent();
    if (iStatus != CFE_SUCCESS)
    {
        (void) CFE_ES_WriteToSysLog("MAVLINK - Failed to init events (0x%08X)\n", (unsigned int)iStatus);
        goto MAVLINK_InitApp_Exit_Tag;
    }
    else
    {
        hasEvents = 1;
    }

    iStatus = MAVLINK_InitPipe();
    if (iStatus != CFE_SUCCESS)
    {
        (void) CFE_EVS_SendEvent(MAVLINK_INIT_ERR_EID, CFE_EVS_ERROR,
                                 "Failed to init pipes (0x%08X)",
                                 (unsigned int)iStatus);
        goto MAVLINK_InitApp_Exit_Tag;
    }

    iStatus = MAVLINK_InitData();
    if (iStatus != CFE_SUCCESS)
    {
        (void) CFE_EVS_SendEvent(MAVLINK_INIT_ERR_EID, CFE_EVS_ERROR,
                                 "Failed to init data (0x%08X)",
                                 (unsigned int)iStatus);
        goto MAVLINK_InitApp_Exit_Tag;
    }

    iStatus = MAVLINK_InitParamTbl();
    if (iStatus != CFE_SUCCESS)
    {
        (void) CFE_EVS_SendEvent(MAVLINK_INIT_ERR_EID, CFE_EVS_ERROR,
                                 "Failed to init config tables (0x%08X)",
                                 (unsigned int)iStatus);
        goto MAVLINK_InitApp_Exit_Tag;
    }

    iStatus = MAVLINK_InitCdsTbl();
    if (iStatus != CFE_SUCCESS)
    {
        (void) CFE_EVS_SendEvent(MAVLINK_INIT_ERR_EID, CFE_EVS_ERROR,
                                 "Failed to init CDS table (0x%08X)",
                                 (unsigned int)iStatus);
        goto MAVLINK_InitApp_Exit_Tag;
    }

    /* Initialize custom code. */
	iStatus = MAVLINK_InitCustom();
	if(iStatus != CFE_SUCCESS)
	{
		CFE_ES_WriteToSysLog("MAVLINK - InitCustom failed (%i)\n", (int)iStatus);
		goto MAVLINK_InitApp_Exit_Tag;
	}

    /* Register the cleanup callback */
    iStatus = OS_TaskInstallDeleteHandler(&MAVLINK_CleanupCallback);
    if (iStatus != CFE_SUCCESS)
    {
        (void) CFE_EVS_SendEvent(MAVLINK_INIT_ERR_EID, CFE_EVS_ERROR,
                                 "Failed to init register cleanup callback (0x%08X)",
                                 (unsigned int)iStatus);
        goto MAVLINK_InitApp_Exit_Tag;
    }

    /* Initialize listener child task. */
    iStatus = MAVLINK_InitChildTasks();
    if(iStatus != CFE_SUCCESS)
    {
        CFE_ES_WriteToSysLog("MAVLINK - Failed to initialize child tasks (0x%08X)\n", (unsigned int)iStatus);
        goto MAVLINK_InitApp_Exit_Tag;
    }

MAVLINK_InitApp_Exit_Tag:
    if (iStatus == CFE_SUCCESS)
    {
        (void) CFE_EVS_SendEvent(MAVLINK_INIT_INF_EID, CFE_EVS_INFORMATION,
                                 "Initialized.  Version %d.%d.%d.%d",
                                 MAVLINK_MAJOR_VERSION,
                                 MAVLINK_MINOR_VERSION,
                                 MAVLINK_REVISION,
                                 MAVLINK_MISSION_REV);
    }
    else
    {
        if (hasEvents == 1)
        {
            (void) CFE_EVS_SendEvent(MAVLINK_INIT_ERR_EID, CFE_EVS_ERROR, "Application failed to initialize");
        }
        else
        {
            (void) CFE_ES_WriteToSysLog("MAVLINK - Application failed to initialize\n");
        }
    }

    return (iStatus);
}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* MAVLINK Cleanup                                                 */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

void MAVLINK_CleanupCallback()
{
	MAVLINK_CleanupCustom();
	MAVLINK_AppData.IngestActive = FALSE;
	//OS_MutSemGive(MAVLINK_AppData.ConfigTblMutex);
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* Spawn Child Task				                                   */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

int32 MAVLINK_InitChildTasks(void)
{
    int32 Status = CFE_SUCCESS;

    MAVLINK_AppData.IngestActive = TRUE;

	Status= CFE_ES_CreateChildTask(&MAVLINK_AppData.ListenerTaskID,
								   MAVLINK_LISTENER_TASK_NAME,
								   MAVLINK_ListenerTaskMain,
								   NULL,
								   MAVLINK_LISTENER_TASK_STACK_SIZE,
								   MAVLINK_LISTENER_TASK_PRIORITY,
								   0);

	if (Status != CFE_SUCCESS)
	{
		goto MAVLINK_InitListenerTask_Exit_Tag;
	}

//	Status= CFE_ES_CreateChildTask(&MAVLINK_AppData.SerialListenerTaskID,
//									MAVLINK_SERIAL_LISTENER_TASK_NAME,
//									MAVLINK_SerializedListenerTaskMain,
//									NULL,
//									MAVLINK_SERIAL_LISTENER_TASK_STACK_SIZE,
//									MAVLINK_SERIAL_LISTENER_TASK_PRIORITY,
//									0);
//
//	if (Status != CFE_SUCCESS)
//	{
//		goto MAVLINK_InitListenerTask_Exit_Tag;
//	}

MAVLINK_InitListenerTask_Exit_Tag:
	if (Status != CFE_SUCCESS)
	{
		CFE_EVS_SendEvent (MAVLINK_LISTENER_CREATE_CHDTASK_ERR_EID,
						   CFE_EVS_ERROR,
						   "Listener child task failed.  CFE_ES_CreateChildTask returned: 0x%08lX",
						   Status);
	}

    return Status;
}

void printMsg(mavlink_message_t *msg)
{
    OS_printf("magic = %i\n", msg->magic );
    OS_printf("len = %i\n", msg->len );
    OS_printf("seq = %i\n", msg->seq );
    OS_printf("sysid = %i\n", msg->sysid );
    OS_printf("compid = %i\n", msg->compid );
    OS_printf("msgid = %i\n", msg->msgid );
    //OS_printf("payload = %i", msg->paload );
    OS_printf("checksum = %i\n\n", msg->checksum );
}

void printHrt(mavlink_heartbeat_t *msg)
{
    OS_printf("type = %u\n", msg->type );
    OS_printf("autopilot = %u\n", msg->autopilot );
    OS_printf("base_mode = %u\n", msg->base_mode );
    OS_printf("custom_mode = %u\n", msg->custom_mode );
    OS_printf("system_status = %u\n", msg->system_status );
    OS_printf("mavlink_version = %u\n", msg->mavlink_version );
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* Child Task Listener Main		                                   */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

void MAVLINK_ListenerTaskMain(void)
{
	int32 Status = -1;
	mavlink_message_t msg;
	mavlink_status_t msg_status;
	int32 index = 0;
    uint32 MsgSize = MAVLINK_MAX_PACKET_LEN;

	Status = CFE_ES_RegisterChildTask();
	if (Status == CFE_SUCCESS)
	{
		/* Ingest Loop */
		do{
			//OS_printf("\nIn MAVLINK ingest loop\n");

			/* Receive mavlink message */
			MAVLINK_ReadMessage(MAVLINK_AppData.IngestBuffer, &MsgSize);

			if(MsgSize == -1)
			{
				MsgSize = MAVLINK_MAX_PACKET_LEN;
				continue;
			}

			/* Decode the message */
			for (index = 0; index < MsgSize; ++index)
			{
				if (mavlink_parse_char(MAVLINK_COMM_0, MAVLINK_AppData.IngestBuffer[index], &msg, &msg_status))
				{
					/* Pass to message router */
					MAVLINK_MessageRouter(msg);
				}
			}

            /* Wait before next iteration */
			OS_TaskDelay(100);
		}while(MAVLINK_AppData.IngestActive == TRUE);

		CFE_ES_ExitChildTask();
	}
	else
	{
		/* Can't send event or write to syslog because this task isn't registered with the cFE. */
		OS_printf("MAVLINK Listener Child Task Registration failed!\n");
	}
}

void MAVLINK_MessageRouter(mavlink_message_t msg)
{
	switch(msg.msgid)
	{
		case MAVLINK_MSG_ID_HEARTBEAT:
		{
			mavlink_heartbeat_t 		decodedMsg;
			mavlink_msg_heartbeat_decode(&msg, &decodedMsg);
			MAVLINK_ProcessHeartbeat(decodedMsg);
			break;
		}
		case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
		{
			OS_printf("QGC requseting params\n");
			MAVLINK_SendParamsToQGC();
			break;
		}
		case MAVLINK_MSG_ID_COMMAND_LONG:
		{
			OS_printf("Recieved command long\n");
			mavlink_command_long_t 		decodedMsg;
			mavlink_msg_command_long_decode(&msg, &decodedMsg);
			//MAVLINK_ProcessHeartbeat(decodedMsg);
			break;
		}



		default:
			OS_printf("\nReceived packet: SYS: %d, COMP: %d, LEN: %d, MSG ID: %d\n", msg.sysid, msg.compid, msg.len, msg.msgid);
			break;
	}
//	OS_printf("\n");
}


void MAVLINK_ProcessHeartbeat(mavlink_heartbeat_t heartbeat)
{
	if(heartbeat.type == MAV_TYPE_GCS)
	{
		OS_printf("Found QGC heartbeat\n");
	}
	else if(heartbeat.type == MAV_TYPE_OCTOROTOR)
	{
		OS_printf("Found our heartbeat\n");
	}
	else
	{
		OS_printf("Found UNKNOWN heartbeat\n");
	}
//	printHrt(&heartbeat);
}


void MAVLINK_SendHeartbeat(void)
{
	uint8 system_id 		= MAVLINK_PARAM_SYSTEM_ID;
	uint8 component_id 		= MAV_COMP_ID_AUTOPILOT1;
	mavlink_message_t msg 	= {0};
	uint8 type				= MAV_TYPE_OCTOROTOR;
	uint8 autopilot 		= MAV_AUTOPILOT_GENERIC;
	uint8 base_mode 		= MAV_MODE_FLAG_STABILIZE_ENABLED;
	uint32 custom_mode 		= 0;
	uint8 system_status 	= 3;
	uint16 msg_size 		= 0;
	uint8 msgBuf[MAVLINK_MAX_PACKET_LEN] = {0};

	mavlink_msg_heartbeat_pack(system_id, component_id, &msg, type,
										 autopilot, base_mode, custom_mode, system_status);
	msg_size = mavlink_msg_to_send_buffer(msgBuf, &msg);

    MAVLINK_SendMessage((char *) &msgBuf, msg_size);
}

void MAVLINK_SendToQGC(void) //DEBUG ONLY
{
	//OS_printf("Sending heartbeat\n");
	mavlink_message_t msg 	= {0};
	mavlink_message_t msg2 	= {0};
	mavlink_message_t msg3 	= {0};
	mavlink_param_value_t p = {0};
	mavlink_param_value_t p2 = {0};
	mavlink_param_value_t p3 = {0};
	uint16 msg_size 		= 0;
	uint8 msgBuf[MAVLINK_MAX_PACKET_LEN] = {0};
	char *name = "TEST_PARAM_1";
	char *name2 = "TEST_PARAM_2";
	char *name3 = "TEST_PARAM_3";

	p.param_index = 0;
	p.param_count = 3;
	p.param_value = 1;
	memcpy(&p.param_id, name, 12);
	p.param_type = 6;

	p2.param_index = 1;
	p2.param_count = 3;
	p2.param_value = 2;
	memcpy(&p2.param_id, name2, 12);
	p2.param_type = 6;

	p3.param_index = 2;
	p3.param_count = 3;
	p3.param_value = 3;
	memcpy(&p3.param_id, name3, 12);
	p3.param_type = 6;

	mavlink_msg_param_value_encode(1,1, &msg, &p);
	mavlink_msg_param_value_encode(1,1, &msg2, &p2);
	mavlink_msg_param_value_encode(1,1, &msg3, &p3);

	msg_size = mavlink_msg_to_send_buffer(msgBuf, &msg);
    MAVLINK_SendMessage((char *) &msgBuf, msg_size);

    msg_size = mavlink_msg_to_send_buffer(msgBuf, &msg2);
    MAVLINK_SendMessage((char *) &msgBuf, msg_size);

	msg_size = mavlink_msg_to_send_buffer(msgBuf, &msg3);
	MAVLINK_SendMessage((char *) &msgBuf, msg_size);
}

void MAVLINK_SendParamsToQGC(void)
{
	//OS_printf("Sending heartbeat\n");
	uint32 i =0;
	uint16 msg_size 		= 0;
	uint8 msgBuf[MAVLINK_MAX_PACKET_LEN] = {0};
	mavlink_param_value_t param_value_msg = {0};
	mavlink_message_t msg 	= {0};


	/* Iterate over table and send each parameter */
	for(i = 0; i < MAVLINK_PARAM_TABLE_MAX_ENTRIES; ++i)
	{
		if (MAVLINK_AppData.ParamTblPtr->params[i].enabled == 1)
		{
			/* Update parameter message with current table index values */
			param_value_msg.param_index = i;
			param_value_msg.param_count = 10;// set equal to total number of params?
			param_value_msg.param_value = MAVLINK_AppData.ParamTblPtr->params[i].value;
			memcpy(&param_value_msg.param_id, MAVLINK_AppData.ParamTblPtr->params[i].name,
					sizeof(MAVLINK_AppData.ParamTblPtr->params[i].name));
			param_value_msg.param_type = MAVLINK_AppData.ParamTblPtr->params[i].type;

			/* Encode mavlink message and send to ground station */
			mavlink_msg_param_value_encode(MAVLINK_PARAM_SYSTEM_ID, MAVLINK_PARAM_COMPONENT_ID, &msg, &param_value_msg);
			msg_size = mavlink_msg_to_send_buffer(msgBuf, &msg);
			MAVLINK_SendMessage((char *) &msgBuf, msg_size);
		}
	}

}

void MAVLINK_SendParamsToSB(void)
{
	int32 Status;
	uint32 i =0;
	uint16 msg_size 		= 0;
	uint8 msgBuf[MAVLINK_MAX_PACKET_LEN] = {0};
	MAVLINK_ParamValue_t ParamValueMsg = {0};
	CFE_SB_MsgPtr_t 	CmdMsgPtr;


	/* Iterate over table and send each parameter */
	for(i = 0; i < MAVLINK_PARAM_TABLE_MAX_ENTRIES; ++i)
	{
		if (MAVLINK_AppData.ParamTblPtr->params[i].enabled == 1)
		{
			/* Update parameter message with current table index values */
			ParamValueMsg.value = MAVLINK_AppData.ParamTblPtr->params[i].value;
			memcpy(ParamValueMsg.name, MAVLINK_AppData.ParamTblPtr->params[i].name,
					sizeof(MAVLINK_AppData.ParamTblPtr->params[i].name));
			//ParamValueMsg.param_type = MAVLINK_AppData.ParamTblPtr->params[i].type; // TODO need this?

			/* Init Airliner message and send to SB */
			CFE_SB_InitMsg(&ParamValueMsg, MAVLINK_PARAM_VALUE_MID, sizeof(MAVLINK_ParamValue_t), FALSE);
			CmdMsgPtr = (CFE_SB_MsgPtr_t)&ParamValueMsg;

			Status = CFE_SB_SendMsg(CmdMsgPtr);
			if (Status != CFE_SUCCESS)
			{
				/* TODO: Decide what to do if the send message fails. */
			}
		}
	}

}

void MAVLINK_AddParam(MAVLINK_SetParamCmd_t* SetParamMsg)
{
	/* Iterate over table to find first empty index */
	for(int i = 0; i < MAVLINK_PARAM_TABLE_MAX_ENTRIES; ++i)
	{
		if (MAVLINK_AppData.ParamTblPtr->params[i].enabled == 0)
		{
			/* Update parameter message with current table index values */
			MAVLINK_AppData.ParamTblPtr->params[i].enabled = 1;
			MAVLINK_AppData.ParamTblPtr->params[i].value = SetParamMsg->value;
			memcpy(MAVLINK_AppData.ParamTblPtr->params[i].name, SetParamMsg->name,
					sizeof(SetParamMsg->name)); //need to clear string?
			MAVLINK_AppData.ParamTblPtr->params[i].type = SetParamMsg->type;
		}
	}
}


void MAVLINK_SetParam(MAVLINK_SetParamCmd_t* SetParamMsg)
{
	boolean paramExists = FALSE;

	/* Iterate over table to find parameter */
	for(int i = 0; i < MAVLINK_PARAM_TABLE_MAX_ENTRIES; ++i)
	{
		/* Only check enabled parameters */
		if (MAVLINK_AppData.ParamTblPtr->params[i].enabled == 1)
		{
			if (strcmp(SetParamMsg->name, MAVLINK_AppData.ParamTblPtr->params[i].name))
			{
				/* Update parameter message with current table index values */
				paramExists = TRUE;
				MAVLINK_AppData.ParamTblPtr->params[i].value = SetParamMsg->value;
				MAVLINK_AppData.ParamTblPtr->params[i].type = SetParamMsg->type;
			}
		}
	}

	if (!paramExists)
	{
		MAVLINK_AddParam(SetParamMsg);
	}
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* Receive and Process Messages                                    */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

int32 MAVLINK_RcvMsg(int32 iBlocking)
{
    int32           iStatus=CFE_SUCCESS;
    CFE_SB_Msg_t*   MsgPtr=NULL;
    CFE_SB_MsgId_t  MsgId;

    /* Stop Performance Log entry */
    CFE_ES_PerfLogExit(MAVLINK_MAIN_TASK_PERF_ID);

    /* Wait for WakeUp messages from scheduler */
    iStatus = CFE_SB_RcvMsg(&MsgPtr, MAVLINK_AppData.SchPipeId, iBlocking);

    /* Start Performance Log entry */
    CFE_ES_PerfLogEntry(MAVLINK_MAIN_TASK_PERF_ID);

    if (iStatus == CFE_SUCCESS)
    {
        MsgId = CFE_SB_GetMsgId(MsgPtr);
        switch (MsgId)
	{
            case MAVLINK_WAKEUP_MID:
                MAVLINK_SendHeartbeat();
                MAVLINK_ProcessNewCmds();
                MAVLINK_ProcessNewData();

                /* TODO:  Add more code here to handle other things when app wakes up */

                /* The last thing to do at the end of this Wakeup cycle should be to
                 * automatically publish new output. */
                MAVLINK_SendOutData();
                break;

            case MAVLINK_SEND_HK_MID:
                MAVLINK_ReportHousekeeping();
                break;

            default:
                (void) CFE_EVS_SendEvent(MAVLINK_MSGID_ERR_EID, CFE_EVS_ERROR,
                                  "Recvd invalid SCH msgId (0x%04X)", (unsigned short)MsgId);
        }
    }
    else if (iStatus == CFE_SB_NO_MESSAGE)
    {
        /* TODO: If there's no incoming message, you can do something here, or 
         * nothing.  Note, this section is dead code only if the iBlocking arg
         * is CFE_SB_PEND_FOREVER. */
        iStatus = CFE_SUCCESS;
    }
    else if (iStatus == CFE_SB_TIME_OUT)
    {
        /* TODO: If there's no incoming message within a specified time (via the
         * iBlocking arg, you can do something here, or nothing.  
         * Note, this section is dead code only if the iBlocking arg
         * is CFE_SB_PEND_FOREVER. */
        iStatus = CFE_SUCCESS;
    }
    else
    {
        /* TODO: This is an example of exiting on an error (either CFE_SB_BAD_ARGUMENT, or
         * CFE_SB_PIPE_RD_ERROR).
         */
        (void) CFE_EVS_SendEvent(MAVLINK_PIPE_ERR_EID, CFE_EVS_ERROR,
			  "SB pipe read error (0x%08X), app will exit", (unsigned int)iStatus);
        MAVLINK_AppData.uiRunStatus= CFE_ES_APP_ERROR;
    }

    return (iStatus);
}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* Process Incoming Data                                           */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

void MAVLINK_ProcessNewData()
{
    int iStatus = CFE_SUCCESS;
    CFE_SB_Msg_t*   DataMsgPtr=NULL;
    CFE_SB_MsgId_t  DataMsgId;

    /* Process telemetry messages till the pipe is empty */
    while (1)
    {
        iStatus = CFE_SB_RcvMsg(&DataMsgPtr, MAVLINK_AppData.DataPipeId, CFE_SB_POLL);
        if (iStatus == CFE_SUCCESS)
        {
            DataMsgId = CFE_SB_GetMsgId(DataMsgPtr);
            switch (DataMsgId)
            {
                /* TODO:  Add code to process all subscribed data here
                **
                ** Example:
                **     case NAV_OUT_DATA_MID:
                **         MAVLINK_ProcessNavData(DataMsgPtr);
                **         break;
                */

                default:
                    (void) CFE_EVS_SendEvent(MAVLINK_MSGID_ERR_EID, CFE_EVS_ERROR,
                                      "Recvd invalid data msgId (0x%04X)", (unsigned short)DataMsgId);
                    break;
            }
        }
        else if (iStatus == CFE_SB_NO_MESSAGE)
        {
            break;
        }
        else
        {
            (void) CFE_EVS_SendEvent(MAVLINK_PIPE_ERR_EID, CFE_EVS_ERROR,
                  "Data pipe read error (0x%08X)", (unsigned int)iStatus);
            MAVLINK_AppData.uiRunStatus = CFE_ES_APP_ERROR;
            break;
        }
    }
}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* Process Incoming Commands                                       */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

void MAVLINK_ProcessNewCmds()
{
    int iStatus = CFE_SUCCESS;
    CFE_SB_Msg_t*   CmdMsgPtr=NULL;
    CFE_SB_MsgId_t  CmdMsgId;

    /* Process command messages till the pipe is empty */
    while (1)
    {
        iStatus = CFE_SB_RcvMsg(&CmdMsgPtr, MAVLINK_AppData.CmdPipeId, CFE_SB_POLL);
        if(iStatus == CFE_SUCCESS)
        {
            CmdMsgId = CFE_SB_GetMsgId(CmdMsgPtr);
            switch (CmdMsgId)
            {
                case MAVLINK_CMD_MID:
                    MAVLINK_ProcessNewAppCmds(CmdMsgPtr);
                    break;

                /* TODO:  Add code to process other subscribed commands here
                **
                ** Example:
                **     case CFE_TIME_DATA_CMD_MID:
                **         MAVLINK_ProcessTimeDataCmd(CmdMsgPtr);
                **         break;
                */

                default:
                    /* Bump the command error counter for an unknown command.
                     * (This should only occur if it was subscribed to with this
                     *  pipe, but not handled in this switch-case.) */
                    MAVLINK_AppData.HkTlm.usCmdErrCnt++;
                    (void) CFE_EVS_SendEvent(MAVLINK_MSGID_ERR_EID, CFE_EVS_ERROR,
                                      "Recvd invalid CMD msgId (0x%04X)", (unsigned short)CmdMsgId);
                    break;
            }
        }
        else if (iStatus == CFE_SB_NO_MESSAGE)
        {
            break;
        }
        else
        {
            (void) CFE_EVS_SendEvent(MAVLINK_PIPE_ERR_EID, CFE_EVS_ERROR,
                  "CMD pipe read error (0x%08X)", (unsigned int)iStatus);
            MAVLINK_AppData.uiRunStatus = CFE_ES_APP_ERROR;
            break;
        }
    }
}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* Process MAVLINK Commands                                            */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

void MAVLINK_ProcessNewAppCmds(CFE_SB_Msg_t* MsgPtr)
{
    uint32  uiCmdCode=0;

    if (MsgPtr != NULL)
    {
        uiCmdCode = CFE_SB_GetCmdCode(MsgPtr);
        switch (uiCmdCode)
        {
            case MAVLINK_NOOP_CC:
                MAVLINK_AppData.HkTlm.usCmdCnt++;
                (void) CFE_EVS_SendEvent(MAVLINK_CMD_INF_EID, CFE_EVS_INFORMATION,
                                  "Recvd NOOP cmd (%u), Version %d.%d.%d.%d",
                                  (unsigned int)uiCmdCode,
                                  MAVLINK_MAJOR_VERSION,
                                  MAVLINK_MINOR_VERSION,
                                  MAVLINK_REVISION,
                                  MAVLINK_MISSION_REV);
                break;

            case MAVLINK_RESET_CC:
                MAVLINK_AppData.HkTlm.usCmdCnt = 0;
                MAVLINK_AppData.HkTlm.usCmdErrCnt = 0;
                (void) CFE_EVS_SendEvent(MAVLINK_CMD_INF_EID, CFE_EVS_INFORMATION,
                                  "Recvd RESET cmd (%u)", (unsigned int)uiCmdCode);
                break;

            case MAVLINK_GET_PARAMS_CC:
            	MAVLINK_SendParamsToSB();
				break;

            case MAVLINK_SET_PARAM_CC:
            	MAVLINK_SetParam(MsgPtr);
            	break;

            default:
                MAVLINK_AppData.HkTlm.usCmdErrCnt++;
                (void) CFE_EVS_SendEvent(MAVLINK_MSGID_ERR_EID, CFE_EVS_ERROR,
                                  "Recvd invalid cmdId (%u)", (unsigned int)uiCmdCode);
                break;
        }
    }
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* Send MAVLINK Housekeeping                                           */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

void MAVLINK_ReportHousekeeping()
{
    /* TODO:  Add code to update housekeeping data, if needed, here.  */

    CFE_SB_TimeStampMsg((CFE_SB_Msg_t*)&MAVLINK_AppData.HkTlm);
    int32 iStatus = CFE_SB_SendMsg((CFE_SB_Msg_t*)&MAVLINK_AppData.HkTlm);
    if (iStatus != CFE_SUCCESS)
    {
        /* TODO: Decide what to do if the send message fails. */
    }
}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* Publish Output Data                                             */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

void MAVLINK_SendOutData()
{
    /* TODO:  Add code to update output data, if needed, here.  */

    CFE_SB_TimeStampMsg((CFE_SB_Msg_t*)&MAVLINK_AppData.OutData);
    int32 iStatus = CFE_SB_SendMsg((CFE_SB_Msg_t*)&MAVLINK_AppData.OutData);
    if (iStatus != CFE_SUCCESS)
    {
        /* TODO: Decide what to do if the send message fails. */
    }
}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* Verify Command Length                                           */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

boolean MAVLINK_VerifyCmdLength(CFE_SB_Msg_t* MsgPtr,
                           uint16 usExpectedLen)
{
    boolean bResult  = TRUE;
    uint16  usMsgLen = 0;

    if (MsgPtr != NULL)
    {
        usMsgLen = CFE_SB_GetTotalMsgLength(MsgPtr);

        if (usExpectedLen != usMsgLen)
        {
            bResult = FALSE;
            CFE_SB_MsgId_t MsgId = CFE_SB_GetMsgId(MsgPtr);
            uint16 usCmdCode = CFE_SB_GetCmdCode(MsgPtr);

            (void) CFE_EVS_SendEvent(MAVLINK_MSGLEN_ERR_EID, CFE_EVS_ERROR,
                              "Rcvd invalid msgLen: msgId=0x%08X, cmdCode=%d, "
                              "msgLen=%d, expectedLen=%d",
                              MsgId, usCmdCode, usMsgLen, usExpectedLen);
            MAVLINK_AppData.HkTlm.usCmdErrCnt++;
        }
    }

    return (bResult);
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* MAVLINK application entry point and main process loop               */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

void MAVLINK_AppMain()
{
    /* Register the application with Executive Services */
    MAVLINK_AppData.uiRunStatus = CFE_ES_APP_RUN;

    int32 iStatus = CFE_ES_RegisterApp();
    if (iStatus != CFE_SUCCESS)
    {
        (void) CFE_ES_WriteToSysLog("MAVLINK - Failed to register the app (0x%08X)\n", (unsigned int)iStatus);
    }

    /* Start Performance Log entry */
    CFE_ES_PerfLogEntry(MAVLINK_MAIN_TASK_PERF_ID);

    /* Perform application initializations */
    if (iStatus == CFE_SUCCESS)
    {
        iStatus = MAVLINK_InitApp();
    }

    if (iStatus == CFE_SUCCESS)
    {
        /* Do not perform performance monitoring on startup sync */
        CFE_ES_PerfLogExit(MAVLINK_MAIN_TASK_PERF_ID);
        CFE_ES_WaitForStartupSync(MAVLINK_STARTUP_TIMEOUT_MSEC);
        CFE_ES_PerfLogEntry(MAVLINK_MAIN_TASK_PERF_ID);
    }
    else
    {
        MAVLINK_AppData.uiRunStatus = CFE_ES_APP_ERROR;
    }

    /* Application main loop */
    while (CFE_ES_RunLoop(&MAVLINK_AppData.uiRunStatus) == TRUE)
    {
        int32 iStatus = MAVLINK_RcvMsg(MAVLINK_SCH_PIPE_PEND_TIME);
        if (iStatus != CFE_SUCCESS)
        {
            /* TODO: Decide what to do for other return values in MAVLINK_RcvMsg(). */
        }
        /* TODO: This is only a suggestion for when to update and save CDS table.
        ** Depends on the nature of the application, the frequency of update
        ** and save can be more or less independently.
        */
        MAVLINK_UpdateCdsTbl();
        MAVLINK_SaveCdsTbl();
        //MAVLINK_SendHeartbeat();
        iStatus = MAVLINK_AcquireConfigPointers();
        if(iStatus != CFE_SUCCESS)
        {
            /* We apparently tried to load a new table but failed.  Terminate the application. */
            MAVLINK_AppData.uiRunStatus = CFE_ES_APP_ERROR;
        }
    }

    /* Stop Performance Log entry */
    CFE_ES_PerfLogExit(MAVLINK_MAIN_TASK_PERF_ID);

    /* Exit the application */
    CFE_ES_ExitApp(MAVLINK_AppData.uiRunStatus);
}

/************************/
/*  End of File Comment */
/************************/
