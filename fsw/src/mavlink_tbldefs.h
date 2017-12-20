    
#ifndef MAVLINK_TBLDEFS_H
#define MAVLINK_TBLDEFS_H

/************************************************************************
** Pragmas
*************************************************************************/

/************************************************************************
** Includes
*************************************************************************/
#include "cfe.h"
#include "mavlink_platform_cfg.h"
#include "mavlink_msg.h"

#ifdef __cplusplus
extern "C" {
#endif

/************************************************************************
** Local Defines
*************************************************************************/
/**
 * \brief Defines the number of entries in the table
 */
#define MAVLINK_PARAM_TABLE_MAX_ENTRIES  (10)

/**
 * \brief Defines the table identification name used for table registration.
 */
#define MAVLINK_PARAM_TABLENAME          ("PARAM_TBL")

/**
 * \brief Defines the table file name used for table registration.
 */
#define MAVLINK_CDS_TABLENAME  ("mavlink_CdsTbl")

/************************************************************************
** Local Structure Declarations
*************************************************************************/

/** \brief Definition for a single config table entry */
typedef struct
{
	uint8 enabled;
	uint8 vehicle_id;
	uint8 component_id;
	MAVLINK_ParamData_t param_data;
} MAVLINK_ParamTblData_t;

/** \brief Definition for a single config table entry */
typedef struct
{
    uint8                   tableID;
    MAVLINK_ParamTblData_t     params[MAVLINK_PARAM_TABLE_MAX_ENTRIES];
} MAVLINK_ParamTblEntry_t;


/** \brief Definition for Critical Data Storage (CDS) table entry */
typedef struct
{
    int32  iParam;

    /* TODO:  Add type declaration for CDS data here. */
} MAVLINK_CdsTbl_t;

/************************************************************************
** External Global Variables
*************************************************************************/

/************************************************************************
** Global Variables
*************************************************************************/

/************************************************************************
** Local Variables
*************************************************************************/

/************************************************************************
** Local Function Prototypes
*************************************************************************/


#ifdef __cplusplus
}
#endif

#endif /* MAVLINK_TBLDEFS_H */

/************************/
/*  End of File Comment */
/************************/
