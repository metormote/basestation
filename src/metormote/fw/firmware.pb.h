/* Automatically generated nanopb header */
/* Generated by nanopb-0.1.9-dev at Thu Jun  6 18:30:29 2013. */

#ifndef _PB_FIRMWARE_PB_H_
#define _PB_FIRMWARE_PB_H_
#include <pb.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Enum definitions */
typedef enum _firmware_Update_Method {
    firmware_Update_Method_ROM = 0,
    firmware_Update_Method_FLASH = 1,
    firmware_Update_Method_ANT = 2,
    firmware_Update_Method_BLE = 3
} firmware_Update_Method;

typedef enum _firmware_Response_Status {
    firmware_Response_Status_SUCCESS = 0,
    firmware_Response_Status_NOT_IN_RANGE = 1,
    firmware_Response_Status_TRANSFER_FAILED = 2
} firmware_Response_Status;

/* Struct definitions */
typedef struct _firmware_Firmware {
    uint64_t target;
    uint64_t crc;
    pb_callback_t hex;
} firmware_Firmware;

typedef struct _firmware_Response {
    firmware_Response_Status status;
    bool has_errorCode;
    int32_t errorCode;
} firmware_Response;

typedef struct _firmware_Update {
    firmware_Update_Method method;
    uint64_t crc;
} firmware_Update;

/* Default values for struct fields */

/* Struct field encoding specification for nanopb */
extern const pb_field_t firmware_Firmware_fields[4];
extern const pb_field_t firmware_Update_fields[3];
extern const pb_field_t firmware_Response_fields[3];

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif
