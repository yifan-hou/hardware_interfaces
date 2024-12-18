#pragma once

//! Status codes
typedef enum
{
    E_SUCCESS               = 0,  //!< No error
    E_NOT_AVAILABLE         = 1,  //!< Device, service or data is not avialable
    E_NO_SENSOR             = 2,  //!< No sensor connected
    E_NOT_INITIALIZED       = 3,  //!< The device is not initialized
    E_ALREADY_RUNNING       = 4,  //!< Service is already running
    E_FEATURE_NOT_SUPPORTED = 5,  //!< The asked feature is not supported
    E_INCONSISTENT_DATA     = 6,  //!< One or more dependent parameters mismatch
    E_TIMEOUT               = 7,  //!< Timeout error
    E_READ_ERROR            = 8,  //!< Error while reading from a device
    E_WRITE_ERROR           = 9,  //!< Error while writing to a device
    E_INSUFFICIENT_RESOURCES= 10, //!< No memory available
    E_CHECKSUM_ERROR        = 11, //!< Checksum error
    E_NO_PARAM_EXPECTED     = 12, //!< No parameters expected
    E_NOT_ENOUGH_PARAMS     = 13, //!< Not enough parameters
    E_CMD_UNKNOWN           = 14, //!< Unknown command
    E_CMD_FORMAT_ERROR      = 15, //!< Comand format error
    E_ACCESS_DENIED         = 16, //!< Access denied
    E_ALREADY_OPEN          = 17, //!< The interface is already open
    E_CMD_FAILED            = 18, //!< Command failed
    E_CMD_ABORTED           = 19, //!< Command aborted
    E_INVALID_HANDLE        = 20, //!< Invalid handle
    E_NOT_FOUND             = 21, //!< Device not found
    E_NOT_OPEN              = 22, //!< Device not open
    E_IO_ERROR              = 23, //!< I/O error
    E_INVALID_PARAMETER     = 24, //!< Invalid parameter
    E_INDEX_OUT_OF_BOUNDS   = 25, //!< Index out of bounds
    E_CMD_PENDING           = 26, //!< Command execution needs more time
    E_OVERRUN               = 27, //!< Data overrun
    E_RANGE_ERROR           = 28, //!< Range error
    E_AXIS_BLOCKED          = 29, //!< Axis is blocked
    E_FILE_EXISTS           = 30 //!< File already exists
} TStat;


typedef struct {
    unsigned short length;  //!< Length of message's payload in bytes
                            //!  (0, if the message has no payload)
    unsigned char id;       //!< ID of the message
    unsigned char *data;    //!< Pointer to the message's payload
} TMESSAGE;                 //!< command message format


typedef struct {
    bool SF_SCRIPT_FAILURE;
    bool SF_SCRIPT_RUNNING;
    bool SF_CMD_FAILURE;
    bool SF_FINGER_FAULT;
    bool SF_CURR_FAULT;     // Engine error
    bool SF_POWER_FAULT;
    bool SF_TEMP_FAULT;
    bool SF_TEMP_WARNING;
    bool SF_FAST_STOP;
    bool SF_FORCECTRL_MODE; // force control mode
    bool SF_TARGET_POS_REACHED;
    bool SF_AXIS_STOPPED;   // a previous command has been stopped using the stop command
    bool SF_SOFT_LIMIT_PLUS;// positive direction soft limit reached
    bool SF_SOFT_LIMIT_MINUS; // negative direction soft limit reached
    bool SF_BLOCKED_PLUS;   // axis is blocked in positive movement direction
    bool SF_BLOCKED_MINUS;  // axis is blocked in negative movement direction
    bool SF_MOVING;         // fingers are currently moving
    bool SF_REFERENCED;     // gripper is referenced and accepts motion commands
} SSTATE;


//! Response Messages
typedef struct
{
    unsigned char id;
    short length;
    TStat status_code;
    unsigned char *data;
} TRESPONSE;


class WSG50Observer
{
public:
    virtual void update(TRESPONSE * response) = 0;

};
