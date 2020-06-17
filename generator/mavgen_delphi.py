#!/usr/bin/env python
'''
parse a MAVLink protocol XML file and generate a C implementation

Copyright Andrew Tridgell 2011
Released under GNU GPL version 3 or later
'''
from __future__ import print_function

from builtins import range
from builtins import object

import os
from . import mavparse, mavtemplate

t = mavtemplate.MAVTemplate()

class msgs(object):
	msgids 	= []
msg = msgs()

def generate_version_h(directory, xml):
    '''generate version.h'''
    f = open(os.path.join(directory, "version.pas"), mode='w')
    t.write(f,'''
(** @file
 *  @brief MAVLink comm protocol built from ${basename}.xml
 *  @see http://mavlink.org
 *)
unit version;
interface

const

MAVLINK_BUILD_DATE = '${parse_time}';
MAVLINK_WIRE_PROTOCOL_VERSION = '${wire_protocol_version}';
MAVLINK_MAX_DIALECT_PAYLOAD_SIZE = ${largest_payload};

implementation

end.
 
''', xml)
    f.close()

def generate_mavlink_h(directory, xml):
    '''generate mavlink.pas'''
    for m in xml.message:
        msg.msgids.append(m.msg_name)
    f = open(os.path.join(directory, "mavlink.pas"), mode='w')
    t.write(f,'''
(** @file
 *  @brief MAVLink comm protocol built from {basename}.xml
 *  @see http://mavlink.org
 *)
unit mavlink;
interface
	
const

  MAVLINK_MESSAGE_CRCS: array[0..255] of byte = (${message_crcs_array});
  MAVLINK_MSG_ID_HEARTBEAT = $00000000;
  MAVLINK_MSG_ID_SYS_STATUS = $00000001;
  MAVLINK_MSG_ID_SYSTEM_TIME = $00000002;
  MAVLINK_MSG_ID_PING = $00000004;
  MAVLINK_MSG_ID_CHANGE_OPERATOR_CONTROL = $00000005;
  MAVLINK_MSG_ID_CHANGE_OPERATOR_CONTROL_ACK = $00000006;
  MAVLINK_MSG_ID_AUTH_KEY = $00000007;
  MAVLINK_MSG_ID_SET_MODE = $0000000B;
  MAVLINK_MSG_ID_PARAM_REQUEST_READ = $00000014;
  MAVLINK_MSG_ID_PARAM_REQUEST_LIST = $00000015;
  MAVLINK_MSG_ID_PARAM_VALUE = $00000016;
  MAVLINK_MSG_ID_PARAM_SET = $00000017;
  MAVLINK_MSG_ID_GPS_RAW_INT = $00000018;
  MAVLINK_MSG_ID_GPS_STATUS = $00000019;
  MAVLINK_MSG_ID_SCALED_IMU = $0000001A;
  MAVLINK_MSG_ID_RAW_IMU = $0000001B;
  MAVLINK_MSG_ID_RAW_PRESSURE = $0000001C;
  MAVLINK_MSG_ID_SCALED_PRESSURE = $0000001D;
  MAVLINK_MSG_ID_ATTITUDE = $0000001E;
  MAVLINK_MSG_ID_ATTITUDE_QUATERNION = $0000001F;
  MAVLINK_MSG_ID_LOCAL_POSITION_NED = $00000020;
  MAVLINK_MSG_ID_GLOBAL_POSITION_INT = $00000021;
  MAVLINK_MSG_ID_RC_CHANNELS_SCALED = $00000022;
  MAVLINK_MSG_ID_RC_CHANNELS_RAW = $00000023;
  MAVLINK_MSG_ID_SERVO_OUTPUT_RAW = $00000024;
  MAVLINK_MSG_ID_MISSION_REQUEST_PARTIAL_LIST = $00000025;
  MAVLINK_MSG_ID_MISSION_WRITE_PARTIAL_LIST = $00000026;
  MAVLINK_MSG_ID_MISSION_ITEM = $00000027;
  MAVLINK_MSG_ID_MISSION_REQUEST = $00000028;
  MAVLINK_MSG_ID_MISSION_SET_CURRENT = $00000029;
  MAVLINK_MSG_ID_MISSION_CURRENT = $0000002A;
  MAVLINK_MSG_ID_MISSION_REQUEST_LIST = $0000002B;
  MAVLINK_MSG_ID_MISSION_COUNT = $0000002C;
  MAVLINK_MSG_ID_MISSION_CLEAR_ALL = $0000002D;
  MAVLINK_MSG_ID_MISSION_ITEM_REACHED = $0000002E;
  MAVLINK_MSG_ID_MISSION_ACK = $0000002F;
  MAVLINK_MSG_ID_SET_GPS_GLOBAL_ORIGIN = $00000030;
  MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN = $00000031;
  MAVLINK_MSG_ID_PARAM_MAP_RC = $00000032;
  MAVLINK_MSG_ID_SAFETY_SET_ALLOWED_AREA = $00000036;
  MAVLINK_MSG_ID_SAFETY_ALLOWED_AREA = $00000037;
  MAVLINK_MSG_ID_ATTITUDE_QUATERNION_COV = $0000003D;
  MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT = $0000003E;
  MAVLINK_MSG_ID_GLOBAL_POSITION_INT_COV = $0000003F;
  MAVLINK_MSG_ID_LOCAL_POSITION_NED_COV = $00000040;
  MAVLINK_MSG_ID_RC_CHANNELS = $00000041;
  MAVLINK_MSG_ID_REQUEST_DATA_STREAM = $00000042;
  MAVLINK_MSG_ID_DATA_STREAM = $00000043;
  MAVLINK_MSG_ID_MANUAL_CONTROL = $00000045;
  MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE = $00000046;
  MAVLINK_MSG_ID_MISSION_ITEM_INT = $00000049;
  MAVLINK_MSG_ID_VFR_HUD = $0000004A;
  MAVLINK_MSG_ID_COMMAND_INT = $0000004B;
  MAVLINK_MSG_ID_COMMAND_LONG = $0000004C;
  MAVLINK_MSG_ID_COMMAND_ACK = $0000004D;
  MAVLINK_MSG_ID_MANUAL_SETPOINT = $00000051;
  MAVLINK_MSG_ID_SET_ATTITUDE_TARGET = $00000052;
  MAVLINK_MSG_ID_ATTITUDE_TARGET = $00000053;
  MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED = $00000054;
  MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED = $00000055;
  MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT = $00000056;
  MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT = $00000057;
  MAVLINK_MSG_ID_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET = $00000059;
  MAVLINK_MSG_ID_HIL_STATE = $0000005A;
  MAVLINK_MSG_ID_HIL_CONTROLS = $0000005B;
  MAVLINK_MSG_ID_HIL_RC_INPUTS_RAW = $0000005C;
  MAVLINK_MSG_ID_OPTICAL_FLOW = $00000064;
  MAVLINK_MSG_ID_GLOBAL_VISION_POSITION_ESTIMATE = $00000065;
  MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE = $00000066;
  MAVLINK_MSG_ID_VISION_SPEED_ESTIMATE = $00000067;
  MAVLINK_MSG_ID_VICON_POSITION_ESTIMATE = $00000068;
  MAVLINK_MSG_ID_HIGHRES_IMU = $00000069;
  MAVLINK_MSG_ID_OPTICAL_FLOW_RAD = $0000006A;
  MAVLINK_MSG_ID_HIL_SENSOR = $0000006B;
  MAVLINK_MSG_ID_SIM_STATE = $0000006C;
  MAVLINK_MSG_ID_RADIO_STATUS = $0000006D;
  MAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL = $0000006E;
  MAVLINK_MSG_ID_TIMESYNC = $0000006F;
  MAVLINK_MSG_ID_HIL_GPS = $00000071;
  MAVLINK_MSG_ID_HIL_OPTICAL_FLOW = $00000072;
  MAVLINK_MSG_ID_HIL_STATE_QUATERNION = $00000073;
  MAVLINK_MSG_ID_SCALED_IMU2 = $00000074;
  MAVLINK_MSG_ID_LOG_REQUEST_LIST = $00000075;
  MAVLINK_MSG_ID_LOG_ENTRY = $00000076;
  MAVLINK_MSG_ID_LOG_REQUEST_DATA = $00000077;
  MAVLINK_MSG_ID_LOG_DATA = $00000078;
  MAVLINK_MSG_ID_LOG_ERASE = $00000079;
  MAVLINK_MSG_ID_LOG_REQUEST_END = $0000007A;
  MAVLINK_MSG_ID_GPS_INJECT_DATA = $0000007B;
  MAVLINK_MSG_ID_GPS2_RAW = $0000007C;
  MAVLINK_MSG_ID_POWER_STATUS = $0000007D;
  MAVLINK_MSG_ID_SERIAL_CONTROL = $0000007E;
  MAVLINK_MSG_ID_GPS_RTK = $0000007F;
  MAVLINK_MSG_ID_GPS2_RTK = $00000080;
  MAVLINK_MSG_ID_SCALED_IMU3 = $00000081;
  MAVLINK_MSG_ID_DATA_TRANSMISSION_HANDSHAKE = $00000082;
  MAVLINK_MSG_ID_ENCAPSULATED_DATA = $00000083;
  MAVLINK_MSG_ID_DISTANCE_SENSOR = $00000084;
  MAVLINK_MSG_ID_TERRAIN_REQUEST = $00000085;
  MAVLINK_MSG_ID_TERRAIN_DATA = $00000086;
  MAVLINK_MSG_ID_TERRAIN_CHECK = $00000087;
  MAVLINK_MSG_ID_TERRAIN_REPORT = $00000088;
  MAVLINK_MSG_ID_SCALED_PRESSURE2 = $00000089;
  MAVLINK_MSG_ID_ATT_POS_MOCAP = $0000008A;
  MAVLINK_MSG_ID_SET_ACTUATOR_CONTROL_TARGET = $0000008B;
  MAVLINK_MSG_ID_ACTUATOR_CONTROL_TARGET = $0000008C;
  MAVLINK_MSG_ID_SCALED_PRESSURE3 = $0000008F;
  MAVLINK_MSG_ID_BATTERY_STATUS = $00000093;
  MAVLINK_MSG_ID_AUTOPILOT_VERSION = $00000094;
  MAVLINK_MSG_ID_LANDING_TARGET = $00000095;
  MAVLINK_MSG_ID_SENSOR_OFFSETS = $00000096;
  MAVLINK_MSG_ID_SET_MAG_OFFSETS = $00000097;
  MAVLINK_MSG_ID_MEMINFO = $00000098;
  MAVLINK_MSG_ID_AP_ADC = $00000099;
  MAVLINK_MSG_ID_DIGICAM_CONFIGURE = $0000009A;
  MAVLINK_MSG_ID_DIGICAM_CONTROL = $0000009B;
  MAVLINK_MSG_ID_MOUNT_CONFIGURE = $0000009C;
  MAVLINK_MSG_ID_MOUNT_CONTROL = $0000009D;
  MAVLINK_MSG_ID_MOUNT_STATUS = $0000009E;
  MAVLINK_MSG_ID_FENCE_POINT = $000000A0;
  MAVLINK_MSG_ID_FENCE_FETCH_POINT = $000000A1;
  MAVLINK_MSG_ID_FENCE_STATUS = $000000A2;
  MAVLINK_MSG_ID_AHRS = $000000A3;
  MAVLINK_MSG_ID_SIMSTATE = $000000A4;
  MAVLINK_MSG_ID_HWSTATUS = $000000A5;
  MAVLINK_MSG_ID_RADIO = $000000A6;
  MAVLINK_MSG_ID_LIMITS_STATUS = $000000A7;
  MAVLINK_MSG_ID_WIND = $000000A8;
  MAVLINK_MSG_ID_DATA16 = $000000A9;
  MAVLINK_MSG_ID_DATA32 = $000000AA;
  MAVLINK_MSG_ID_DATA64 = $000000AB;
  MAVLINK_MSG_ID_DATA96 = $000000AC;
  MAVLINK_MSG_ID_RANGEFINDER = $000000AD;
  MAVLINK_MSG_ID_AIRSPEED_AUTOCAL = $000000AE;
  MAVLINK_MSG_ID_RALLY_POINT = $000000AF;
  MAVLINK_MSG_ID_RALLY_FETCH_POINT = $000000B0;
  MAVLINK_MSG_ID_COMPASSMOT_STATUS = $000000B1;
  MAVLINK_MSG_ID_AHRS2 = $000000B2;
  MAVLINK_MSG_ID_CAMERA_STATUS = $000000B3;
  MAVLINK_MSG_ID_CAMERA_FEEDBACK = $000000B4;
  MAVLINK_MSG_ID_BATTERY2 = $000000B5;
  MAVLINK_MSG_ID_AHRS3 = $000000B6;
  MAVLINK_MSG_ID_AUTOPILOT_VERSION_REQUEST = $000000B7;
  MAVLINK_MSG_ID_REMOTE_LOG_DATA_BLOCK = $000000B8;
  MAVLINK_MSG_ID_REMOTE_LOG_BLOCK_STATUS = $000000B9;
  MAVLINK_MSG_ID_LED_CONTROL = $000000BA;
  MAVLINK_MSG_ID_MAG_CAL_PROGRESS = $000000BF;
  MAVLINK_MSG_ID_MAG_CAL_REPORT = $000000C0;
  MAVLINK_MSG_ID_EKF_STATUS_REPORT = $000000C1;
  MAVLINK_MSG_ID_PID_TUNING = $000000C2;
  MAVLINK_MSG_ID_GIMBAL_REPORT = $000000C8;
  MAVLINK_MSG_ID_GIMBAL_CONTROL = $000000C9;
  MAVLINK_MSG_ID_GIMBAL_RESET = $000000CA;
  MAVLINK_MSG_ID_GIMBAL_AXIS_CALIBRATION_PROGRESS = $000000CB;
  MAVLINK_MSG_ID_GIMBAL_SET_HOME_OFFSETS = $000000CC;
  MAVLINK_MSG_ID_GIMBAL_HOME_OFFSET_CALIBRATION_RESULT = $000000CD;
  MAVLINK_MSG_ID_GIMBAL_SET_FACTORY_PARAMETERS = $000000CE;
  MAVLINK_MSG_ID_GIMBAL_FACTORY_PARAMETERS_LOADED = $000000CF;
  MAVLINK_MSG_ID_GIMBAL_ERASE_FIRMWARE_AND_CONFIG = $000000D0;
  MAVLINK_MSG_ID_GIMBAL_PERFORM_FACTORY_TESTS = $000000D1;
  MAVLINK_MSG_ID_GIMBAL_REPORT_FACTORY_TESTS_PROGRESS = $000000D2;
  MAVLINK_MSG_ID_GIMBAL_REQUEST_AXIS_CALIBRATION_STATUS = $000000D3;
  MAVLINK_MSG_ID_GIMBAL_REPORT_AXIS_CALIBRATION_STATUS = $000000D4;
  MAVLINK_MSG_ID_GIMBAL_REQUEST_AXIS_CALIBRATION = $000000D5;
  MAVLINK_MSG_ID_GOPRO_HEARTBEAT = $000000D7;
  MAVLINK_MSG_ID_GOPRO_GET_REQUEST = $000000D8;
  MAVLINK_MSG_ID_GOPRO_GET_RESPONSE = $000000D9;
  MAVLINK_MSG_ID_GOPRO_SET_REQUEST = $000000DA;
  MAVLINK_MSG_ID_GOPRO_SET_RESPONSE = $000000DB;
  MAVLINK_MSG_ID_RPM = $000000E2;
  MAVLINK_MSG_ID_VIBRATION = $000000F1;
  MAVLINK_MSG_ID_HOME_POSITION = $000000F2;
  MAVLINK_MSG_ID_SET_HOME_POSITION = $000000F3;
  MAVLINK_MSG_ID_ADSB_VEHICLE = $000000F6;
  MAVLINK_MSG_ID_V2_EXTENSION = $000000F8;
  MAVLINK_MSG_ID_MEMORY_VECT = $000000F9;
  MAVLINK_MSG_ID_DEBUG_VECT = $000000FA;
  MAVLINK_MSG_ID_NAMED_VALUE_FLOAT = $000000FB;
  MAVLINK_MSG_ID_NAMED_VALUE_INT = $000000FC;
  MAVLINK_MSG_ID_STATUSTEXT = $000000FD;
  MAVLINK_MSG_ID_DEBUG = $000000FE;



implementation

end.
''', xml)
    f.close()

def generate_main_h(directory, xml):
    '''generate main header per XML file'''
    f = open(os.path.join(directory, "mavlink_"+xml.basename + ".pas"), mode='w')
    t.write(f, '''
(** @file
 *  @brief MAVLink comm protocol generated from ${basename}.xml
 *  @see http://mavlink.org
 *)
unit mavlink_${basename};
interface
uses 
	// MESSAGE DEFINITIONS
	${{message: //mavlink_msg_${name_lower},
	}}
	// base include
	${{include_list: // "../${base}/${base}.pas",
	}}
	mavlink_types;
	
// MESSAGE LENGTHS AND CRCS

//MAVLINK_MESSAGE_CRCS: array[0..255] of byte = (50, 124, 137, 0, 237, 217, 104, 119, 0, 0, 0, 89, 0, 0, 0, 0, 0, 0, 0, 0, 214, 159, 220, 168, 24, 23, 170, 144, 67, 115, 39, 246, 185, 104, 237, 244, 222, 212, 9, 254, 230, 28, 28, 132, 221, 232, 11, 153, 41, 39, 78, 0, 0, 0, 15, 3, 0, 0, 0, 0, 0, 153, 183, 51, 82, 118, 148, 21, 0, 243, 124, 0, 0, 38, 20, 158, 152, 143, 0, 0, 0, 106, 49, 22, 143, 140, 5, 150, 0, 231, 183, 63, 54, 0, 0, 0, 0, 0, 0, 0, 175, 102, 158, 208, 56, 93, 138, 108, 32, 185, 84, 34, 0,
//    124, 237, 4, 76, 128, 56, 116, 134, 237, 203, 250, 87, 203, 220, 25, 226, 46, 29, 223, 85, 6, 229, 203, 1, 195, 109, 168, 181, 0, 0, 131, 0, 0, 0, 154, 178, 200, 134, 219, 208, 188, 84, 22, 19, 21, 134, 0, 78, 68, 189, 127, 154, 21, 21, 144, 1, 234, 73, 181, 22, 83, 167, 138, 234, 240, 47, 189, 52, 174, 229, 85, 159, 186, 72, 0, 0, 0, 0, 92, 36, 71, 98, 0, 0, 0, 0, 0, 134, 205, 94, 128, 54, 63, 112, 201, 221, 226, 238, 103, 235, 14, 0, 77, 50, 163, 115, 47, 0, 0, 0, 0, 0, 0, 207, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 90, 104, 85, 0, 0, 184, 0, 8, 204, 49, 170, 44, 83, 46, 0);
//  X25_INIT_CRC: integer = $ffff;
//  MAVLINK_MESSAGE_LENGTHS: array[0..255] of byte = (9, 31, 12, 0, 14, 28, 3, 32, 0, 0, 0, 6, 0, 0, 0, 0, 0, 0, 0, 0, 20, 2, 25, 23, 30, 101, 22, 26, 16, 14, 28, 32, 28, 28, 22, 22, 21, 6, 6, 37, 4, 4, 2, 2, 4, 2, 2, 3, 13, 12, 37, 0, 0, 0, 27, 25, 0, 0, 0, 0, 0, 68, 26, 185, 181, 42, 6, 4, 0, 11, 18, 0, 0, 37, 20, 35, 33, 3, 0, 0, 0, 22, 39, 37, 53, 51, 53, 51, 0, 28, 56, 42, 33, 0, 0, 0, 0, 0, 0, 0, 26, 32, 32, 20, 32, 62, 44, 64, 84, 9, 254, 16, 0, 36, 44, 64, 22, 6, 14, 12, 97, 2, 2, 113, 35, 6, 79, 35,
//    35, 22, 13, 255, 14, 18, 43, 8, 22, 14, 36, 43, 41, 0, 0, 14, 0, 0, 0, 36, 60, 30, 42, 8, 4, 12, 15, 13, 6, 15, 14, 0, 12, 3, 8, 28, 44, 3, 9, 22, 12, 18, 34, 66, 98, 8, 48, 19, 3, 20, 24, 29, 45, 4, 40, 2, 206, 7, 29, 0, 0, 0, 0, 27, 44, 22, 25, 0, 0, 0, 0, 0, 42, 14, 2, 3, 2, 1, 33, 1, 6, 2, 4, 2, 3, 2, 0, 1, 3, 2, 4, 2, 0, 0, 0, 0, 0, 0, 8, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 32, 52, 53, 0, 0, 38, 0, 254, 36, 30, 18, 18, 51, 9, 0);


//#include "../protocol.h"

//#define MAVLINK_ENABLED_${basename_upper}
// MAVLINK VERSION
const
	MAVLINK_VERSION = ${version};

// ENUM DEFINITIONS
type
${{enum:
(** @brief ${description} *)
//{$ifndef HAVE_ENUM_${name} }
//#define HAVE_ENUM_${name}
${name}=
(
${{entry:   ${name}=${value},  {${description} |${{param:${description}| }} } 
}}
   ${name}_DUMMY
);
//{$EndIf}
}}

implementation

end.
''', xml)
    f.close()
            

def generate_message_h(directory, m):
    '''generate per-message header for a XML file'''
    f = open(os.path.join(directory, 'mavlink_msg_%s.pas' % m.name_lower), mode='w')
    t.write(f, '''
unit mavlink_msg_${name_lower};
// MESSAGE ${name} PACKING
interface
uses 
	mavlink_types, mavlink_helpers, SysUtils;
	
const MAVLINK_MSG_ID_${name} = ${id};

type

mavlink_${name_lower}_t = packed record
${{ordered_fields: ${name}${array_suffix} ${type}; (*< ${description}*)
}}
end;

const
	MAVLINK_MSG_ID_${name}_LEN = ${wire_length};
	MAVLINK_MSG_ID_${name}_MIN_LEN = ${wire_min_length};
	MAVLINK_MSG_ID_${id}_LEN = ${wire_length};
	MAVLINK_MSG_ID_${id}_MIN_LEN = ${wire_min_length};

	MAVLINK_MSG_ID_${name}_CRC = ${crc_extra};
	MAVLINK_MSG_ID_${id}_CRC = ${crc_extra};

${{array_fields:MAVLINK_MSG_${msg_name}_FIELD_${name_upper}_LEN = ${array_length};
}}

(*
{$if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_${name} { \\
    ${id}, \\
    "${name}", \\
    ${num_fields}, \\
    { ${{ordered_fields: { "${name}", ${c_print_format}, MAVLINK_TYPE_${type_upper}, ${array_length}, ${wire_offset}, offsetof(mavlink_${name_lower}_t, ${name}) }, \\
        }} } \\
}
{$ELSE}
#define MAVLINK_MESSAGE_INFO_${name} { \\
    "${name}", \\
    ${num_fields}, \\
    { ${{ordered_fields: { "${name}", ${c_print_format}, MAVLINK_TYPE_${type_upper}, ${array_length}, ${wire_offset}, offsetof(mavlink_${name_lower}_t, ${name}) }, \\
        }} } \\
}
{$IFEND}
*)

function mavlink_msg_${name_lower}_pack(system_id: uint8_t; component_id: uint8_t; var msg: mavlink_message_t ${{arg_fields:; ${array_prefix}${name}:${array_const} }}):uint16;
function mavlink_msg_${name_lower}_pack_chan(system_id: uint8_t; component_id: uint8_t; 
									chan: uint8_t;
									msg: mavlink_message_t ${{arg_fields:; ${array_prefix}${name}:${array_const} }}):uint16;
function mavlink_msg_${name_lower}_encode(system_id: uint8_t; component_id: uint8_t; 
									msg: mavlink_message_t; var ${name_lower}: mavlink_${name_lower}_t ):Uint16; inline;
function mavlink_msg_${name_lower}_encode_chan(system_id: uint8_t; component_id: uint8_t; 
									chan: uint8;
									msg: mavlink_message_t; ${name_lower}: mavlink_${name_lower}_t ):Uint16; inline;
procedure mavlink_msg_${name_lower}_decode(msg: mavlink_message_t; var ${name_lower}: mavlink_${name_lower}_t); inline;
(*${{fields:function mavlink_msg_${name_lower}_get_${name}(msg${get_arg}: mavlink_message_t): ${return_type}; inline;
}}*)

implementation

(**
 * @brief Pack a ${name_lower} message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
${{arg_fields: * @param ${name} ${description}
}}
 * @return length of the message in bytes (excluding serial stream start sign)
 *)
function mavlink_msg_${name_lower}_pack(system_id: uint8_t; component_id: uint8_t; var msg: mavlink_message_t ${{arg_fields:; ${array_prefix}${name}:${array_const} }}):uint16;
var
	packet: mavlink_${name_lower}_t;
begin
   
${{scalar_fields:    packet.${name} := ${putname};
}}
${{array_fields:    move(${name}, packet.${name}, sizeof(${type})*${array_length});
}}
//        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_${name}_LEN);
    move(packet, msg.payload[0], MAVLINK_MSG_ID_${name}_LEN);

    msg.msgid := MAVLINK_MSG_ID_${name};
    Result:= mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_${name}_MIN_LEN, MAVLINK_MSG_ID_${name}_LEN, MAVLINK_MSG_ID_${name}_CRC);
end;

(**
 * @brief Pack a ${name_lower} message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
${{arg_fields: * @param ${name} ${description}
}}
 * @return length of the message in bytes (excluding serial stream start sign)
 *)
function mavlink_msg_${name_lower}_pack_chan(system_id: uint8_t; component_id: uint8_t; 
									chan: uint8_t;
									msg: mavlink_message_t ${{arg_fields:; ${array_prefix}${name}:${array_const} }}):uint16;
var
	packet: mavlink_${name_lower}_t;
begin
(*
{$if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_${name}_LEN];
${{scalar_fields:    _mav_put_${type}(buf, ${wire_offset}, ${putname});
}}
${{array_fields:    _mav_put_${type}_array(buf, ${wire_offset}, ${name}, ${array_length});
}}
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_${name}_LEN);
{$ELSE}
*)
//    mavlink_${name_lower}_t packet;
${{scalar_fields:    packet.${name} := ${putname};
}}
${{array_fields:    move(${name}, packet.${name}, sizeof(${type})*${array_length});
}}
        move(packet, msg.payload[0], MAVLINK_MSG_ID_${name}_LEN);
//        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_${name}_LEN);
//{$IFEND}

    msg.msgid := MAVLINK_MSG_ID_${name};
    Result:= mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_${name}_MIN_LEN, MAVLINK_MSG_ID_${name}_LEN, MAVLINK_MSG_ID_${name}_CRC);
end;

(**
 * @brief Encode a ${name_lower} struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param ${name_lower} C-struct to read the message contents from
 *)
function mavlink_msg_${name_lower}_encode(system_id: uint8_t; component_id: uint8_t; 
									msg: mavlink_message_t; var ${name_lower}: mavlink_${name_lower}_t ):Uint16; inline;
begin
    Result:= mavlink_msg_${name_lower}_pack(system_id, component_id, msg,${{arg_fields: ${name_lower}.${name},}});
end;

(**
 * @brief Encode a ${name_lower} struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param ${name_lower} C-struct to read the message contents from
 *)
function mavlink_msg_${name_lower}_encode_chan(system_id: uint8_t; component_id: uint8_t; 
									chan: uint8;
									msg: mavlink_message_t; ${name_lower}: mavlink_${name_lower}_t ):Uint16; inline;
begin
    Result:= mavlink_msg_${name_lower}_pack_chan(system_id, component_id, chan, msg,${{arg_fields: ${name_lower}.${name},}});
end;

(**
 * @brief Send a ${name_lower} message
 * @param chan MAVLink channel to send the message
 *
${{arg_fields: * @param ${name} ${description}
}}
 *)

// MESSAGE ${name} UNPACKING

${{fields:
(**
 * @brief Get field ${name} from ${name_lower} message
 *
 * @return ${description}
 *)
(*
function mavlink_msg_${name_lower}_get_${name}(msg${get_arg}: mavlink_message_t): ${return_type}; inline;
begin
    Result:= msg.payload[${wire_offset}];
end;
}}
*)

(**
 * @brief Decode a ${name_lower} message into a struct
 *
 * @param msg The message to decode
 * @param ${name_lower} C-struct to decode the message contents into
 *)
procedure mavlink_msg_${name_lower}_decode(msg: mavlink_message_t; var ${name_lower}: mavlink_${name_lower}_t); inline;
var
	len: uint8;
begin
(*
{$if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
${{ordered_fields:    ${decode_left}mavlink_msg_${name_lower}_get_${name}(msg${decode_right});
}}
{$ELSE}
*)
		if (msg.len < MAVLINK_MSG_ID_${name}_LEN) then
			len := msg.len
		else len:= MAVLINK_MSG_ID_${name}_LEN;
        FillChar(${name_lower}, MAVLINK_MSG_ID_${name}_LEN, 0);
		move(msg.payload[0], ${name_lower}, len);
//    memcpy(${name_lower}, msg.payload[0], len);
//{$IFEND}
end;

end.
''', m)
    f.close()


def generate_testsuite_h(directory, xml):
    '''generate testsuite.h per XML file'''
    f = open(os.path.join(directory, "testsuite.h"), mode='w')
    t.write(f, '''
(** @file
 *    @brief MAVLink comm protocol testsuite generated from ${basename}.xml
 *    @see http://qgroundcontrol.org/mavlink/
 *)
#pragma once
{$ifndef ${basename_upper}_TESTSUITE_H
#define ${basename_upper}_TESTSUITE_H

{$ifdef __cplusplus
extern "C" {
{$EndIf}

{$ifndef MAVLINK_TEST_ALL
#define MAVLINK_TEST_ALL
${{include_list:static void mavlink_test_${base}(uint8_t, uint8_t, mavlink_message_t *last_msg);
}}
static void mavlink_test_${basename}(uint8_t, uint8_t, mavlink_message_t *last_msg);

static void mavlink_test_all(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
${{include_list:    mavlink_test_${base}(system_id, component_id, last_msg);
}}
    mavlink_test_${basename}(system_id, component_id, last_msg);
}
{$EndIf}

${{include_list:#include "../${base}/testsuite.h"
}}

${{message:
static void mavlink_test_${name_lower}(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
{$ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status.flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_${name} >= 256) {
            return;
        }
{$EndIf}
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_${name_lower}_t packet_in = {
        ${{ordered_fields:${c_test_value},}}
    };
    mavlink_${name_lower}_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        ${{scalar_fields:packet1.${name} = packet_in.${name};
        }}
        ${{array_fields:mav_array_memcpy(packet1.${name}, packet_in.${name}, sizeof(${type})*${array_length});
        }}
{$ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status.flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_${name}_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_${name}_MIN_LEN);
        }
{$EndIf}
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_${name_lower}_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_${name_lower}_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_${name_lower}_pack(system_id, component_id, &msg ${{arg_fields:, packet1.${name} }});
    mavlink_msg_${name_lower}_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_${name_lower}_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg ${{arg_fields:, packet1.${name} }});
    mavlink_msg_${name_lower}_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_${name_lower}_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_${name_lower}_send(MAVLINK_COMM_1 ${{arg_fields:, packet1.${name} }});
    mavlink_msg_${name_lower}_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}
}}

static void mavlink_test_${basename}(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
${{message:    mavlink_test_${name_lower}(system_id, component_id, last_msg);
}}
}

{$ifdef __cplusplus
}
{$EndIf} // __cplusplus
{$EndIf} // ${basename_upper}_TESTSUITE_H
''', xml)

    f.close()

def copy_fixed_headers(directory, xml):
    '''copy the fixed protocol headers to the target directory'''
    import shutil, filecmp
    hlist = {
        "0.9": [ 'protocol.h', 'mavlink_helpers.h', 'mavlink_types.h', 'checksum.h' ],
        "1.0": [ 'mavlink_protocol.pas', 'mavlink_helpers.pas', 'mavlink_types.pas', 'checksum.pas', 'mavlink_conversions.pas' ],
        "2.0": [ 'mavlink_protocol.pas', 'mavlink_helpers.pas', 'mavlink_types.pas', 'checksum.pas', 'mavlink_conversions.pas',
                 'mavlink_get_info.h', 'mavlink_sha256.h' ]
        }
    basepath = os.path.dirname(os.path.realpath(__file__))
    srcpath = os.path.join(basepath, 'delphi/units_v%s' % xml.wire_protocol_version)
    print("Copying fixed pascal headers for protocol %s to %s" % (xml.wire_protocol_version, directory))
    for h in hlist[xml.wire_protocol_version]:
        src = os.path.realpath(os.path.join(srcpath, h))
        dest = os.path.realpath(os.path.join(directory, h))
        if src == dest or (os.path.exists(dest) and filecmp.cmp(src, dest)):
            continue
        shutil.copy(src, dest)

class mav_include(object):
    def __init__(self, base):
        self.base = base

def generate_one(basename, xml):
    '''generate headers for one XML file'''

    directory = os.path.join(basename, xml.basename)

    print("Generating Delphi implementation in directory %s" % directory)
    mavparse.mkdir_p(directory)

    if xml.little_endian:
        xml.mavlink_endian = "MAVLINK_LITTLE_ENDIAN"
    else:
        xml.mavlink_endian = "MAVLINK_BIG_ENDIAN"

    if xml.crc_extra:
        xml.crc_extra_define = "1"
    else:
        xml.crc_extra_define = "0"

    if xml.command_24bit:
        xml.command_24bit_define = "1"
    else:
        xml.command_24bit_define = "0"

    if xml.sort_fields:
        xml.aligned_fields_define = "1"
    else:
        xml.aligned_fields_define = "0"

    # work out the included headers
    xml.include_list = []
    for i in xml.include:
        base = i[:-4]
        xml.include_list.append(mav_include(base))

    # form message lengths array
    xml.message_lengths_array = ''
    if not xml.command_24bit:
        for msgid in range(256):
            mlen = xml.message_min_lengths.get(msgid, 0)
            xml.message_lengths_array += '%u, ' % mlen
        xml.message_lengths_array = xml.message_lengths_array[:-2]

    # and message CRCs array
    xml.message_crcs_array = ''
    if xml.command_24bit:
        # we sort with primary key msgid
        for msgid in sorted(xml.message_crcs.keys()):
            xml.message_crcs_array += '{%u, %u, %u, %u, %u, %u}, ' % (msgid,
                                                                      xml.message_crcs[msgid],
                                                                      xml.message_min_lengths[msgid],
                                                                      xml.message_flags[msgid],
                                                                      xml.message_target_system_ofs[msgid],
                                                                      xml.message_target_component_ofs[msgid])
    else:
        for msgid in range(256):
            crc = xml.message_crcs.get(msgid, 0)
            xml.message_crcs_array += '%u, ' % crc
    xml.message_crcs_array = xml.message_crcs_array[:-2]

    # form message info array
    xml.message_info_array = ''
    if xml.command_24bit:
        # we sort with primary key msgid
        for msgid in sorted(xml.message_names.keys()):
            name = xml.message_names[msgid]
            xml.message_info_array += 'MAVLINK_MESSAGE_INFO_%s, ' % name
    else:
        for msgid in range(256):
            name = xml.message_names.get(msgid, None)
            if name is not None:
                xml.message_info_array += 'MAVLINK_MESSAGE_INFO_%s, ' % name
            else:
                # Several C compilers don't accept {NULL} for
                # multi-dimensional arrays and structs
                # feed the compiler a "filled" empty message
                xml.message_info_array += '{"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, '
    xml.message_info_array = xml.message_info_array[:-2]

    # add some extra field attributes for convenience with arrays
    for m in xml.message:
        m.msg_name = m.name
        if xml.crc_extra:
            m.crc_extra_arg = ", %s" % m.crc_extra
        else:
            m.crc_extra_arg = ""
        for f in m.fields:
            if f.name == "type":
                f.name = 'a%s' % (f.name)
            if f.print_format is None:
                f.c_print_format = 'NULL'
            else:
                f.c_print_format = '"%s"' % f.print_format
            if f.array_length != 0:
                f.array_suffix = ': array [0..%u] of' % (f.array_length - 1)
                f.array_prefix = 'const '
                f.array_tag = '_array'
                f.array_arg = ', %u' % f.array_length
                f.array_return_arg = '%s, %u, ' % (f.name, f.array_length)
                f.decode_left = ''
                f.decode_right = ', %s.%s' % (m.name_lower, f.name)
                f.return_type = 'Uint16'
                f.get_arg = ', %s *%s' % (f.type, f.name)
                if f.type == 'char':
                    f.c_test_value = '"%s"' % f.test_value
                    f.type = 'AnsiChar'
                else:
                    test_strings = []
                    for v in f.test_value:
                        test_strings.append(str(v))
                    f.c_test_value = '{ %s }' % ', '.join(test_strings)
                f.array_const = ' array of %s' % (f.type)
            else:
                f.array_suffix = ':'
                f.array_prefix = ''
                f.array_tag = ''
                f.array_arg = ''
                f.array_return_arg = ''
                f.decode_left = "%s.%s = " % (m.name_lower, f.name)
                f.decode_right = ''
                f.get_arg = ''
                f.return_type = f.type
                if f.type == 'char':
                    f.c_test_value = "'%s'" % f.test_value
                    f.type = 'AnsiChar'
                elif f.type == 'Uint64':
                    f.c_test_value = "%sULL" % f.test_value                    
                elif f.type == 'Int64':
                    f.c_test_value = "%sLL" % f.test_value                    
                else:
                    f.c_test_value = f.test_value
                f.array_const = f.type

    # cope with uint8_t_mavlink_version
    for m in xml.message:
        m.arg_fields = []
        m.array_fields = []
        m.scalar_fields = []
        for f in m.ordered_fields:
            if f.array_length != 0:
                m.array_fields.append(f)
            else:
                m.scalar_fields.append(f)
        for f in m.fields:
            if not f.omit_arg:
                m.arg_fields.append(f)
                f.putname = f.name
            else:
                f.putname = f.const_value

    generate_mavlink_h(directory, xml)
    generate_version_h(directory, xml)
    generate_main_h(directory, xml)
    for m in xml.message:
        generate_message_h(directory, m)
#    generate_testsuite_h(directory, xml)


def generate(basename, xml_list):
    '''generate complete MAVLink Delphi implemenation'''

    for idx in range(len(xml_list)):
        xml = xml_list[idx]
        xml.xml_idx = idx
        generate_one(basename, xml)
    copy_fixed_headers(basename, xml_list[0])
