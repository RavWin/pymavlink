#!/usr/bin/env python
'''
parse a MAVLink protocol XML file and generate a Delphi implementation

Copyright Andrew Tridgell 2011
Copyright Andrew Kambaroff a.k.a RaJa 2017-2020
Released under GNU GPL version 3 or later
'''
from __future__ import print_function

from builtins import range
from builtins import object

import os
from . import mavparse_delphi, mavtemplate

t = mavtemplate.MAVTemplate()

class msgs(object):
	msgids 	= []
msg = msgs()
message_crcs = []
crc_list = ''
def generate_version_pas(directory, xml):
    '''generate version.pas'''
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

def generate_mavlink_msg_inc(directory, xml):
    for m in xml.message:
        msg.msgids.append(m.msg_name)
    f = open(os.path.join(directory, "mavlink_msg_"+xml.basename+".inc"), mode='w')
    t.write(f,'''
// uses {$I mavlink_msg_${basename}.inc}
${{message:	mavlink_msg_${name_lower},
}}
''', xml)
    f.close()
def generate_mavlink_crcs(directory, xml):
    f = open(os.path.join(directory, 'mavlink_crcs.pas'), mode='w')
    t.write(f, '''
unit mavlink_crcs;

interface

//const
//  MAVLINK_MESSAGE_CRCS: array[0..255] of byte = 
//  (${message_crcs_array});

implementation

end.
''', xml)
    f.close()

def generate_main_pas(directory, xml):
    '''generate main header per XML file'''
    f = open(os.path.join(directory, 'mavlink_'+xml.basename + ".pas"), mode='w')
    t.write(f, '''
(** @file
 *  @brief MAVLink comm protocol generated from ${basename}.xml
 *  @see http://mavlink.org
 *)
unit mavlink_${basename};
interface
uses 
	mavlink;
// MESSAGE LENGTHS AND CRCS

const
	MAVLINK_MESSAGE_CRCS: TCrcArray = 
	(${message_crcs_array});
	MAVLINK_MESSAGE_LENGTHS: TCrcArray = 
	(${message_lengths_array});

{$define MAVLINK_ENABLED_${basename_upper}}
	MAVLINK_VERSION = ${version};

// MESSAGE DEFINITIONS
${{message:	MAVLINK_MSG_ID_${name} = ${id};
}}

	
// ENUM DEFINITIONS
type
${{enum:
(** @brief ${description} *)
${name}=
(
${{entry:   ${name}=${value},  { ${description}${{param:|}} } 
}}
   ${name}_DUMMY
);
}}


implementation

end.
''', xml)
    f.close()
            

def generate_message_pas(directory, m):
    '''generate per-message header for a XML file'''
    f = open(os.path.join(directory, 'mavlink_msg_%s.pas' % m.name_lower), mode='w')
    t.write(f, '''
unit mavlink_msg_${name_lower};
// MESSAGE ${name} PACKING
interface
uses 
	mavlink, SysUtils;
	
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
    Result := mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_${name}_MIN_LEN, MAVLINK_MSG_ID_${name}_LEN, MAVLINK_MSG_ID_${name}_CRC);
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
    Result := mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_${name}_MIN_LEN, MAVLINK_MSG_ID_${name}_LEN, MAVLINK_MSG_ID_${name}_CRC);
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
    Result := mavlink_msg_${name_lower}_pack(system_id, component_id, msg,${{arg_fields: ${name_lower}.${name},}});
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
    Result := mavlink_msg_${name_lower}_pack_chan(system_id, component_id, chan, msg,${{arg_fields: ${name_lower}.${name},}});
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
    Result := msg.payload[${wire_offset}];
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


def copy_fixed_headers(directory, xml):
    '''copy the fixed protocol headers to the target directory'''
    import shutil, filecmp
    hlist = {
        "0.9": [  ],
        "1.0": [ 'mavlink.pas', 'mavlink_conversions.pas' ],
        "2.0": [ 'mavlink.pas', 'mavlink_conversions.pas',
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

    directory = basename#os.path.join(basename, xml.basename)

    print("Generating Delphi implementation in directory %s" % directory)
    mavparse_delphi.mkdir_p(directory)

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
            xml.message_crcs_array[msgid] = '{%u, %u, %u, %u, %u, %u}, ' % (msgid,
                                                                      xml.message_crcs[msgid],
                                                                      xml.message_min_lengths[msgid],
                                                                      xml.message_flags[msgid],
                                                                      xml.message_target_system_ofs[msgid],
                                                                      xml.message_target_component_ofs[msgid])
    else:
        for msgid in range(256):
            # if not xml.message_crcs[msgid]:
            crc = xml.message_crcs.get(msgid, 0)
            xml.message_crcs_array += '%u, ' % crc			
            if crc != 0:
                message_crcs[msgid] = crc
    xml.message_crcs_array = xml.message_crcs_array[:-2]
    # for msgid in range[256]:
        # if message_crcs[msgid] != 0:
            # message_crcs[msgid] = xml.message_crcs[msgid]# print(xml.basename)

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

    generate_mavlink_msg_inc(directory, xml)
    generate_main_pas(directory, xml)
    for m in xml.message:
        generate_message_pas(directory, m)
#    generate_testsuite_h(directory, xml)


def generate(basename, xml_list):
    '''generate complete MAVLink Delphi implemenation'''

    # crc = 0
    # message_crcs = []
    id = 0;
    crclist = ''
    while id<256:
        message_crcs.append( 0)
        id+=1
    for idx in range(len(xml_list)):
        xml = xml_list[idx]
        xml.xml_idx = idx
        generate_one(basename, xml)
    for msgid in range(256):
        # if not xml.message_crcs[msgid]:
        crc = message_crcs[msgid]
        crclist += '%u, ' % crc			
    # crclist = crclist[:-2]# xml.message_crcs_array = message_crcs;
    xml.message_crcs_array = crclist[:-2]
    #generate_mavlink_crcs(basename, xml)
    # print(xml.message_crcs_array)# crc = 0#xml.message_crcs[msgid]
    # print(crclist)# crc = 0#xml.message_crcs[msgid]
    generate_version_pas(basename, xml)
    copy_fixed_headers(basename, xml_list[0])
