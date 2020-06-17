unit mavlink_types;

interface
//uses MAVLink;

const

  MAVLINK_MAX_PAYLOAD_LEN= 255;
  MAVLINK_NUM_CHECKSUM_BYTES=2;
  MAVLINK_CORE_HEADER_LEN= 5; ///< Length of core header (of the comm. layer): message length (1 byte) + message sequence (1 byte) + message system id (1 byte) + message component id (1 byte) + message type id (1 byte)
  MAVLINK_NUM_HEADER_BYTES =(MAVLINK_CORE_HEADER_LEN + 1); ///< Length of all header bytes, including core and checksum
  MAVLINK_NUM_NON_PAYLOAD_BYTES =(MAVLINK_NUM_HEADER_BYTES + MAVLINK_NUM_CHECKSUM_BYTES);

type

uint8_t=uint8;
uint16_t=uint16;
uint32_t=uint32;
uint64_t=uint64;
int8_t=int8;
int16_t=int16;
int32_t=int32;
int64_t=int64;
float=single;

mavlink_message_t= packed record
	checksum: uint16; ///< sent at end of packet
	magic : UInt8;   ///< protocol magic marker
	len : UInt8;     ///< Length of payload
	seq : UInt8;     ///< Sequence of packet
	sysid : UInt8;   ///< ID of message sender system/aircraft
	compid : UInt8;  ///< ID of the message sender component
	msgid : UInt8;   ///< ID of message in payload
  case Integer of
    0:(payload64 : array [0..(MAVLINK_MAX_PAYLOAD_LEN+MAVLINK_NUM_CHECKSUM_BYTES+7) div 8-1] of uint64);
    1:(payload : array [0..((MAVLINK_MAX_PAYLOAD_LEN+MAVLINK_NUM_CHECKSUM_BYTES+7) div 8-1)*SizeOf(UInt64)] of uint8);
end;
mavlink_message_p=^mavlink_message_t;

///< The state machine for the comm parser
mavlink_parse_state_t=(
    MAVLINK_PARSE_STATE_UNINIT=0,
    MAVLINK_PARSE_STATE_IDLE,
    MAVLINK_PARSE_STATE_GOT_STX,
    MAVLINK_PARSE_STATE_GOT_SEQ,
    MAVLINK_PARSE_STATE_GOT_LENGTH,
    MAVLINK_PARSE_STATE_GOT_SYSID,
    MAVLINK_PARSE_STATE_GOT_COMPID,
    MAVLINK_PARSE_STATE_GOT_MSGID,
    MAVLINK_PARSE_STATE_GOT_PAYLOAD,
    MAVLINK_PARSE_STATE_GOT_CRC1,
    MAVLINK_PARSE_STATE_GOT_BAD_CRC1);

mavlink_framing_t=( MAVLINK_FRAMING_INCOMPLETE=0,
    MAVLINK_FRAMING_OK=1,
    MAVLINK_FRAMING_BAD_CRC=2);

 mavlink_channel_t=(
    MAVLINK_COMM_0,
    MAVLINK_COMM_1,
    MAVLINK_COMM_2,
    MAVLINK_COMM_3
);



mavlink_status_t=packed record
    msg_received : mavlink_framing_t;               ///< Number of received messages
    buffer_overrun : UInt8;             ///< Number of buffer overruns
    parse_error : UInt8;                ///< Number of parse errors
    parse_state: mavlink_parse_state_t;  ///< Parsing state machine
    packet_idx : UInt8;                 ///< Index in current packet
    current_rx_seq : UInt8;             ///< Sequence number of last packet received
    current_tx_seq : UInt8;             ///< Sequence number of last packet sent
    packet_rx_success_count : UInt16;   ///< Received packets
    packet_rx_drop_count : UInt16;      ///< Number of packet drops
end;
mavlink_status_p=^mavlink_status_t;




implementation

end.
