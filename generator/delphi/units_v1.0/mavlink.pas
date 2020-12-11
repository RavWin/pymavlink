unit mavlink;

interface

uses
  SysUtils;

//============== mavlink_types ===============
const

  MAVLINK_MAX_PAYLOAD_LEN= 255;
  MAVLINK_NUM_CHECKSUM_BYTES=2;
  MAVLINK_CORE_HEADER_LEN= 5; ///< Length of core header (of the comm. layer): message length (1 byte) + message sequence (1 byte) + message system id (1 byte) + message component id (1 byte) + message type id (1 byte)
  MAVLINK_NUM_HEADER_BYTES =(MAVLINK_CORE_HEADER_LEN + 1); ///< Length of all header bytes, including core and checksum
  MAVLINK_NUM_NON_PAYLOAD_BYTES =(MAVLINK_NUM_HEADER_BYTES + MAVLINK_NUM_CHECKSUM_BYTES);

  MAVLINK_BIG_ENDIAN    = 0;
  MAVLINK_LITTLE_ENDIAN = 1;

type

  TCrcArray = array[0..255] of byte;
  PCrcArray = ^TCrcArray;

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

  //============== mavlink parser ===============

  TMavlinkParser = class
  private
    rxmsg: mavlink_message_t;
    status: mavlink_status_t;
    fCRCS : TCrcArray;
    procedure startChecksum;
    procedure updateChecksum(c: uint8);
    function  getSendBufferLength: uint16;
  public
    function  parseChar(c: UInt8): mavlink_framing_t;
    function  frameCharBuffer(var  c: uint8): mavlink_framing_t;
    function  finalizeMessage( system_id: uint8; component_id: uint8; min_length: uint8; length: uint8; crc_extra: uint8): UInt16;
    function  msgToSendBuffer(var buffer: TBytes; const msg: mavlink_message_t): uint16;
    procedure resetState;

    property  msg : mavlink_message_t read rxmsg;
    property  CRCs:  TCrcArray read fCRCS write fCRCS;
  end;

//============== mavlink_checksum ===============
const
  X25_INIT_CRC = $ffff;
  X25_VALIDATE_CRC = $f0b8;

  procedure crc_accumulate(data : Uint8; var crcAccum : UInt16);
  procedure crc_init(var crcAccum : UInt16);
  function  crc_calculate(pBuffer : PByteArray; length : Uint16) : Uint16;
  procedure crc_accumulate_buffer(var crcAccum : UInt16; pBuffer : PByteArray; length : UInt16);

//============== mavlink_helpers ===============

const
  MAVLINK_COMM_NUM_BUFFERS = 16;
  MAVLINK_STX = 254;
  MAVLINK_ENDIAN = MAVLINK_LITTLE_ENDIAN;
  MAVLINK_ALIGNED_FIELDS = 1;
  MAVLINK_CRC_EXTRA = 1;
  MAVLINK_COMMAND_24BIT = 0;

  function mavlink_finalize_message(var msg: mavlink_message_t; system_id: uint8; component_id: uint8; min_length: uint8; length: uint8; crc_extra: uint8): uint16;
  function mavlink_finalize_message_chan(var msg: mavlink_message_t; system_id: uint8; component_id: uint8; chan: uint8; min_length: uint8; length: uint8; crc_extra: uint8): UInt16;
  procedure mav_array_memcpy(const src; var dest; n: Integer);

implementation

uses
  Windows;

var
  m_mavlink_txseq: array[0..MAVLINK_COMM_NUM_BUFFERS-1] of UInt8;

(**
 * @brief Put a bitfield of length 1-32 bit into the buffer
 *
 * @param b the value to add, will be encoded in the bitfield
 * @param bits number of bits to use to encode b, e.g. 1 for boolean, 2, 3, etc.
 * @param packet_index the position in the packet (the index of the first byte to use)
 * @param bit_index the position in the byte (the index of the first bit to use)
 * @param buffer packet buffer to write into
 * @return new position of the last used byte in the buffer
 *)
function put_bitfield_n_by_index(b: int32; bits: uint8; packet_index: uint8; bit_index: uint8; r_bit_index: PByte; buffer: PByte): uint8;
begin
//	uint16_t bits_remain = bits;
//	// Transform number into network order
//	int32_t v;
//	uint8_t i_bit_index, i_byte_index, curr_bits_n;
//#if MAVLINK_NEED_BYTE_SWAP
//	union {
//		int32_t i;
//		uint8_t b[4];
//	} bin, bout;
//	bin.i = b;
//	bout.b[0] = bin.b[3];
//	bout.b[1] = bin.b[2];
//	bout.b[2] = bin.b[1];
//	bout.b[3] = bin.b[0];
//	v = bout.i;
//#else
//	v = b;
//#endif
//
//	// buffer in
//	// 01100000 01000000 00000000 11110001
//	// buffer out
//	// 11110001 00000000 01000000 01100000
//
//	// Existing partly filled byte (four free slots)
//	// 0111xxxx
//
//	// Mask n free bits
//	// 00001111 = 2^0 + 2^1 + 2^2 + 2^3 = 2^n - 1
//	// = ((uint32_t)(1 << n)) - 1; // = 2^n - 1
//
//	// Shift n bits into the right position
//	// out = in >> n;
//
//	// Mask and shift bytes
//	i_bit_index = bit_index;
//	i_byte_index = packet_index;
//	if (bit_index > 0)
//	{
//		// If bits were available at start, they were available
//		// in the byte before the current index
//		i_byte_index--;
//	}
//
//	// While bits have not been packed yet
//	while (bits_remain > 0)
//	{
//		// Bits still have to be packed
//		// there can be more than 8 bits, so
//		// we might have to pack them into more than one byte
//
//		// First pack everything we can into the current 'open' byte
//		//curr_bits_n = bits_remain << 3; // Equals  bits_remain mod 8
//		//FIXME
//		if (bits_remain <= (uint8_t)(8 - i_bit_index))
//		{
//			// Enough space
//			curr_bits_n = (uint8_t)bits_remain;
//		}
//		else
//		{
//			curr_bits_n = (8 - i_bit_index);
//		}
//
//		// Pack these n bits into the current byte
//		// Mask out whatever was at that position with ones (xxx11111)
//		buffer[i_byte_index] &= (0xFF >> (8 - curr_bits_n));
//		// Put content to this position, by masking out the non-used part
//		buffer[i_byte_index] |= ((0x00 << curr_bits_n) & v);
//
//		// Increment the bit index
//		i_bit_index += curr_bits_n;
//
//		// Now proceed to the next byte, if necessary
//		bits_remain -= curr_bits_n;
//		if (bits_remain > 0)
//		{
//			// Offer another 8 bits / one byte
//			i_byte_index++;
//			i_bit_index = 0;
//		}
//	}
//
//	*r_bit_index = i_bit_index;
//	// If a partly filled byte is present, mark this as consumed
//	if (i_bit_index != 7) i_byte_index++;
//	return i_byte_index - packet_index;
end;

(**
 * @brief Pack a message to send it over a serial byte stream
 *)

function mavlink_finalize_message_chan(var msg: mavlink_message_t; system_id: uint8; component_id: uint8; chan: uint8; min_length: uint8; length: uint8; crc_extra: uint8): UInt16;
begin
	// This code part is the same for all messages;
  msg.magic := MAVLINK_STX;
  msg.len := length;
  msg.sysid := system_id;
  msg.compid := component_id;
	// One sequence number per channel
  msg.seq := m_mavlink_txseq[chan];
  m_mavlink_txseq[chan] := m_mavlink_txseq[chan] + 1;
  msg.checksum := crc_calculate(PByteArray(@msg.len), MAVLINK_CORE_HEADER_LEN);
  crc_accumulate_buffer(msg.checksum, @msg.payload, msg.len);
//#if MAVLINK_CRC_EXTRA
	crc_accumulate(crc_extra, &msg.checksum);
//#endif
  // checksum is immediately after the payload bytes
//	mavlink_ck_a(msg) = (uint8_t)(msg.checksum & 0xFF);
//	mavlink_ck_b(msg) = (uint8_t)(msg.checksum >> 8);
  msg.payload[msg.len] := (msg.checksum and $FF);
  msg.payload[msg.len + 1] := (msg.checksum shr 8);

  Result := length + MAVLINK_NUM_NON_PAYLOAD_BYTES;
end;

function mavlink_finalize_message(var msg: mavlink_message_t; system_id: uint8; component_id: uint8; min_length: uint8; length: uint8; crc_extra: uint8): uint16;
begin
  Result := mavlink_finalize_message_chan(msg, system_id, component_id, Ord(MAVLINK_COMM_0), min_length, length, crc_extra);
end;

(*
  like memcpy(), but if src is NULL, do a memset to zero
*)
procedure mav_array_memcpy(const src; var dest; n: Integer);
begin
  if (SizeOf(src) = 0) then
  begin
    FillChar(dest, n, 0);
  end
  else
  begin
    move(src, dest, n);
  end;
end;

//function mavlink_msg_get_send_buffer_length(msg: mavlink_message_t): uint16;
//begin
//	Result :=msg.len + MAVLINK_NUM_NON_PAYLOAD_BYTES;
//end;

procedure crc_accumulate(data : Uint8; var crcAccum : UInt16);
var tmp : UInt8;
begin
        //Accumulate one byte of data into the CRC

        tmp := data xor (crcAccum and $ff);
        tmp := tmp xor (tmp shl 4);
        crcAccum := (crcAccum shr 8) xor (tmp shl 8) xor (tmp shl 3) xor (tmp shr 4);
end;

procedure crc_init(var crcAccum : UInt16);
begin
        crcAccum := X25_INIT_CRC;
end;

function crc_calculate(pBuffer : PByteArray; length : Uint16) : Uint16;
var crcTmp : UInt16;
  I: Integer;
begin
        crc_init(crcTmp);
        for I := 0 to length - 1 do
            crc_accumulate(pBuffer[I], crcTmp);
        Result := crcTmp;
end;

procedure crc_accumulate_buffer(var crcAccum : UInt16; pBuffer : PByteArray; length : UInt16);
var
  I: Integer;
begin
     for I := 0 to length - 1 do
        crc_accumulate(pBuffer[I], crcAccum);
end;


{ TMavlinkParser }

function TMavlinkParser.finalizeMessage(system_id, component_id, min_length, length, crc_extra: uint8): UInt16;
begin
	// This code part is the same for all messages;
  rxmsg.magic := MAVLINK_STX;
  rxmsg.len := length;
  rxmsg.sysid := system_id;
  rxmsg.compid := component_id;
	// One sequence number per channel
  rxmsg.seq := status.current_tx_seq;
  status.current_tx_seq := status.current_tx_seq + 1;
//	msg.checksum := crc_calculate(((const uint8_t*)(msg)) + 3, MAVLINK_CORE_HEADER_LEN);
//	crc_accumulate_buffer(&msg.checksum, _MAV_PAYLOAD(msg), msg.len);
  rxmsg.checksum := crc_calculate(PByteArray(@rxmsg.len), MAVLINK_CORE_HEADER_LEN);
  crc_accumulate_buffer(rxmsg.checksum, @rxmsg.payload, rxmsg.len);
//#if MAVLINK_CRC_EXTRA
	crc_accumulate(crc_extra, &rxmsg.checksum);
//#endif
  // checksum is immediately after the payload bytes
//	mavlink_ck_a(msg) = (uint8_t)(msg.checksum & 0xFF);
//	mavlink_ck_b(msg) = (uint8_t)(msg.checksum >> 8);
  rxmsg.payload[rxmsg.len] := (rxmsg.checksum and $FF);
  rxmsg.payload[rxmsg.len + 1] := (rxmsg.checksum shr 8);

  Result := length + MAVLINK_NUM_NON_PAYLOAD_BYTES;
end;

function TMavlinkParser.frameCharBuffer(var c: uint8): mavlink_framing_t;
begin
  status.msg_received := MAVLINK_FRAMING_INCOMPLETE;

  case (status.parse_state) of

    MAVLINK_PARSE_STATE_UNINIT, MAVLINK_PARSE_STATE_IDLE:
      begin
        if (c = MAVLINK_STX) then
        begin
          status.parse_state := MAVLINK_PARSE_STATE_GOT_STX;
          rxmsg.len := 0;
          rxmsg.magic := c;
          startChecksum;
        end;
      end;

    MAVLINK_PARSE_STATE_GOT_STX:
      begin
        if (status.msg_received <> MAVLINK_FRAMING_INCOMPLETE) then
        begin
    (* Support shorter buffers than the
       default maximum packet size *)
          Inc(status.buffer_overrun);
          Inc(status.parse_error);
          status.msg_received := MAVLINK_FRAMING_INCOMPLETE;
          status.parse_state := MAVLINK_PARSE_STATE_IDLE;
        end
        else
        begin
        // NOT counting STX, LENGTH, SEQ, SYSID, COMPID, MSGID, CRC1 and CRC2
          rxmsg.len := c;
          status.packet_idx := 0;
          updateChecksum(c);
          status.parse_state := MAVLINK_PARSE_STATE_GOT_LENGTH;
        end;
      end;

    MAVLINK_PARSE_STATE_GOT_LENGTH:
      begin
        rxmsg.seq := c;
        updateChecksum(c);
        status.parse_state := MAVLINK_PARSE_STATE_GOT_SEQ;
      end;

    MAVLINK_PARSE_STATE_GOT_SEQ:
      begin
        rxmsg.sysid := c;
        updateChecksum(c);
        status.parse_state := MAVLINK_PARSE_STATE_GOT_SYSID;
      end;

    MAVLINK_PARSE_STATE_GOT_SYSID:
      begin
        rxmsg.compid := c;
        updateChecksum(c);
        status.parse_state := MAVLINK_PARSE_STATE_GOT_COMPID;
      end;

    MAVLINK_PARSE_STATE_GOT_COMPID:
      begin
  //#ifdef MAVLINK_CHECK_MESSAGE_LENGTH
  //	        if (rxmsg.len != MAVLINK_MESSAGE_LENGTH(c))
  //		{
  //			status.parse_error++;
  //			status.parse_state := MAVLINK_PARSE_STATE_IDLE;
  //			break;
  //	    }
  //#endif
        rxmsg.msgid := c;
        updateChecksum(c);
        if (rxmsg.len = 0) then
        begin
          status.parse_state := MAVLINK_PARSE_STATE_GOT_PAYLOAD;
        end
        else
        begin
          status.parse_state := MAVLINK_PARSE_STATE_GOT_MSGID;
        end;
      end;

    MAVLINK_PARSE_STATE_GOT_MSGID:
      begin
        rxmsg.payload[status.packet_idx] := c;
        Inc(status.packet_idx);
        updateChecksum(c);
        if (status.packet_idx = rxmsg.len) then
        begin
          status.parse_state := MAVLINK_PARSE_STATE_GOT_PAYLOAD;
        end;
      end;

    MAVLINK_PARSE_STATE_GOT_PAYLOAD:
      begin
  //#if MAVLINK_CRC_EXTRA
        updateChecksum(fCRCS[rxmsg.msgid]);
  //#endif
        if (c <> (rxmsg.checksum and $FF)) then
        begin
          status.parse_state := MAVLINK_PARSE_STATE_GOT_BAD_CRC1;
        end
        else
        begin
          status.parse_state := MAVLINK_PARSE_STATE_GOT_CRC1;
        end;
        rxmsg.payload[status.packet_idx] := c;
      end;

    MAVLINK_PARSE_STATE_GOT_CRC1, MAVLINK_PARSE_STATE_GOT_BAD_CRC1:
      begin
        if ((status.parse_state = MAVLINK_PARSE_STATE_GOT_BAD_CRC1) or (c <> (rxmsg.checksum shr 8))) then
        begin
        // got a bad CRC message
          status.msg_received := MAVLINK_FRAMING_BAD_CRC;
        end
        else
        begin
        // Successfully got message
          status.msg_received := MAVLINK_FRAMING_OK;
        end;
        status.parse_state := MAVLINK_PARSE_STATE_IDLE;
        rxmsg.payload[status.packet_idx + 1] := c;
//        Move(rxmsg, r_message, SizeOf(mavlink_message_t));
      end;
  end;

//  Inc(bufferIndex);
	// If a message has been sucessfully decoded, check index
  if (status.msg_received = MAVLINK_FRAMING_OK) then
  begin
		//while(status.current_seq != rxmsg.seq)
		//{
		//	status.packet_rx_drop_count++;
		//               status.current_seq++;
		//}
    status.current_rx_seq := rxmsg.seq;
		// Initial condition: If no packet has been received so far, drop count is undefined
    if (status.packet_rx_success_count = 0) then
      status.packet_rx_drop_count := 0;
		// Count this packet as received
    Inc(status.packet_rx_success_count);
  end;

  // Provide visibility on how far we are into current msg
  inc(status.current_rx_seq);
  status.parse_error := 0;

  if (status.msg_received = MAVLINK_FRAMING_BAD_CRC) then
  begin
		(*
		  the CRC came out wrong. We now need to overwrite the
		  msg CRC with the one on the wire so that if the
		  caller decides to forward the message anyway that
		  mavlink_msg_to_send_buffer() won't overwrite the
		  checksum
		 *)
    rxmsg.checksum := rxmsg.payload[status.packet_idx] or (rxmsg.payload[status.packet_idx + 1] shl 8);
  end;

  Result := status.msg_received;
end;

function TMavlinkParser.getSendBufferLength: uint16;
begin
	Result :=rxmsg.len + MAVLINK_NUM_NON_PAYLOAD_BYTES;
end;

procedure TMavlinkParser.startChecksum;
begin
  crc_init(rxmsg.checksum);
end;

procedure TMavlinkParser.updateChecksum( c: uint8);
begin
  crc_accumulate(c, rxmsg.checksum);
end;

function TMavlinkParser.msgToSendBuffer(var buffer: TBytes;  const msg: mavlink_message_t): uint16;
var
  ck: Pbyte;
begin
  SetLength(buffer, MAVLINK_NUM_NON_PAYLOAD_BYTES + uint16(msg.len));
  Move(&msg.magic, buffer[0], MAVLINK_NUM_HEADER_BYTES + uint16(msg.len));

  ck := PByte(Uint32(@buffer[0]) + MAVLINK_NUM_HEADER_BYTES + uint16(msg.len));

  ck[0] := uint8(msg.checksum and $FF);
  ck[1] := uint8(msg.checksum shr 8);

  Result := MAVLINK_NUM_NON_PAYLOAD_BYTES + msg.len;
end;

function TMavlinkParser.parseChar(c: UInt8): mavlink_framing_t;
var
  msg_received: mavlink_framing_t;
begin
  msg_received := frameCharBuffer(c);
  if (msg_received = MAVLINK_FRAMING_BAD_CRC) then
  begin
    // we got a bad CRC. Treat as a parse failure
//     rxmsg := mavlink_get_channel_buffer(chan);
//    status := mavlink_get_channel_status(chan);
//    rxmsg := m_mavlink_buffer[chan];
//    status := @m_mavlink_status[chan];
    Inc(status.parse_error);
    status.msg_received := MAVLINK_FRAMING_INCOMPLETE;
    status.parse_state := MAVLINK_PARSE_STATE_IDLE;
    if (c = MAVLINK_STX) then
    begin
      status.parse_state := MAVLINK_PARSE_STATE_GOT_STX;
      rxmsg.len := 0;
      startChecksum;
    end;
    Result := MAVLINK_FRAMING_BAD_CRC;
    exit;
  end;
  Result := msg_received;
end;

procedure TMavlinkParser.resetState;
begin
//  m_mavlink_status[chan].parse_state := MAVLINK_PARSE_STATE_IDLE;
  status.parse_state := MAVLINK_PARSE_STATE_IDLE;
end;

end.

