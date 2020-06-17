unit mavlink_helpers;

interface

uses
  mavlink_types, checksum, mavlink, SysUtils;

const
  MAVLINK_COMM_NUM_BUFFERS = 16;
  MAVLINK_STX = 254;
//  MAVLINK_ENDIAN = MAVLINK_LITTLE_ENDIAN;
  MAVLINK_ALIGNED_FIELDS = 1;
  MAVLINK_CRC_EXTRA = 1;
  MAVLINK_COMMAND_24BIT = 0;

//  MAVLINK_MESSAGE_CRCS: array[0..255] of byte = (50, 124, 137, 0, 237, 217, 104, 119, 0, 0, 0, 89, 0, 0, 0, 0, 0, 0, 0, 0, 214, 159, 220, 168, 24, 23, 170, 144, 67, 115, 39, 246, 185, 104, 237, 244, 222, 212, 9, 254, 230, 28, 28, 132, 221, 232, 11, 153, 41, 39, 78, 196, 0, 0, 15, 3, 0, 0, 0, 0, 0, 167, 183, 119, 191, 118, 148, 21, 0, 243, 124, 0, 0, 38, 20, 158, 152, 143, 0, 0, 0, 106, 49, 22, 143, 140, 5, 150, 0, 231, 183, 63, 54, 47, 0, 0, 0, 0, 0, 0, 175, 102, 158, 208, 56, 93, 138, 108, 32, 185, 84, 34, 174, 124, 237, 4, 76, 128, 56, 116, 134, 237, 203, 250, 87, 203, 220, 25, 226, 46, 29, 223, 85, 6, 229, 203, 1, 195, 109, 168, 181, 47, 72, 131, 127, 0, 103, 154, 178, 200, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 163, 105, 151, 35, 150, 0, 0, 0, 0, 0, 0, 90, 104, 85, 95, 130, 184, 81, 8, 204, 49, 170, 44, 83, 46, 0);

function mavlink_parse_char(chan: UInt8; c: UInt8; var r_message: mavlink_message_t; var r_mavlink_status: mavlink_status_t): mavlink_framing_t;
function mavlink_frame_char_buffer(var rxmsg: mavlink_message_t; var status: mavlink_status_t; c: uint8; var r_message: mavlink_message_t; var r_mavlink_status: mavlink_status_t): mavlink_framing_t;
function mavlink_frame_char(chan: UInt8; c: UInt8; var r_message: mavlink_message_t; var r_mavlink_status: mavlink_status_t): mavlink_framing_t;
procedure mavlink_update_checksum(var msg: mavlink_message_t; c: uint8);
procedure mavlink_start_checksum(var msg: mavlink_message_t);
procedure mavlink_reset_channel_status(chan: UInt8);
function mavlink_finalize_message(var msg: mavlink_message_t; system_id: uint8; component_id: uint8; min_length: uint8; length: uint8; crc_extra: uint8): uint16;
function mavlink_finalize_message_chan(var msg: mavlink_message_t; system_id: uint8; component_id: uint8; chan: uint8; min_length: uint8; length: uint8; crc_extra: uint8): UInt16;
procedure mav_array_memcpy(const src; var dest; n: Integer);
function mavlink_msg_to_send_buffer(var buffer: TBytes; const msg: mavlink_message_t): uint16;

implementation

uses
  Windows;

var
  m_mavlink_buffer: array[0..MAVLINK_COMM_NUM_BUFFERS - 1] of mavlink_message_t;
  m_mavlink_status: array[0..MAVLINK_COMM_NUM_BUFFERS - 1] of mavlink_status_t;

function mavlink_frame_char_buffer(var rxmsg: mavlink_message_t; var status: mavlink_status_t; c: uint8; var r_message: mavlink_message_t; var r_mavlink_status: mavlink_status_t): mavlink_framing_t;
var
  bufferIndex: integer;
begin
  bufferIndex := 0;

  status.msg_received := MAVLINK_FRAMING_INCOMPLETE;

  case (status.parse_state) of

    MAVLINK_PARSE_STATE_UNINIT, MAVLINK_PARSE_STATE_IDLE:
      begin
        if (c = MAVLINK_STX) then
        begin
          status.parse_state := MAVLINK_PARSE_STATE_GOT_STX;
          rxmsg.len := 0;
          rxmsg.magic := c;
          mavlink_start_checksum(rxmsg);
        end;
      end;

    MAVLINK_PARSE_STATE_GOT_STX:
      begin
        if (status.msg_received <> MAVLINK_FRAMING_INCOMPLETE) then
        begin

    (* Support shorter buffers than the
       default maximum packet size *)
  //    #if (MAVLINK_MAX_PAYLOAD_LEN < 255)
  //            || c > MAVLINK_MAX_PAYLOAD_LEN
  //    #endif
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
          mavlink_update_checksum(rxmsg, c);
          status.parse_state := MAVLINK_PARSE_STATE_GOT_LENGTH;
        end;
      end;

    MAVLINK_PARSE_STATE_GOT_LENGTH:
      begin
        rxmsg.seq := c;
        mavlink_update_checksum(rxmsg, c);
        status.parse_state := MAVLINK_PARSE_STATE_GOT_SEQ;
      end;

    MAVLINK_PARSE_STATE_GOT_SEQ:
      begin
        rxmsg.sysid := c;
        mavlink_update_checksum(rxmsg, c);
        status.parse_state := MAVLINK_PARSE_STATE_GOT_SYSID;
      end;

    MAVLINK_PARSE_STATE_GOT_SYSID:
      begin
        rxmsg.compid := c;
        mavlink_update_checksum(rxmsg, c);
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
        mavlink_update_checksum(rxmsg, c);
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
        mavlink_update_checksum(rxmsg, c);
        if (status.packet_idx = rxmsg.len) then
        begin
          status.parse_state := MAVLINK_PARSE_STATE_GOT_PAYLOAD;
        end;
      end;

    MAVLINK_PARSE_STATE_GOT_PAYLOAD:
      begin
  //#if MAVLINK_CRC_EXTRA
        mavlink_update_checksum(rxmsg, MAVLINK_MESSAGE_CRCS[rxmsg.msgid]);
  //#endif
//        if rxmsg.msgid=$B4 then
//          OutputDebugString('Payload received');

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
        Move(rxmsg, r_message, SizeOf(mavlink_message_t));
      end;
  end;

  Inc(bufferIndex);
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

  r_message.len := rxmsg.len; // Provide visibility on how far we are into current msg
  r_mavlink_status.parse_state := status.parse_state;
  r_mavlink_status.packet_idx := status.packet_idx;
  r_mavlink_status.current_rx_seq := status.current_rx_seq + 1;
  r_mavlink_status.packet_rx_success_count := status.packet_rx_success_count;
  r_mavlink_status.packet_rx_drop_count := status.parse_error;
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
    r_message.checksum := rxmsg.payload[status.packet_idx] or (rxmsg.payload[status.packet_idx + 1] shl 8);
  end;

  Result := status.msg_received;
end;

function mavlink_frame_char(chan: UInt8; c: UInt8; var r_message: mavlink_message_t; var r_mavlink_status: mavlink_status_t): mavlink_framing_t;
begin
  Result := mavlink_frame_char_buffer(m_mavlink_buffer[chan], m_mavlink_status[chan], c, r_message, r_mavlink_status);
end;

function mavlink_parse_char(chan: UInt8; c: UInt8; var r_message: mavlink_message_t; var r_mavlink_status: mavlink_status_t): mavlink_framing_t;
var
  msg_received: mavlink_framing_t;
  rxmsg: mavlink_message_t;
  status: mavlink_status_p;
begin
  msg_received := mavlink_frame_char(chan, c, r_message, r_mavlink_status);
  if (msg_received = MAVLINK_FRAMING_BAD_CRC) then
  begin
    // we got a bad CRC. Treat as a parse failure
//     rxmsg := mavlink_get_channel_buffer(chan);
//    status := mavlink_get_channel_status(chan);
    rxmsg := m_mavlink_buffer[chan];
    status := @m_mavlink_status[chan];
    Inc(status.parse_error);
    status.msg_received := MAVLINK_FRAMING_INCOMPLETE;
    status.parse_state := MAVLINK_PARSE_STATE_IDLE;
    if (c = MAVLINK_STX) then
    begin
      status.parse_state := MAVLINK_PARSE_STATE_GOT_STX;
      rxmsg.len := 0;
      mavlink_start_checksum(rxmsg);
    end;
    Result := MAVLINK_FRAMING_BAD_CRC;
    exit;
  end;
  Result := msg_received;
end;

procedure mavlink_start_checksum(var msg: mavlink_message_t);
begin
  crc_init(msg.checksum);
end;

procedure mavlink_update_checksum(var msg: mavlink_message_t; c: uint8);
begin
  crc_accumulate(c, msg.checksum);
end;

procedure mavlink_reset_channel_status(chan: UInt8);
begin
//	mavlink_status_t *status = mavlink_get_channel_status(chan);
  m_mavlink_status[chan].parse_state := MAVLINK_PARSE_STATE_IDLE;
end;

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
function mavlink_msg_to_send_buffer(var buffer: TBytes; const msg: mavlink_message_t): uint16;
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

function mavlink_finalize_message_chan(var msg: mavlink_message_t; system_id: uint8; component_id: uint8; chan: uint8; min_length: uint8; length: uint8; crc_extra: uint8): UInt16;
begin
	// This code part is the same for all messages;
  msg.magic := MAVLINK_STX;
  msg.len := length;
  msg.sysid := system_id;
  msg.compid := component_id;
	// One sequence number per channel
  msg.seq := m_mavlink_status[chan].current_tx_seq;
  m_mavlink_status[chan].current_tx_seq := m_mavlink_status[chan].current_tx_seq + 1;
//	msg.checksum := crc_calculate(((const uint8_t*)(msg)) + 3, MAVLINK_CORE_HEADER_LEN);
//	crc_accumulate_buffer(&msg.checksum, _MAV_PAYLOAD(msg), msg.len);
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

end.

