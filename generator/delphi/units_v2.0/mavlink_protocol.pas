unit mavlink_protocol;

interface
uses mavlink_types;

implementation

function mavlink_msg_get_send_buffer_length(msg: mavlink_message_t): uint16;
begin
	Result:=msg.len + MAVLINK_NUM_NON_PAYLOAD_BYTES;
end;

end.
