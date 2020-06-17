unit checksum;

interface
uses SysUtils;

const
  X25_INIT_CRC = $ffff;
  X25_VALIDATE_CRC = $f0b8;

procedure crc_accumulate(data : Uint8; var crcAccum : UInt16);
procedure crc_init(var crcAccum : UInt16);
function crc_calculate(pBuffer : PByteArray; length : Uint16) : Uint16;
procedure crc_accumulate_buffer(var crcAccum : UInt16; pBuffer : PByteArray; length : UInt16);

implementation
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

end.
