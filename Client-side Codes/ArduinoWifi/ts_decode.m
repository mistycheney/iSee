function val = ts_decode(ts, nbyte)

val = uint32(0);
for i = 1:nbyte
    val = bitshift(uint32(val), 6);
    val = bitand(val, bitcmp(uint32(63)));
    val = bitor(val, uint32((ts(i) - 48)));
end
