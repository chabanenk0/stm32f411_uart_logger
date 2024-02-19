filename = 'stm32_dump_20240210_2301_bits_throttle_only.bin'

file = open(filename, 'rb')
file.seek(0x14ac)

while True:
        bits = file.read(1)
        print(format(bits[0], '08b'))
