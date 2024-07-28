import socket
from typing import List

ENCODING = "ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789!@#$%^&*()[]<>|;:',./?~_-\n "

print(ENCODING.__len__())

def decompress(input: bytes) -> str:
    xarr = str()
    for byte in input:
        binary_str_x = bin(byte)[2:]
        xarr += "0"*(8-len(binary_str_x)) + binary_str_x

    out = str()
    for i in range(0, len(xarr), 6):
        try:
            out += ENCODING[int(xarr[i:i+6], 2)]
        except:
            print("Invalid character in message.")
            continue

    return out
    
def parse(input: bytes) -> List[str]:
    decompressed = decompress(input)
    parsed = [x.split(",") for x in decompressed.split("\n") if len(x) > 1]

    return parsed

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
server_address = ("10.8.46.2", 5808) # Server IP, PORT

if __name__ == "__main__":
    try:
        sent = sock.sendto("~".encode(), server_address)
        while True:
            data, server = sock.recvfrom(4096)
            [print(f'{x}') for x in parse(data)]
               
    finally:
        print('Error, exiting...')
        sock.close()