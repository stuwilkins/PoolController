from bluepy.btle import Peripheral
from struct import unpack

pool_chars = ['67ff8149-1199-4700-a494-00f721975a41',
              '3bf88493-4af6-4413-aa53-9bd788d1e7db',
              '54990715-32a0-4f9f-8c8d-e8fffc3b0cae',
              '0d37c89b-e707-4090-bf85-5172764c2966',
              '4f051f2a-8c0c-4a45-b71e-18d29b4a1d41']

types = ['>i', '>i', '>i', 'B', '>i']

factors = [1000, 1000, 1000, 1, 1000]


def read_and_convert(char, f=1.0, t='>i'):
    _val = char.read()
    print(_val)
    val = unpack(t, _val)
    return val[0] / f

p = Peripheral("DD:9B:C2:11:E2:A4", "random")
service = p.getServiceByUUID("036451ed-6956-41ae-9840-5aca6c150ec7")

vals = list()
for c, f, t in zip(pool_chars, factors, types):
    _c = service.getCharacteristics(c)[0]
    vals.append(read_and_convert(_c, f, t))

print(vals)
