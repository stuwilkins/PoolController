import time
import keen
from bluepy.btle import Peripheral
from struct import unpack

keen.project_id = "586556c88db53dfda8a7e515"
keen.write_key = "9E61EACC110C94C74923258CDA228480891F857717B7C61DA861097C52ABC3B06E9DDA2648C84E06222815A2E3883F969A75AE12CA1390A025EC0AE2456A373092C1118EE35AB9474E1C5C12FCA310136B0A9FAE68DD8C7F2D1F6BD5FD4E47B4"

pool_chars = ['67ff8149-1199-4700-a494-00f721975a41',
              '3bf88493-4af6-4413-aa53-9bd788d1e7db',
              '54990715-32a0-4f9f-8c8d-e8fffc3b0cae',
              '0d37c89b-e707-4090-bf85-5172764c2966',
              '4f051f2a-8c0c-4a45-b71e-18d29b4a1d41']

types = ['>i', '>i', '>i', 'B', '>i']

factors = [1000, 1000, 1000, 1, 1000]


def read_and_convert(char, f=1.0, t='>i'):
    _val = char.read()
    val = unpack(t, _val)
    return val[0] / f


def read_controller():
    print("--- Connecting")
    p = Peripheral("DD:9B:C2:11:E2:A4", "random")
    service = p.getServiceByUUID("036451ed-6956-41ae-9840-5aca6c150ec7")

    vals = list()
    for c, f, t in zip(pool_chars, factors, types):
        _c = service.getCharacteristics(c)[0]
        _v = read_and_convert(_c, f, t)
        print(_c, _v)
        vals.append(_v)

    p.disconnect()
    print("--- Disconnected")
    return vals


if __name__ == "__main__":
    while True:
        try:
            vals = read_controller()
        except:
            continue
        else:
            print(vals)
            try:
                keen.add_event("Home_Temp_Logging", {
                    'pool_water_temp': vals[0],
                    'air_temp': vals[1],
                    'air_humidity': vals[2],
                    'flow': vals[3],
                    'water_level': vals[4]
                })
            except:
                print("*** Error Sending to KeenIO ***")

        time.sleep(300)
