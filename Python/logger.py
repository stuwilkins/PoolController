import time
import keen
from bluepy.btle import Peripheral
from struct import unpack
from keys import project_id, write_key

keen.project_id = project_id
keen.write_key = write_key

pool_chars = ['00001001-0000-1000-8000-00805f9b34fb',
              '00001002-0000-1000-8000-00805f9b34fb',
              '00001003-0000-1000-8000-00805f9b34fb',
              '00001004-0000-1000-8000-00805f9b34fb',
              '00001005-0000-1000-8000-00805f9b34fb',
              '00001006-0000-1000-8000-00805f9b34fb',
              '00001007-0000-1000-8000-00805f9b34fb',
              '00001008-0000-1000-8000-00805f9b34fb',
              '00001009-0000-1000-8000-00805f9b34fb']

types = ['>i', '>i', '>i', 'B', '>i', '>h', '>h', '>h', '>l']

factors = [1000, 1000, 1000, 1, 1000, 100, 1, 1, 1]


def read_and_convert(char, f=1.0, t='>i'):
    _val = char.read()
    print(_val, f, t)
    val = unpack(t, _val)
    return val[0] / f


def read_controller():
    print("--- Connecting")
    p = Peripheral("DD:9B:C2:11:E2:A4", "random")

    service = p.getServiceByUUID("036451ed-6956-41ae-9840-5aca6c150ec7")

    vals = list()
    for c, f, t in zip(pool_chars, factors, types):
        _c = service.getCharacteristics(c)
        _c = _c[0]
        _v = read_and_convert(_c, f, t)
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
                ev = {'pool_water_temp': vals[0],
                      'air_temp': vals[1],
                      'air_humidity': vals[2],
                      'flow': vals[3],
                      'water_level': vals[4],
                      'UV_index': vals[5],
                      'visible_light': vals[6],
                      'ir_light': vals[7]}
                print(ev)
                keen.add_event("Home_Temp_Logging", ev)
            except:
                print("*** Error Sending to KeenIO ***")

        time.sleep(300)
