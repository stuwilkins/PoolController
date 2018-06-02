import keen
import logging
import yaml
from bluepy.btle import Peripheral
from struct import unpack


logger = logging.getLogger('bluetooth_le_logger')
logger.setLevel(logging.DEBUG)
ch = logging.StreamHandler()
ch.setLevel(logging.DEBUG)
# create formatter and add it to the handlers
formatter = logging.Formatter("%(asctime)s - %(name)s - "
                              "%(levelname)s - %(message)s")
ch.setFormatter(formatter)
logger.addHandler(ch)


def read_config(filedir):
    with open(filedir + 'keen.yaml', 'r') as ymlfile:
        cfg = yaml.load(ymlfile)
    keen.project_id = cfg['keen.io']['project_id']
    keen.write_key = cfg['keen.io']['write_key']
    keen.read_key = cfg['keen.io']['read_key']

    with open(filedir + 'pool_controller.yaml', 'r') as yamlfile:
        cfg = yaml.load(yamlfile)

    return cfg


def read_and_convert(char, f=1.0, t='>i'):
    _val = char.read()
    val = unpack(t, _val)
    return val[0] / f


def read_controller(host, chars):
    logger.info("Connecting to %s", host)
    p = Peripheral(host, "random")
    service = p.getServiceByUUID("036451ed-6956-41ae-9840-5aca6c150ec7")
    logger.info("Connected")

    for c in chars.values():
        logger.info("Reading char %s", c['char'])
        _c = service.getCharacteristics(c['char'])
        _c = _c[0]
        logger.info("Converting char %s with factor %f and type %s",
                    c['char'], c['factor'], c['type'])
        _v = read_and_convert(_c, c['factor'], c['type'])
        logger.info("Char %s has value %s", c['char'], str(_v))
        logger.info("Adding char %s as event %s", c['char'], c['ev_name'])
        c['value'] = _v

    p.disconnect()

    logger.info('Disconnected')

    return chars


def send_to_keen_io(event):
    keen.add_event("Home_Temp_Logging", event)
    return
