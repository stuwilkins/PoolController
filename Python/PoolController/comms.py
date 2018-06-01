import logging
import yaml
from bluepy.btle import Peripheral
from struct import unpack, pack

from .bluetooth_setup import pool_chars

def read_controller(host):
    logger.info("Connecting to %s", host)
    p = Peripheral(host, "random")
    service = p.getServiceByUUID("036451ed-6956-41ae-9840-5aca6c150ec7")
    logger.info("Connected")

    event = dict()
    for c in pool_chars.values():
        logger.info("Reading char %s", c['char'])
        _c = service.getCharacteristics(c['char'])
        _c = _c[0]
        logger.info("Converting char %s with factor %f and type %s",
                    c['char'], c['factor'], c['type'])
        _v = read_and_convert(_c, c['factor'], c['type'])
        logger.info("Char %s has value %s", c['char'], str(_v))
        logger.info("Adding char %s as event %s", c['char'], c['ev_name'])
        event[c['ev_name']] = _v

    p.disconnect()

    logger.info('Disconnected')

