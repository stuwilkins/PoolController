pool_chars = {'Pool Water Temp' : {
               'char': '00001001-0000-1000-8000-00805f9b34fb',
               'type': '>i',
               'factor': 1000,
               'ev_name': 'pool_water_temp',
               'mqtt_topic': 'myhome/pool/water_temp'},
              'Air Temperature': {
               'char': '00001002-0000-1000-8000-00805f9b34fb',
               'type': '>i',
               'factor': 1000,
               'ev_name': 'air_temp'},
             'Air Humidity': {
               'char': '00001003-0000-1000-8000-00805f9b34fb',
               'type': '>i',
               'factor': 1000,
               'ev_name': 'air_humidity'},
             'Water Flow Sensor': {
               'char': '00001004-0000-1000-8000-00805f9b34fb',
               'type': 'B',
               'factor': 1,
               'ev_name': 'flow'},
             'Water Level': {
               'char': '00001005-0000-1000-8000-00805f9b34fb',
               'type': '>i',
               'factor': 1000,
               'ev_name': 'water_level'},
             'UV Index': {
               'char': '00001006-0000-1000-8000-00805f9b34fb',
               'type': '>h',
               'factor': 100,
               'ev_name': 'UV_index'},
             'Visible Light Intensity': {
               'char': '00001007-0000-1000-8000-00805f9b34fb',
               'type': '>h',
               'factor': 1,
               'ev_name': 'visible_light'},
             'IR Light Intensity': {
               'char': '00001008-0000-1000-8000-00805f9b34fb',
               'type': '>h',
               'factor': 1,
               'ev_name': 'ir_light'},
             'Time Unix Epoch': {
               'char': '00001009-0000-1000-8000-00805f9b34fb',
               'type': '>l',
               'factor': 1,
               'ev_name': 'measurement_time'},
             'Pump Speed': {
               'char': '00001010-0000-1000-8000-00805f9b34fb',
               'type': '>h',
               'factor': 1,
               'ev_name': 'pump_speed'},
             'Error Status': {
               'char': '00001012-0000-1000-8000-00805f9b34fb',
               'type': 'B',
               'factor': 1,
               'ev_name': 'error_status' }
            }
