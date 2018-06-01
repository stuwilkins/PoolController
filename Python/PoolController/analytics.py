import keen
import yaml
import datetime
import time


def read_config(filename):
    with open(filename, 'r') as ymlfile:
        cfg = yaml.load(ymlfile)
    keen.project_id = cfg['keen.io']['project_id']
    keen.write_key = cfg['keen.io']['write_key']
    keen.read_key = cfg['keen.io']['read_key']
    return cfg


if __name__ == "__main__":
    read_config("/etc/pool_controller.yaml")
    filters = [{"operator": "gt",
                "property_name": "pool_water_temp",
                "property_value": 5},
               {"operator": "lt",
                "property_name": "pool_water_temp",
                "property_value": 50}]

    print(datetime.date.today())
    print(datetime.date.today() - datetime.timedelta(days=1))

    stop = datetime.date.today()
    for day in range(1, 10):
        start = stop - datetime.timedelta(days=1)
        timeframe1 = {"end": "{}T20:00:00.000".format(stop),
                      "start": "{}T20:00:00.000".format(start)}
        timeframe2 = {"end": "{}T22:00:00.000".format(stop),
                      "start": "{}T06:00:00.000".format(stop)}
        print(timeframe1, timeframe2)
        minimum = keen.minimum("Home_Temp_Logging",
                               target_property="pool_water_temp",
                               filters=filters,
                               timeframe=timeframe1,
                               timezone="US/Eastern")

        maximum = keen.maximum("Home_Temp_Logging",
                               target_property="pool_water_temp",
                               filters=filters,
                               timeframe=timeframe1,
                               timezone="US/Eastern")

        temp_gain = 0
        if (minimum is not None) and (maximum is not None):
            print(day, minimum, maximum, maximum - minimum)
            temp_gain = maximum - minimum

        x = keen.average("Home_Temp_Logging",
                         target_property="ir_light",
                         filters=[{"operator": "gt",
                                   "property_name": "ir_light",
                                   "property_value": 0}],
                         timeframe=timeframe2,
                         interval="hourly",
                         timezone="US/Eastern")
        ir_int = 0
        for a in x:
            if(a['value'] is not None):
                ir_int = ir_int + a['value']
        print(ir_int)

        keen.add_event("Analyticals",
                       {"keen": {"timestamp":
                                 "{}T20:00:00.000".format(stop)},
                        "pool_temp_gain": temp_gain,
                        "ir_total": ir_int})
        stop = stop - datetime.timedelta(days=1)
        start = stop - datetime.timedelta(days=1)
        time.sleep(60)
