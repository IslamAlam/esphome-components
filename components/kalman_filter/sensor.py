# FILE: /esphome-components/esphome-components/components/kalman_filter/sensor.py

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import CONF_ID, CONF_NAME

DEPENDENCIES = ['sensor']
AUTO_LOAD = ['sensor']

kalman_filter_ns = cg.esphome_ns.namespace('kalman_filter')
KalmanFilterSensor = kalman_filter_ns.class_('KalmanFilterSensor', sensor.Sensor, cg.Component)

CONF_SOURCE_ID = 'source_id'
CONF_PROCESS_NOISE = 'process_noise'
CONF_MEASUREMENT_NOISE = 'measurement_noise'
CONF_ESTIMATED_ERROR = 'estimated_error'

CONFIG_SCHEMA = sensor.sensor_schema().extend({
    cv.GenerateID(): cv.declare_id(KalmanFilterSensor),
    cv.Required(CONF_SOURCE_ID): cv.use_id(sensor.Sensor),
    cv.Optional(CONF_PROCESS_NOISE, default=0.01): cv.float_range(min=0),
    cv.Optional(CONF_MEASUREMENT_NOISE, default=0.1): cv.float_range(min=0),
    cv.Optional(CONF_ESTIMATED_ERROR, default=1.0): cv.float_range(min=0),
}).extend(cv.COMPONENT_SCHEMA)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await sensor.register_sensor(var, config)

    source = await cg.get_variable(config[CONF_SOURCE_ID])
    cg.add(var.set_source(source))
    cg.add(var.set_process_noise(config[CONF_PROCESS_NOISE]))
    cg.add(var.set_measurement_noise(config[CONF_MEASUREMENT_NOISE]))
    cg.add(var.set_estimated_error(config[CONF_ESTIMATED_ERROR]))