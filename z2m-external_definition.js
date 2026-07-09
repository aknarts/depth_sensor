const {identify, numeric, temperature, light} = require('zigbee-herdsman-converters/lib/modernExtend');

const definition = {
    zigbeeModel: ['Depth.Sensor'],
    model: 'Depth.Sensor',
    vendor: 'Acheta',
    description: 'Automatically generated definition',
    extend: [identify(), light({"color": true, "effect": false, "powerOnBehavior": false}), temperature(), numeric({
        name: 'depth',
        cluster: 'genAnalogOutput',
        attribute: 'presentValue',
        reporting: {min: '10_SECONDS', max: '1_HOUR', change: 1},
        description: 'Measure distance from sensor',
        unit: 'mm',
        valueMin: 0,
        valueMax: 5000,
        access: 'STATE_GET',
    })],
    meta: {},
};

module.exports = definition;
