const {identify, numeric, temperature, light} = require('zigbee-herdsman-converters/lib/modernExtend');

const depthSensors = [
    {name: 'tank_1', endpoint: 1, description: 'Tank 1 depth'},
    {name: 'tank_2', endpoint: 2, description: 'Tank 2 depth'},
];

const depthSensorExtends = depthSensors.map((sensor) => numeric({
    name: 'depth',
    cluster: 'genAnalogOutput',
    attribute: 'presentValue',
    endpointNames: [sensor.name],
    reporting: {min: '10_SECONDS', max: '1_HOUR', change: 1},
    description: sensor.description,
    unit: 'mm',
    valueMin: 0,
    valueMax: 5000,
    access: 'STATE_GET',
}));

const definition = {
    zigbeeModel: ['Depth.Sensor'],
    model: 'Depth.Sensor',
    vendor: 'Acheta',
    description: 'ESP32-C6 Zigbee depth sensor',
    endpoint: () => Object.fromEntries(depthSensors.map((sensor) => [sensor.name, sensor.endpoint])),
    extend: [
        identify(),
        light({"color": true, "effect": false, "powerOnBehavior": false}),
        temperature(),
        ...depthSensorExtends,
    ],
    meta: {multiEndpoint: true},
};

module.exports = definition;
