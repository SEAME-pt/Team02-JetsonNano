
# Current Structure of the project files in the Jetson
/opt/vehicle
├── bin
│   ├── CombinedController
│   ├── LaneDetector
│   ├── MiddleWare
│   ├── VehicleSystem
│   └── XboxController
├── etc
│   ├── systemd            **Missing services located**
│   │   ├── controller.service
│   │   ├── middleware.service
│   │   └── vehiclesystem.service
│   └── zenoh
│       ├── ControllerConfig.json
│       ├── LaneDetectorConfig.json
│       ├── MiddleWareConfig.json
│       ├── PidControllerConfig.json
│       ├── VehicleSystemConfig.json
│       └── XboxControllerConfig.json
├── tools
│   └── system_monitor
│       ├── monitor_config.json
│       └── system_monitor
│      └── zenoh-router
│           ├── docker-compose.yml
│           └── zenoh-docker
│               ├── config
│               │   └── zenoh.json5
│               └── lib
│                   ├── libzenoh_backend_fs.so
│                   ├── libzenoh_backend_influxdb.so
│                   ├── libzenoh_backend_influxdb2.so
│                   ├── libzenoh_plugin_rest.so
│                   └── libzenoh_plugin_storage_manager.so
└── vss ...

## Zenoh Router Notes

The zenoh-router should be located at opt/vehicle/zenoh-router

/opt/vehicle/zenoh-router
├── docker-compose.yml
└── zenoh-docker/
    ├── lib/
    └── config/
        └── zenoh.json5  

## System Monitor Notes

The system_monitor folder will be located at opt/vehicle/tools/system_monitor

/opt/vehicle/tools/system_monitor
├── system_monitor                
└── monitor_config.json         

## Notes:
 - Docker compose requires sudo permissions to be run in the service
 - The zenoh router requires a library folder / files 
