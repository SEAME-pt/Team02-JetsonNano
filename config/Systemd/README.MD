## Zenoh Router Notes

The zenoh-router should be located at opt/vehicle/zenoh-router

/opt/vehicle/zenoh-router
├── docker-compose.yml
└── zenoh-docker/
    └── config/
        ├── zenoh.json5                     - configuration file

## System Monitor Notes

The system_monitor folder will be located at opt/vehicle/tools/system_monitor

/opt/vehicle/tools/system_monitor
├── system_monitor                 - executble
└── monitor_config.json            - configuration file required by the executable to establish connections

## Notes:
 - Docker compose requires sudo permissions to be run in the service
 - The zenoh router requires a library folder / files 
