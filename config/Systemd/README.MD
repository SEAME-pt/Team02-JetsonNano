## Zenoh Router Notes

The zenoh-router should be located at opt/vehicle/zenoh-router

/opt/vehicle/zenoh-router
├── docker-compose.yml
└── zenoh-docker/
    └── config/
        ├── zenoh.json5                     - last working version before adding router
        └── monitoring_zenoh.json5          - Version that should replace the previous to access the router correctly with the new features


## System Monitor Notes

The system_monitor folder will be located at opt/vehicle/tools/system_monitor

/opt/vehicle/tools/system_monitor
├── system_monitor                 - executble
└── monitor_config.json            - configuration file required by the executable to establish connectios