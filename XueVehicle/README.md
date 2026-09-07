# Hello_Vehicle

`Hello_Vehicle` is a minimal ArduPilot application for checking `AP_Param`.
It follows the `AP_Vehicle` application shape (`setup()` and `loop()` through
`AP_HAL_MAIN_CALLBACKS`) without pulling vehicle sensors, flight modes,
controllers, or missions into the application itself.  Its small
`GCS_Hello`/`GCS_MAVLINK_Hello` adapter uses ArduPilot's existing MAVLink
parameter handlers so Mission Planner can discover, read, change, and save
the `XUE_*` values over USB (SERIAL0).

The test registers and loads these parameters through the existing
`AP_Param` library:

`Hello_Vehicle` owns a conventional `Parameters g` object.  The `Xue` object
inside it is registered with `AP_SUBGROUPINFO`, matching the parameter layout
used by ArduPilot vehicles.

- `XUE_GAIN` (default `1.0`)
- `XUE_SPEED` (default `2.5`)
- `XUE_ENABLE` (default `1`)
- `XUE_TEST` (default `42`)

Build and run it on SITL with:

```sh
./waf configure --board sitl
./waf build --target tool/Hello_Vehicle
./build/sitl/tool/Hello_Vehicle -M rover --console
```

On startup it loads saved parameter values and starts the USB MAVLink link.

## Mission Planner

Flash `build/fmuv2/bin/Hello_Vehicle.apj`, connect the board's USB port, and
select its COM port in Mission Planner.  The firmware sends a MAVLink heartbeat
on SERIAL0 and uses ArduPilot's standard handlers for `PARAM_REQUEST_LIST`,
`PARAM_REQUEST_READ`, and `PARAM_SET`.  The four `XUE_*` values therefore
appear in the Full Parameter List and changed values persist across reboot.
