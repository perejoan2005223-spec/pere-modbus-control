# Pere: Modbus, fan and valve control

Module for [greenhouse-busta](https://github.com/BurningStratus/greenhouse-busta).

## Current status

Initial scaffold only. The seven source/header files are intentionally empty.
There is no implemented Modbus, fan, valve or queue behavior yet.
They were copied from commit `47d8003` on the group's `pere/modbus-control`
branch. That original branch remains available.

## Responsibilities

- Modbus communication: reuse the template's ModbusClient and PicoOsUart.
- Fan interface and valve interface: `src/actuators/`.
- Fan/valve control task consuming sensor data: `src/control/`.
- Sensor data contract to agree with Fabien: `src/shared/SensorData.h`.

Agree on queue ownership, error/validity reporting, and synchronization of
complete Modbus transactions before implementing the interfaces.

## Building

A standalone scaffold compilation (host compiler, no hardware behavior):

```sh
cmake -S . -B build
cmake --build build
```

For future integration into the group's firmware, after initializing the Pico
SDK and FreeRTOS, add this module from the parent CMake project:

```cmake
add_subdirectory(pere-modbus-control)
target_link_libraries(your_firmware_target PRIVATE pere_modbus_control)
```

Replace `your_firmware_target` with the actual firmware target. Add the needed
Pico/FreeRTOS/driver dependencies to this module's target when the empty source
files begin using those APIs. The module does not create its own main() or
scheduler. Registering a Git submodule does not automatically compile or run it.

## Working with the Git submodule

From the greenhouse-busta root, initialize the recorded versions with:

```sh
git submodule update --init --recursive
```

Initialization checks out the recorded commit, usually in detached HEAD mode.
Before editing inside this submodule, switch to its development branch:

```sh
cd pere-modbus-control
git switch main
```

Commit and push module changes in this repository first. Then, from the parent
repository, commit the updated `pere-modbus-control` pointer on a work branch
and propose that update through a pull request. The parent pins a commit; it
does not automatically follow the module's latest changes.
