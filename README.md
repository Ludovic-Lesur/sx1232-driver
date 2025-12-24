# Description

This repository contains the **SX1232** RF transceiver driver.

# Dependencies

The driver relies on:

* An external `types.h` header file defining the **standard C types** of the targeted MCU.
* The **embedded utility functions** defined in the [embedded-utils](https://github.com/Ludovic-Lesur/embedded-utils) repository.

Here is the versions compatibility table:

| **sx1232-driver** | **embedded-utils** |
|:---:|:---:|
| [sw1.9](https://github.com/Ludovic-Lesur/sx1232-driver/releases/tag/sw1.9) | >= [sw5.0](https://github.com/Ludovic-Lesur/embedded-utils/releases/tag/sw5.0) |
| [sw1.8](https://github.com/Ludovic-Lesur/sx1232-driver/releases/tag/sw1.8) | >= [sw5.0](https://github.com/Ludovic-Lesur/embedded-utils/releases/tag/sw5.0) |
| [sw1.7](https://github.com/Ludovic-Lesur/sx1232-driver/releases/tag/sw1.7) | >= [sw5.0](https://github.com/Ludovic-Lesur/embedded-utils/releases/tag/sw5.0) |
| [sw1.6](https://github.com/Ludovic-Lesur/sx1232-driver/releases/tag/sw1.6) | >= [sw5.0](https://github.com/Ludovic-Lesur/embedded-utils/releases/tag/sw5.0) |
| [sw1.5](https://github.com/Ludovic-Lesur/sx1232-driver/releases/tag/sw1.5) | >= [sw5.0](https://github.com/Ludovic-Lesur/embedded-utils/releases/tag/sw5.0) |
| [sw1.4](https://github.com/Ludovic-Lesur/sx1232-driver/releases/tag/sw1.4) | >= [sw1.3](https://github.com/Ludovic-Lesur/embedded-utils/releases/tag/sw1.3) |
| [sw1.3](https://github.com/Ludovic-Lesur/sx1232-driver/releases/tag/sw1.3) | >= [sw1.3](https://github.com/Ludovic-Lesur/embedded-utils/releases/tag/sw1.3) |
| [sw1.2](https://github.com/Ludovic-Lesur/sx1232-driver/releases/tag/sw1.2) | >= [sw1.3](https://github.com/Ludovic-Lesur/embedded-utils/releases/tag/sw1.3) |
| [sw1.1](https://github.com/Ludovic-Lesur/sx1232-driver/releases/tag/sw1.1) | >= [sw1.3](https://github.com/Ludovic-Lesur/embedded-utils/releases/tag/sw1.3) |
| [sw1.0](https://github.com/Ludovic-Lesur/sx1232-driver/releases/tag/sw1.0) | >= [sw1.3](https://github.com/Ludovic-Lesur/embedded-utils/releases/tag/sw1.3) |

# Compilation flags

| **Flag name** | **Value** | **Description** |
|:---:|:---:|:---:|
| `SX1232_DRIVER_DISABLE_FLAGS_FILE` | `defined` / `undefined` | Disable the `sx1232_driver_flags.h` header file inclusion when compilation flags are given in the project settings or by command line. |
| `SX1232_DRIVER_DISABLE` | `defined` / `undefined` | Disable the SX1232 driver. |
| `SX1232_DRIVER_SPI_ERROR_BASE_LAST` | `<value>` | Last error base of the low level SPI driver. |
| `SX1232_DRIVER_DELAY_ERROR_BASE_LAST` | `<value>` | Last error base of the low level delay driver. |
| `SX1232_DRIVER_FXOSC_HZ` | `<value>` | Oscillator frequency in Hz. |
| `SX1232_DRIVER_TX_ENABLE` | `defined` / `undefined` | Enable radio transmission functions. |
| `SX1232_DRIVER_RX_ENABLE` | `defined` / `undefined` | Enable radio reception functions. |

# Build

A static library can be compiled by command line with `cmake`.

```bash
mkdir build
cd build
cmake -DCMAKE_TOOLCHAIN_FILE="<toolchain_file_path>" \
      -DTOOLCHAIN_PATH="<arm-none-eabi-gcc_path>" \
      -DTYPES_PATH="<types_file_path>" \
      -DEMBEDDED_UTILS_PATH="<embedded-utils_path>" \
      -DSX1232_DRIVER_SPI_ERROR_BASE_LAST=0 \
      -DSX1232_DRIVER_DELAY_ERROR_BASE_LAST=0 \
      -DSX1232_DRIVER_FXOSC_HZ=32000000 \
      -DSX1232_DRIVER_TX_ENABLE=ON \
      -DSX1232_DRIVER_RX_ENABLE=ON \
      -G "Unix Makefiles" ..
make all
```
