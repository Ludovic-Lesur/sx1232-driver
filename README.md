# Description

This repository contains the **SX1232** RF transceiver driver.

# Dependencies

The driver relies on:

* An external `types.h` header file defining the **standard C types** of the targeted MCU.
* The **embedded utility functions** defined in the [embedded-utils](https://github.com/Ludovic-Lesur/embedded-utils) repository.

Here is the versions compatibility table:

| **sx1232-driver** | **embedded-utils** |
|:---:|:---:|
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
