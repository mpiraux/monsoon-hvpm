# Monsoon HVPM Rust library & tools

An unofficial library & tools for the [Monsoon High Voltage Power Monitor](https://www.msoon.com/high-voltage-power-monitor) written in Rust.

This project reimplements some of the features in the official [Monsoon](https://github.com/msoon/PyMonsoon) Python library.

## Restrictions

This has only been tested on Linux so far. It only supports the latest hardware and software revisions, i.e. hardware model 2 and firmware version 32.

## CLI Usage

This project bundles a simple CLI to interact with the power monitor. it prints out collected samples as CSV on stdout.

```
Usage: msoon [OPTIONS]

Options:
      --set-voltage <SET_VOLTAGE>
          Sets the voltage and enables the power supply output
      --usb-passthrough <USB_PASSTHROUGH>
          Sets the USB passthrough mode [default: off] [possible values: off, on, auto]
      --keep-output-enabled
          Keeps the output enabled when the program exits. Defaults to false
      --sampling-duration <SAMPLING_DURATION>
          Starts sampling for the given duration in seconds [default: 0]
      --aggregate-samples
          Aggregates collected samples per seconds. Defaults to false
  -h, --help
          Print help
  -V, --version
          Print version
```
