# Hardware Connection Guide

This guide provides detailed instructions for connecting the nRF52 DK development board with the required sensors and power management components via I2C.

## Overview

This setup is designed for development with the following SoCs:
- **nRF52805** (192KB Flash, 24KB RAM)
- **nRF52810** (192KB Flash, 24KB RAM)
- **nRF52832** (512KB Flash, 64KB RAM)

### Components

1. **nRF52 DK** - Main development board
   - [Product Page](https://www.nordicsemi.com/Products/Development-hardware/nRF52-DK)

2. **nPM1100 EK** - Power management evaluation kit
   - [Product Page](https://www.nordicsemi.com/Products/Development-hardware/nPM1100-EK)

3. **Heart Rate 2 Click (MIKROE-4037)** - MAXM86161EFD+T optical sensor
   - [Product Page](https://www.mikroe.com/heart-rate-2-click)

4. **Accel 5 Click (MIKROE-3149)** - BMA400 accelerometer
   - [Product Page](https://www.mikroe.com/accel-5-click)

5. **MAX30208EVSYS** - MAX30208CLB+T temperature sensor evaluation system
   - [Product Page](https://www.analog.com/en/resources/evaluation-hardware-and-software/evaluation-boards-kits/max30208evsys.html)

## I2C Bus Configuration

The nRF52 DK uses **I2C0** (TWI0) for communication with all sensors. The I2C bus is configured as follows:

- **SCL (Serial Clock)**: GPIO P0.26 (Pin 26)
- **SDA (Serial Data)**: GPIO P0.27 (Pin 27)
- **Pull-up resistors**: 4.7kΩ (typically included on Click boards)

### nRF52 DK Pin Assignments

| Function | nRF52 DK Pin | GPIO | Description |
|----------|-------------|------|-------------|
| I2C SCL  | P0.26       | 26   | Clock line   |
| I2C SDA  | P0.27       | 27   | Data line    |
| 3.3V     | 3.3V        | -    | Power supply |
| GND      | GND         | -    | Ground       |

## Sensor Connections

### 1. Heart Rate 2 Click (MAXM86161EFD+T)

The Heart Rate 2 Click board features the MAXM86161 optical sensor for heart rate and SpO2 monitoring.

**I2C Address**: `0x5E` (7-bit address)

**Connections**:

| Heart Rate 2 Click | nRF52 DK | Description |
|-------------------|----------|-------------|
| SCL                | P0.26    | I2C Clock   |
| SDA                | P0.27    | I2C Data    |
| 3.3V               | 3.3V     | Power (3.3V)|
| GND                | GND      | Ground      |
| INT                | P0.28    | Interrupt (optional) |

**Note**: The Heart Rate 2 Click board has a voltage selection jumper (JP1). Set it to the **left position** for 3.3V operation.

### 2. Accel 5 Click (BMA400)

The Accel 5 Click board features the BMA400 low-power accelerometer.

**I2C Address**: `0x18` or `0x19` (7-bit address, depends on SA0 pin)

**Connections**:

| Accel 5 Click | nRF52 DK | Description |
|--------------|----------|-------------|
| SCL           | P0.26    | I2C Clock   |
| SDA           | P0.27    | I2C Data    |
| 3.3V          | 3.3V     | Power (3.3V)|
| GND           | GND      | Ground      |
| INT1          | P0.29    | Interrupt 1 (optional) |
| INT2          | P0.30    | Interrupt 2 (optional) |

**Note**: Check the Accel 5 Click board documentation for the SA0 pin configuration to determine the I2C address.

### 3. MAX30208EVSYS (MAX30208CLB+T)

The MAX30208EVSYS evaluation system includes the MAX30208 temperature sensor.

**I2C Address**: `0x50` (7-bit address, default)

**Connections**:

| MAX30208EVSYS | nRF52 DK | Description |
|---------------|----------|-------------|
| SCL            | P0.26    | I2C Clock   |
| SDA            | P0.27    | I2C Data    |
| 3.3V           | 3.3V     | Power (3.3V)|
| GND            | GND      | Ground      |
| INT            | P0.31    | Interrupt (optional) |

**Note**: Refer to the MAX30208EVSYS documentation for specific pin assignments on the evaluation board.

## Power Management with nPM1100 EK

The nPM1100 Evaluation Kit provides power management capabilities for the development setup.

### nPM1100 EK Configuration

1. **Power Supply Connection**:
   - Connect the nPM1100 EK to a USB power source or battery
   - The nPM1100 EK provides regulated output voltage

2. **Output Voltage Selection**:
   - Use the DIP switches on the nPM1100 EK to configure:
     - Output voltage (typically 3.3V for this setup)
     - Current limits
     - Charging parameters

3. **Connection to nRF52 DK**:
   - Connect the nPM1100 EK output to the nRF52 DK power input
   - Ensure common ground connection

4. **LED Indicators**:
   - Monitor the nPM1100 EK LEDs for:
     - Charge status
     - Error conditions
     - Power state

### Power Distribution

```
nPM1100 EK (VOUT)
    |
    +---> nRF52 DK (3.3V)
    |
    +---> Heart Rate 2 Click (3.3V)
    |
    +---> Accel 5 Click (3.3V)
    |
    +---> MAX30208EVSYS (3.3V)
    |
    +---> Common GND (all devices)
```

## Complete Wiring Diagram

```
                    nRF52 DK
                    ┌─────────┐
                    │         │
                    │  P0.26  │───┬─── SCL (I2C Bus)
                    │  P0.27  │───┼─── SDA (I2C Bus)
                    │  3.3V   │───┼─── Power
                    │  GND    │───┼─── Ground
                    │         │   │
                    └─────────┘   │
                                  │
                    ┌─────────────┼─────────────┐
                    │             │             │
         Heart Rate │   Accel 5   │  MAX30208   │
         2 Click    │    Click    │   EVSYS     │
         (0x5E)     │   (0x18)    │   (0x50)    │
                    │             │             │
                    └─────────────┴─────────────┘
```

## I2C Address Summary

| Device | I2C Address (7-bit) | I2C Address (8-bit) | Notes |
|--------|---------------------|---------------------|-------|
| BMA400 (Accel 5 Click) | 0x18 or 0x19 | 0x30 or 0x32 | Depends on SA0 pin |
| MAX30208 | 0x50 | 0xA0 | Default address |
| MAXM86161 (Heart Rate 2 Click) | 0x5E | 0xBC | Fixed address |

## Board File Configuration

The board files for each SoC are configured with the following I2C devices:

- **BMA400**: Address `0x18`, compatible string `"bosch,bma400"`
- **MAX30208**: Address `0x50`, compatible string `"maxim,max30208"`
- **MAXM86161**: Address `0x5E`, compatible string `"maxim,maxm86161"`

All devices are connected to I2C0 with:
- Clock frequency: Standard I2C (100 kHz)
- SCL: GPIO P0.26
- SDA: GPIO P0.27

## Development Setup

### 1. Hardware Assembly

1. Place the nRF52 DK on a stable surface
2. Connect the nPM1100 EK for power management (optional for initial testing)
3. Connect all Click boards and evaluation boards to the I2C bus
4. Ensure all devices share a common ground
5. Verify power supply voltage (3.3V) for all devices

### 2. Software Configuration

1. Select the appropriate board configuration:
   ```bash
   # For nRF52805
   west build -b nRF52805-dev
   
   # For nRF52810
   west build -b nRF52810-dev
   
   # For nRF52832
   west build -b nRF52832-dev
   ```

2. The board files automatically configure:
   - I2C0 peripheral
   - Device tree nodes for all sensors
   - Pin assignments

### 3. Testing I2C Communication

Use I2C scanner tools or sample applications to verify communication with each sensor:

- Verify each device responds at its expected I2C address
- Check for I2C bus errors or conflicts
- Test reading device identification registers

## Troubleshooting

### Common Issues

1. **I2C Bus Not Responding**
   - Check SCL/SDA connections
   - Verify pull-up resistors (4.7kΩ)
   - Ensure devices are powered

2. **Wrong I2C Address**
   - Verify device address configuration
   - Check for address conflicts
   - Review device datasheets for address selection

3. **Power Issues**
   - Verify 3.3V power supply
   - Check current consumption limits
   - Ensure proper ground connections

4. **Device Not Detected**
   - Verify device tree configuration
   - Check compatible strings match driver
   - Review device status in device tree

## Additional Resources

- [nRF52 DK User Guide](https://infocenter.nordicsemi.com/topic/ug_nrf52_dk/UG/nrf52_DK/nRF52_DK_intro.html)
- [nPM1100 EK Documentation](https://infocenter.nordicsemi.com/topic/ug_npm1100_ek/UG/npm1100_ek/npm1100_ek_intro.html)
- [Heart Rate 2 Click Documentation](https://www.mikroe.com/heart-rate-2-click)
- [Accel 5 Click Documentation](https://www.mikroe.com/accel-5-click)
- [MAX30208 Datasheet](https://www.analog.com/media/en/technical-documentation/data-sheets/max30208.pdf)
- [MAXM86161 Datasheet](https://www.analog.com/media/en/technical-documentation/data-sheets/maxm86161.pdf)
- [BMA400 Datasheet](https://www.bosch-sensortec.com/products/motion-sensors/accelerometers/bma400/)

## Notes

- All I2C devices share the same bus (I2C0)
- Interrupt pins are optional but recommended for efficient operation
- The nPM1100 EK is optional for development but recommended for power management features
- Ensure all devices operate at 3.3V logic levels
- The board files are configured for development with the nRF52 DK hardware

