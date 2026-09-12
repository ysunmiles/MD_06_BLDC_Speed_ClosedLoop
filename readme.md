# MD_06_BLDC_Speed_ClosedLoop

STM32F407 BLDC motor-control firmware based on STM32 HAL, CMSIS-RTOS V2 and FreeRTOS. The current application implements Hall-sensor six-step commutation, forward/reverse control, closed-loop speed PI control, ADC-DMA monitoring, OLED display, and UART FireWater output for VOFA+.

## Features

- Hall-sensor six-step BLDC commutation
- Forward and reverse start/stop control through two EXTI buttons
- TIM1 three-channel PWM for the high-side phases
- GPIO-controlled low-side phase outputs
- Hall speed measurement using TIM5
- 12-sample moving-average speed filter
- PI speed controller with a target speed of `1000 rpm`
- ADC1 and ADC3 continuous conversion with circular DMA
- OLED speed display
- USART1 FireWater output for VOFA+

This is an experimental closed-loop speed-control prototype. Current limiting, robust fault handling, hardware dead-time verification, and production-grade direction-change sequencing still require further validation.

## Runtime Flow

1. `main()` initializes GPIO, DMA, USART1, ADC1, ADC3, TIM1, and TIM5.
2. `MX_FREERTOS_Init()` creates `MonitorTask` and `MotorCtrlTask`.
3. `MonitorTask` starts ADC1/ADC3 DMA and waits for Hall-period notifications.
4. `HAL_GPIO_EXTI_Callback()` measures the time between Hall transitions with TIM5 and notifies `MonitorTask`.
5. `MonitorTask` calculates speed, applies the moving-average filter, updates ADC measurements, sends a FireWater frame, and refreshes the OLED.
6. TIM1 update interrupts call `MotorCtrl_PWMCallback()`, which calculates PI duty and applies the current Hall commutation state.
7. The two buttons notify `MotorCtrlTask` to start or stop forward/reverse operation.

## Motor Control

The forward six-step table is:

| Hall state | High-side PWM | Low-side output |
| --- | --- | --- |
| 5 | U+ | V- |
| 1 | U+ | W- |
| 3 | V+ | W- |
| 2 | V+ | U- |
| 6 | W+ | U- |
| 4 | W+ | V- |

The reverse table is implemented in `APP/motorCtrl.c` and must be verified against the actual Hall sequence and motor phase wiring.

Before each commutation state, all three PWM channels and low-side GPIO outputs are disabled. TIM1 is configured as follows:

```text
TIM1 clock = 168 MHz
Prescaler  = 21 - 1
Period     = 400 - 1
PWM        = 168 MHz / 21 / 400 = 20 kHz
```

`duty` is the raw TIM1 compare value, not a percentage. The timer full-scale value is `400` (`ARR = 399`). The current PI output is limited to `0`-`100` for testing and is then written to the compare register. The low-side outputs are ordinary GPIOs rather than TIM1 complementary outputs. TIM1 dead-time is configured as zero, so dead-time and shoot-through protection must be handled by the gate driver or added before higher-power testing.

## Pinout

| Function | MCU pin | Configuration |
| --- | --- | --- |
| Forward button `KEY1` | PE3 | Falling-edge EXTI, pull-up |
| Reverse button `KEY2` | PE4 | Falling-edge EXTI, pull-up |
| U/V/W back-EMF inputs | PF9 / PF8 / PF7 | ADC3 channels 7 / 6 / 5 |
| Temperature input `VTEMP` | PA0 | ADC3 channel 0 |
| Phase current U/V/W | PB0 / PA6 / PA3 | ADC1 channels 8 / 6 / 3 |
| DC bus voltage `VBUS` | PB1 | ADC1 channel 9 |
| Hall U/V/W | PH10 / PH11 / PH12 | Rising/falling-edge EXTI inputs |
| Driver shutdown/enable | PF10 | `CTRL_SD` GPIO output |
| U/V/W high-side PWM | PA8 / PA9 / PA10 | TIM1 channels 1 / 2 / 3 |
| U/V/W low-side outputs | PB13 / PB14 / PB15 | GPIO outputs |
| OLED SCL/SDA | PD14 / PD0 | Software GPIO interface |

Verify the driver IC enable polarity before powering the motor. The firmware controls the driver through `CTRL_SD` and stops the motor by disabling PWM channels and resetting all low-side GPIO outputs.

## VOFA+ FireWater Output

USART1 is configured for **115200 baud, 8 data bits, no parity, 1 stop bit**. Each line contains the following 13 comma-separated values:

```text
BEMFu,BEMFv,BEMFw,Iu,Iv,Iw,Vbus,temp,Hallu,Hallv,Hallw,speed,duty
```

Example:

```text
12.345,12.346,12.347,1.200,1.201,1.202,24.000,35.600,1,0,1,1000.000,35
```

In VOFA+, select the serial port, set the baud rate to 115200, and choose the FireWater protocol. The first eight fields and `speed` are floating-point values; the three Hall fields are logic levels; `duty` is the integer TIM1 compare value, with a full scale of `400`.

## Project Layout

- `APP/motorCtrl.c`: Hall commutation, direction control, PI speed control, and driver control
- `APP/monitor.c`: Hall speed measurement, moving-average filtering, ADC-DMA acquisition, OLED update, and FireWater UART output
- `APP/monitor.h`: `MotorDatasType` definition
- `Core/`: STM32CubeMX-generated initialization, interrupts, and RTOS setup
- `BSP/`: OLED driver
- `Drivers/`: STM32F4 CMSIS and HAL drivers
- `Middlewares/Third_Party/FreeRTOS/`: FreeRTOS kernel and CMSIS-RTOS adapter
- `config.ioc`: STM32CubeMX configuration source
- `CMakeLists.txt` and `CMakePresets.json`: CMake/Ninja build configuration

## Build

### Prerequisites

- CMake 3.22 or later
- Ninja
- Arm GNU Toolchain with `arm-none-eabi-gcc`
- ST-LINK or another compatible programmer/debugger

From the project root:

```powershell
cmake --preset Debug
cmake --build --preset Debug
```

For an optimized build:

```powershell
cmake --preset Release
cmake --build --preset Release
```

Build outputs are placed in `build/Debug` or `build/Release`. The executable target is named `config`.

## Debug Checklist

When the motor runs in the wrong direction or trips over-current, check these signals in order:

1. Confirm the correct button EXTI is reached: `KEY1` for forward and `KEY2` for reverse.
2. Record the Hall sequence during rotation; valid states are `1` through `6`, not `0` or `7`.
3. Confirm the reverse Hall commutation table matches the actual motor phase order.
4. Measure PA8/PA9/PA10 for 20 kHz PWM and PB13/PB14/PB15 for the selected low-side output.
5. Start with a low fixed duty cycle before enabling the PI controller.
6. Test with a current-limited supply until dead-time, current limiting, and fault handling are verified.

## Regenerating CubeMX Files

Update peripheral and pin settings in `config.ioc`, then regenerate the project with STM32CubeMX. Keep application logic in the user source files and preserve the generated source list in `CMakeLists.txt` when adding new application files.
