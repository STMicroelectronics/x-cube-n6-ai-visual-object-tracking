# Release Notes for x-cube-n6-ai-visual-object-tracking Application

## Purpose

Computer Vision application demonstrating the deployment of a Visual Object Tracking (VOT) model on the STM32N6570-DK board.

This application is prebuilt with the LightTrackM VOT model for STM32N6570-DK.

## Key Features

- Multi-threaded application flow (FreeRTOS)
- Single-object tracking with basic re-identification support
- Touchscreen target selection
- NPU accelerated quantized AI model inference
- Dual DCMIPP pipes with downscale
- LTDC dual-layer implementation
- DCMIPP ISP usage
- Dev mode
- Boot from External Flash

## Software components

| Name                               | Version             | Release notes
|-----                               | -------             | -------------
| stedgeai-lib                       | 12.0.1              |
| stm32-mw-camera                    | v1.5.1              | [release notes](Lib/stm32-mw-camera/Release_Notes.md)
| NemaGFX SDK                        | v1.4.0              | [release notes](Lib/NemaGFX/doc/ChangeLog.md)
| stm32-vision-models-postprocessing | v0.15.0             | [release notes](Lib/stm32-vision-models-postprocessing/lib_vision_models_pp/README.md)
| ai-postprocessing-wrapper          | v1.1.1              | [release notes](Lib/ai-postprocessing-wrapper/Release_Notes.md)
| CMSIS                              | V6.2.0              | [release notes](STM32Cube_FW_N6/Drivers/CMSIS/Documentation/index.html)
| STM32N6xx CMSIS Device             | V1.4.0              | [release notes](STM32Cube_FW_N6/Drivers/CMSIS/Device/ST/STM32N6xx/Release_Notes.html)
| STM32N6xx HAL/LL Drivers           | V1.4.0              | [release notes](STM32Cube_FW_N6/Drivers/STM32N6xx_HAL_Driver/Release_Notes.html)
| STM32N6570-DK BSP Drivers          | V1.4.1              | [release notes](STM32Cube_FW_N6/Drivers/BSP/STM32N6570-DK/Release_Notes.html)
| BSP Component aps256xx             | V1.0.6              | [release notes](STM32Cube_FW_N6/Drivers/BSP/Components/aps256xx/Release_Notes.html)
| BSP Component Common               | V7.3.0              | [release notes](STM32Cube_FW_N6/Drivers/BSP/Components/Common/Release_Notes.html)
| BSP Component mx66uw1g45g          | V1.1.0              | [release notes](STM32Cube_FW_N6/Drivers/BSP/Components/mx66uw1g45g/Release_Notes.html)
| BSP Component mx25um51245g         | V1.0.1              | [release notes](STM32Cube_FW_N6/Drivers/BSP/Components/mx25um51245g/Release_Notes.html)
| BSP Component rk050hr18            | V1.0.1              | [release notes](STM32Cube_FW_N6/Drivers/BSP/Components/rk050hr18/Release_Notes.html)
| BSP Component gt911                | V1.0.1              | [release notes](STM32Cube_FW_N6/Drivers/BSP/Components/gt911/Release_Notes.html)
| FreeRTOS kernel                    | v10.6.2             | [release notes](Lib/stm32-mw-freertos/Source/History.txt)
|                                    | ST modified 20241011| [ST release notes](Lib/stm32-mw-freertos/Source/st_readme.txt)
| Fonts utility                      | V2.0.4              | [release notes](STM32Cube_FW_N6/Utilities/Fonts/Release_Notes.html)
| LCD utility                        | V2.2.0              | [release notes](STM32Cube_FW_N6/Utilities/lcd/Release_Notes.html)
| AI FSBL                            | v1.4.0              | [release notes](FSBL/Release_Notes.md)

## Update history

### V1.0.0 / October 2026

Initial Version
