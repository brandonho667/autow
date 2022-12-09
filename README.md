# Autow

Automatic Towing Project for ECE148 - Autonomous Vehicles

[![tow.jpg](https://lh3.googleusercontent.com/ETM5nEdAuAPoGptogoDfgaenrlEyppe_SOuuu8noqBOiiAg38JaPkK7FJOQbmcTWUQcz_k8Oiygi9rJC3J5B162V2X2V-AnOIJCrNhd43orNk5mG9inGdFcmefWZK0gGHl2EkfREPboFY4oDCO2aYbZbv1Mlg80dhlfARRCrXyB8pXBNj9gsPxlCetLIlfAeORHmw0AcK3trFkH6JEfDNI63wjzGivzUV39nY97w52a06E9iAE9VmUp9aUxxmd_b2NH2M-A10rjM7EXSKLnjSJpLq8Qxj0HrPWxjVYOKHRDlCg5KycvBOqkY9VWr_T5o_AhDPcOCwj6F3lNzzm2yjNF6z9wNRELbE8c4USmZdGjYM8aQnwyIXMbQfgGsyU--AiKElGRS1zAhT0PNj9FWozDvDpwsGDFhHxOlsac_kK7pN864eTFoK5ZK-HDAutKRm9YrZGRQwQ16XOHWd2ksiGHmrOaeRCObQt8khzZH2J65_WVCT42XMRn_DCoNuaOkHC_XqD8Kev0lvubKffwE-6NXZE-HwIPV3IzBBtaNmADNL7jWhPwChmdVCRD9PeH2n7hjDVdZQjzox6SZCAB_PG73CbsGkDtXTp6K9uxk3gnuvyNh-1SSoexMd2ubRTCS7bCbXQs1-OuhbAlDs5rhrG7G3jZ5Lq5QN8m-kD3qw-psBV0uTyemW9qEB8Mcu863T05sF0Ei-3JYCuwVWUODId0TUoyCApIPn8Y3ciHbdXBjqEWufL5Sqr0_YCU9_YwjF2hxq744mBxeY5NSS2XtuV0YflVN4-bgI8csDoDm2uUGurpRIHjvPEBtQze3taeYMIYQqq256bp4y6xISDF2L8AH6ksTOStezwp3T8SokizGbKssfJpl0GhwgUYnqW4vZPygXGjp9PHgCE_H0JI6hOSFa1VnnC_jJNZgtgUfO_z3OPIGiL15N1_QZCYs0GJ0S6YWevzA58BJX9ZYWgmb4liD9-DzpTe1lrYFirj9WO6c7TGINuekGVE2=w1848-h835-no?authuser=0)](https://www.youtube.com/watch?v=caYrD2hRw2U)

<p style="text-align:center"> Autow Demo Video </p>

## Features

- Joystick RC car control
- AR Tag Detection and Pose Estimation
- Autonomous tow hitch lineup to trailer with dynamic steering
- Asynchronous driver cancellation during autonomous hitching

## Steps for running

1. `cd` to your `ros2_ws`
2. `colcon build --packages-select autow_ros`
3. `source install/setup.bash`
4. `ros2 launch autow_ros autow_launch.yaml`

## Helpful Docker commands on Robocar

- `robocar_docker ros_container`
- `` docker start  `docker ps -q -l`  ``
- `` docker attach `docker ps -q -l`  ``

## Audio on Jetson Nano

- `sudo pulseaudio --system -D` to start PulseAudio
- `pax11publish -r` to enable connection to PulseAudio
