This is a very simple example of how to generate SVPWM using the STM32f103c8 blue pill board.
The code is based on the HAL lib from ST, the config is generated using CUBEMX.
There are still some debug code there to generate sinwave as well

The board looks like this: The PA8,PA9,PA10 are the three phases output 
![Alt text](./stm32_bp.jpg?raw=true "Wave form pic2")


The wave form looks like this on a 2 channels osc:

![Alt text](./OSC48Ximg201028071127.jpg?raw=true "Wave form pic1")

![Alt text](./OSC48Ximg201028071202.jpg?raw=true "Wave form pic2")

