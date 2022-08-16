# S-curve-stepdir

Here is a tiny S-curve acceleration/deceleration algorithm for step motors (using external step/dir driver), and a visulization tool for that.
Expected to be used on low-cost microcontrollers such as Arduino Uno / (maybe even Arduino Nano) / STM32F10xxx. It also can be run on more powerful systems, but it would be better to use there more complex algorithms with calculations on-the-fly and exponential curves.

## Key features:
* Smooth velocity function consists of 3 stages : quadratic (increasing) acceleration, linear acceleration and quadratic (decreasing) acceleration to target speed
* Only simple calculations: float-point multiplications, sums and divisions
* Calculate acceleration table before moving and saving that in memory
* Short code in raw C
* Equal area discretization of pulse curve
* Newyhon's iteration method instead of accurate solving improves calculation speed on microcontrollers without FPU

![alt text](https://github.com/arabel1a/hg-set/blob/86c0ac5c8a4a52a7e100a826815487dfb3b82b9d/graphs.png?raw=true)

# How to use

## Backend
Just add source code to your project. 
Then you need to specify acceleration parameters. Create struct "profile" and fill theese fields:
*   `float f_0`            Starting speed, (micro)steps/s. Usually it's just a maximan speed that controller can reach from standstill state.
*   `float f_c`            Desired speed, (micro)steps/s.
*   `float t_c`            Desired acceleration time, s. will be overwritten, if trajectory planner is used.
*   `float max_t_c`			   Maximum acceleration time, s.
*   `float c`              Maximum allowed acceleration, (micro)steps/s^2.
*   `float * timings`      Pointer to table of timings. Can be just a massive or FLASH memory adress.
*   `int ksi`              Guaranteed path that will be passed with constant speed, steps.


In my application, it is more nessesary to pass some way with constant speed, than to reach concrette target speed in concrette time. If it's possible with choosen `c`, algorithm will pass the intreval `[center-ksi, center+ksi]`, where center means center of path, with speed `f_c`. If not, this interval will be passed with maximal aviable constant speed. In other intervals, it chooses the sloqwest possible acceleration to decrease vibrations and chance of step loss.  If it's not so for you, just write your own trajectory planner, main mathematic will be the same.

After cpecifying this patrameters just call `trajectory_planner`. 

Now, you have acceleration table `[your_sructure_name].timings`. Just make a step pulse (using timer interrupts or just a delay) with time between i-th and (i+1)th equal `[your_sructure_name].timings[i]`. When you reach the end of a table, you should make pulses with delay equal to the lats timing, untill you get to deceleration position. Then pass this (or another, computed the same way? if you need) table in inverse order. That's all. For more details, see example.

## Visualization

For debug and aesthetic purposes, there is a visualization tool. To run it, you need python3 with numpy and matplotlib and jupyter notebook. Or you can run it in google colab. Juct specify required parameters in 1st cell, click "run all" and enjoy pretty graphs.

## Example
![alt text](https://github.com/arabel1a/hg-set/blob/80d84c4c87f63d60765237c2049142040c9b7a02/example.png?raw=true)
There is a short example for STM32F103RB, written as MDK-ARM project. 
/*/*/**/*/*/* TODO example description.

