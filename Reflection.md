# Reflection
---


## Effect of the P, I, D Components

### Effect of P

This is the coeffecient of cte. Set it to be 0, then there is no response to the angle between the car and the road. Set it bigger, then small angle can result in a big steering value.

When Kp=-1 

![negative P](gif/PID-control_P_is_-1.gif)

When Kp=0.001

![Small P](gif/PID-control_P_is_0.001.gif)

When Kp=1

![Larger P](gif/PID-control_P_is_1.gif)


### Effect of I
This is the innertial bias of the car. Large positive value will send the car to the left: 

When Ki=1

![I=1](gif/PID-control_positive_I.gif)


Extreme negative value will steer the car to the right: 

When Ki=-1
![I=-1](gif/PID-control_negative_I.gif)

### Effect of D
Proper value of D can reduce the oscillating effect from P.

When Kd=0.1

![D=0.1](gif/PID-control_P_is_0.2._I0_D_0.1.gif)

When D is too large, it will introduce high frequency oscillation. 

When Kd=100

![D=100](gif/PID-control_P_is_0.2._I0_D_100.gif)

This is essentially because the solution of a second order differential equation with constant coefficient is a trig function (like sin(kx)), and larger Kd leads to larger k in sin(kx) and hence the high frequency alternation.

## Optimizing Kp, Ki, Kd

I implemented twiddle inside `PID.cpp` (mainly inside `UpdateError()`). My approach was mainly using twiddle but with the help of visual observation to save twiddle's searching time.

I implemented two hyper-parameter searching algorithms: one with twiddle(inside main_twiddle.cpp), one with gradient decent(inside PID.cpp). 

I tried starting with 

```
Kp = 1
Ki = 1
Kd = 1
```
and very short road (`PID::epoch_length_=100`). It does improve the error along time but slowly. Observed that the car tends to steer left when `Ki` is positive and right if negative, I decide to use small `Ki`, i.e. 0.

So I initiated twiddle with 

```
Kp = 1
Ki = 0
Kd = 1
```
and longer road to test (`PID::epoch_length_=200`). This time I saw a trend of increasing `Kd`. Theoretically, I could spend more time and find better and better choices. 


But then I decide to try starting twiddle with 

```
Kp = 0.2
Ki = 0.01
Kd = 3
```
which is taken from the course. 

With the parameters above, the car tends to steer to the left at the beginning. 

I started with a road segment with `epoch_length_=200`. Ran `twiddle` for about 20 times and found `Ki` need to be small quickly. 

Then I creased the length of testing road segment, and initiate the parameters with 

```
Kp = 0.2
Ki = 0.003
Kd = 3
```
where `Ki` from the optimized one from the previous step.

I increased the length of the road segment to `epoch_length_=1000`. Left the program to run overnight, below is the value of Kp, Ki, Kd in the last few rounds:

```
0.232032,0.00099099,8.14317;
0.232032,0.00109009,8.14317;
0.255235,0.00109009,8.14317;
0.255235,0.00109009,7.54353;
0.280758,0.00109009,7.54353;
0.306027,0.00109009,7.54353;
0.306027,0.00116956,7.54353;
0.333822,0.00116956,7.54353;
0.364396,0.000985987,7.54353;
0.364396,0.000985987,7.89407;
0.391638,0.000985987,7.89407;
0.421605,0.000985987,7.89407;
0.421605,0.000908879,7.89407;
0.454567,0.000908879,7.89407;
0.454567,0.000908879,8.17517;
0.481,0.000908879,8.17517;
0.481,0.000964528,8.17517;
0.567671,0.000964528,10.1;
0.6101,0.000964528,10.1;
0.6101,0.000964528,9.45046;
0.656771,0.000964528,9.45046;
0.702976,0.000964528,9.45046;
```

So with the final values:

```
Kp = 0.702976
Ki = 0.000964528
Kd = 9.45046
```
I got the smallest error.

However visually I feel smaller Kd gives smoother drive. This could be because larger Kd increase steering, and thus reduced speed, and thus shorter path, and thus smaller error.

