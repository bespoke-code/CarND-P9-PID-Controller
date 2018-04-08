# CarND-P9-PID-Controller
PID controller implementation in C++ to maneuver a vehicle around a track in the Udacity CarND simulator

---

## Reflection

### How PID control works

PID controllers are quite short to implement in code, but some knowledge for their
parameters and characteristics are required to tune them properly in order to get a
satisfactory result.

The parameters work as following:
- Kp is the coefficient representing the 'proportional' (P) part of the PID controller.
In our case, the car Cross-Track Error (CTE) for a given step is multiplied by negative Kp,
in order to provide a steering output proportional to the CTE. What this means is that a
greater CTE will induce a sharper turn towards the desired trajectory.
- Kd is the coefficient representing the 'derivative' (D) part of the PID controller.
The Kd is multiplied by the difference of the last two CTEs (in other words, the rate of change
in the CTE). A negative change in successive CTE measurements (driving towards the desired
 trajectory) will dampen/smoothen the turn towards the desired trajectory, but a greater
 change in successive CTE measurements will provoke a sharper turn towards the trajectory.
 A properly tuned Kd (or derivative part) balances the proportional part in a way,
 effectively damping/smoothing the oscillations caused by the proportional control part.
- Ki is the coefficient representing the 'integral' (I) part of the PID controller. We use Ki
to correct long-term drifts to the left or right of a lane, by multiplying Ki to the sum of
all the CTEs from the beginning to the present moment. Deviations occuring for a longer period
of time will (with time) be corrected (i.e. constantly driving to the left of the track
inside the simulator will be corrected faster, as the sum of all CTE errors increases.

### The effect of the P, I, D components on the driving experience

While tuning the P, I and D coefficients, I noted the change of the behaviour of the car.

- When using a P-controller to go around the track, it became evident to me that even a very
small Kp coefficient (<0.5) can cause big amplified oscillations while driving, especially
after coming to a curve or at higher speed.
- Kp - induced oscillations usually had big amplitudes (big CTE while driving). A constant
offset to the side of the road was evident after driving to the first curve.
- Curves were definitely a menace for the PD controller, since the controller itself didn't
compensate for the accumulated error (offset) to the side of the road. Slowly increasing the
Kd parameter showed a big improvement, but after a few turns, the car could still end up in
bushes or in the lake.
- I observed that the Kd parameter had to be cranked up really high
(around 10x the value of Kp) to not allow oscillations from the P component to dominate
the steering. To speed up the slow reaction in curves, Kp also had to go up in value.
- Setting values "too high" (Kp = 0.8, Kd = 8) shrunk the error faster, but the car was jerking
wildly on the road, which was a no-go for me.
- Tuning Kp at different speeds clearly shows that Kp should be decreased as speed increases.
It is somewhat similar to modern 'weighted steering', maybe? This is something I'll have in
mind when I come back to this project in the future to try and implement more precise and
gentle control for the throttle.

All parameters induced a car behaviour as expected during testing.
Tuning them exposed some inner dependencies for a smooth ride control.


### How the coefficients were tuned

Although the lessons elaborated on Twiddle as an algorithm for PID parameter tuning,
I decided to roughly tune the PID controller coefficients manually, following a procedure
I found in Thomas Bräunl's book, Embedded Robotics:

1. Select a typical operating setting for the desired speed, set Kd = Ki = 0. Increase Kp
to maximum, or until oscillation occurs.
2. If system oscillates, divide Kp by 2.
3. Increase Kd and observe behaviour vhen increasing/decreasing the desired speed by about 5%.
Choose a value of Kd which gives a damped response.
4. Slowly increase Ki until oscillation starts. Then divide Ki by 2 or 3.
5. Check whether overall controller performance is satisfactory under typical system conditions.

After this procedure, I fine-tuned the parameters manually until a fully satisfactory solution
was obtained. In the end, the tuned PID parameters are as following:
- Kp = 0.1
- Kd = 1
- Ki = 0.002