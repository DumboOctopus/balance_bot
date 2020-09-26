# balance_bot
2 wheeled self balancing robot

# First attempt

First I tried modeling the physics of the two wheel robot. The main idea is that when the robot is at a certain angle, it will fall at a certain speed.
In order to stop the falling, we must move the wheels fast enough to cancel out **the horizontal component** of the falling velocity. As long as motor propels
the bot at that speed, the falling will stop (I think). 

In practice, it sorta worked but the problem was that it could not react fast enough and it was kinda jerky. I think maybe my math is missing a term.
Or maybe the whole concept is wrong. IDk


# Second attempt

PID control. This one is fairly straight forwards. Use a PID control. Tune the constants. 

The error is calculated like this:

```
error = deg_from_gyro - angle_where_it_balances
```

