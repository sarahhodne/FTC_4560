/**
 * TeleOp program for team 4560's FTC robot.
 * All code written by Henrik Hodne unless otherwise noted. All code written by
 * Henrik Hodne is released under the MIT license (see the LICENSE file).
 */

#include "JoystickDriver.c"
#include "4560_Common.h"

// The latest reading from the left joystick on controller 1
float x_val, y_val;

/**
 * Get the angle part of the left joystick and return it.
 *
 * @return Angle part of the left joystick (converted to a polar coordinate).
 */
int getAngle()
{
  return radiansToDegrees(atan2(x_val, y_val));
}

/**
 * Set up the robot (initialize sensors, etc.)
 *
 * Nothing should move in this phase, and servos shouldn't be set to their
 * initial position (use aboutToStart() for that).
 */
void initializeRobot()
{
  compassSetup();
}

/**
 * Set up servos and other initializing things that make the robot move.
 */
void aboutToStart()
{
  compassUp();
}

/**
 * The task handling the driving. This will get the joystick settings, and move
 * accordingly.
 *
 * The driving is all handled by the first game controller. The left joystick
 * drives the robot in the direction it's tilted. The right joystick spins the
 * robot in the direction it's tilted (clockwise is to the right, counter-
 * clockwise to the left).
 */
task drivingTask()
{
  while (true)
  {
    getJoystickSettings(joystick);

    x_val = scaleJoystick(joystick.joy1_x1);
    y_val = scaleJoystick(joystick.joy1_y1);

    // Magnitude part of a vector
    int speedMagnitude = (int)sqrt(pow(abs(x_val), 2) + pow(abs(y_val), 2));

    // Angle part of a vector
    int speedDirection = getAngle();

    if (x_val > 10 || x_val < -10 || y_val > 10 || y_val < -10)
      // We want to move
      moveRobot(speedMagnitude, speedDirection);
    else if (joystick.joy1_x2 > 10 || joystick.joy1_x2 < -10)
      // We want to spin
      spin(scaleJoystick(joystick.joy1_x2));
    else
      spin(0);
  }
}

/**
 * The task handling the arm. This will get the joystick settings, and move the
 * arm accordingly.
 *
 * The arm is all handled by the second game controller. The D-pad moves the
 * arm up and down, as well as button 6 and 8 (the buttons moves in steps, the
 * D-pad). This task (and possibly the whole program) hangs if you try moving
 * the arm too far with steps, as it never reaches where it wants to). Buttons
 * 2, 3 and 4 starts, stops and reverses the sweeper, respectively.
 */
task armTask()
{
  while (true)
  {
    getJoystickSettings(joystick);
    if (joy2Btn(2))
      sweeperOn();
    if (joy2Btn(4))
      sweeperReverse();
    if (joy2Btn(3))
      sweeperOff();
    if (joy2Btn(6))
      armStepUp();
    if (joy2Btn(8))
      armStepDown();
    if (joystick.joy2_TopHat == TopHat_Up)
      motor[motorArm] = 40;
    if (joystick.joy2_TopHat == TopHat_Down)
      motor[motorArm] = -40;
    if (joystick.joy2_TopHat == TopHat_Idle)
      motor[motorArm] = 0;
  }
}

/**
 * The first task to get started. This will initialize the robot, wait for the
 * start signal from the FCS, fire up the other tasks, then just idle until the
 * program ends.
 */
task main()
{
  initializeRobot();
  waitForStart();
  aboutToStart();
  StartTask(drivingTask);
  StartTask(armTask);

  // So the program doesn't just exit.
  while (true)
  {
    // We don't want to hog the CPU here...
    wait1Msec(5);
  }
