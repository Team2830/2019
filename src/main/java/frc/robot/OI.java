/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
  //// CREATING BUTTONS
  // One type of button is a joystick button which is any button on a
  //// joystick.
  // You create one by telling it which joystick it's on and which button
  // number it is.
  // Joystick stick = new Joystick(port);
  // Button button = new JoystickButton(stick, buttonNumber);

  // There are a few additional built in buttons you can use. Additionally,
  // by subclassing Button you can create custom triggers and bind those to
  // commands the same as any other Button.

  //// TRIGGERING COMMANDS WITH BUTTONS
  // Once you have a button, it's trivial to bind it to a button in one of
  // three ways:

  // Start the command when the button is pressed and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenPressed(new ExampleCommand());

  // Run the command while the button is being held down and interrupt it once
  // the button is released.
  // button.whileHeld(new ExampleCommand());

  // Start the command when the button is released and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenReleased(new ExampleCommand());
  public  Joystick joystick = new Joystick(0);
  Joystick driverStick; 
	Joystick operatorStick;

	// Start the command when the button is released and let it run the command
	// until it is finished as determined by it's isFinished method.
	// button.whenReleased(new ExampleCommand());
  Button button1, button2, button3, button4, button5, button6, button7, button8, button9,button10, dbutton5, dbutton6,dbutton7, dbutton8;
  public OI(){
		driverStick = new Joystick(0);
    operatorStick = new Joystick(1);


    
    /** button1  = new JoystickButton(operatorStick, 3);
    button1.whenPressed(new CargoUp());
    
    button2  = new JoystickButton(operatorStick, 2);
    button2.whenPressed(new CargoOuttake());
    
    button3  = new JoystickButton(operatorStick, 1);
    button3.whenPressed(new CargoIntake());

    button4  = new JoystickButton(operatorStick, 2);
    button4.whenPressed(new CargoStopMotors());

    button5  = new JoystickButton(operatorStick, 5);
    button5.whenPressed(new CargoDown());
 
    button6  = new JoystickButton(operatorStick, 6);
    button6.whenPressed(new HatchOut());

    button7  = new JoystickButton(operatorStick, 7);
    button7.whenPressed(new HatchIn());

    button8  = new JoystickButton(operatorStick, 8);
    button8.whenPressed(new HatchDown());

    button9 = new JoystickButton(operatorStick, 10);
    button9.whenPressed(new HatchAcquire());

    button1 = new JoystickButton(operatorStick, 1);

    button2 = new JoystickButton(operatorStick, 2);*/
   
    /**dbutton5  = new JoystickButton(driverStick, 5);
    dbutton5.whenPressed(new ClampClimber());
    dbutton6  = new JoystickButton(driverStick, 6);
    dbutton6.whenPressed(new Unclamp());


    button7  = new JoystickButton(operatorStick, 7);
    button7.whenPressed(new ForwardClimber());
    button7.whenReleased(new StopClimber());

    button9 = new JoystickButton(driverStick, 9);
    button9.whenPressed(new BackwardsClimber());
    button9.whenReleased(new StopClimber());

    dbutton8 = new JoystickButton(driverStick, 8);
    dbutton8.whenPressed(new StopClimber());*/

  }
	public Joystick getDriverJoystick(){
		return driverStick;
  }
  
	
	public Joystick getOperatorJoystick(){
		return operatorStick;
  }
}

