/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.ButtonMonitor;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.commands.ClimberBackDown;
import frc.robot.commands.ClimberBackUp;
import frc.robot.commands.ClimberEnabled;
import frc.robot.commands.ClimberFrontDown;
import frc.robot.commands.ClimberFrontUp;
import frc.robot.commands.FollowTrajectory;
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
 // public  Joystick joystick = new Joystick(0);
 // Joystick driverStick; 
 // Joystick operatorStick;
  Button driverButtonA,driverButtonB,driverButtonX,driverButtonY, backButton;
  XboxController driverController, operatorController;

	// Start the command when the button is released and let it run the command
	// until it is finished as determined by it's isFinished method.
	// button.whenReleased(new ExampleCommand());
  
  public OI(){
    driverController = new XboxController(0);
    operatorController = new XboxController(1);
    backButton = new JoystickButton(driverController, 8);
  // button1.whenPressed(new FollowTrajectory());//"/home/lvuser/deploy/righthab2torightfrontcargo_right.csv","/home/lvuser/deploy/righthab2torightfrontcargo_left.csv"));
    
  backButton.whenPressed(new FollowTrajectory("/home/lvuser/deploy/righthab2torightfrontcargo_left.csv","/home/lvuser/deploy/righthab2torightfrontcargo_right.csv"));
  //backButton.whenPressed(new FollowTrajectory());
  //"home/lvuser/deploy/straight7_right.csv", "home/lvuser/deploy/straight7_left.csv"));
 /** if (driverController.getBackButton()){
      new FollowTrajectory();
    }*/
  }

  public XboxController getDriverController(){
    return driverController;
  }
  
  public XboxController getOperatorController(){
    return operatorController;
  }

}

