/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.commands.operateHatch;

import java.util.Map;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Spark;

/**
 * This is a subsystem that manipulates the hatch game piece.
 */
public class Hatch extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  static private final int forwardChannel = 2;
  static private final int reverseChannel = 6;
  static DoubleSolenoid hatchSolenoid = new DoubleSolenoid(forwardChannel, reverseChannel);
  static Spark hatchIntake = new Spark(2);
  boolean hatchIn = true;
  boolean hatchDown = false;
  boolean hatchUp = true;
  DigitalInput hatchDownLimit = new DigitalInput(1);
  NetworkTableEntry hatchDownEntry, hatchInEntry;

  public Hatch(){
    ShuffleboardLayout hatchMappingList = Shuffleboard.getTab("Hatch")
    .getLayout("Mapping", BuiltInLayouts.kList)
    .withSize(2,5)
    .withPosition(0,0)
    .withProperties(Map.of("Label position", "LEFT"));
    hatchMappingList.add("Intake Motor", hatchIntake.getChannel());
    hatchMappingList.add("Limit Switch", hatchDownLimit.getChannel());
    hatchMappingList.add("Solenoid Forward Chan", forwardChannel);
    hatchMappingList.add("Solenoid Reverse Chan", reverseChannel);
    Shuffleboard.getTab("Hatch").add("Hatch Motor",hatchIntake).withPosition(2,0).withWidget(BuiltInWidgets.kSpeedController);
    hatchInEntry = Shuffleboard.getTab("Hatch").add("Hatch in",hatchIn).withPosition(2,1).getEntry();
    hatchDownEntry = Shuffleboard.getTab("Hatch").add("Hatch Down",hatchDown).withPosition(3,1).getEntry();
    hatchInEntry = Shuffleboard.getTab("Driver View").add("Hatch in",hatchIn).withPosition(1,5).getEntry();
  }
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new operateHatch());
  }

  /**
   * Acquires the object: sets the solenoid (pneumatics) to run and sets motor to run
   * */
   public void hatchAcquire(){
     //hatchDown();
     hatchOut();
     hatchUp();
     hatchIn();
  }
 
  
  /**
   * Scores the object: stops the solenoid (pneumatics)
   */
   public void hatchScore(){
     hatchOut();
     hatchDown();
     hatchIn();
     //hatchUp();
  }
  
  /**
   * Sets the motor to move the hatch down
   */
  public void hatchDown(){
    // if (hatchDown == true){
    //   hatchIntake.set(0);
    // }
    // else if (Robot.pdp.getCurrent(4) > 4){
    //   hatchIntake.set(0);
    //   hatchDown = true;
    // }
    // else {
    //   hatchIntake.set(-.5);
    //   hatchDown = false;
    // }
    
  }

  /**
   * Sets the motor to move the hatch up
   */
  public void hatchUp(){
/**    if (hatchUp == true){
      hatchIntake.set(0);
    }
    else*/
    //SmartDashboard.putNumber("Hatch Amps", Robot.pdp.getCurrent(4));

    // if (Robot.pdp.getCurrent(4) > 4){
    //   hatchIntake.set(0);
    //   hatchUp = true;
    // }
    // else {
    //   hatchIntake.set(.5);
    //   hatchUp = false;
    // } 
  }

  /**
   * Run hatch intake motor to set speed
   */
  public void driveHatch(double speed){
    if(! hatchDownLimit.get() && speed > 0){
      speed = 0;
    }
    hatchIntake.set(speed);
  //  SmartDashboard.putBoolean("Hatch Limit", hatchDownLimit.get());
  }
  
  /**
   * Brings hatch out (solenoid runs forward)
   */
  public void hatchOut(){
    hatchSolenoid.set(DoubleSolenoid.Value.kForward);
    hatchIn = false;
  }
  
  /**
   * Brings hatch in (solenoid runs reverse)
   */
  public void hatchIn(){
    hatchSolenoid.set(DoubleSolenoid.Value.kReverse);
    hatchIn = true;
  }
 
  
  /**
   * Stops the hatch motor
   **/
  public void stopHatchIntake(){
    hatchIntake.stopMotor();
  }  

  public void writeToSmartDashboard(){
    hatchDownEntry.setBoolean(hatchDown);
    hatchInEntry.setBoolean(hatchIn);
  }
}
  
