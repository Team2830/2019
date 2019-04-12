/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;

import java.util.Map;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Spark;
import frc.robot.commands.operateCargo;


/**
 * This is a subsystem that manipulates the cargo game piece.
 */
public class Cargo extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  static private final int forwardChannel = 0;
  static private final int reverseChannel = 1;
  static DoubleSolenoid cargoSolenoid = new DoubleSolenoid(forwardChannel, reverseChannel);
  static Spark cargoIntakeFront = new Spark(0);
  static Spark cargoIntakeBack = new Spark(1);
  boolean cargoUp = true;
//  NetworkTableEntry leftEncoderEntry, leftVelocityEntry, rightEncoderEntry, rightVelocityEntry;
  public Cargo(){
     ShuffleboardLayout cargoMappingList = Shuffleboard.getTab("Cargo")
    .getLayout("Mapping", BuiltInLayouts.kList)
    .withSize(2,5)
    .withPosition(0,0)
    .withProperties(Map.of("Label position", "LEFT"));
    cargoMappingList.add("Front Motor", cargoIntakeFront.getChannel());
    cargoMappingList.add("Back Motor", cargoIntakeBack.getChannel());
    cargoMappingList.add("Solenoid Forward Chan", forwardChannel);
    cargoMappingList.add("Solenoid Reverse Chan", reverseChannel);
  }
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    //public void setDefaultCommand(new CargoIntake());
    setDefaultCommand(new operateCargo());
    cargoIntakeBack.setInverted(true);
    
  }

  /**
   * Sets the motors to intake the object into cargo
   */
  public void cargoIntakeIn(){
    cargoIntakeFront.set(-1);
    cargoIntakeBack.set(-1);
  }
    
  /**
   * Sets the motors to outtake the object out of cargo
   */ 
  public void cargoIntakeOut(){
    cargoIntakeFront.set(1);
    cargoIntakeBack.set(1);
  }
  
  /**
   * Stops the intake motors
   */
  public void stopCargoIntake(){
    cargoIntakeFront.stopMotor();
    cargoIntakeBack.stopMotor();
  }

  
  /**
   * Sets the solenoid (pneumatics) to run, making the cargo go up
   */   
  public void cargoUp(){
    cargoSolenoid.set(DoubleSolenoid.Value.kForward);
    cargoUp = true;
  }
  
  /**
   * Sets the solenoid (pneumatics) to stop, making the cargo go down
   */
  public void cargoDown(){
    //TODO: Test if the Hatch is in or not. if not, don't put cargo down
    cargoSolenoid.set(DoubleSolenoid.Value.kReverse);
    cargoUp = false;
  }

  public boolean isCargoUp(){
    return cargoUp;
  }
}
  
