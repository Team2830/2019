/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/**
 * TODO: Need Documentation Here
 */
public class Vision extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  boolean hasTargets = false;
  double horizontalOffset = 0;
  double verticalOffset = 0;

  public Vision(){
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("stream").setNumber(0);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(0);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0);

  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  /**
   * TODO: Need Documentation Here
   */
  public double getVerticalOffset(){
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
  }

  /**
   * TODO: Need Documentation Here
   * @return
   */
  public double getHorizontalOffset(){
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
  }

  /**
   * TODO: Need Documentation Here
   */
  public boolean hasTargets(){
    if(NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0) == 1)
      return true;
    return false;
  }
public double get3DYaw(){
  return NetworkTableInstance.getDefault().getTable("limelight").getEntry("camtran").getDoubleArray(new double[]{})[4];
}

public double get3DX(){
  return NetworkTableInstance.getDefault().getTable("limelight").getEntry("camtran").getDoubleArray(new double[]{})[0];
}

public double get3DY(){
  return NetworkTableInstance.getDefault().getTable("limelight").getEntry("camtran").getDoubleArray(new double[]{})[1];
}
  /**
   * TODO: Need Documentation Here
   */
  public double getDistance(){
    if(hasTargets()){
      double a2 = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
      double a1= 28.4;
      double h1= 13.9;
      double h2 = 26;
      return (h2-h1)/(Math.tan((a1+a2)*Math.PI/180));
    }
    return 0;
  }

  public void changeStream(){
    if (    NetworkTableInstance.getDefault().getTable("limelight").getEntry("stream").getDouble(0)>1){
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(0);
    } else {
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(
         NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").getDouble(0)+1);
    }
  
  }
  public void writeToSmartDashboard(){
    SmartDashboard.putBoolean("LimelightHasTargets", hasTargets());
    SmartDashboard.putNumber("LimelightVerticalOffset", getVerticalOffset());
    SmartDashboard.putNumber("LimelightHorizontalOffset", getHorizontalOffset());
    SmartDashboard.putNumber("LimelightGetDistance", getDistance());
    try {
      SmartDashboard.putNumber("Yaw", NetworkTableInstance.getDefault().getTable("limelight").getEntry("camtran").getDoubleArray(new double[]{})[4]);
      SmartDashboard.putNumber("CamtranX", NetworkTableInstance.getDefault().getTable("limelight").getEntry("camtran").getDoubleArray(new double[]{})[0]);
      SmartDashboard.putNumber("CamtranY", NetworkTableInstance.getDefault().getTable("limelight").getEntry("camtran").getDoubleArray(new double[]{})[1]);
    } catch (Exception e) {
      System.out.println(e);
    }

  }

  public void setPipeline(int pipeline){
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(pipeline);
  }
}