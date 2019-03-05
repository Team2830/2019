/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.Robot;
import frc.robot.commands.DriveCommand;

/**
 * TODO: Need Documentation Here
 */
public class DriveTrain extends Subsystem {

  public static WPI_TalonSRX talonLeft;
  public static WPI_TalonSRX talonRight;
  static WPI_VictorSPX victorLeft;
  static WPI_VictorSPX victorRight;
  static AHRS ahrs;
  static DifferentialDrive m_drive;
  int collisionCount = 0;
  boolean collisionDetected = false;

  public static int MotionMagicLoop = 0;
  public static int TimesInMotionMagic = 0;
  final static double kCollisionThreshold_Delta6 = 2f;

  static int kPIDLoopIdx = 0;
  static int kSlotIdx = 0;
  static int kTimeoutMs = 30;
  static double kP = 0.2;
  static double kI = 0.0;
  static double kD = 0.0;
  static double peakOutput = 1.0;
  static int distance = 0;

  boolean m_LimelightHasValidTarget=  false;
  double m_LimelightDriveCommand = 0.0;
  double m_LimelightSteerCommand = 0.0;

  /**
   * TODO: Need Documentation Here (what happens in the constructor)
   */
  public DriveTrain() {
    talonLeft = new WPI_TalonSRX(15);
    talonRight = new WPI_TalonSRX(20);
    victorLeft = new WPI_VictorSPX(14);
    victorRight = new WPI_VictorSPX(21);
    ahrs = new AHRS(SPI.Port.kMXP);
  
    talonLeft.configFactoryDefault();
    talonRight.configFactoryDefault();
    talonRight.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
    talonLeft.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
    
    talonLeft.configContinuousCurrentLimit(20, 0);
    talonLeft.configPeakCurrentLimit(35, 0);
    talonLeft.configPeakCurrentDuration(100, 0);
    talonLeft.enableCurrentLimit(true);
    talonLeft.configOpenloopRamp(0.2, 0);

    talonRight.configContinuousCurrentLimit(20, 0);
    talonRight.configPeakCurrentLimit(35, 0);
    talonRight.configPeakCurrentDuration(100, 0);
    talonRight.enableCurrentLimit(true);
    talonRight.configOpenloopRamp(0.2, 0);

    talonRight.setInverted(false);
    talonLeft.setInverted(false);
    talonRight.setSensorPhase(false);
    talonLeft.setSensorPhase(true);
    victorLeft.follow(talonLeft);
    victorRight.follow(talonRight);
    victorLeft.setInverted(InvertType.FollowMaster);
    victorRight.setInverted(InvertType.FollowMaster);
  
    m_drive = new DifferentialDrive(talonLeft, talonRight);
  
    resetCounters();
  }
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public double joystickDeadband =0.04;

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
     setDefaultCommand(new DriveCommand());
  }
/**
 * TODO: Need Documentation Here
 * @param joystick
 */
  public void tankDrive(Joystick joystick){
    m_drive.tankDrive(-joystick.getRawAxis(5),-joystick.getRawAxis(1));
    //TODO: remove debugging SmartDashboard calls
    SmartDashboard.putNumber("left Tank",-joystick.getRawAxis(5));
    SmartDashboard.putNumber("right Tank",-joystick.getRawAxis(1));
  }

  /**
   * TODO: Need Documentation Here
   */
  public void arcadeDrive(Joystick joystick){
    double throttle = deadbanded((-1*joystick.getRawAxis(2))+joystick.getRawAxis(3), joystickDeadband);
    double steering = deadbanded(joystick.getRawAxis(0), joystickDeadband);
    //TODO: remove debugging SmartDashboard calls
    SmartDashboard.putNumber("Throttle", throttle);
    SmartDashboard.putNumber("Steering", steering);
    m_drive.arcadeDrive(throttle, steering);
  }

  /**
   * TODO: Need Documentation Here
   * 
   * @param joystick
   */
  public void curvatureDrive(Joystick joystick){
    double throttle = deadbanded((-1*joystick.getRawAxis(2))+joystick.getRawAxis(3), joystickDeadband);
    double steering = deadbanded(-joystick.getRawAxis(0), joystickDeadband);
    m_drive.curvatureDrive(throttle, steering, joystick.getRawButton(2));
  }

/*
  public void arcadeDrive(Joystick joystick){

    double[] values = drive(joystick);
    double direction = values[0], leftSpeed = values[1], rightSpeed = values[2];
    if(direction == 0 || direction == 3){
      talonLeft.set(rightSpeed);
      talonRight.set(leftSpeed);
    } else{
      talonRight.set(rightSpeed);
      talonLeft.set(leftSpeed);
    }
  }
*/

  /**
   * TODO: Need Documentation Here
   * 
   * @param joystick
   * @return double[] 3 values, the direction, the leftSpeed and the right speed
   */
  public double[] drive(Joystick joystick){
    double throttle = deadbanded((-1*joystick.getRawAxis(2))+joystick.getRawAxis(3), joystickDeadband);
    if (Math.abs(throttle) > 1){
      throttle = Math.copySign(1, throttle);
    }
    double steering = 0.6*deadbanded(-joystick.getRawAxis(0), joystickDeadband); 
    double maxInput = Math.copySign(Math.max(Math.abs(throttle), Math.abs(steering)), throttle);
    double speed, direction;
    if (throttle >= 0){
      if(steering >= 0){
        direction = 0;
        speed = throttle-steering;
      }else{
        direction = 1;
        speed = throttle+steering;
      }

    }else{
      if(steering >= 0){
        direction = 2;
        speed = throttle+steering;
      }else{
        direction = 3;
        speed = throttle-steering;
      }
    }
    
    double[] set = {direction,speed,maxInput};
    DetectCollision();
    if (collisionDetected == true){
      if (collisionCount < 5){
        speed = 0;
        maxInput = 0;
        collisionCount++;
      }
      else {
        collisionDetected = false;
        collisionCount = 0;
      }
    }
    return set;
  }

  /**
   *  Vision Drive used to turn based off vision target location
   * 
   * @param joystick
   */
  public void visionDrive(Joystick joystick){

    
    Update_Limelight_Tracking();
    double throttle = deadbanded((-1*joystick.getRawAxis(2))+joystick.getRawAxis(3), joystickDeadband);
    m_drive.curvatureDrive(throttle, m_LimelightSteerCommand, false);
    
  }

  public void Update_Limelight_Tracking(){
    final double STEER_K = 0.03;
    final double DRIVE_K = 0.26;
    final double DESIRED_TARGET_AREA = 13.0;
    final double MAX_DRIVE = 0.7;

    double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
    double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    double ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);

    if (tv < 1.0){
      m_LimelightHasValidTarget = false;
      m_LimelightDriveCommand = 0.0;
      m_LimelightSteerCommand = 0.0;
      return;
    }

    m_LimelightHasValidTarget = true;

    // Start with proportional steering
    double steer_cmd = tx * STEER_K;
    m_LimelightSteerCommand = steer_cmd;
    SmartDashboard.putNumber("steerValue",m_LimelightSteerCommand);

    // try to drive forward until the target area reaches our desired area
    double drive_cmd = (DESIRED_TARGET_AREA - ta) * DRIVE_K;

    // don't let the robot drive too fast into goal.
    if (drive_cmd > MAX_DRIVE){
      drive_cmd = MAX_DRIVE;
    }
    m_LimelightDriveCommand = drive_cmd;
  }

  /**
   * TODO: Need Documentation Here
   */
  public void resetCounters(){
    ahrs.zeroYaw();
    talonLeft.setSelectedSensorPosition(0, 0, 10);
    talonRight.setSelectedSensorPosition(0, 0, 10);
  }


/**
 * TODO: Need Documentation Here
 * 
 * @param inches
 */
  public void driveForward(double inches){
    resetCounters();
    talonRight.selectProfileSlot(kSlotIdx, kPIDLoopIdx);
    talonLeft.selectProfileSlot(kSlotIdx, kPIDLoopIdx);
    talonLeft.config_kP(kSlotIdx, kP, kTimeoutMs);
    talonRight.config_kP(kSlotIdx, kP, kTimeoutMs);
    talonLeft.config_kI(kSlotIdx, kI, kTimeoutMs);
    talonRight.config_kI(kSlotIdx, kI, kTimeoutMs);
    talonLeft.config_kD(kSlotIdx, kD, kTimeoutMs);
    talonRight.config_kD(kSlotIdx, kD, kTimeoutMs);

    distance = getPulsesFromInches(inches);
    talonRight.set(ControlMode.MotionMagic, distance, DemandType.AuxPID, ahrs.getYaw());
    talonLeft.follow(DriveTrain.talonRight, FollowerType.AuxOutput1);
  }

  /**
   * TODO: Need Documentation Here
   * 
   * @return
   */
  public boolean isDriveForwardComplete(){
    if(distance >= getSensorAverage()){
      return true;
    }
    return false;
  }

  /**
   * TODO: Need Documentation Here
   * 
   * @return
   */
  public static int getSensorAverage(){
    return (talonLeft.getSelectedSensorPosition()+talonRight.getSelectedSensorPosition())/2;
  }

  /**
   * 
   *  Function to set input based off deadband
   * 
   * @param input
   * @param deadband
   * @return
   */
  public double deadbanded(double input, double deadband){
    if(Math.abs(input)>Math.abs(deadband)){
      return input;
    }else{
      return 0;
    }
  }
  
  /**
   * TODO: Need Documentation Here
   * 
   */
  public void DetectCollision(){
    double lastWorldLinearAccelX = 0;
    double lastWorldLinearAccelY = 0;

    double currWorldLinearAccelX = ahrs.getWorldLinearAccelX();
    double currentJerkX = currWorldLinearAccelX - lastWorldLinearAccelX;
    lastWorldLinearAccelX = currWorldLinearAccelX;
    double currWorldLinearAccelY = ahrs.getWorldLinearAccelY();
    double currentJerkY = currWorldLinearAccelY - lastWorldLinearAccelY;
    lastWorldLinearAccelY = currWorldLinearAccelY;

    if ((Math.abs(currentJerkX) > kCollisionThreshold_Delta6) ||
      (Math.abs(currentJerkY) > kCollisionThreshold_Delta6)) {
        collisionDetected = true;
    }
  }
    
  public static int getPulsesFromInches(double inches){
		return (int)(240/Math.PI*inches);
	}

	public double getInchesFromPulses(double d){
		return d*Math.PI/240;
  }
  
  public double getAngle(){
    return ahrs.getAngle();
  }

  public double getDistance(){
		return(talonLeft.getSelectedSensorPosition(0) + talonRight.getSelectedSensorPosition(0))/2;
	}
  

  /**
   *  Writes to the dashboard values (can be used for debugging)
   */
  public void writeToSmartDashboard(){
    SmartDashboard.putNumber("Left Encoder Distance", talonLeft.getSelectedSensorPosition());
    SmartDashboard.putNumber("Right Encoder Distance", talonRight.getSelectedSensorPosition());
    SmartDashboard.putNumber("Right Encoder Velocity", talonRight.getSelectedSensorVelocity());
    SmartDashboard.putNumber("Left Encoder Velocity", talonLeft.getSelectedSensorVelocity());
    SmartDashboard.putNumber("Left Talon Current", talonLeft.getOutputCurrent());
    SmartDashboard.putNumber("Right Talon Current", talonRight.getOutputCurrent());
    SmartDashboard.putNumber("LeftTalon Current", Robot.pdp.getCurrent(15));
    SmartDashboard.putNumber("LeftVictor Current", Robot.pdp.getCurrent(14));
    SmartDashboard.putNumber("RightTalon Current", Robot.pdp.getCurrent(1));
    SmartDashboard.putNumber("RigtVictor Current", Robot.pdp.getCurrent(0));
    SmartDashboard.putData(ahrs);
    SmartDashboard.putData(talonLeft);
    SmartDashboard.putBoolean("Collision Detected", collisionDetected);
    // SmartDashboard.putNumber("Sensor Velocity", talonRight.getSelectedSensorPosition(kPIDLoopIdx));
    // SmartDashboard.putNumber("Sensor Position", talonRight.getActiveTrajectoryVelocity());
    SmartDashboard.putNumber("Motor Output Precent", talonRight.getMotorOutputPercent());
    SmartDashboard.putNumber("Closed Loop Error", talonRight.getClosedLoopError(kPIDLoopIdx));
  }

  public void configArcadeDrive() {
    /*
    talonRight.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
    talonLeft.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
    talonRight.setInverted(false);
    victorRight.setInverted(false);
    talonLeft.setInverted(true);
    victorLeft.setInverted(true);
    talonRight.setSensorPhase(true);
    talonLeft.setSensorPhase(true);
    victorLeft.follow(talonLeft);
    victorRight.follow(talonRight);
    */
  }
}
