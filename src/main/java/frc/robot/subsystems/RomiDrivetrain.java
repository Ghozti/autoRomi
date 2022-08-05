// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.sql.Time;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RomiDrivetrain extends SubsystemBase {
  private static final double kCountsPerRevolution = 1440.0;
  private static final double kWheelDiameterInch = 2.75591; // 70 mm

  // The Romi has the left and right motors set to
  // PWM channels 0 and 1 respectively
  private final Spark m_leftMotor = new Spark(0);
  private final Spark m_rightMotor = new Spark(1);

  // The Romi has onboard encoders that are hardcoded
  // to use DIO pins 4/5 and 6/7 for the left and right
  private final Encoder m_leftEncoder = new Encoder(4, 5);
  private final Encoder m_rightEncoder = new Encoder(6, 7);

  // Set up the differential drive controller
  private final DifferentialDrive m_diffDrive = new DifferentialDrive(m_leftMotor, m_rightMotor);

  //PID for auto
  double kP = 0.2;
  double kI = 0.02;
  double kD = 0.02;
  double iLimit = .5;

  //auto stuff
  double distanceTargetInches = 40;
  double error = 0;
  double errorSum = 0;
  double lastError = 0;
  double errorRate = 0;
  double speedOutput = 0;
  double lastTimestamp = 0;
  double currentTime = 0;

  RomiGyro gyro = new RomiGyro();

  /** Creates a new RomiDrivetrain. */
  public RomiDrivetrain() {
    // Use inches as unit for encoder distances
    m_leftEncoder.setDistancePerPulse((Math.PI * kWheelDiameterInch) / kCountsPerRevolution);
    m_rightEncoder.setDistancePerPulse((Math.PI * kWheelDiameterInch) / kCountsPerRevolution);
    resetEncoders();

    // Invert right side since motor is flipped
    m_rightMotor.setInverted(true);
  }

  public void arcadeDrive(double xaxisSpeed, double zaxisRotate) {
    m_diffDrive.arcadeDrive(xaxisSpeed, zaxisRotate);
  }

  public void resetEncoders() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
    errorSum = 0;
    lastError = 0;
    lastTimestamp = Timer.getFPGATimestamp();
  }

  public double getLeftDistanceInch() {
    return m_leftEncoder.getDistance();
  }

  public double getRightDistanceInch() {
    return m_rightEncoder.getDistance();
  }

  //gyro pid stuff

  double gP = .01;
  double gI = .0001;
  double gD;
  double gilimit = 30;

  double output;
  double gerror;
  double maxTolerance = .8;
  double minTolerance = -.8;
  double gErrorSum;

  public void autonomousPeriodic(){
    /*
    error = distanceTargetInches - ((getLeftDistanceInch() + getRightDistanceInch())/2);
    currentTime = Timer.getFPGATimestamp() - lastTimestamp;

    if(Math.abs(error) < iLimit){
      errorSum += error + currentTime;
    }

    errorRate = (error - lastError)/currentTime;

    speedOutput = (kP * error) + (kI * errorSum) + (kD * errorRate);
    arcadeDrive(speedOutput, 0);

    //update time stamp & error
    lastTimestamp = Timer.getFPGATimestamp();
    lastError = error;
    */

    currentTime = Timer.getFPGATimestamp() - lastTimestamp;

    if(gyro.getAngleZ() > maxTolerance){
      gerror = maxTolerance - gyro.getAngleZ();
    }
    if(gyro.getAngleZ() < minTolerance){
      gerror = minTolerance - gyro.getAngleZ();
    }

    if(Math.abs(gerror) < gilimit && Math.round(gerror) != 0){
      gErrorSum += gerror + currentTime;
    }

    output = (gP* gerror) + (gI * gErrorSum);

    arcadeDrive(.35, output);

    lastTimestamp = Timer.getFPGATimestamp();
  }

  public RomiGyro getGyro(){
    return gyro;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(((gerror < gilimit) || (gerror > gilimit)) && Math.round(gerror - .3) != 0){
      SmartDashboard.putNumber("error", Math.abs(gerror));
      SmartDashboard.putBoolean("past limit", true);
    }else{
      SmartDashboard.putBoolean("past limit", false);
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation'
  }
}
