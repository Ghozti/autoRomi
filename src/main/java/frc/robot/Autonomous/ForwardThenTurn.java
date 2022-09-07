// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autonomous;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.RomiDrivetrain;
import frc.robot.subsystems.RomiGyro;

/** Add your docs here. */
public class ForwardThenTurn implements Autonomous{

    RomiDrivetrain drivetrain;
    RomiGyro gyro;

    public ForwardThenTurn(RomiDrivetrain drivetrain, RomiGyro gyro){
        this.drivetrain = drivetrain;
        this.gyro = gyro;
    }

    double kP = 0.055;
    double kI = 0.00055;
    double kD = 0.0005;
    double iLimit = 9;

    //gyro
    double gP = .004;
    double gI = .001;
    double gD = 0.02;
    double gilimit = 90;
    double gerror = 0;
    double gerrorSum = 0;

    double distanceTargetInches = 10;
    double turnDegrees = 90;
    double minTolreance = 0.8;
    double maxTolerance = 0.8;
    double error = 0;
    double errorSum = 0;
    double lastError = 0;
    double errorRate = 0;
    double speedOutput = 0;
    double lastTimestamp = 0;
    double currentTime = 0;
    double output;

    @Override
    public void init() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void update() {
        // TODO Auto-generated method stub

        if(drivetrain.getLeftDistanceInch() < 10 || drivetrain.getLeftDistanceInch() < 9.7){
            error = distanceTargetInches - ((drivetrain.getLeftDistanceInch() + drivetrain.getRightDistanceInch())/2);
            currentTime = Timer.getFPGATimestamp() - lastTimestamp;

            if(Math.abs(error) < iLimit){
                errorSum += error + currentTime;
            }

            errorRate = (error - lastError)/currentTime;
            speedOutput = (kP*error) + (kI * errorSum) + (kD * errorRate);
            drivetrain.arcadeDrive(speedOutput, 0);

            lastTimestamp = Timer.getFPGATimestamp();
            lastError = error;
        }   
        if(Math.round(drivetrain.getLeftDistanceInch()) >= 10){
            currentTime = Timer.getFPGATimestamp() - lastTimestamp;

            if(gyro.getAngleZ() < 90 - minTolreance){
                gerror = turnDegrees - gyro.getAngleZ();
            }
            if(gyro.getAngleZ() > 90 + maxTolerance){
                gerror = turnDegrees + gyro.getAngleZ();
            }

            if(Math.round(gerror) < 75-minTolreance || Math.round(gerror) > 75 + maxTolerance){
                gerrorSum += gerror + currentTime;
            }
        
            output = (gP * gerror) + (gI * gerrorSum);
            System.out.println(gerror);
            System.out.println(gerrorSum);
            System.out.println(output);
            System.out.println("***");

            drivetrain.arcadeDrive(0, output);

            lastTimestamp = Timer.getFPGATimestamp();
        }
    }

    @Override
    public void end() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void reset() {
        // TODO Auto-generated method stub
        errorSum = 0;
        lastError = 0;
        lastTimestamp = Timer.getFPGATimestamp();
    }

    @Override
    public boolean isDone() {
        // TODO Auto-generated method stub
        return false;
    }

}
