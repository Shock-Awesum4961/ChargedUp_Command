// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DrivetrainSystem extends SubsystemBase {

    CANSparkMax frontLeft = null;
    CANSparkMax rearLeft = null;
    CANSparkMax rearRight = null;
    CANSparkMax frontRight = null;

    MecanumDrive m_robotDrive = null;

    AHRS gyro = null;

  /** Creates a new Drivetrain. */
  public DrivetrainSystem() {
    frontLeft = new CANSparkMax(Constants.MotorCANIds.FRONT_LEFT, MotorType.kBrushless);
    rearLeft = new CANSparkMax(Constants.MotorCANIds.REAR_LEFT, MotorType.kBrushless);
    rearRight = new CANSparkMax(Constants.MotorCANIds.REAR_RIGHT, MotorType.kBrushless);
    frontRight = new CANSparkMax(Constants.MotorCANIds.FRONT_RIGHT, MotorType.kBrushless);

    m_robotDrive = new MecanumDrive(frontLeft, rearLeft, frontRight, rearRight);

    m_robotDrive.setMaxOutput(.85);

    m_robotDrive.setDeadband(0);

    gyro = new AHRS();

  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public CommandBase exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void drive(double translation, double strafe, double rotation, boolean isSlowMode) {
    if(isSlowMode){
      m_robotDrive.driveCartesian(translation/2,0,rotation/2, Rotation2d.fromDegrees(gyro.getAngle()));
    } else {
      m_robotDrive.driveCartesian(translation,0,rotation/2, Rotation2d.fromDegrees(gyro.getAngle()));
    }
  }  
}
