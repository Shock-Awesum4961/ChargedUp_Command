// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.Conversions;
import frc.robot.Constants;

public class DrivetrainSystem extends SubsystemBase {

    CANSparkMax frontLeft = null;
    CANSparkMax rearLeft = null;
    CANSparkMax rearRight = null;
    CANSparkMax frontRight = null;

    RelativeEncoder m_frontLeftEncoder = null;
    RelativeEncoder m_frontRightEncoder = null;
    RelativeEncoder m_backLeftEncoder = null;
    RelativeEncoder m_backRightEncoder = null;

    MecanumDrive m_robotDrive = null;

    AHRS gyro = null;

    MecanumDriveOdometry m_odometry;
    Pose2d m_pose;

  /** Creates a new Drivetrain. */
  public DrivetrainSystem() {
    frontLeft = new CANSparkMax(Constants.MotorCANIds.FRONT_LEFT, MotorType.kBrushless);
    rearLeft = new CANSparkMax(Constants.MotorCANIds.REAR_LEFT, MotorType.kBrushless);
    rearRight = new CANSparkMax(Constants.MotorCANIds.REAR_RIGHT, MotorType.kBrushless);
    frontRight = new CANSparkMax(Constants.MotorCANIds.FRONT_RIGHT, MotorType.kBrushless);

    m_frontLeftEncoder = frontLeft.getEncoder();
    m_frontRightEncoder = frontRight.getEncoder();
    m_backLeftEncoder = rearLeft.getEncoder();
    m_backRightEncoder = rearRight.getEncoder();

    m_frontLeftEncoder.setPosition(0.0);
    m_frontRightEncoder.setPosition(0.0);
    m_backLeftEncoder.setPosition(0.0);
    m_backRightEncoder.setPosition(0.0);

    m_robotDrive = new MecanumDrive(frontLeft, rearLeft, frontRight, rearRight);

    m_robotDrive.setMaxOutput(.85);

    m_robotDrive.setDeadband(0);

    gyro = new AHRS();

    Translation2d m_frontLeftLocation = new Translation2d(0.381, 0.381); // TODO update based on current robot
    Translation2d m_frontRightLocation = new Translation2d(0.381, -0.381);
    Translation2d m_backLeftLocation = new Translation2d(-0.381, 0.381);
    Translation2d m_backRightLocation = new Translation2d(-0.381, -0.381);

    // Creating my kinematics object using the wheel locations.
    MecanumDriveKinematics m_kinematics = new MecanumDriveKinematics(
      m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation
    );

    // Creating my odometry object from the kinematics object and the initial wheel positions.
    // Here, our starting pose is 5 meters along the long end of the field and in the
    // center of the field along the short end, facing the opposing alliance wall.
    m_odometry = new MecanumDriveOdometry(
      m_kinematics,
      gyro.getRotation2d(),
      new MecanumDriveWheelPositions(
        Conversions.neoToMeters(m_frontLeftEncoder.getPosition(), Constants.wheelCircumference, Constants.driveGearRatio), 
        Conversions.neoToMeters(m_frontRightEncoder.getPosition(), Constants.wheelCircumference, Constants.driveGearRatio), 
        Conversions.neoToMeters(m_backLeftEncoder.getPosition(), Constants.wheelCircumference, Constants.driveGearRatio), 
        Conversions.neoToMeters(m_backRightEncoder.getPosition(), Constants.wheelCircumference, Constants.driveGearRatio) 
      ),
      new Pose2d()
    );


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
    var wheelPositions = new MecanumDriveWheelPositions(
      Conversions.neoToMeters(m_frontLeftEncoder.getPosition(), Constants.wheelCircumference, Constants.driveGearRatio), 
      Conversions.neoToMeters(m_frontRightEncoder.getPosition(), Constants.wheelCircumference, Constants.driveGearRatio), 
      Conversions.neoToMeters(m_backLeftEncoder.getPosition(), Constants.wheelCircumference, Constants.driveGearRatio), 
      Conversions.neoToMeters(m_backRightEncoder.getPosition(), Constants.wheelCircumference, Constants.driveGearRatio) 
    );
  
    // Get the rotation of the robot from the gyro.
    var gyroAngle = gyro.getRotation2d();
  
    // Update the pose
    m_pose = m_odometry.update(gyroAngle, wheelPositions);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void drive(double translation, double strafe, double rotation, boolean isSlowMode) {
    if(isSlowMode){
      m_robotDrive.driveCartesian(translation/2,0,rotation/2, gyro.getRotation2d());
    } else {
      m_robotDrive.driveCartesian(translation,0,rotation/2, gyro.getRotation2d());
    }
  }  
}
