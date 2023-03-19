package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class Drivetrain extends CommandBase {    
    private DrivetrainSystem drivetrainSystem;    
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier slowModeSup;

    public Drivetrain(DrivetrainSystem drivetrainSystem,DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier slowModeSup){
        this.drivetrainSystem = drivetrainSystem;
        addRequirements(drivetrainSystem);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.slowModeSup = slowModeSup;
    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);
        double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);
        

        /* Drive */
        drivetrainSystem.drive(
            translationVal,
            strafeVal,
            rotationVal,
            slowModeSup.getAsBoolean()
        );
    }
}