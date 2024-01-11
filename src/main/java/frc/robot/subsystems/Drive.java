package frc.robot.subsystems;

import java.io.File;
import java.io.IOException;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveParser;

enum DriveStates {
    FIELD_ABSOLUTE,
    FIELD_RELATIVE,
    FORWARD
}

public class Drive {
    static final double DEADBAND = 0.05;
    SwerveParser swerveParser;
    SwerveDrive drive;
    DriveStates driveStates = DriveStates.FIELD_ABSOLUTE;
    Robot robot = null;
    final int WHEEL_DIAMETER = 4;
    final double DRIVE_GEAR_RATIO = 6.12;
    final double ANGLE_GEAR_RATIO = 21.4286;
    final double ENCODER_RESOLUTION = 42;

    public Drive(Robot robot) {
        this.robot = robot;

        double driveConversionFactor = SwerveMath.calculateMetersPerRotation(Units.inchesToMeters(WHEEL_DIAMETER), DRIVE_GEAR_RATIO, 1);
        double angleConversionFactor = SwerveMath.calculateDegreesPerSteeringRotation(ANGLE_GEAR_RATIO, 1);

        try {
            swerveParser = new SwerveParser(new File(Filesystem.getDeployDirectory(), "swerve/neo"));
            drive = swerveParser.createSwerveDrive(Units.feetToMeters(14.5), angleConversionFactor, driveConversionFactor);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public void periodic() {
        String state = "";
        double xMovement = MathUtil.applyDeadband(robot.controller.getLeftY(), DEADBAND);
        double yMovement = MathUtil.applyDeadband(robot.controller.getLeftX(), DEADBAND);
        double rotation = robot.controller.getRightX();

        if (driveStates == DriveStates.FIELD_ABSOLUTE) {
            state = "Field Absolute";
            drive.drive(
                    new Translation2d(xMovement, yMovement),
                    rotation,
                    false,
                    false);
            
            if (robot.controller.getBButtonPressed()) {
                driveStates = DriveStates.FIELD_RELATIVE;
                System.out.println("FIELDS relative changed");
            }

            if (robot.controller.getYButtonPressed()) {
                driveStates = DriveStates.FORWARD;
            }
        } else if (driveStates == DriveStates.FIELD_RELATIVE) {
            state = "Field Relative";
            if (robot.controller.getAButtonPressed()) {
                drive.zeroGyro();
            }

            drive.drive(
                    new Translation2d(xMovement, yMovement),
                    rotation,
                    true,
                    false);
            
            if (robot.controller.getBButtonPressed()) {
                driveStates = DriveStates.FIELD_ABSOLUTE;
            }
        } else {
            state = "Forward Test";
            drive.drive(
                new Translation2d(-1, 0),
                0,
                false,
                false);

            if (robot.controller.getYButtonPressed()) {
                driveStates = DriveStates.FIELD_ABSOLUTE;
            }
        }

        SmartDashboard.putString("Drive State", state);
    }

    public void addVisionMeasurement(Pose2d pose, double timestamp) {
        drive.addVisionMeasurement(pose, timestamp);
    }
}
