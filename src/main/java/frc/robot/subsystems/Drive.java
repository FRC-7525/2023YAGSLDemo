package frc.robot.subsystems;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import swervelib.SwerveDrive;
import swervelib.SwerveModule;
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

    BufferedWriter csvBackLeft;
    BufferedWriter csvBackRight;
    BufferedWriter csvFrontLeft;
    BufferedWriter csvFrontRight;

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

        try {
            csvBackLeft = new BufferedWriter(new FileWriter("/tmp/backleft.csv"));
            csvBackRight = new BufferedWriter(new FileWriter("/tmp/backright.csv"));
            csvFrontLeft = new BufferedWriter(new FileWriter("/tmp/frontleft.csv"));
            csvFrontRight = new BufferedWriter(new FileWriter("/tmp/frontright.csv"));

            csvBackLeft.write("\"Input voltage\",\"Output voltage\",\"Velocity\"\n");
            csvBackRight.write("\"Input voltage\",\"Output voltage\",\"Velocity\"\n");
            csvFrontLeft.write("\"Input voltage\",\"Output voltage\",\"Velocity\"\n");
            csvFrontRight.write("\"Input voltage\",\"Output voltage\",\"Velocity\"\n");
        } catch (IOException e) {
            System.out.println("Error");
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

        for (SwerveModule m : drive.getModules()) {
            System.out.println("Module Name: "+m.configuration.name);
            //CANSparkMax steeringMotor = (CANSparkMax)m.configuration.angleMotor.getMotor();
            CANSparkMax driveMotor = (CANSparkMax)m.configuration.driveMotor.getMotor();
            
            RelativeEncoder relativeEncoder = driveMotor.getEncoder();

            try {
                if (m.configuration.name.equals("backleft")) {
                    csvBackLeft.write(driveMotor.getBusVoltage() + "," + driveMotor.getAppliedOutput() + "," + relativeEncoder.getVelocity() + "\n");
                } else if (m.configuration.name.equals("backright")) {
                    csvBackRight.write(driveMotor.getBusVoltage() + "," + driveMotor.getAppliedOutput() + "," + relativeEncoder.getVelocity() + "\n");
                } else if (m.configuration.name.equals("frontleft")) {
                    csvFrontLeft.write(driveMotor.getBusVoltage() + "," + driveMotor.getAppliedOutput() + "," + relativeEncoder.getVelocity() + "\n");
                } else if (m.configuration.name.equals("frontright")) {
                    csvFrontRight.write(driveMotor.getBusVoltage() + "," + driveMotor.getAppliedOutput() + "," + relativeEncoder.getVelocity() + "\n");
                }
            } catch (IOException e) {
                System.out.println("Error");
                e.printStackTrace();
            }
        }
    }

    public void addVisionMeasurement(Pose2d pose, double timestamp) {
        drive.addVisionMeasurement(pose, timestamp);
    }
}
