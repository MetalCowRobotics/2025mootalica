package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.lib.util.COTSFalconSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {
    public static final double stickDeadband = 0.1;
    public static final XboxController operatorControls = new XboxController(1);


    public static final class Swerve {
        public static final int pigeonID = 14;
        public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

        public static final COTSFalconSwerveConstants chosenModule =  //TODO: This must be tuned to specific robot
            // COTSFalconSwerveConstants.SDSMK4i(COTSFalconSwerveConstants.driveGearRatios.SDSMK4i_L2);
            COTSFalconSwerveConstants.SDSMK4(COTSFalconSwerveConstants.driveGearRatios.SDSMK4_L3);

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(21.25); //TODO: This must be tuned to specific robot
        public static final double wheelBase = Units.inchesToMeters(21.25); //TODO: This must be tuned to specific robot
        public static final double wheelCircumference = chosenModule.wheelCircumference;


        /* Swerve Kinematics 
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
         public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        /* Motor Inverts */
        public static final boolean angleMotorInvert = chosenModule.angleMotorInvert;
        public static final boolean driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final boolean canCoderInvert = chosenModule.canCoderInvert;

        /* Swerve Current Limiting */
        public static final int angleContinuousCurrentLimit = 15;
        public static final int anglePeakCurrentLimit = 20;
        public static final double anglePeakCurrentDuration = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveContinuousCurrentLimit = 20;
        public static final int drivePeakCurrentLimit = 30;
        public static final double drivePeakCurrentDuration = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = 0.5;
        public static final double closedLoopRamp = 0.25;

        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;
        public static final double angleKF = chosenModule.angleKF;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.05; //TODO: This must be tuned to specific robot
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values 
         * Divide SYSID values by 12 to convert from volts to percent output for CTRE */
        public static final double driveKS = (0.32 / 12); //TODO: This must be tuned to specific robot
        public static final double driveKV = (1.51 / 12);
        public static final double driveKA = (0.27 / 12);

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 3.3; //TODO: This must be tuned to specific robot
        public static final double maxAutoSpeed = 1.5;
        /** Radians per Second */
        public static final double maxAngularVelocity = (3 * Math.PI) / 2; //TODO: This must be tuned to specific robot

        /* Neutral Modes */
        public static final NeutralMode angleNeutralMode = NeutralMode.Coast;
        public static final NeutralMode driveNeutralMode = NeutralMode.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 4;
            public static final int angleMotorID = 5;
            public static final int canCoderID = 6;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(271.8);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 10;
            public static final int angleMotorID = 11;
            public static final int canCoderID = 12;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(348.1);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 7;
            public static final int angleMotorID = 8;
            public static final int canCoderID = 9;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(149.);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 2;
            public static final int canCoderID = 3;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(96.8);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
    }

    public static final class AutoConstants { //TODO: The below constants are used in the example auto, and must be tuned to specific robot
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    
        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;
    
        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }

    public static final class IntakeConstants {
        public static final int INTAKE_MOTOR = 16;
        public static final double INTAKE_SPEED = 0.95;
        public static Boolean intakeRunning = false;
        public static final JoystickButton CONT_INTAKE_RUN = new JoystickButton(operatorControls, XboxController.Button.kRightBumper.value);
        public static final JoystickButton CONT_INTAKE_RUN_REV = new JoystickButton(operatorControls, XboxController.Button.kLeftBumper.value);

        public static final int CONE_SENSOR_DIO = 1;
        public static final int CUBE_SENSOR_DIO = 0;

    }

    public static final class ArmConstants {
        // CALIBRATION INSTRUCTIONS
        // 1. Set all pot offsets to zero
        // 2. Deploy the code to the robot
        // 3. Power off the robot
        // 4. Insert the bars and get the wrist to 90 degrees
        // 5. Turn on the robot, but do not remove the bars
        // 6. Look on Shuffleboard for Shoulder Angle, Elbow Angle, and Wrist Angle
        // 7. Put those angle readings in as the offsets for each joint (Elbow Angle becomes the Elbow Offset, Shoulder Angle becomes the Shoulder Offset, Wrist Angle becomes the Wrist Offset)
        public static final class Offsets {
            public static final int ELBOW_POT_ANALOG_ID = 0;
            public static final double ELBOW_POT_OFFSET = 1814;

            public static final int SHOULDER_POT_ANALOG_ID = 1;
            public static final double SHOULDER_POT_OFFSET = 1814;

            public static final int WRIST_POT_ANALOG_ID = 0;
            public static final double WRIST_POT_OFFSET = 1706+47;
        } 
        public static final class SubstationCone {

            public static final double SHOULDER_ANGLE = 6;
            public static final double ELBOW_ANGLE = 14.50;
            public static final double WRIST_ANGLE = 45;
        }
        public static final class SubstationCube {
            public static final double SHOULDER_ANGLE = 40;
            public static final double ELBOW_ANGLE = 50;
            public static final double WRIST_ANGLE = 90;
        }
        public static final class GroundCone {
            public static final double SHOULDER_ANGLE = 103;
            public static final double ELBOW_ANGLE = -50;
            public static final double WRIST_ANGLE = -23;

        }
        public static final class GroundCube {
            public static final double SHOULDER_ANGLE = 102;
            public static final double ELBOW_ANGLE = -49;
            public static final double WRIST_ANGLE = -3;
        }
        public static final class MidScoring {
            public static final double SHOULDER_ANGLE = 63;
            public static final double ELBOW_ANGLE = 36;

            public static final double WRIST_ANGLE = 70;

        }
        public static final class LowScoring {
            public static final double SHOULDER_ANGLE = 0;
            public static final double ELBOW_ANGLE = 0;
            public static final double WRIST_ANGLE = 72;
        }
        public static final class HighScoring {
            public static final double SHOULDER_ANGLE = 110;
            public static final double ELBOW_ANGLE = 80;
            public static final double WRIST_ANGLE = 46;
        }
    }

    public static final class CoordinateConstants {
        public static final class BlueField {
            public static final double PoofsStep1X = -5.867;
            public static final double PoofsStep1Y = -0.2;

            public static final double PoofsStep2X = -5.867;
            public static final double PoofsStep2Y = 0.406;
        
            public static final double PoofsStep3X = 0;
            public static final double PoofsStep3Y = 0.559;
        }
        public static final class RedField {
            public static final double PoofsStep1X = BlueField.PoofsStep1X;
            public static final double PoofsStep1Y = -BlueField.PoofsStep1Y;

            public static final double PoofsStep2X = BlueField.PoofsStep2X;
            public static final double PoofsStep2Y = -BlueField.PoofsStep2Y;

            public static final double PoofsStep3X = BlueField.PoofsStep3X;
            public static final double PoofsStep3Y = -BlueField.PoofsStep3Y;
        }
    }

}
