package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AlignToPoint;
import frc.robot.commands.AlignToSubstation;
import frc.robot.commands.DrivePath;
import frc.robot.commands.DriveToPoint;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.Swerve;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /*WaitCommand time = new WaitCommand(3.0);*/

    /* Subsystems */
    private Swerve m_swerve = new Swerve();

    /* Controllers */
    private final Joystick driver = new Joystick(0);
   private final Joystick operator = new Joystick(1);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton moveToCenter = new JoystickButton(driver, XboxController.Button.kA.value);
    private final JoystickButton moveToLeft = new JoystickButton(driver, XboxController.Button.kX.value);
    private final JoystickButton moveToRight = new JoystickButton(driver, XboxController.Button.kB.value);
    
    /* Operator Buttons */
    private final JoystickButton cubeSubstationIntakePosition = new JoystickButton(operator, XboxController.Button.kRightBumper.value);
    private final JoystickButton coneSubstationIntakePosition = new JoystickButton(operator, XboxController.Button.kLeftBumper.value);
    // private final JoystickButton cubeFloorIntakePosition = new JoystickButton(operator, XboxController.Axis.kLeftY);
    // private final JoystickButton coneFloorIntakePosition = new JoystickButton(operator, XboxController.Button.kX.value);
    private final JoystickButton lowScoringPosition = new JoystickButton(operator, XboxController.Button.kB.value);
    private final JoystickButton midScoringPosition = new JoystickButton(operator, XboxController.Button.kY.value);
    private final JoystickButton stow = new JoystickButton(operator, XboxController.Button.kA.value);

    Trigger crawl = new Trigger(() -> driver.getRawAxis(XboxController.Axis.kLeftTrigger.value) > 0.8);
    Trigger sprint = new Trigger(() -> driver.getRawAxis(XboxController.Axis.kRightTrigger.value) > 0.8);

    Trigger cubeFloorIntakePosition = new Trigger(() -> operator.getRawAxis(XboxController.Axis.kLeftY.value) > 0.8);
    Trigger coneFloorIntakePosition = new Trigger(() -> operator.getRawAxis(XboxController.Axis.kRightY.value) > 0.8);

    Trigger intakeForward = new Trigger(() -> operator.getRawAxis(XboxController.Axis.kLeftTrigger.value) > 0.7);
    Trigger intakeReverse = new Trigger(() -> operator.getRawButtonPressed(XboxController.Button.kBack.value));
    Trigger stopIntake = new Trigger(() -> operator.getRawButtonPressed(XboxController.Button.kX.value));
    Trigger eject = new Trigger(() -> operator.getRawAxis(XboxController.Axis.kRightTrigger.value) > 0.7);

    Trigger substationRight = new Trigger(() -> driver.getRawButtonPressed(XboxController.Button.kRightBumper.value));
    Trigger substationLeft = new Trigger(() -> driver.getRawButtonPressed(XboxController.Button.kLeftBumper.value));

    Trigger wristUp = new Trigger(() -> operator.getRawAxis(XboxController.Axis.kLeftX.value) > 0.7);
    Trigger wristDown = new Trigger(() -> operator.getRawAxis(XboxController.Axis.kRightX.value) > 0.7);

    Trigger shootHigh = new Trigger(() -> operator.getRawButtonPressed(XboxController.Button.kStart.value));
    // Trigger toggleLED = new Trigger(() -> driver.getRawButtonPressed(XboxController.Button.kY.value));
    // private final JoystickButton stopstow = new JoystickButton(operator, XboxController.Button.kB.value);

    Trigger balance = new Trigger(() -> driver.getRawButton(XboxController.Button.kStart.value));

    /* Subsystems */
    // private Swerve m_swerve = new Swerve();
    // private ShoulderSubsystem m_shoulderSubsystem;
    // private ElbowSubsystem m_elbowSubsystem;
    // private WristSubsystem m_wristSubsystem;
    // private IntakeSubsystem m_IntakeSubsystem;
    // private LEDSubsystem m_LEDSubsystem;

    Trigger drive = new Trigger(() -> 
        (Math.abs(driver.getRawAxis(XboxController.Axis.kLeftX.value)) > 0.1 || Math.abs(driver.getRawAxis(XboxController.Axis.kLeftY.value)) > 0.1) || 
        (Math.abs(driver.getRawAxis(XboxController.Axis.kRightX.value)) > 0.1 || Math.abs(driver.getRawAxis(XboxController.Axis.kRightY.value)) > 0.1)
    );
    // private final JoystickButton stopstow = new JoystickButton(operator, XboxController.Button.kB.value);
    
    /* Autos */
    private double armMovementTimeout = 3;
    private SendableChooser<Command> m_autoSelector;               
    
    private Command chargeStationScoreMobilityDock;
    private Command chargeStationScoreDock;
    private Command substationScoreMobilityDockBlue;
    private Command substationScoreMobilityDockRed;
    private Command substationScoreMobility;
    private Command cableRunScoreMobility;
    private Command armTest;
    private Command twoPieceAutoBlueMidCubeLowCube;
    private Command twoPieceAutoRedMidCubeLowCube;
    private Command twoPieceTwoCube;
    private Command twoPieceHighCubeHighCone;

    private Command twoPieceAutoBlueHighCubeLowCube;
    private Command twoPieceAutoRedHighCubeLowCube;


    private Command twoPieceAutoBlueMidConeLowCube;
    private Command twoPieceAutoBlueMidCubeMidCone;
    private Command twoPieceAutoBluePoofsTest;

    private Command threePiece;
    
    private Command alignToMiddle;
    private Command alignToLeft;
    private Command alignToRight;
    private Command alignToSubstationRight;
    private Command changeColor;
    private Command balanceCommand;
    private Command alignToSubstationLeft;

    private Command noAuto = new InstantCommand(() -> m_swerve.zeroGyro(180));

    /*change the color
    Trigger changeLightColor = new Trigger(() -> m_IntakeSubsystem.Identification());*/

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        
        m_swerve = new Swerve();
        m_autoSelector = new SendableChooser<Command>();

        

        threePiece = new SequentialCommandGroup(
            new DrivePath(m_swerve, "3 Piece Auto")
        );

        if (DriverStation.getAlliance().equals(Alliance.Blue)) {
            alignToMiddle = new AlignToPoint(m_swerve, -0.53, -0.15, 180);
            alignToLeft = new AlignToPoint(m_swerve, -0.53, 0.48, 180);
            alignToRight = new AlignToPoint(m_swerve, -0.53, -0.7, 180);
        } else {
            alignToMiddle = new AlignToPoint(m_swerve, -0.53, -0.15, 180);
            alignToLeft = new AlignToPoint(m_swerve, -0.53, -0.7, 180);
            alignToRight = new AlignToPoint(m_swerve, -0.53, 0.48, 180);
        }

        alignToSubstationLeft = new AlignToSubstation(m_swerve, 0.42, 0.0);
        alignToSubstationRight = new AlignToSubstation(m_swerve, 0.42, 0.0);
        
        m_autoSelector.addOption("Charge Station Score + Dock", chargeStationScoreDock);
        m_autoSelector.addOption("Charge Station Score + Mobility + Dock", chargeStationScoreMobilityDock);
        m_autoSelector.addOption("Blue Substation Score + Mobility + Dock", substationScoreMobilityDockBlue);
        m_autoSelector.addOption("Red Substation Score + Mobility + Dock", substationScoreMobilityDockRed);
        m_autoSelector.addOption("Substation Score + Mobility", substationScoreMobility);
        m_autoSelector.addOption("Cable Run Score Mobility", cableRunScoreMobility);
        m_autoSelector.addOption("Blue Two Piece Mid Cube Low Cube", twoPieceAutoBlueMidCubeLowCube);
        m_autoSelector.addOption("Red Two Piece Mid Cube Low Cube", twoPieceAutoRedMidCubeLowCube);
        m_autoSelector.addOption("Blue Two Piece High Cube Low Cube", twoPieceAutoBlueHighCubeLowCube);
        m_autoSelector.addOption("Red Two Piece High Cube Low Cube", twoPieceAutoRedHighCubeLowCube);

        m_autoSelector.setDefaultOption("2 piece High cube Mid cube ", twoPieceTwoCube);
        m_autoSelector.setDefaultOption("2 piece High cube High cone ", twoPieceHighCubeHighCone);

        m_autoSelector.addOption("Blue Two Piece Mid Cube Mid Cone IN TESTING", twoPieceAutoBlueMidCubeMidCone);
        m_autoSelector.addOption("Blue Two Piece Mid Cone Low Cube IN TESTING", twoPieceAutoBlueMidConeLowCube);
        m_autoSelector.addOption("Blue Two Piece Poofs TEST", twoPieceAutoBluePoofsTest);
        m_autoSelector.addOption("arm test", armTest);
        m_autoSelector.addOption("three piece", threePiece);
        // m_autoSelector.setDefaultOption("None", noAuto);
        SmartDashboard.putData(m_autoSelector);
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    public void configureButtonBindings() {
        

        /* Driver Buttons */
        m_swerve.setDefaultCommand(
            new TeleopSwerve(
                m_swerve, 
                () -> driver.getRawAxis(translationAxis), 
                () -> driver.getRawAxis(strafeAxis), 
                () -> -driver.getRawAxis(rotationAxis), 
                () -> robotCentric.getAsBoolean()
            )
        );
        
        zeroGyro.onTrue(new InstantCommand(() -> m_swerve.zeroGyro()));
        
        moveToCenter.onTrue(alignToMiddle);
        moveToCenter.onFalse(new InstantCommand(() -> CommandScheduler.getInstance().cancel(alignToMiddle)));

        moveToLeft.onTrue(alignToLeft);
        moveToLeft.onFalse(new InstantCommand(() -> CommandScheduler.getInstance().cancel(alignToLeft)));

        moveToRight.onTrue(alignToRight);
        moveToRight.onFalse(new InstantCommand(() -> CommandScheduler.getInstance().cancel(alignToRight)));

        balance.onTrue(balanceCommand);
        drive.onTrue(new InstantCommand(() -> CommandScheduler.getInstance().cancel(balanceCommand)));

        // changeColor = new ToggleColor(m_LEDSubsystem);
        //toggleLED.onTrue(changeColor);

        m_swerve.enableVision();

        crawl.onTrue(new InstantCommand(() -> m_swerve.setCrawl()));
        crawl.onFalse(new InstantCommand(() -> m_swerve.setBase()));

        sprint.onTrue(new InstantCommand(() -> m_swerve.setSprint()));
        sprint.onFalse(new InstantCommand(() -> m_swerve.setBase()));
    }


    public Command getAutonomousCommand() {
        CommandScheduler.getInstance().removeDefaultCommand(m_swerve);
        return m_autoSelector.getSelected();
    }
}
