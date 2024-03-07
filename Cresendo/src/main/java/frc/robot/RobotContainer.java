package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AprilTagCommand;
import frc.robot.commands.ClimbCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.IntakeElevator;
import frc.robot.subsystems.Intakes;
import frc.robot.subsystems.LED;
//import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Climb.Positions;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  //private Field2d field = new Field2d();
  private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController joystick = new CommandXboxController(0); // My joystick
  public final CommandSwerveDrivetrain drivetrain; // My drivetrain

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  /* Path follower */
  //private Command runAuto = drivetrain.getAutoPath("Tests");

  private final Telemetry logger = new Telemetry(MaxSpeed);

  //private void configureBindings() {
   // }

    /* Controllers */
   //private final Joystick driver = new Joystick(0);
    private final CommandXboxController op = new CommandXboxController(1);

   // /* Drive Controls */
   // private final int translationAxis = XboxController.Axis.kLeftY.value;
   // private final int strafeAxis = XboxController.Axis.kLeftX.value;
   // private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
   // private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
   // private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);

    /* Subsystems */
    //private final Swerve s_Swerve = new Swerve();
    private final IntakeElevator elevator = new IntakeElevator();
    private final Intakes s_Intakes = new Intakes(elevator);
    private final Climb s_Climb = new Climb();

    private final LED led = new LED();

    //auto
    private final SendableChooser<Command> autoChooser;

    public AprilTagCommand getTagCommand(){
        return new AprilTagCommand(drivetrain);
    }
    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        
        //field.setRobotPose(drivetrain)
        SmartDashboard.putData("intake",s_Intakes);
        SmartDashboard.putData("climb",s_Climb);
        //SmartDashboard.putData("swerve",s_Swerve);
        CommandScheduler.getInstance().schedule(led.setAllianceColorLed());
        //limelight periodic
        
       
       

        //named commands
        NamedCommands.registerCommand("Apriltags",getTagCommand());
        NamedCommands.registerCommand("Outtake", new ParallelCommandGroup(s_Intakes.AmpOuttake()));
        NamedCommands.registerCommand("GoDown", s_Intakes.GoDown());
        NamedCommands.registerCommand("HPInktake",s_Intakes.HPin());
        NamedCommands.registerCommand("GoUp", s_Intakes.goUp());
        drivetrain = TunerConstants.DriveTrain;

        CommandScheduler.getInstance().schedule(Commands.repeatingSequence(new AprilTagCommand(drivetrain),waitSeconds(5)));
         // Configure the button bindings
         configureButtonBindings();

        // Build an auto chooser. This will use Commands.none() as the default option.
            
        autoChooser = AutoBuilder.buildAutoChooser();

    // Another option that allows you to specify the default auto by its name
    // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");

    SmartDashboard.putData("Auto Chooser", autoChooser);
    
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ).ignoringDisable(true));

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain
         .applyRequest(() -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

    // reset the field-centric heading on left bumper press
    joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    drivetrain.registerTelemetry(logger::telemeterize);
        
    joystick.pov(0).whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(0.5).withVelocityY(0)));
    joystick.pov(180).whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(-0.5).withVelocityY(0)));

    joystick.x().onTrue(s_Intakes.setTopIntakeState(Intakes.state.OUT)).onFalse(s_Intakes.setTopIntakeState(Intakes.state.OFF));
    joystick.b().onTrue(s_Intakes.setTopIntakeState(Intakes.state.HP)).onFalse(s_Intakes.setTopIntakeState(Intakes.state.OFF));
    joystick.rightTrigger().whileTrue(new RepeatCommand(s_Intakes.setBottomIntakeState(Intakes.state.GROUND))).onFalse(s_Intakes.setTopIntakeState(Intakes.state.OFF));
    joystick.leftTrigger().whileTrue(new RepeatCommand(s_Intakes.setBottomIntakeState(Intakes.state.GROUNDOUT))).onFalse(s_Intakes.setTopIntakeState(Intakes.state.OFF));
    /* Bindings for drivetrain characterization */
    /* These bindings require multiple buttons pushed to swap between quastatic and dynamic */
    /* Back/Start select dynamic/quasistatic, Y/X select forward/reverse direction 
    joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));*/

        //op
        op.y().onTrue(s_Intakes.AmpOuttake());
           
        op.x()
            .onTrue(s_Intakes.HPin());
        op.b()
        .onTrue(s_Intakes.GroundPickUP());
            
        if(Robot.isSimulation()){
            op.button(5)
            .onTrue(s_Intakes.goUp());
            op.button(6)
            .onTrue(s_Intakes.GoDown());
        }

        op.rightBumper()
            .onTrue(s_Intakes.goUp());
        op.leftBumper()
            .onTrue(s_Intakes.GoDown());

        

        op.leftTrigger().and(op.y()).onTrue(s_Intakes.goUp());
        op.leftTrigger().and(op.x()).onTrue(s_Intakes.GoDown());
        op.leftTrigger().and(op.x()).onTrue(s_Intakes.setTopIntakeState(Intakes.state.HP)).onFalse(s_Intakes.setTopIntakeState(Intakes.state.OFF));
        op.leftTrigger().and(op.x()).onTrue(s_Intakes.setTopIntakeState(Intakes.state.OUT)).onFalse(s_Intakes.setTopIntakeState(Intakes.state.OFF));

       //  op.pov(90).onTrue(s_Climb.goToClimberPosition(Positions.TOP));
        op.pov(0).onTrue(new SequentialCommandGroup(s_Climb.unlockClimb(),s_Climb.goToClimberPosition(Positions.TOP)));
        op.pov(180).onTrue(new SequentialCommandGroup(s_Climb.goToClimberPosition(Positions.BOTTOM),s_Climb.lockClimb()));
       // op.povLeft().onTrue(s_Climb.AutoZero());

      // op.leftTrigger().whileTrue(s_Climb.manualDown(op::getLeftTriggerAxis));
            
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return autoChooser.getSelected();
    }
}