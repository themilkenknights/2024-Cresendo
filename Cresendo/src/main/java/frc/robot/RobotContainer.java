package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;

import java.util.List;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.autos.AutoOptions;
import frc.robot.commands.Align;
import frc.robot.commands.AprilTagCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Climb;
//import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Climb.Positions;
import frc.robot.subsystems.IntakeElevator;
import frc.robot.subsystems.Intakes;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
        // private Field2d field = new Field2d();
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
        private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
        private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

        /* Path follower */
        // private Command runAuto = drivetrain.getAutoPath("Tests");

        private final Telemetry logger = new Telemetry(MaxSpeed);

        // private void configureBindings() {
        // }

        /* Controllers */
        // private final Joystick driver = new Joystick(0);
        private final CommandXboxController op = new CommandXboxController(1);

        // /* Drive Controls */
        // private final int translationAxis = XboxController.Axis.kLeftY.value;
        // private final int strafeAxis = XboxController.Axis.kLeftX.value;
        // private final int rotationAxis = XboxController.Axis.kRightX.value;

        /* Driver Buttons */
        // private final JoystickButton zeroGyro = new JoystickButton(driver,
        // XboxController.Button.kY.value);
        // private final JoystickButton robotCentric = new JoystickButton(driver,
        // XboxController.Button.kLeftBumper.value);

        /* Subsystems */
        // private final Swerve s_Swerve = new Swerve();
        private final IntakeElevator elevator = new IntakeElevator();
        private final Intakes s_Intakes = new Intakes(elevator);
        private final Climb s_Climb = new Climb();

        // auto
        //private final SendableChooser<Command> autoChooser;

        private final SendableChooser<AutoOptions.StartingPostions> StartingPostionsChooser = new SendableChooser<>();
        private final SendableChooser<AutoOptions.Types> AutoTypeChooser = new SendableChooser<>();
        private final SendableChooser<AutoOptions.Notes> AutoNoteChooser = new SendableChooser<>();

        public AprilTagCommand getTagCommand() {
                return new AprilTagCommand(drivetrain);
        }

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {

                // field.setRobotPose(drivetrain)
                SmartDashboard.putData("intake", s_Intakes);
                SmartDashboard.putData("climb", s_Climb);
                // SmartDashboard.putData("swerve",s_Swerve);

                // limelight periodic

                /////////////////////////////////////////////////////
                // NAMEDCOMMANDS //
                /////////////////////////////////////////////////////

                NamedCommands.registerCommand("Apriltags", getTagCommand());
                NamedCommands.registerCommand("Outtake", s_Intakes.AutoAmpOuttake());
                NamedCommands.registerCommand("GoDown", s_Intakes.GoDown());
                NamedCommands.registerCommand("HPInktake", s_Intakes.AutoHPin());
                NamedCommands.registerCommand("GoUp", s_Intakes.goUp());
                NamedCommands.registerCommand("ground", s_Intakes.AutoGroundPickUP());
                drivetrain = TunerConstants.DriveTrain;

                CommandScheduler.getInstance()
                                .schedule(Commands.repeatingSequence(new AprilTagCommand(drivetrain).ignoringDisable(true), waitSeconds(5).ignoringDisable(true)));
                // Configure the button bindings
                configureButtonBindings();

                // Build an auto chooser. This will use Commands.none() as the default option.

                //autoChooser = AutoBuilder.buildAutoChooser();

                StartingPostionsChooser.addOption("TOP", AutoOptions.StartingPostions.TOP);
                StartingPostionsChooser.addOption("MID", AutoOptions.StartingPostions.MID);
                StartingPostionsChooser.addOption("BOTTOM", AutoOptions.StartingPostions.BOTTOM);
                StartingPostionsChooser.setDefaultOption("TOP", AutoOptions.StartingPostions.TOP);

                AutoNoteChooser.addOption("TOP", AutoOptions.Notes.TOP);
                AutoNoteChooser.addOption("MID", AutoOptions.Notes.MID);
                AutoNoteChooser.addOption("BOTTOM", AutoOptions.Notes.BOTTOM);

                AutoNoteChooser.setDefaultOption("TOP", AutoOptions.Notes.TOP);
                ;

                AutoTypeChooser.addOption("SINGLE", AutoOptions.Types.SINGLE);
                AutoTypeChooser.addOption("TAKE", AutoOptions.Types.TAKE);
                AutoTypeChooser.addOption("Double Take", AutoOptions.Types.DOUBLE_TAKE);
                AutoTypeChooser.addOption("Defend", AutoOptions.Types.DEFENCE);

                AutoTypeChooser.setDefaultOption("TAKE", AutoOptions.Types.TAKE);

                // Another option that allows you to specify the default auto by its name
                // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");

                // SmartDashboard.putData("Auto Chooser", autoChooser);

                Shuffleboard.getTab("Autos").add("Positons", StartingPostionsChooser).withSize(2, 1)
                                .withWidget(BuiltInWidgets.kSplitButtonChooser);
                Shuffleboard.getTab("Autos").add("Type", AutoTypeChooser).withSize(2, 1)
                                .withWidget(BuiltInWidgets.kSplitButtonChooser);
                Shuffleboard.getTab("Autos").add("Note", AutoNoteChooser).withSize(2, 1)
                                .withWidget(BuiltInWidgets.kSplitButtonChooser);
                //lightLime
               // new Blind(drivetrain.getRotation3d()::getAngle);

        }

        /**
         * Use this method to define your button->command mappings. Buttons can be
         * created by
         * instantiating a {@link GenericHID} or one of its subclasses ({@link
         * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
         * it to a {@link
         * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
         */
        private void configureButtonBindings() {
                drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
                                drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive
                                                                                                                   // forward
                                                                                                                   // with
                                                                                                                   // negative
                                                                                                                   // Y
                                                                                                                   // (forward)
                                                .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with
                                                                                                // negative X (left)
                                                .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive
                                                                                                            // counterclockwise
                                                                                                            // with
                                                                                                            // negative
                                                                                                            // X (left)
                                ).ignoringDisable(true));

                joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
                joystick.b().whileTrue(drivetrain
                                .applyRequest(
                                                () -> point.withModuleDirection(new Rotation2d(joystick.getLeftY(),
                                                                joystick.getLeftX()))));
                drivetrain.registerTelemetry(logger::telemeterize);

                /////////////////////////////////////////////////////
                // DRIVER CONTROL //
                /////////////////////////////////////////////////////

                joystick.y().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));
                joystick.pov(90).whileTrue(
                                drivetrain.applyRequest(() -> forwardStraight.withVelocityX(0.5).withVelocityY(0)));
                joystick.pov(270)
                                .whileTrue(drivetrain.applyRequest(
                                                () -> forwardStraight.withVelocityX(-0.5).withVelocityY(0)));
                joystick.pov(0)
                //                 .onTrue(new SequentialCommandGroup(
                //                     s_Climb.unlockClimb(),
                //                     s_Climb.goToClimberPosition(Positions.TOP)
                //                 ));
                                .onTrue(new SequentialCommandGroup(
                                    s_Climb.unlockClimb(),
                                    s_Climb.goToClimberPosition(Positions.TOP),
                                    s_Climb.lockClimb()
                                ));
                // joystick.pov(180).onTrue(new SequentialCommandGroup(s_Climb.goToClimberPosition(Positions.BOTTOM),
                //                 s_Climb.lockClimb(), new InstantCommand(() -> s_Climb.disable())));
                joystick.pov(180).onTrue(new SequentialCommandGroup(
                                s_Climb.goToClimberPosition(Positions.BOTTOM)));
                joystick.x().whileTrue(s_Intakes.setTopIntakeState(Intakes.state.OUT))
                                .onFalse(s_Intakes.setTopIntakeState(Intakes.state.OFF));
                joystick.b().whileTrue(s_Intakes.setTopIntakeState(Intakes.state.HP))
                                .onFalse(s_Intakes.setTopIntakeState(Intakes.state.OFF));
               /*  joystick.rightTrigger()
                                .whileTrue(new RepeatCommand(s_Intakes.setBottomIntakeState(Intakes.state.GROUND)))
                                .onFalse(s_Intakes.setTopIntakeState(Intakes.state.OFF));
                joystick.leftTrigger()
                                .whileTrue(new RepeatCommand(s_Intakes.setBottomIntakeState(Intakes.state.GROUNDOUT)))
                                .onFalse(s_Intakes.setTopIntakeState(Intakes.state.OFF));*/
                joystick.leftTrigger().whileTrue(Align.LeftHPAlign());
                joystick.rightTrigger().whileTrue(Align.RightHPAlign());
                joystick.a().whileTrue(Align.AMPAlign());

                joystick.rightBumper().whileTrue(s_Intakes.Flashorange());
                joystick.leftBumper().whileTrue(s_Intakes.FlashBlue());

                /////////////////////////////////////////////////////
                // OP CONTROL //
                /////////////////////////////////////////////////////
                // op
                op.y().onTrue(s_Intakes.goUp());
                op.a().onTrue(s_Intakes.GoDown());
                // op.leftTrigger().whileTrue(new SequentialCommandGroup(new
                // InstantCommand(()->s_Intakes.getCurrentCommand().cancel()),//s_Intakes.setTopIntakeState(Intakes.state.OUT))).onFalse(s_Intakes.setTopIntakeState(Intakes.state.OFF));

                // op.rightTrigger().whileTrue(s_Intakes.setTopIntakeState(Intakes.state.HP)).onFalse(s_Intakes.setTopIntakeState(Intakes.state.OFF));
                op.x()
                                .onTrue(s_Intakes.getElevator().gotoHeight(IntakeElevator.Positions.AMP));
                // .onTrue(s_Intakes.AutoHPin());

                op.b()
                                .onTrue(s_Intakes.AutoGroundPickUP());
                op.leftBumper().onTrue(s_Intakes.HPin());
                op.rightBumper().onTrue(s_Intakes.AmpOuttake());

                if (Robot.isSimulation()) {
                        op.button(5).onTrue(s_Intakes.goUp());
                        op.button(6).onTrue(s_Intakes.GoDown());
                }

                // op.rightBumper().onTrue(s_Intakes.goUp());
                // op.leftBumper() .onTrue(s_Intakes.GoDown());
                // op.rightBumper() .onTrue(s_Intakes.goUp());

                // .until(()->s_Climb.getController().getPositionError()<0.5)
                // op.leftTrigger().and(op.y()).onTrue(s_Intakes.goUp());
                // op.leftTrigger().and(op.x()).onTrue(s_Intakes.GoDown());
                // op.leftTrigger().and(op.x()).onTrue(s_Intakes.setTopIntakeState(Intakes.state.HP)).onFalse(s_Intakes.setTopIntakeState(Intakes.state.OFF));
                // op.leftTrigger().and(op.x()).onTrue(s_Intakes.setTopIntakeState(Intakes.state.OUT)).onFalse(s_Intakes.setTopIntakeState(Intakes.state.OFF));

                // op.pov(90).onTrue(s_Climb.goToClimberPosition(Positions.TOP));

                // op.povLeft().onTrue(s_Climb.AutoZero());

                // op.leftTrigger().whileTrue(s_Climb.manualDown(op::getLeftTriggerAxis));

        }

        public void reEnable() {
                s_Climb.resetMesurement();
                s_Climb.setSetpoint(0);
                s_Intakes.getElevator().onReEnable();
                // CommandScheduler.getInstance().cancelAll();
        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {
                // An ExampleCommand will run in autonomous

                switch (AutoTypeChooser.getSelected()) {
                        case SINGLE:
                                return AutoBuilder
                                                .buildAuto(StartingPostionsChooser.getSelected().toString() + "Single");
                        case TAKE:
                                return AutoBuilder.buildAuto(StartingPostionsChooser.getSelected().toString() + "_OUT");
                        case DOUBLE_TAKE:
                                return AutoBuilder.buildAuto(StartingPostionsChooser.getSelected().toString() + "_"
                                                + AutoNoteChooser.getSelected().toString() + "_Take");
                        case DEFENCE:
                                return AutoBuilder.buildAuto("DEFENCE");
                        case STRAIGHT:
                                return AutoBuilder.followPath(new PathPlannerPath((List<Translation2d>) new Translation2d(2.5, 0), new PathConstraints(3, 3, 500, 500), new GoalEndState(0, new Rotation2d())));
                        default:
                                return new InstantCommand();
                }
        }
}