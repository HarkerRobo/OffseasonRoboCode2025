// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.Function;
import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.EE.IntakeAlgae;
import frc.robot.commands.EE.IntakeCoralActive;
import frc.robot.commands.EE.EEManual;
import frc.robot.commands.EE.Score;
import frc.robot.commands.EE.ScoreAlgae;
import frc.robot.commands.EE.TuskMoveToPosition;
import frc.robot.commands.EE.TuskMoveToPositionSimple;
import frc.robot.commands.EE.ZeroTusk;
import frc.robot.commands.climb.ClimbManual;
import frc.robot.commands.drivetrain.DriveToPoseCommand;
import frc.robot.commands.elevator.ElevatorManual;
import frc.robot.commands.elevator.MoveToPosition;
import frc.robot.commands.elevator.ZeroElevator;
import frc.robot.subsystems.swerve.Drivetrain;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.swerve.Modules;
import harkerrobolib.joysticks.HSXboxController;

public class RobotContainer {
    public enum AlignDirection
    {
        Left,
        Right,
        Algae,
        LeftBarge,
        MidBarge,
        RightBarge
    }
    private static RobotContainer instance = RobotContainer.getInstance();

    private double MaxSpeed = Constants.Swerve.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second // max angular velocity

    private double MaxSpeedSlow = MaxSpeed * 0.6;
    private double MaxAngularRateSlow = MaxAngularRate * 1.0;

    private AlignDirection direction = AlignDirection.Left;

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.02).withRotationalDeadband(MaxAngularRate * 0.01) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final SlewRateLimiter xLimiter = new SlewRateLimiter(MaxSpeed*1.5);
    private final SlewRateLimiter yLimiter = new SlewRateLimiter(MaxSpeed*1.5);
    // private final SlewRateLimiter rotLimiter = new SlewRateLimiter(MaxAngularRate*2);
    private final SlewRateLimiter xLimiterExtended = new SlewRateLimiter(MaxSpeedSlow*1.5);
    private final SlewRateLimiter yLimiterExtended = new SlewRateLimiter(MaxSpeedSlow*1.5);
    // private final SlewRateLimiter rotLimiterExtended = new SlewRateLimiter(MaxAngularRateSlow*1.5);

    private final Telemetry logger = new Telemetry();

    private final HSXboxController driver = new HSXboxController(0);
    private final HSXboxController operator = new HSXboxController(1);

    public final Drivetrain drivetrain = Modules.createDrivetrain();
    private final Elevator elevator = Elevator.getInstance();
    private final EndEffector endEffector = EndEffector.getInstance();
    private final Climb climb = Climb.getInstance();

    private final SendableChooser<Command> autoChooser;
    private int selectedLevel;

    Function<RobotContainer.AlignDirection, Command> setDirectionFactory = ((AlignDirection direction) ->
        new Command() {
            public void execute () {System.out.println(direction); RobotContainer.getInstance().setAlignDirection(direction);}
            public boolean isFinished () {return true;}});

    Command alignLeft = setDirectionFactory.apply(AlignDirection.Left);
    Command alignAlgae = setDirectionFactory.apply(AlignDirection.Algae);
    Command alignRight = setDirectionFactory.apply(AlignDirection.Right);

    public RobotContainer() {
        NamedCommands.registerCommand("ZeroElevator", new ZeroElevator().asProxy());
        NamedCommands.registerCommand("ZeroTusk", new ZeroTusk().asProxy());
        NamedCommands.registerCommand("ElevatorL2",
                new MoveToPosition(Constants.Elevator.CORAL_HEIGHTS[1]).asProxy());
        NamedCommands.registerCommand("ElevatorL3",
                new MoveToPosition(Constants.Elevator.CORAL_HEIGHTS[2]).asProxy());
        NamedCommands.registerCommand("ElevatorL4",
                new MoveToPosition(Constants.Elevator.CORAL_HEIGHTS[3]).asProxy());
        NamedCommands.registerCommand("Score", new Score().asProxy());
        NamedCommands.registerCommand("ZeroElevatorFast", new MoveToPosition(0.2).andThen(new MoveToPosition(0.03)).asProxy());
        NamedCommands.registerCommand("IntakeCoralActive", new IntakeCoralActive().asProxy());

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auton Chooser", autoChooser);

        SignalLogger.start();

        configureBindings();
    }

    private void configureBindings() {
        elevator.setDefaultCommand(new ElevatorManual());

        endEffector.setDefaultCommand(new EEManual());

        climb.setDefaultCommand(new ClimbManual());

        drivetrain.registerTelemetry(logger::telemeterizeDrive);
        logger.telemeterize(elevator, endEffector);
        updateTelemetry();

                // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() -> {
                    
                    boolean slow = true;
                    //boolean slow = driver.getLeftTriggerAxis() > 0.3;
                    double velocityX = -driver.getLeftY() * (slow ? MaxSpeedSlow : MaxSpeed);
                    double velocityY = -driver.getLeftX() * (slow ? MaxSpeedSlow : MaxSpeed);
                    double rotRate = -driver.getRightX() * (slow ? MaxAngularRateSlow : MaxAngularRate);

                    return drive.withVelocityX(elevator.isExtended() ? xLimiterExtended.calculate(velocityX) : xLimiter.calculate(velocityX)) // Drive forward with negative Y (forward)
                                .withVelocityY(elevator.isExtended() ? yLimiterExtended.calculate(velocityY) : yLimiter.calculate(velocityY)) // Drive left with negative X (left)
                                .withRotationalRate(rotRate);// Drive counterclockwise with negative X (left)
                }
        ));

        // operator.getLeftDPad().onTrue(alignLeft); 
        driver.button(7).onTrue(alignLeft);
        // driver left/right bottom button
        
        // operator.getUpDPad().onTrue(alignAlgae);
  
        // operator.getRightDPad().onTrue(alignRight);
        driver.button(8).onTrue(alignRight);

        configureDriverBindings();

    }

    public void configureDriverBindings ()
    {
        driver.leftTrigger().and(()->EndEffector.getInstance().algaeIn()).onTrue(
            new MoveToPosition(Constants.Elevator.ALGAE_HEIGHTS[3])
            .andThen(new TuskMoveToPosition(Constants.EndEffector.REEF_TUSK_POSITION))
            .andThen(new ScoreAlgae())
            .andThen(new TuskMoveToPosition(Constants.EndEffector.ALGAE_HOLD_POSITION))
            .andThen(new ZeroElevator()));

        driver.rightTrigger().and(()->EndEffector.getInstance().algaeIn()).onTrue(
            new MoveToPosition(Constants.Elevator.ALGAE_HEIGHTS[0])
            .andThen(new TuskMoveToPosition(Constants.EndEffector.PROCESSOR_TUSK_POSITION))
            .andThen(new ScoreAlgae())
            .andThen(new TuskMoveToPosition(Constants.EndEffector.ALGAE_HOLD_POSITION))
            .andThen(new ZeroElevator()));


        Function<Integer /*level (1=low, 2=high)*/, Command> intakeAlgae = (Integer level)->
            drivetrain.run(()->setAlignDirection(AlignDirection.Algae))
            .andThen(new MoveToPosition(Constants.Elevator.ALGAE_HEIGHTS[level]))
            .andThen(new TuskMoveToPosition(Constants.EndEffector.REEF_TUSK_POSITION))
            .andThen(new IntakeAlgae())
            .andThen(new TuskMoveToPosition(Constants.EndEffector.ALGAE_HOLD_POSITION))
            .andThen(new ZeroElevator());

        driver.leftTrigger().and(()->!EndEffector.getInstance().algaeIn()).onTrue(
            intakeAlgae.apply(1));
        
        driver.rightTrigger().and(()->!EndEffector.getInstance().algaeIn()).onTrue(
            intakeAlgae.apply(2));

        
        Function<AlignDirection, Command> fullScoreAutomation = (AlignDirection alignDirection) ->
            drivetrain.run(()->setAlignDirection(alignDirection))
            .andThen(new MoveToPosition(Constants.Elevator.CORAL_HEIGHTS[selectedLevel]))
            .andThen(new DriveToPoseCommand(drivetrain))
            .andThen(new Score())
            .andThen(new ZeroElevator());

    
        driver.leftBumper().onTrue(fullScoreAutomation.apply(AlignDirection.Left));

        driver.rightBumper().onTrue(fullScoreAutomation.apply(AlignDirection.Right));

        driver.b().onTrue(endEffector.runOnce(() -> setSelectedLevel(1)));

        driver.x().onTrue(endEffector.runOnce(() -> setSelectedLevel(3)));

        driver.a().onTrue(endEffector.runOnce(() -> setSelectedLevel(2))); 

        driver.y().onTrue(endEffector.runOnce(() -> setSelectedLevel(4))); 



    }

    public void setSelectedLevel (int level)
    {
        selectedLevel = level;
    }


    public void updateTelemetry() {
        logger.telemeterize(elevator, endEffector);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public HSXboxController getOperator() {
        return operator;
    }

    public HSXboxController getDriver() {
        return driver;
    }

    public AlignDirection getAlignDirection ()
    {
        return direction;
    }

    public void setAlignDirection (AlignDirection direction)
    {
        this.direction = direction;
    }

    public static RobotContainer getInstance() {
        if (instance == null)
            instance = new RobotContainer();
        return instance;
    }
}