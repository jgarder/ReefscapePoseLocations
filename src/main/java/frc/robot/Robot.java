// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructEntry;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.AlphaBots.AprilTag;
import frc.robot.AlphaBots.NT;
import frc.robot.generated.TunerConstants;
import frc.robot.AlphaBots.AprilTag.TagType;
import frc.robot.subsystems.AprilTagManager;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;

/** This is a sample program to demonstrate the use of elevator simulation. */
public class Robot extends TimedRobot {
  private final CommandXboxController m_joystick = new CommandXboxController(Constants.kJoystickPort);
  private final Elevator m_elevator = new Elevator();
  //private final advantageKitBootstrap akit = new advantageKitBootstrap(this);
  public static final  CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
  private final AprilTagManager ATMan = new AprilTagManager(drivetrain); 
  private int chosenAprilTagID = 0;

  StructPublisher<Pose2d> publisher = NetworkTableInstance.getDefault()
  .getStructTopic("MyPose", Pose2d.struct).publish();
  StructEntry<Pose2d> NT_myloc = NT.getStructEntry_Pose2D("Poses","TagLoc",new Pose2d());
  StructEntry<Pose2d> NT_myStraightloc = NT.getStructEntry_Pose2D("Poses","StraightLoc",new Pose2d());//straight out from the april tag. a centered pick for de-algae/processor/pickup 
  StructEntry<Pose2d> NT_Leftloc = NT.getStructEntry_Pose2D("Poses","leftLoc",new Pose2d());//LEFT FROM Robot TOWARDS tag view!
  StructEntry<Pose2d> NT_rightloc = NT.getStructEntry_Pose2D("Poses","rightLoc",new Pose2d());//Right Given View Robot TOWARDS tag!
  StructEntry<Pose2d> NT_ClosestTag = NT.getStructEntry_Pose2D("Poses","ClosestTag",new Pose2d());//shows closest tag to robotchassis
  //StructEntry<Pose2d> NT_Simloc = NT.getStructEntry_Pose2D("Poses","ChassisLoc",new Pose2d());
  StructEntry<Pose2d> NT_ClosestSource = NT.getStructEntry_Pose2D("Poses","ClosestSource",new Pose2d());
  StructEntry<Pose2d> NT_ClosestReef = NT.getStructEntry_Pose2D("Poses","ClosestReef",new Pose2d());
  StructEntry<Pose2d> NT_ClosestProcessor = NT.getStructEntry_Pose2D("Poses","ClosestProcessor",new Pose2d());
  StructEntry<Pose2d> NT_ClosestBarge = NT.getStructEntry_Pose2D("Poses","ClosestBarge",new Pose2d());
  DoubleEntry NT_tagID = NT.getDoubleEntry("", "TagID", chosenAprilTagID);
 
  
  public Robot() {
    DriverStation.silenceJoystickConnectionWarning(true);
    //akit.startAdvantageKitLogger();//before robot container even boots we log.
    //m_joystick.a().onTrue(new InstantCommand(()->{publisher.set(AprilTagLocations.Blue.Processor.Pose);}));
    //m_joystick.b().onTrue(new InstantCommand(()->{publisher.set(AprilTagLocations.Blue.blueBarge.Pose);}));
    m_joystick.povUp().onTrue(new InstantCommand(()->{chosenAprilTagID++;}));
    m_joystick.povDown().onTrue(new InstantCommand(()->{chosenAprilTagID--;}));
  }

  @Override
  public void robotPeriodic() {
    // Update the telemetry, including mechanism visualization, regardless of mode.
    NT_tagID.set(chosenAprilTagID);
    m_elevator.updateTelemetry();
    CommandScheduler.getInstance().run();
  }
  public static Pose2d SimRobotChassisLoc = new Pose2d();
  double simTranlatespeed = .3;
  double simRotateSpeed = 5;
  @Override
  public void simulationPeriodic() {
    // Update the simulation model.
    SimRobotChassisLoc = new Pose2d(SimRobotChassisLoc.getX()+(simTranlatespeed*m_joystick.getLeftX()),
                                    SimRobotChassisLoc.getY()+(simTranlatespeed*-m_joystick.getLeftY()),
                                    SimRobotChassisLoc.getRotation().plus(Rotation2d.fromDegrees(simRotateSpeed*-m_joystick.getRightX())));
    //NT_Simloc.set(SimRobotChassisLoc);//handled inside apriltagmanager
    m_elevator.simulationPeriodic();
  }
  double ReefWidthCenteronCenter = Units.inchesToMeters(15);
  @Override
  public void teleopPeriodic() {
    AprilTag ClosestTag = AprilTagManager.getClosestTagToRobotCenter(SimRobotChassisLoc);
    NT_ClosestTag.set(ClosestTag.Pose);
    AprilTag ourtag = AprilTagManager.getTagbyID(chosenAprilTagID);
    System.out.println(ourtag.ID + " ID : tag found : " + ourtag.name);
    NT_myloc.set(ourtag.Pose);
    NT_myStraightloc.set(AprilTagManager.getStraightOutLoc(ClosestTag.ID, Units.inchesToMeters(6)));
    NT_Leftloc.set(AprilTagManager.getOffSet90Loc(ClosestTag.ID, Units.inchesToMeters(6), ReefWidthCenteronCenter,true));
    NT_rightloc.set(AprilTagManager.getOffSet90Loc(ClosestTag.ID, Units.inchesToMeters(6), ReefWidthCenteronCenter,false));
    //Pose2d RobotChassisLoc = AprilTagManager.getStraightOutLoc(chosenAprilTagID, Units.inchesToMeters(6));

    NT_ClosestSource.set(AprilTagManager.getClosestTagofTypeToRobotCenter(SimRobotChassisLoc,TagType.Source).Pose);
    NT_ClosestProcessor.set(AprilTagManager.getClosestTagofTypeToRobotCenter(SimRobotChassisLoc,TagType.Processor).Pose);
    NT_ClosestReef.set(AprilTagManager.getClosestTagofTypeToRobotCenter(SimRobotChassisLoc,TagType.Reef).Pose);
    NT_ClosestBarge.set(AprilTagManager.getClosestTagofTypeToRobotCenter(SimRobotChassisLoc,(DriverStation.getAlliance().isPresent() & DriverStation.getAlliance().get().equals(Alliance.Blue))? TagType.BlueBarge:TagType.RedBarge).Pose);

   
    if (m_joystick.getHID().getBackButton()) {
      // Here, we set the constant setpoint of 0.75 meters.
      m_elevator.reachGoal(Constants.kSetpointMeters);
    } else {
      // Otherwise, we update the setpoint to 0.
      m_elevator.reachGoal(0.0);
    }
  }

  @Override
  public void disabledInit() {
    // This just makes sure that our simulation code knows that the motor's off.
    m_elevator.stop();
  }

  @Override
  public void close() {
    m_elevator.close();
    super.close();
  }
}
