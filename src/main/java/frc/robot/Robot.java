// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructEntry;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.AlphaBots.NT;
import frc.robot.AlphaBots.advantageKitBootstrap;
import frc.robot.subsystems.Elevator;

/** This is a sample program to demonstrate the use of elevator simulation. */
public class Robot extends TimedRobot {
  private final CommandXboxController m_joystick = new CommandXboxController(Constants.kJoystickPort);
  private final Elevator m_elevator = new Elevator();
  //private final advantageKitBootstrap akit = new advantageKitBootstrap(this);
  private final AprilTagManager apriltaglocations = new AprilTagManager();
  private int chosenAprilTagID = 0;

  StructPublisher<Pose2d> publisher = NetworkTableInstance.getDefault()
  .getStructTopic("MyPose", Pose2d.struct).publish();
  StructEntry<Pose2d> NT_myloc = NT.getStructEntry_Pose2D("Poses","TagLoc",new Pose2d());
  StructEntry<Pose2d> NT_myStraightloc = NT.getStructEntry_Pose2D("Poses","StraightLoc",new Pose2d());//straight out from the april tag. a centered pick for de-algae/processor/pickup 
  StructEntry<Pose2d> NT_Leftloc = NT.getStructEntry_Pose2D("Poses","leftLoc",new Pose2d());//LEFT FROM Robot TOWARDS tag view!
  StructEntry<Pose2d> NT_rightloc = NT.getStructEntry_Pose2D("Poses","rightLoc",new Pose2d());//Right Given View Robot TOWARDS tag!
  DoubleEntry NT_tagID = NT.getDoubleEntry("", "TagID", chosenAprilTagID);
 
  
  public Robot() {

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

  @Override
  public void simulationPeriodic() {
    // Update the simulation model.
    m_elevator.simulationPeriodic();
  }
  double ReefWidthCenteronCenter = Units.inchesToMeters(15);
  @Override
  public void teleopPeriodic() {
    AprilTag ourtag = AprilTagManager.getTagbyID(chosenAprilTagID);
    System.out.println(ourtag.ID + " ID : tag found : " + ourtag.name);
    NT_myloc.set(ourtag.Pose);
    NT_myStraightloc.set(AprilTagManager.getStraightOutLoc(chosenAprilTagID, Units.inchesToMeters(6)));
    NT_Leftloc.set(apriltaglocations.getOffSet90Loc(chosenAprilTagID, Units.inchesToMeters(6), ReefWidthCenteronCenter,true));
    NT_rightloc.set(apriltaglocations.getOffSet90Loc(chosenAprilTagID, Units.inchesToMeters(6), ReefWidthCenteronCenter,false));
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
