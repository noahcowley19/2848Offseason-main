package frc.robot.subsystems;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.signals.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import edu.wpi.first.util.sendable.SendableBuilder;

import com.ctre.phoenix6.hardware.TalonFX;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.ShooterConstants.*;
import static frc.robot.RangerHelpers.*;
public class FlyWheel extends SubsystemBase{
private double robotToHub = 0;
private double angleToHub = 0;
private Translation2d translation = new Translation2d();




  private final TalonFX m_FlywheelLeftLeader;
  private final TalonFX m_FlywheelRightFollower;
  private final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);
  private final DutyCycleOut test = new DutyCycleOut(kFlywheelLeftMotorID);

    public FlyWheel(){
     m_FlywheelLeftLeader = new TalonFX(kFlywheelLeftMotorID);
     m_FlywheelRightFollower = new TalonFX(kFlywheelRightMotorID);
      TalonFXConfiguration flywheelConfig = new TalonFXConfiguration();

    flywheelConfig.Slot0.kS = 0;
    flywheelConfig.Slot0.kV = 0.12;
    flywheelConfig.Slot0.kP = 0.9;
    //flywheelConfig.Slot0.kI = 0.08;
    flywheelConfig.Slot0.kI = 0;
    flywheelConfig.Slot0.kD = 0;
    flywheelConfig.Slot0.kA = 0;
    flywheelConfig.Voltage
        .withPeakForwardVoltage(Volts.of(kFlywheelPeakVoltage))
        .withPeakReverseVoltage(Volts.of(-1 * kFlywheelPeakVoltage));
    flywheelConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    setupTalonFx(m_FlywheelLeftLeader, flywheelConfig);
    m_FlywheelRightFollower.setControl(new Follower(m_FlywheelLeftLeader.getDeviceID(), MotorAlignmentValue.Aligned));
    }


    public void periodic(){

        
      angleToHub = RobotContainer.getTurret().getAngleToHub();
      robotToHub = RobotContainer.getTurret().getDistanceToHub();
      double height = 1.2; //meters
      double angle2 = Math.atan(height/robotToHub);
      double gravity = 9.81;
      double fVelocity = (robotToHub/Math.cos(angle2))*(Math.sqrt(gravity/(2*(robotToHub*Math.tan(angle2)-1.2))));
      double motorVelocity = fVelocity * 3/4;

      m_FlywheelLeftLeader.setControl(m_request.withVelocity(motorVelocity));
      
      SmartDashboard.putData(this);






    }
    public double getError()
    {
      double error = Math.abs(66-m_FlywheelLeftLeader.getVelocity().getValueAsDouble());
      return error;
    }
    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("Velocity Output", () -> m_FlywheelLeftLeader.getVelocity().getValueAsDouble(), null);
       builder.addDoubleProperty("Voltage", () -> m_FlywheelLeftLeader.getMotorVoltage().getValueAsDouble(), null);



   
    }

    
    
}






    
