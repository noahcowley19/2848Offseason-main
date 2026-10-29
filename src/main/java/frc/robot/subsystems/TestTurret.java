package frc.robot.subsystems;

import static frc.robot.Constants.ShooterConstants.kTurretCANcoderID;
import static frc.robot.Constants.ShooterConstants.kTurretMotorID;
import static frc.robot.Constants.ShooterConstants.redHubPose;
import static frc.robot.RangerHelpers.retryConfigApply;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class TestTurret extends SubsystemBase{
     private final TalonFX m_Turret = new TalonFX(kTurretMotorID);
    private final CANcoder m_Encoder = new CANcoder(kTurretCANcoderID);

    private Field2d field = new Field2d();

   private Translation2d robotToHub = new Translation2d();
    
     private double kP = (.5*180)/Math.PI;
     private double target = Math.PI/2;
     private double position = 0;
     private double error = 0;
     private double dcycOutput = 0;
     private double offset = -0.327881;
     private double kTestVolts = 2;
     private boolean closedLoop = true;
     
     private double angleToHub = 0;
     private double heading = 0;
     private double correction = 0;

     private double tempTarget = 0;

     public TestTurret() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;

        config.MotorOutput.Inverted=InvertedValue.Clockwise_Positive;
        CANcoderConfiguration CANconfig = new CANcoderConfiguration();

        CANconfig.MagnetSensor.SensorDirection=SensorDirectionValue.Clockwise_Positive;
        CANconfig.MagnetSensor.MagnetOffset = offset;
        retryConfigApply(()->m_Encoder.getConfigurator().apply(CANconfig));
        m_Turret.getConfigurator().apply(config);
     }
     public void periodic(){

      field.setRobotPose(RobotContainer.getDrivetrain().getState().Pose);

      SmartDashboard.putData(field);

      SmartDashboard.putData(this);

        //position = m_Turret.getPosition().getValue().magnitude();

       position = m_Encoder.getAbsolutePosition().getValueAsDouble() * Math.PI*2;

       heading = RobotContainer.getDrivetrain().getState().Pose.getRotation().getRadians();




      robotToHub = redHubPose.minus(RobotContainer.getDrivetrain().getState().Pose.getTranslation());

      angleToHub = MathUtil.angleModulus(robotToHub.getAngle().getRadians());

      correction = MathUtil.angleModulus(angleToHub-heading);

      setTarget(correction);
      





       


        
        error = target - position;

        dcycOutput = error*kP;

        if(dcycOutput>100)
            dcycOutput = 100;
        else if(dcycOutput<-100)
            dcycOutput = -100;

        // skip closed loop while runVolts is driving the motor
        if(closedLoop)
            m_Turret.set(dcycOutput/100);
     }
     public void setTarget(double radians)
     {
        target = radians; //MathUtil.clamp(radians, kMinTurretRad, kMaxTurretRad);
     }
     public Command goTo(double radians)
     {
        return runOnce(() -> setTarget(radians));
     }
     public Command goToDeg(double degrees)
     {
        return goTo(Math.toRadians(degrees));
     }
    //  public Command runVolts()
    //  {
    //     return startEnd(
    //         () -> {
    //             closedLoop = false;
    //             m_Turret.setVoltage(kTestVolts);
    //         },
    //         () -> {
    //             setTarget(position);
    //             closedLoop = true;
    //         });
    //  }
     public boolean atTarget()
     {
        return Math.abs(error) < Math.toRadians(2);
     }



     @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("Turret Angle",
        ()->target,
        value -> setTarget(value));

        builder.addDoubleProperty("Measured Angle",
        ()->m_Encoder.getAbsolutePosition().getValueAsDouble(),
        null);
        builder.addDoubleProperty("Angle to Hub", ()-> angleToHub, null);

        builder.addDoubleProperty("COrrectio", ()->correction,null);

        



    }
        








     }

    
    



