package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.Constants;
import frc.robot.Constants.ExtendConstants;

public class ElevatorSubsystem extends SubsystemBase{
    private final CANSparkMax elevatorMotorRt = new CANSparkMax(Constants.LiftConstants.LIFT_RT, MotorType.kBrushless);
    private final CANSparkMax elevatorMotorLt= new CANSparkMax(Constants.LiftConstants.LIFT_LT, MotorType.kBrushless);
    private final MotorControllerGroup a_rightMotors = new MotorControllerGroup(elevatorMotorRt);
    private final MotorControllerGroup a_leftMotors = new MotorControllerGroup(elevatorMotorLt);
    private boolean stopped = true;
    private RelativeEncoder m_encoder;
    private double currSpeed = 0;
    private double stoppedPos;
    private double maxExtension = 0; // Difference between high and low encode values
    private double lowLimit = 0;
    private double speedLimitPoint = 2.0;
    private final DifferentialDrive extend = new DifferentialDrive(a_leftMotors, a_rightMotors);

    public ElevatorSubsystem() {
        elevatorMotorRt.setIdleMode(IdleMode.kBrake);
        elevatorMotorLt.setIdleMode(IdleMode.kBrake);
        elevatorMotorRt.setInverted(false);
        elevatorMotorLt.setInverted(true);
        m_encoder = elevatorMotorLt.getEncoder();       
/*             
        m_pidController.setP(0.6);
        m_pidController.setI(0.0);
        m_pidController.setD(0.0);
*/
        stoppedPos = m_encoder.getPosition();
        SmartDashboard.putNumber(ExtendConstants.EXTEND_RANGE_LABEL, maxExtension);
    }

    public void init() {
        lowLimit = SmartDashboard.getNumber(ExtendConstants.EXTEND_LOW_LIMIT, lowLimit);
        maxExtension = SmartDashboard.getNumber(ExtendConstants.EXTEND_RANGE_LABEL, maxExtension);
    }

    public void setPosition(double position) {
        m_encoder.setPosition(position);
    }

    public double getPosition() {
        return m_encoder.getPosition();
    }

    // Returns true if target reached
   
    public CommandBase extendArmCommand(DoubleSupplier speed) {
        return run(() -> {
            extend.arcadeDrive(speed.getAsDouble(), 0);
            System.out.println("Arm is exxtending.... with speed: " + speed.getAsDouble());
        }); 
    }
    
    public CommandBase lowerArmCommand(DoubleSupplier speed) {
        return run(() -> {
            extend.arcadeDrive(speed.getAsDouble(), 0);
            System.out.println("Arm is retracting.... with speed: " + speed.getAsDouble());
        });
    }
    

    public boolean isStopped() {
        return stopped;
    }

    public void stop() {
        double currentPos = m_encoder.getPosition();
        if (!stopped) {
            currSpeed *= 0.5;
            stoppedPos = currentPos;
            if (currSpeed<0.25 && currSpeed>-0.25) {
                extend.stopMotor();
                stopped = true;
                System.out.println("Stopped extend at pos:"+stoppedPos);
            } else
                System.out.println("Slowing extend motor..speed:"+currSpeed);
        }
        extend.arcadeDrive(0, 0);  
    }

    public double getCurrentSpeed() {
        return currSpeed;
    }
}