
package frc.robot.archive;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Add your docs here.
 */
public class Limit_Switch {
    // test  function to run motor when limit switch is test. May or may not be useful

    static TalonSRX Switcher = new TalonSRX(13);
    static DigitalInput LimitSwitchTest = new DigitalInput(9);

    public static void LimitSwitchTester() {
        if (LimitSwitchTest.get()) {
            Switcher.set(ControlMode.PercentOutput, 0.0);

        }
        if (!LimitSwitchTest.get()) {
            Switcher.set(ControlMode.PercentOutput, 0.1);

        }
        SmartDashboard.putBoolean("STOPGOSWITCH", LimitSwitchTest.get());

    }
}
