/*
MIT License

Copyright (c) 2023 Trobotix 8696

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

package org.firstinspires.ftc.teamcode.HAT;

import java.util.List;
import java.util.Map;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

// import com.qualcomm.robotcore.hardware.*; // No.
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.AccelerationSensor;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.CompassSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.IrSeekerSensor;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(name="Hardware Advanced Tester", group="HAT")
@Disabled

public class HAT extends LinearOpMode {

    private int value = 0;
    private int delta = 1;
    
    @Override
    public void runOpMode() {
        telemetry.setAutoClear(false);

        lineMessage("Hardware Advanced Tester (HAT)");
        lineMessage(hardwareMap.size() + " configured devices deteced.");
        lineMessage("Use the dpad to select a device to test. Press (A) to confirm selection.");
        
        HardwareMap.DeviceMapping<? extends HardwareDevice> deviceMapping = promptDeviceMapping();
        HardwareDevice hardwareDevice = promptHardwareDevice(deviceMapping);

        if (hardwareDevice != null) {
            lineMessage("Connection Info: " + hardwareDevice.getConnectionInfo());
            lineMessage("Specific Device Name: " + hardwareDevice.getDeviceName());
            lineMessage("Manufacturer: " + hardwareDevice.getManufacturer());
            lineMessage("Version: " + hardwareDevice.getVersion());    
        }

        lineMessage("Note: Use the dpad to adjust integral values.");
        lineMessage("Press play to continue.");
        
        waitForStart();

        lineMessage("Press (B) to stop.");

        if (hardwareDevice == null) {
            lineMessage("No devices available.");
        } else if (hardwareDevice instanceof AccelerationSensor) {
            testAccelerationSensor((AccelerationSensor)hardwareDevice);
        } else if (hardwareDevice instanceof AnalogInput) {
            testAnalogInput((AnalogInput)hardwareDevice);
        } else if (hardwareDevice instanceof ColorSensor) {
            testColorSensor((ColorSensor)hardwareDevice);
        } else if (hardwareDevice instanceof CompassSensor) {
            testCompassSensor((CompassSensor)hardwareDevice);
        } else if (hardwareDevice instanceof CRServo) {
            testCRServo((CRServo)hardwareDevice);
        } else if (hardwareDevice instanceof DigitalChannel) {
            testDigitalChannel((DigitalChannel)hardwareDevice);
        } else if (hardwareDevice instanceof GyroSensor) {
            testGyroSensor((GyroSensor)hardwareDevice);
        } else if (hardwareDevice instanceof I2cDevice) {
            testI2cDevice((I2cDevice)hardwareDevice);
        } else if (hardwareDevice instanceof IrSeekerSensor) {
            testIrSeekerSensor((IrSeekerSensor)hardwareDevice);
        } else if (hardwareDevice instanceof LED) {
            // May never trigger due to LED never explicitly implementing HardwareDevice...
            testLED((LED)hardwareDevice);
        } else if (hardwareDevice instanceof LightSensor 
            || hardwareDevice instanceof OpticalDistanceSensor
        ) {
            // The OpticalDistanceSensor is effectively the same thing as a LightSensor, but it's values have been normalized.
            testLightSensor((LightSensor)hardwareDevice);
        } else if (hardwareDevice instanceof Servo) {
            testServo((Servo)hardwareDevice);
        } else if (hardwareDevice instanceof TouchSensor) {
            testTouchSensor((TouchSensor)hardwareDevice);
        } else if (hardwareDevice instanceof UltrasonicSensor) {
            testUltrasonicSensor((UltrasonicSensor)hardwareDevice);
        } else if (hardwareDevice instanceof VoltageSensor) {
            testVoltageSensor((VoltageSensor)hardwareDevice);
        } else if (hardwareDevice instanceof DcMotor 
            || hardwareDevice instanceof DcMotorEx
        ) {
            testDcMotor((DcMotor)hardwareDevice);
        } else {
            lineMessage("Unsupported device type.");
        }

        lineMessage("Testing finished. Press (B) to exit.");
        waitExit();

        telemetry.setAutoClear(true);
    }

    private void testAccelerationSensor(AccelerationSensor sensor) {
        lineMessage("Status: " + sensor.status());
        while (bstop()) {
            telemetry.addData("Acceleration (g's)", sensor.getAcceleration());
            telemetry.update();
        }
    }

    private void testAnalogInput(AnalogInput input) {
        lineMessage("Maximum Voltage: " + input.getMaxVoltage());
        while (bstop()) {
            telemetry.addData("Voltage", input.getVoltage());
            telemetry.update();
        }
    }

    private void testColorSensor(ColorSensor sensor) {
        lineMessage("Press (A) to enable and (X) to disable LED if available.");
        while (bstop()) {
            telemetry.addData("Alpha (brightness)", sensor.alpha());
            telemetry.addData("ARGB Color Value", sensor.argb());
            telemetry.addData("Red", sensor.red());
            telemetry.addData("Green", sensor.green());
            telemetry.addData("Blue", sensor.blue());
            telemetry.update();

            if (gamepad1.a) {
                sensor.enableLed(true);
            } else if (gamepad1.x) {
                sensor.enableLed(false);
            }
        }
    }

    private void testCompassSensor(CompassSensor sensor) {
        lineMessage("Status: " + sensor.status());
        lineMessage("Press (A) to enter measurement mode and (X) to enter calibration mode.");
        while (bstop()) {
            telemetry.addData("Calibration Failed", sensor.calibrationFailed());
            telemetry.addData("Direction [0, 360)", sensor.getDirection());
            telemetry.update();

            if (gamepad1.a) {
                sensor.setMode(CompassSensor.CompassMode.MEASUREMENT_MODE);
            } else if (gamepad1.x) {
                sensor.setMode(CompassSensor.CompassMode.CALIBRATION_MODE);
            }
        }
    }

    private void testCRServo(CRServo servo) {
        value = 0;
        delta = 10;
        lineMessage("Port Number: " + servo.getPortNumber());
        lineMessage("Press (A) to set direction to FORWARD and (X) to set direction to REVERSE.");
        while (bstop()) {
            queryValue();
            double power = value / 100.0;
            telemetry.addData("Power", power);
            telemetry.addData("Direction", servo.getDirection());
            servo.setPower(power);

            if (gamepad1.a) {
                servo.setDirection(CRServo.Direction.FORWARD);
            } else if (gamepad1.x) {
                servo.setDirection(CRServo.Direction.REVERSE);
            }
        }
    }

    private void testDigitalChannel(DigitalChannel channel) {
        lineMessage("Press (A) to enter INPUT mode and (X) to enter OUTPUT mode.");
        lineMessage("Press bumpers to toggle channel state.");
        while (bstop()) {
            boolean state = channel.getState();
            telemetry.addData("State", state);
            telemetry.addData("Mode", channel.getMode());

            if (gamepad1.a) {
                channel.setMode(DigitalChannel.Mode.INPUT);
            } else if (gamepad1.x) {
                channel.setMode(DigitalChannel.Mode.OUTPUT);
            }
            if (gamepad1.left_bumper) {
                channel.setState(true);
            } else if (gamepad1.right_bumper) {
                channel.setState(false);
            }
        }
    }

    private void testGyroSensor(GyroSensor gyro) {
        lineMessage("Status: " + gyro.status());
        lineMessage("Press (A) to recalibrate and (X) to reset the z-axis integrator.");
        while (bstop()) {
            try {
                boolean calibrating = gyro.isCalibrating();
                telemetry.addData("Calibrating", calibrating);
                telemetry.addData("Heading (z-axis)", gyro.getHeading());
                telemetry.addData("Rotational Fraction", gyro.getRotationFraction());
                telemetry.addData("Raw X", gyro.rawX());
                telemetry.addData("Raw Y", gyro.rawY());
                telemetry.addData("Raw Z", gyro.rawZ());
                telemetry.update();
    
                if (gamepad1.a && !calibrating) {
                    gyro.calibrate();
                }
                if (gamepad1.x) {
                    gyro.resetZAxisIntegrator();
                }
            } catch (Exception e) {
                lineMessage("Gyro encountered an unsupported operation. Ending test.");
                break;
            }
        }
    }

    private void testI2cDevice(I2cDevice device) {
        lineMessage("Note: Lack of support.");
        while (bstop()) {
            String readData = "";
            byte[] readBuffer = device.getCopyOfReadBuffer();
            for (byte i : readBuffer) {
                readData += i + " ";
            }
            telemetry.addData("Read Data (excluding four-byte header section)", readData);
            telemetry.update();
        }
    }

    private void testIrSeekerSensor(IrSeekerSensor sensor) {
        value = (int)(sensor.getSignalDetectedThreshold() * 1000); // 1000 is currently an arbitrary constant.
        delta = 100;
        lineMessage("Press (A) to use 1200 Hz detection and (X) to use 600 Hz detection.");
        while (bstop()) {
            queryValue();
            telemetry.addData("Angle", sensor.getAngle());
            telemetry.addData("Mode", sensor.getMode());
            telemetry.addData("Detection Threshold", sensor.getSignalDetectedThreshold());
            telemetry.addData("Strength", sensor.getStrength());
            telemetry.addData("Signal Detected", sensor.signalDetected());

            IrSeekerSensor.IrSeekerIndividualSensor[] individualSensors = sensor.getIndividualSensors();
            for (int i = 0; i < individualSensors.length; ++i) {
                telemetry.addData(
                    "Individual Sensor " + i, 
                    "Angle (" + individualSensors[i].getSensorAngle() + ") Strength (" + individualSensors[i].getSensorStrength() + ")"
                );
            }

            sensor.setSignalDetectedThreshold(value / 1000.0);

            if (gamepad1.a) {
                sensor.setMode(IrSeekerSensor.Mode.MODE_1200HZ);
            } else if (gamepad1.x) {
                sensor.setMode(IrSeekerSensor.Mode.MODE_600HZ);
            }
        }
    }

    private void testLED(LED led) {
        lineMessage("Press (A) to enable, (X) to disable, and bumpers (left --> true, right --> false) to go the same to the light.");
        while (bstop()) {
            telemetry.addData("Light On", led.isLightOn());
            telemetry.update();

            if (gamepad1.a) {
                led.enable(true);
            } else if (gamepad1.x) {
                led.enable(false);
            }

            if (gamepad1.left_bumper) {
                led.enableLight(true);
            } else if (gamepad1.right_bumper) {
                led.enableLight(false);
            }
        }
    }

    private void testLightSensor(LightSensor sensor) {
        lineMessage("Press (A) to enable the LED light and (X) to disable.");
        lineMessage("Status: " + sensor.status());
        lineMessage("Raw Light Maximum: " + sensor.getRawLightDetectedMax());
        while (bstop()) {
            telemetry.addData("Light Detected", sensor.getLightDetected());
            telemetry.addData("Raw Light Detected", sensor.getRawLightDetected());
            telemetry.update();

            if (gamepad1.a) {
                sensor.enableLed(true);
            } else if (gamepad1.x) {
                sensor.enableLed(false);
            }
        }
    }

    private void testServo(Servo servo) {
        value = (int)(servo.getPosition() * 100);
        delta = 10;
        lineMessage("Servo Maximum Position: " + Servo.MAX_POSITION);
        lineMessage("Servo Minimum Position: " + Servo.MIN_POSITION);
        lineMessage("Port Number: " + servo.getPortNumber());
        lineMessage("Press (A) to set direction to FORWARD and (X) to set direction to REVERSE.");
        while (bstop()) {
            queryValue();
            double requestedPosition = value / 100.0;
            telemetry.addData("Actual Position", servo.getPosition());
            telemetry.addData("Direction", servo.getDirection());
            telemetry.addData("Requested Position", requestedPosition);
            
            // Implementing scaleRange doesn't appear to have any apparent utility.

            servo.setPosition(requestedPosition);

            if (gamepad1.a) {
                servo.setDirection(Servo.Direction.FORWARD);
            } else if (gamepad1.x) {
                servo.setDirection(Servo.Direction.REVERSE);
            }
        }
    }

    private void testTouchSensor(TouchSensor sensor) {
        while (bstop()) {
            telemetry.addData("Force Applied", sensor.getValue());
            telemetry.addData("Pressed", sensor.isPressed());
            telemetry.update();
        }
    }

    private void testUltrasonicSensor(UltrasonicSensor sensor) {
        lineMessage("Status: " + sensor.status());
        while (bstop()) {
            telemetry.addData("Ultrasonic Level", sensor.getUltrasonicLevel());
            telemetry.update();
        }
    }

    private void testVoltageSensor(VoltageSensor sensor) {
        while (bstop()) {
            telemetry.addData("Voltage", sensor.getVoltage());
        }
    }

    private void testDcMotor(DcMotor motor) {
        value = 0;
        delta = 100;
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        boolean ex = motor instanceof DcMotorEx;
        MotorConfigurationType motorConfigType = motor.getMotorType();
        lineMessage("Max RPM Fraction: " + motorConfigType.getAchieveableMaxRPMFraction());
        lineMessage("Max Ticks Per Second: " + motorConfigType.getAchieveableMaxTicksPerSecond());
        lineMessage("Distributor: " + motorConfigType.getDistributorInfo().getDistributor());
        lineMessage("Mode: " + motorConfigType.getDistributorInfo().getModel());
        lineMessage("Gearing: " + motorConfigType.getGearing());
        // If the toString method from PIDFCoefficients is insufficient, then those fields can be obtained from the object directly.
        lineMessage("Default Position PIDF Coeffcients: " + motorConfigType.getHubPositionParams().getPidfCoefficients());
        lineMessage("Default Velocity PIDF Coeffcients: " + motorConfigType.getHubVelocityParams().getPidfCoefficients());
        lineMessage("Max RPM: " + motorConfigType.getMaxRPM());
        lineMessage("Ticks per Revolution:" + motorConfigType.getTicksPerRev());
        // Many other configuration properties not listed due to lack of purpose in documentation.
        lineMessage("Port Number: " + motor.getPortNumber());
        lineMessage("================");
        lineMessage("Press (A) to run to position, (X) to enter zero power brake, (Y) to enter zero power float.");
        lineMessage("Press bumpers to set direction (left --> FORWARD, right --> REVERSE)");
        lineMessage("Use the right joystick to manually control the motor.");
        lineMessage("Before running to position (A), use the left joystick to control that operations's power.");
        while (bstop()) {
            queryValue();
            DcMotor.RunMode runMode = motor.getMode();
            telemetry.addData("Encoder Position", motor.getCurrentPosition());
            telemetry.addData("Mode", runMode);
            telemetry.addData("Power Float", motor.getPowerFloat());
            telemetry.addData("Zero Power Behavior", motor.getZeroPowerBehavior());
            telemetry.addData("Set Target Position", value);

            double runToPower = gamepad1.left_stick_y;
            telemetry.addData("Run To Position Power (left joystick)", runToPower);
            boolean busy = motor.isBusy();
            if (runMode == DcMotor.RunMode.RUN_TO_POSITION) {
                telemetry.addData("Busy", busy);
                telemetry.addData("Target Position", motor.getTargetPosition());
            }

            if (gamepad1.a && !busy) {
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motor.setTargetPosition(value);
                motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motor.setPower(runToPower);
                sleep(100);
            }

            if (gamepad1.x) {
                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            } else if (gamepad1.y) {
                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            }

            double manualPower = gamepad1.right_stick_y;
            telemetry.addData("Manual Power", manualPower);
            if (Math.abs(manualPower) > 0) {
                if (runMode == DcMotor.RunMode.RUN_TO_POSITION) {
                    motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                }
                motor.setPower(manualPower);
            }

            if (ex) {
                DcMotorEx motorEx = (DcMotorEx)motor;
                telemetry.addData("Current (AMPS)", motorEx.getCurrent(CurrentUnit.AMPS));
                telemetry.addData("Current (MILLIAMPS)", motorEx.getCurrent(CurrentUnit.MILLIAMPS));
                telemetry.addData("PIDF Coefficients (RUN_USING_ENCODER)", motorEx.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));
                telemetry.addData("PIDF Coefficients (RUN_TO_POSITION)", motorEx.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION));
                telemetry.addData("Target Position Tolerance", motorEx.getTargetPositionTolerance());
                telemetry.addData("Velocity (TPS)", motorEx.getVelocity());
                telemetry.addData("Over Current", motorEx.isOverCurrent());
                // Some redundant properties not used.
            }

            if (gamepad1.left_bumper) {
                motor.setDirection(DcMotor.Direction.FORWARD);
            } else if (gamepad1.right_bumper) {
                motor.setDirection(DcMotor.Direction.REVERSE);
            }
        }
    }

    private void queryValue() {
        telemetry.addData("Value", value);
        telemetry.addData("Delta", delta);
        telemetry.update();

        if (gamepad1.dpad_up) {
            value += delta;
            menuWait();
        } else if (gamepad1.dpad_down) {
            value -= delta;
            menuWait();
        }

        if (gamepad1.dpad_left && delta >= 10) {
            delta /= 10;
            menuWait();
        } else if (gamepad1.dpad_right) {
            delta *= 10;
            menuWait();
        }
    }

    private boolean bstop() {
        return !gamepad1.b && !isStopRequested();
    }

    private void lineMessage(String message) {
        telemetry.addLine(message);
        telemetry.update();
    }

    private HardwareMap.DeviceMapping<? extends HardwareDevice> promptDeviceMapping() {
        List<HardwareMap.DeviceMapping<? extends HardwareDevice>> deviceMappings = hardwareMap.allDeviceMappings;
        int size = deviceMappings.size();

        int index = 0;
        HardwareMap.DeviceMapping<? extends HardwareDevice> ret = deviceMappings.get(index);

        Telemetry.Item item = telemetry.addData("", "");

        while (!gamepad1.a && !isStopRequested()) {
            item.setCaption(ret.getDeviceTypeClass().getSimpleName());
            item.setValue(ret.size() + " devices");
            telemetry.update();

            if (gamepad1.dpad_up && index < size - 1) {
                ++index;
                ret = deviceMappings.get(index);
            }
            else if (gamepad1.dpad_down && index > 0) {
                --index;
                ret = deviceMappings.get(index);
            }

            menuWait();
        }
        while (gamepad1.a) {
            idle();
        }

        return ret;
    }

    private HardwareDevice promptHardwareDevice(HardwareMap.DeviceMapping<? extends HardwareDevice> deviceMapping) {
        Map.Entry<String, ? extends HardwareDevice>[] hardwareDevices = (Map.Entry<String, ? extends HardwareDevice>[])deviceMapping.entrySet().toArray();
        int size = hardwareDevices.length;

        if (size == 0) {
            return null;
        }

        int index = 0;
        Map.Entry<String, ? extends HardwareDevice> ret = hardwareDevices[index];

        Telemetry.Item item = telemetry.addData("", "");

        while (!gamepad1.a && !isStopRequested()) {
            item.setCaption(ret.getKey());
            item.setValue(ret.getValue().getDeviceName());
            telemetry.update();

            if (gamepad1.dpad_up && index < size - 1) {
                ++index;
                ret = hardwareDevices[index];
            }
            else if (gamepad1.dpad_down && index > 0) {
                --index;
                ret = hardwareDevices[index];
            }

            menuWait();
        }
        while (gamepad1.a) {
            idle();
        }

        return ret.getValue();
    }

    private void menuWait() {
        sleep(250);
    }

    private void waitExit() {
        while (!gamepad1.b && !isStopRequested()) {
            idle();
        }
    }
}
