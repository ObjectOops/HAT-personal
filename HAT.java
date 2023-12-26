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

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.AccelerationSensor;
import com.qualcomm.robotcore.hardware.AnalogInput;

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

        lineMessage("Press play to continue.");
        
        waitForStart();

        lineMessage("Press (B) to stop.");

        if (hardwareDevice == null) {
            lineMessage("No devices available.");
        } else if (hardwareDevice instanceof AccelerationSensor) {
            testAccelerationSensor((AccelerationSensor)hardwareDevice);
        } else if (hardwareDevice instanceof AnalogInput) {
            testAnalogInput((AnalogInput)hardwareDevice);
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
