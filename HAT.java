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

@TeleOp(name="Hardware Advanced Tester", group="HAT")
@Disabled

public class HAT extends LinearOpMode {
    
    @Override
    public void runOpMode() {
        telemetry.setAutoClear(false);

        lineMessage("Hardware Advanced Tester (HAT)");
        lineMessage(hardwareMap.size() + " configured devices deteced.");
        lineMessage("Use the dpad to select a device to test. Press (A) to confirm selection.");
        
        HardwareMap.DeviceMapping<? extends HardwareDevice> deviceMapping = promptDeviceMapping();
        HardwareDevice hardwareDevice = promptHardwareDevice(deviceMapping);

        lineMessage("Press play to continue.");
        
        waitForStart();

        if (hardwareDevice instanceof AccelerationSensor) {
            // testAccelerationSensor(hardwareDevice);
        } else {
            lineMessage("Unsupported device type.");
        }

        lineMessage("Testing finished. Press (A) to exit.");
        waitExit();

        telemetry.setAutoClear(true);
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
        }
        while (gamepad1.a) {
            idle();
        }

        return ret;
    }

    private HardwareDevice promptHardwareDevice(HardwareMap.DeviceMapping<? extends HardwareDevice> deviceMapping) {
        Map.Entry<String, ? extends HardwareDevice>[] hardwareDevices = (Map.Entry<String, ? extends HardwareDevice>[])deviceMapping.entrySet().toArray();
        int size = hardwareDevices.length;

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
        }
        while (gamepad1.a) {
            idle();
        }

        return ret.getValue();
    }

    private void waitExit() {
        while (!gamepad1.a && !isStopRequested()) {
            idle();
        }
    }
}
