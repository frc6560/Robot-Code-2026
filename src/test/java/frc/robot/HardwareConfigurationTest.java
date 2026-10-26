package frc.robot;

import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.assertEquals;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import java.nio.file.Path;
import java.util.HashSet;
import java.util.Set;
import org.junit.jupiter.api.Test;

class HardwareConfigurationTest {
    private final Set<String> devices = new HashSet<>();

    private void add(String bus, String type, int id) {
        assertTrue(id >= 0 && id <= 62, "Invalid CAN ID: " + id);
        assertTrue(devices.add(bus + "/" + type + "/" + id),
            "Duplicate CAN address: " + bus + "/" + type + "/" + id);
    }

    @Test
    void installedMechanismsAndDeployedSwerveHaveUniqueCanAddresses() throws Exception {
        add(Constants.ShooterConstants.CAN_BUS, "talonfx", Constants.ShooterConstants.LEFT_FLYWHEEL_ID);
        add(Constants.ShooterConstants.CAN_BUS, "talonfx", Constants.ShooterConstants.RIGHT_FLYWHEEL_ID);
        add(Constants.IndexerConstants.CAN_BUS, "talonfx", Constants.IndexerConstants.FLOOR_MOTOR_ID);
        add(Constants.IndexerConstants.CAN_BUS, "talonfx", Constants.IndexerConstants.TUNNEL_MOTOR_ID);
        if (Constants.HardwareConstants.RACK_INTAKE_INSTALLED) {
            add(Constants.IntakeConstants.CAN_BUS, "talonfx", Constants.IntakeConstants.DEPLOY_MOTOR_ID);
        }
        if (Constants.HardwareConstants.LEGACY_TURRET_INSTALLED) {
            add("rio", "talonfx", Constants.TurretConstants.MOTOR_ID);
            add("rio", "cancoder", Constants.TurretConstants.ENCODER_ID);
        }
        if (Constants.HardwareConstants.LEGACY_FEEDER_INSTALLED) {
            add(Constants.FeederConstants.CAN_BUS, "talonfx", Constants.FeederConstants.PAN_MOTOR_ID);
            add(Constants.FeederConstants.CAN_BUS, "talonfx", Constants.FeederConstants.PUSHER_MOTOR_ID);
            add(Constants.FeederConstants.CAN_BUS, "talonfx", Constants.FeederConstants.FLOOR_ID);
        }
        ObjectMapper mapper = new ObjectMapper();
        Path root = Path.of("src/main/deploy/swerve/falcon");
        JsonNode swerve = mapper.readTree(root.resolve("swervedrive.json").toFile());
        JsonNode imu = swerve.get("imu");
        add(imu.get("canbus").asText(), imu.get("type").asText(), imu.get("id").asInt());
        for (JsonNode name : swerve.get("modules")) {
            JsonNode module = mapper.readTree(root.resolve("modules").resolve(name.asText()).toFile());
            for (String component : new String[] {"drive", "angle", "encoder"}) {
                JsonNode device = module.get(component);
                String type = device.get("type").asText();
                // X44 and X60 both use the Talon FX device type on CAN.
                if (type.startsWith("kraken") || type.equals("talonfx")) {
                    type = "talonfx";
                }
                add(device.get("canbus").asText(), type, device.get("id").asInt());
            }
        }
    }

    @Test
    void rackGeometryMatchesConfirmedGearTrainAndPitchDiameter() {
        assertEquals(2.1875, Constants.IntakeConstants.DEPLOY_GEAR_RATIO, 1e-9);
        assertEquals(1.0, Constants.IntakeConstants.PINION_DIAMETER_INCHES, 1e-9);
        assertEquals(Math.PI / 2.1875,
            Constants.IntakeConstants.INCHES_PER_PINION_ROTATION
                / Constants.IntakeConstants.DEPLOY_GEAR_RATIO, 1e-9);
    }

    @Test
    void rackEnableTargetMatchesEightyPercentOfMeasuredTravel() {
        assertEquals(26, Constants.IntakeConstants.DEPLOY_MOTOR_ID,
            "Extension motor's confirmed CAN ID");
        assertEquals("Canivore", Constants.IntakeConstants.CAN_BUS,
            "Extension motor is wired to the CANivore");
        assertTrue(Constants.IntakeConstants.RETRACTED_POSITION_ROTATIONS
            < Constants.IntakeConstants.SHOOT_OSCILLATION_REAR_POSITION_ROTATIONS);
        assertTrue(Constants.IntakeConstants.SHOOT_OSCILLATION_REAR_POSITION_ROTATIONS
            < Constants.IntakeConstants.SHOOT_OSCILLATION_FORWARD_POSITION_ROTATIONS);
        assertTrue(Constants.IntakeConstants.SHOOT_OSCILLATION_FORWARD_POSITION_ROTATIONS
            <= Constants.IntakeConstants.EXTENDED_POSITION_ROTATIONS);
        assertEquals(28.5 / 2.54, Constants.IntakeConstants.MAX_EXTENSION_INCHES, 1e-9);
        assertEquals(22.8 / 2.54 / Math.PI * 2.1875,
            Constants.IntakeConstants.ENABLE_POSITION_ROTATIONS, 1e-9);
        assertTrue(Constants.IntakeConstants.ENABLE_POSITION_ROTATIONS
            < Constants.IntakeConstants.EXTENDED_POSITION_ROTATIONS);
        assertEquals(1000.0, Constants.IntakeConstants.ROLLER_RPM);
    }
}
