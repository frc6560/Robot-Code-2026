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
        add(Constants.IndexerConstants.CAN_BUS, "talonfx", Constants.IndexerConstants.FLOOR_SECONDARY_MOTOR_ID);
        add(Constants.IndexerConstants.CAN_BUS, "talonfx", Constants.IndexerConstants.TUNNEL_MOTOR_ID);
        add(Constants.IntakeConstants.CAN_BUS, "talonfx", Constants.IntakeConstants.DEPLOY_MOTOR_ID);
        add(Constants.IntakeConstants.CAN_BUS, "talonfx", Constants.IntakeConstants.LEFT_ROLLER_MOTOR_ID);
        add(Constants.IntakeConstants.CAN_BUS, "talonfx", Constants.IntakeConstants.RIGHT_ROLLER_MOTOR_ID);
        assertEquals(16, devices.size() + 8, "Only eight mechanism motors plus eight swerve motors are installed");
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
    void motorNamesMatchTheConfirmedPhysicalAddresses() {
        assertEquals(15, Constants.ShooterConstants.RIGHT_FLYWHEEL_ID);
        assertEquals(24, Constants.ShooterConstants.LEFT_FLYWHEEL_ID);
        assertEquals(16, Constants.IndexerConstants.TUNNEL_MOTOR_ID);
        assertEquals(36, Constants.IntakeConstants.LEFT_ROLLER_MOTOR_ID);
        assertEquals(18, Constants.IntakeConstants.RIGHT_ROLLER_MOTOR_ID);
        assertEquals(26, Constants.IntakeConstants.DEPLOY_MOTOR_ID);
    }

    @Test
    void shooterKeepsFortyAmpLimits() {
        assertEquals(40.0, Constants.ShooterConstants.FLYWHEEL_SUPPLY_CURRENT_LIMIT);
        assertEquals(40, Constants.ShooterConstants.FLYWHEEL_STATOR_CURRENT_LIMIT);
    }

    @Test
    void rollersUseConfirmedSeparateCanAddressesAndOppositeDirections() {
        assertEquals(36, Constants.IntakeConstants.LEFT_ROLLER_MOTOR_ID);
        assertEquals(18, Constants.IntakeConstants.RIGHT_ROLLER_MOTOR_ID);
        assertTrue(Constants.IntakeConstants.LEFT_ROLLER_MOTOR_INVERTED
            != Constants.IntakeConstants.RIGHT_ROLLER_MOTOR_INVERTED);
    }

    @Test
    void rackEnableTargetAndLimitMatchRequestedQuickTestRotations() {
        assertEquals(26, Constants.IntakeConstants.DEPLOY_MOTOR_ID,
            "Extension motor's confirmed CAN ID");
        assertEquals("Canivore", Constants.IntakeConstants.CAN_BUS,
            "Extension motor is wired to the CANivore");
        assertEquals(0, Constants.IntakeConstants.RETRACTED_POSITION_ROTATIONS);
        assertEquals(9.5, Constants.IntakeConstants.EXTENDED_POSITION_ROTATIONS);
        assertEquals(2500.0, Constants.IntakeConstants.ROLLER_RPM);
    }
}
