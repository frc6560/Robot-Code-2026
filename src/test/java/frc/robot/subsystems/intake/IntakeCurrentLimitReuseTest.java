package frc.robot.subsystems.intake;

import static org.junit.jupiter.api.Assertions.*;
import org.junit.jupiter.api.Test;
import frc.robot.Constants.IntakeConstants;

class IntakeCurrentLimitReuseTest {
    @Test void limitTransitionsReuseSameConfigurationAndKeepSafetySettings() {
        var config = IntakeIOTalonFX.deployCurrentLimits(IntakeConstants.DEPLOY_STATOR_CURRENT_LIMIT);
        assertSame(config, IntakeIOTalonFX.updateDeployCurrentLimits(config, IntakeConstants.DEPLOY_STARTUP_STATOR_CURRENT_LIMIT));
        assertEquals(IntakeConstants.DEPLOY_STARTUP_STATOR_CURRENT_LIMIT, config.StatorCurrentLimit);
        assertTrue(config.StatorCurrentLimitEnable);
        assertTrue(config.SupplyCurrentLimitEnable);
        assertEquals(IntakeConstants.DEPLOY_SUPPLY_CURRENT_LIMIT, config.SupplyCurrentLimit);
        assertSame(config, IntakeIOTalonFX.updateDeployCurrentLimits(config, IntakeConstants.DEPLOY_STATOR_CURRENT_LIMIT));
        assertEquals(IntakeConstants.DEPLOY_STATOR_CURRENT_LIMIT, config.StatorCurrentLimit);
    }
}
