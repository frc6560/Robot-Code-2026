package frc.robot.subsystems.intake;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.fail;

import java.lang.reflect.Field;
import org.junit.jupiter.api.Test;
import org.littletonrobotics.junction.LogTable;

class IntakeIOInputsTest {
    @Test
    void everyInputSurvivesLoggingAndReplay() throws Exception {
        IntakeIO.IntakeIOInputs original = new IntakeIO.IntakeIOInputs();
        Field[] fields = IntakeIO.IntakeIOInputs.class.getFields();
        for (int i = 0; i < fields.length; i++) {
            Field field = fields[i];
            if (field.getType() == boolean.class) {
                field.setBoolean(original, true);
            } else if (field.getType() == double.class) {
                field.setDouble(original, i + 0.125);
            } else if (field.getType() == String.class) {
                field.set(original, "recorded-" + field.getName());
            } else {
                fail("Add a logging test value for " + field.getName());
            }
        }
        LogTable table = new LogTable(0);
        original.toLog(table);
        IntakeIO.IntakeIOInputs replayed = new IntakeIO.IntakeIOInputs();
        replayed.fromLog(table);
        for (Field field : fields) {
            assertEquals(field.get(original), field.get(replayed), field.getName());
        }
    }
}
