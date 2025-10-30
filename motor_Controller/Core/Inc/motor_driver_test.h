#ifndef MOTOR_DRIVER_TEST_H
#define MOTOR_DRIVER_TEST_H

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    MOTOR_TEST_STOP = 0,
    MOTOR_TEST_FORWARD,
    MOTOR_TEST_BACKWARD,
    MOTOR_TEST_LEFT,
    MOTOR_TEST_RIGHT
} MotorTestState;

void MotorDriver_TestInitialize(void);
void MotorDriver_ApplyState(MotorTestState state);
void MotorDriver_TestRunSequence(void);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_DRIVER_TEST_H */
