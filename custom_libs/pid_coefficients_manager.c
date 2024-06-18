#include "pid_coefficients_manager.h"

extern void send_new_pid_coeffs(struct PidCoefficients *pidCoefs);

void callback_on_new_raw_coef_received(uint8_t *rawCoefBuf)
{
	struct PidCoefficients newCoefs;
	memcpy((void *)&newCoefs, rawCoefBuf, sizeof(struct PidCoefficients));
	send_new_pid_coeffs(&newCoefs);
}
