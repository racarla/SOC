/*
Class definitions for Control Functions

See: LICENSE.md for Copyright and License Agreement

History:
Chris Regan
2017-11-12 - Chris Regan - Defined cntrlPIDamp class and methods

*/

#include "cntrlFunc.hxx"

// cntrlPIDamp
void cntrlPIDamp::setParam(float kP_, float kI_, float kDamp_, float cmdMin_, float cmdMax_)
{ // Constructor
	runMode = 0; // Initialize in Standby
	iErr = 0.0; // Initialize the Integrator

	kP = kP_;
	kI = kI_;
	kDamp = kDamp_;

	cmdMin = cmdMin_;
	cmdMax = cmdMax_;
}

float cntrlPIDamp::compute(const float ref, const float meas, const float dMeas, const float dt)
{ // PI Controller plus Damper, initialize state with cntrlPI_InitState
	// Compute the Error
	float err = ref - meas;
	float dErr = dMeas; // Measurement for the Damper

	switch(runMode) {
		case -1:
			// Reset - Zero the State then Compute Commands
			iErr = 0.0;

			cmd = cntrlPIDamp::calcCmd(err, iErr, dErr);
         	break;
  		case 0:
  			// Standby - Do Nothing
         	break;
		case 1:
			// Hold - Hold Integrator State, Compute Commands
			cmd = cntrlPIDamp::calcCmd(err, iErr, dErr);
         	break;
  		case 2:
			// Init - Initialize State then Compute Commands
			iErr = cntrlPIDamp::initState(cmd, err, dErr);

			cmd = cntrlPIDamp::calcCmd(err, iErr, dErr);
         	break;
		case 3:
			// Engaged - Update the State then Compute Commands
			iErr += (dt * err); // Update the state

			cmd = cntrlPIDamp::calcCmd(err, iErr, dErr);
         	break;
  		default:
  			// Standby - Do Nothing
         	break;
	}

	return cmd;
}


float cntrlPIDamp::calcCmd(float err, float iErr, float dErr)
{ // Compute the Command for the PID or PI+Damper controllers
	float pCmd = kP * err;
	float iCmd = kI * iErr;
	float dCmd = kDamp * dErr;
	float cmd = pCmd + iCmd + dCmd;

	// saturate cmd, set iErr to limit that produces saturated cmd
	if (cmd <= cmdMin) {
		cmd = cmdMin;
		iErr = cntrlPIDamp::initState(cmd, err, dErr); // Re-compute the integrator state
	} else if (cmd >= cmdMax) {
		cmd = cmdMax;
		iErr = cntrlPIDamp::initState(cmd, err, dErr); // Re-compute the integrator state
	}
	
	return cmd;
}

float cntrlPIDamp::initState(float cmd, float err, float dErr)
{ // Initialize a PI, PID, or PI+Damper Controller for near-zero transient
	iErr = 0.0;

	if (kI != 0.0) { // Protect for kI == 0
		iErr = (cmd - (kP * err + kDamp * dErr)) / kI; // Compute the required state
	}

	return iErr;
}


// cntrlPID
void cntrlPID::setParam(float kP_, float kI_, float kD_, float cmdMin_, float cmdMax_)
{ // Constructor
	runMode = 0; // Initialize in Standby
	iErr = 0.0; // Initialize the Integrator
	errPrev = 0.0;

	kP = kP_;
	kI = kI_;
	kD = kD_;

	cmdMin = cmdMin_;
	cmdMax = cmdMax_;
}

float cntrlPID::compute(const float ref, const float meas, const float dt)
{ // PI Controller plus Damper, initialize state with cntrlPI_InitState
	// Compute the Error
	float err = ref - meas;
	float dErr = (err - errPrev) / dt; // Measurement for the Damper

	switch(runMode) {
		case -1:
			// Reset - Zero the State then Compute Commands
			iErr = 0.0;
			errPrev = 0.0;

			cmd = cntrlPID::calcCmd(err, iErr, dErr);
         	break;
  		case 0:
  			// Standby - Do Nothing
         	break;
		case 1:
			// Hold - Compute Commands
			cmd = cntrlPID::calcCmd(err, iErr, dErr);
         	break;
  		case 2:
			// Init - Initialize State then Compute Commands
			iErr = cntrlPID::initState(cmd, err, dErr);

			cmd = cntrlPID::calcCmd(err, iErr, dErr);
         	break;
		case 3:
			// Engaged - Update the State then Compute Commands
			iErr += (dt * err); // Update the state

			cmd = cntrlPID::calcCmd(err, iErr, dErr);
         	break;
  		default:
  			// Standby - Do Nothing
         	break;
	}

	return cmd;
}

float cntrlPID::calcCmd(float err, float iErr, float dErr)
{ // Compute the Command for the PID or PI+Damper controllers
	float pCmd = kP * err;
	float iCmd = kI * iErr;
	float dCmd = kD * dErr;
	float cmd = pCmd + iCmd + dCmd;

	// saturate cmd, set iErr to limit that produces saturated cmd
	if (cmd <= cmdMin) {
		cmd = cmdMin;
		iErr = cntrlPID::initState(cmd, err, dErr); // Re-compute the integrator state
	} else if (cmd >= cmdMax) {
		cmd = cmdMax;
		iErr = cntrlPID::initState(cmd, err, dErr); // Re-compute the integrator state
	}
	
	return cmd;
}

float cntrlPID::initState(float cmd, float err, float dErr)
{ // Initialize a PI, PID, or PI+Damper Controller for near-zero transient
	iErr = 0.0;

	if (kI != 0.0) { // Protect for kI == 0
		iErr = (cmd - (kP * err + kD * dErr)) / kI; // Compute the required state
	}

	return iErr;
}
