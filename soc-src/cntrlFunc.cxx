/*
Classes and functions for Control Functions

See: LICENSE.md for Copyright and License Agreement

History:
2017-11-12 - Chris Regan - Defined CntrlPiDamp class and methods
2017-11-12 - Chris Regan - Defined CntrlPid class and methods
2017-11-12 - Chris Regan - Defined CntrlDamp class and methods
*/

#include "cntrlFunc.hxx"

// CntrlPid
// Constructor
CntrlPid::CntrlPid()
{
	runMode = Standby; // Initialize in Standby
	iErr = 0.0; // Initialize Integrator State
	errPrev = 0.0; // Set previous error to zero

	cmd = 0.0; // Initialize the command to zero
}

// Set Parameters for the tunable controller
void CntrlPid::SetParam(float kP_, float kI_, float kD_, float cmdMin_, float cmdMax_)
{
	kP = kP_;
	kI = kI_;
	kD = kD_;

	cmdMin = cmdMin_;
	cmdMax = cmdMax_;
}

// PID Controller
float CntrlPid::Compute(const float ref, const float meas, const float dt)
{
	// Compute the Error
	float err = ref - meas;
	float dErr = (err - errPrev) / dt; // Derivative of Error

	switch(runMode) {
		case Reset: // Zero the State and Command
			iErr = 0.0;
			errPrev = 0.0;
			cmd = 0.0;
         	break;

  		case Standby: // Do Nothing, State and Command are unchanged
         	break;

		case Hold: // Compute Commands
			cmd = CntrlPid::CalcCmd(err, iErr, dErr);
         	break;

  		case Init: // Initialize State then Compute Commands
			iErr = CntrlPid::InitState(cmd, err, dErr);

			cmd = CntrlPid::CalcCmd(err, iErr, dErr);
         	break;

		case Engage: // Update the State then Compute Commands
			iErr += (dt * err); // Update the state

			cmd = CntrlPid::CalcCmd(err, iErr, dErr);
         	break;
	}

	return cmd;
}

// Compute the Command for the Pid or PI+Damper controllers
float CntrlPid::CalcCmd(float err, float iErr, float dErr)
{
	float pCmd = kP * err;
	float iCmd = kI * iErr;
	float dCmd = kD * dErr;
	float cmd = pCmd + iCmd + dCmd;

	// saturate cmd, set iErr to limit that produces saturated cmd
	if (cmd <= cmdMin) {
		cmd = cmdMin;
		iErr = CntrlPid::InitState(cmd, err, dErr); // Re-compute the integrator state
	} else if (cmd >= cmdMax) {
		cmd = cmdMax;
		iErr = CntrlPid::InitState(cmd, err, dErr); // Re-compute the integrator state
	}
	
	return cmd;
}

// Initialize a PI, Pid, or PI+Damper Controller for near-zero transient
float CntrlPid::InitState(float cmd, float err, float dErr)
{
	iErr = 0.0;

	if (kI != 0.0) { // Protect for kI == 0
		iErr = (cmd - (kP * err + kD * dErr)) / kI; // Compute the required state
	}

	return iErr;
}


// CntrlPiDamp
// Constructor
CntrlPiDamp::CntrlPiDamp()
{
	runMode = Standby; // Initialize in Standby
	iErr = 0.0; // Initialize the Integrator

	cmd = 0.0; // Initialize the command to zero
}

// Set Parameters for the tunable controller
void CntrlPiDamp::SetParam(float kP_, float kI_, float kDamp_, float cmdMin_, float cmdMax_)
{
	kP = kP_;
	kI = kI_;
	kDamp = kDamp_;

	cmdMin = cmdMin_;
	cmdMax = cmdMax_;
}

// PI Controller plus Damper
float CntrlPiDamp::Compute(const float ref, const float meas, const float dMeas, const float dt)
{
	float err = ref - meas; // Compute the Error
	float dErr = dMeas; // Measurement for the Damper

	switch(runMode) {
		case Reset: // Zero the state and command
			iErr = 0.0;
			cmd = 0.0;
         	break;

  		case Standby: // Do Nothing, State and Command unchanged
         	break;

		case Hold: // Hold Integrator State, Compute Commands
			cmd = CntrlPiDamp::CalcCmd(err, iErr, dErr);
         	break;

  		case Init: // Initialize State then Compute Commands
			iErr = CntrlPiDamp::InitState(cmd, err, dErr);

			cmd = CntrlPiDamp::CalcCmd(err, iErr, dErr);
         	break;

		case Engage: // Update the State then Compute Commands
			iErr += (dt * err); // Update the state

			cmd = CntrlPiDamp::CalcCmd(err, iErr, dErr);
         	break;
	}

	return cmd;
}

// Compute the Command for the Pid or PI+Damper controllers
float CntrlPiDamp::CalcCmd(float err, float iErr, float dErr)
{
	float pCmd = kP * err;
	float iCmd = kI * iErr;
	float dCmd = kDamp * dErr;
	float cmd = pCmd + iCmd - dCmd; // NOTE THE SIGN!!

	// saturate cmd, set iErr to limit that produces saturated cmd
	if (cmd <= cmdMin) {
		cmd = cmdMin;
		iErr = CntrlPiDamp::InitState(cmd, err, dErr); // Re-compute the integrator state
	} else if (cmd >= cmdMax) {
		cmd = cmdMax;
		iErr = CntrlPiDamp::InitState(cmd, err, dErr); // Re-compute the integrator state
	}
	
	return cmd;
}

// Initialize a PI, Pid, or PI+Damper Controller for near-zero transient
float CntrlPiDamp::InitState(float cmd, float err, float dErr)
{
	iErr = 0.0;

	if (kI != 0.0) { // Protect for kI == 0
		iErr = (cmd - (kP * err + kDamp * dErr)) / kI; // Compute the required state
	}

	return iErr;
}

// CntrlDamp
// Constructor
CntrlDamp::CntrlDamp()
{
	runMode = Standby; // Initialize in Standby

	cmd = 0.0; // Initialize the command to zero
}

// Set Parameters for the tunable controller
void CntrlDamp::SetParam(float kDamp_, float cmdMin_, float cmdMax_)
{
	kDamp = kDamp_;

	cmdMin = cmdMin_;
	cmdMax = cmdMax_;
}

// Damper Controller
float CntrlDamp::Compute(const float dMeas)
{
	switch(runMode) {
		case Reset: // Zero the state and command
			cmd = 0.0;
         	break;

  		case Standby: // Do Nothing, State and Command unchanged
         	break;

		case Engage: // Update the State then Compute Commands

			cmd = CntrlDamp::CalcCmd(dMeas);
         	break;
	}

	return cmd;
}

// Compute the Command for the Damper controller
float CntrlDamp::CalcCmd(float dMeas)
{
	float cmd = -kDamp * dMeas; // NOTE THE SIGN!!

	// saturate cmd, set iErr to limit that produces saturated cmd
	if (cmd <= cmdMin) {
		cmd = cmdMin;
	} else if (cmd >= cmdMax) {
		cmd = cmdMax;
	}
	
	return cmd;
}
