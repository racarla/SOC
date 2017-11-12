/*
Class definitions for Control Functions

See: LICENSE.md for Copyright and License Agreement

History:
Chris Regan
2017-11-12 - Chris Regan - Defined cntrlPIDamp class and methods

*/

#ifndef CNTRLFUNC_HXX_
#define CNTRLFUNC_HXX_

// Define cntrlPID Class
class cntrlPID {
private:
	float kP, kI, kD, cmdMin, cmdMax;
	float ref, meas, dt;
	float cmd;

	float initState(float cmd, float err, float dErr);
	float calcCmd(float err, float iErr, float dErr);

public:
	int runMode;
	float iErr;
	float errPrev;

	void setParam(float kP_, float kI_, float kD_, float cmdMin_, float cmdMax_);
	float compute(float ref, float meas, float dt);
};

// Define cntrlPIDamp Class
class cntrlPIDamp {
private:
	float kP, kI, kDamp, cmdMin, cmdMax;
	float ref, meas, dMeas, dt;
	float cmd;

	float initState(float cmd, float err, float dErr);
	float calcCmd(float err, float iErr, float dErr);

public:
	int runMode;
	float iErr;

	void setParam(float kP_, float kI_, float kDamp_, float cmdMin_, float cmdMax_);
	float compute(float ref, float meas, float dMeas, float dt);
};

#endif // CNTRLFUNC_HXX_
