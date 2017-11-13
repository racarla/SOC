/*
Classes and functions for Control Functions

See: LICENSE.md for Copyright and License Agreement

History:
2017-11-12 - Chris Regan - Defined CntrlPiDamp class and methods
2017-11-12 - Chris Regan - Defined CntrlPid class and methods
2017-11-12 - Chris Regan - Defined CntrlDamp class and methods
*/

#ifndef CNTRLFUNC_HXX_
#define CNTRLFUNC_HXX_

enum CntrlOpMode {Reset = -1, Standby = 0, Hold = 1, Init = 2, Engage = 3};

// Define CntrlPid Class
class CntrlPid {
private:
	float kP, kI, kD, cmdMin, cmdMax;
	float ref, meas, dt;
	float cmd;

	float InitState(float cmd, float err, float dErr);
	float CalcCmd(float err, float iErr, float dErr);

public:
	CntrlOpMode runMode;
	float iErr;
	float errPrev;

	CntrlPid();
	void SetParam(float kP_, float kI_, float kD_, float cmdMin_, float cmdMax_);
	float Compute(float ref, float meas, float dt);
};

// Define CntrlPiDamp Class
class CntrlPiDamp {
private:
	float kP, kI, kDamp, cmdMin, cmdMax;
	float ref, meas, dMeas, dt;
	float cmd;

	float InitState(float cmd, float err, float dErr);
	float CalcCmd(float err, float iErr, float dErr);

public:
	CntrlOpMode runMode;
	float iErr;

	CntrlPiDamp();
	void SetParam(float kP_, float kI_, float kDamp_, float cmdMin_, float cmdMax_);
	float Compute(float ref, float meas, float dMeas, float dt);
};


// Define CntrlDamp Class
class CntrlDamp {
private:
	float kDamp, cmdMin, cmdMax;
	float dMeas;
	float cmd;

	float CalcCmd(float dMeas);

public:
	CntrlOpMode runMode;

	CntrlDamp();
	void SetParam(float kDamp_, float cmdMin_, float cmdMax_);
	float Compute(float dMeas);
};

#endif // CNTRLFUNC_HXX_
