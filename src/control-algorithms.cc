
#include "control-algorithms.hxx"

void __PIDClass::Configure(float Kp, float Ki, float Kd, float Tf, float b, float c, bool SatFlag, float OutMax, float OutMin) {
  Clear();  // Set Defaults

  Kp_ = Kp;
  Kd_ = Kd;
  Ki_ = Ki;
  Tf_ = Tf;
  b_ = b;
  c_ = c;
  SatFlag_ = SatFlag;
  OutMax_ = OutMax;
  OutMin_ = OutMin;
}

void __PIDClass::Run(GenericFunction::Mode mode,float Reference,float Feedback,float dt,float *Output,int8_t *Saturated) {

  // error for proportional
  ProportionalError_ = b_ * Reference - Feedback;
  // error for integral
  IntegralError_ = Reference - Feedback;
  // error for derivative
  DerivativeError_ = c_ * Reference - Feedback;

  mode_ = mode;
  switch(mode_) {
    case GenericFunction::Mode::kStandby: {
      break;
    }
    case GenericFunction::Mode::kArm: {
      Reset();
      ComputeDerivative(dt);
      InitializeState(0.0f);
      CalculateCommand();
      break;
    }
    case GenericFunction::Mode::kHold: {
      ComputeDerivative(dt);
      CalculateCommand();
      break;
    }
    case GenericFunction::Mode::kEngage: {
      ComputeDerivative(dt);
      UpdateState(dt);
      CalculateCommand();
      break;
    }
  }
  *Output = Output_;
  *Saturated = Saturated_;
}

void __PIDClass::InitializeState(float Command) {
  // Protect for Ki == 0
  if (Ki_ != 0.0f) {
    IntegralErrorState_ = (Command - (Kp_*ProportionalError_ + Kd_*DerivativeErrorState_)) / Ki_;
  } else {
    IntegralErrorState_ = 0.0f;
  }
}

void __PIDClass::ComputeDerivative(float dt) {

  float DerivativeErrorChange = DerivativeError_ - PreviousDerivativeError_;
  float DerivativeFiltVal = 0.0f;

  if (DerivativeErrorChange != 0) {
    DerivativeFiltVal = Tf_ + dt / (DerivativeError_ - PreviousDerivativeError_);

    if (DerivativeFiltVal != 0) {
      DerivativeErrorState_ = 1.0f / DerivativeFiltVal;
    } else {
      DerivativeErrorState_ = 0.0f;
    }
  } else {
    DerivativeErrorState_ = 0.0f;
  }

  PreviousDerivativeError_ = DerivativeError_;
}

void __PIDClass::UpdateState(float dt) {
  // Protect for unlimited windup when Ki == 0
  if (Ki_ != 0.0f) {
    IntegralErrorState_ += (dt*IntegralError_);
  } else {
    IntegralErrorState_ = 0.0;
  }
}

void __PIDClass::CalculateCommand() {
  float ProportionalCommand = Kp_*ProportionalError_;
  float IntegralCommand = Ki_*IntegralErrorState_;
  float DerivativeCommand = Kd_*DerivativeErrorState_;
  Output_ = ProportionalCommand + IntegralCommand + DerivativeCommand;

  // saturate cmd, set iErr to limit that produces saturated cmd
  // saturate command
  if (SatFlag_) {
    if (Output_ <= OutMin_) {
      Output_ = OutMin_;
      Saturated_ = -1;
      // Re-compute the integrator state
      InitializeState(Output_);
    } else if (Output_ >= OutMax_) {
      Output_ = OutMax_;
      Saturated_ = 1;
      // Re-compute the integrator state
      InitializeState(Output_);
    } else {
      Saturated_ = 0;
    }
  }
}

void __PIDClass::Reset() {
  // Reset internal values
  ProportionalError_ = 0.0f;
  DerivativeError_ = 0.0f;
  IntegralError_ = 0.0f;
  DerivativeErrorState_ = 0.0f;

  // Reset States
  PreviousDerivativeError_ = 0.0f;
  IntegralErrorState_ = 0.0f;

  // reset mode
  mode_ = GenericFunction::Mode::kStandby;
}

void __PIDClass::Clear() {
  Reset();

  Kp_ = 1.0f;
  Ki_ = 0.0f;
  Kd_ = 0.0f;
  Tf_ = 0.0f;
  b_ = 1.0f;
  c_ = 1.0f;
  SatFlag_ = false;
  OutMax_ = 0.0f;
  OutMin_ = 0.0f;

  Saturated_ = 0;
  Output_ = 0.0f;
}


/* State Space */
void __SSClass::Configure(Eigen::MatrixXf A, Eigen::MatrixXf B, Eigen::MatrixXf C, Eigen::MatrixXf D, bool SatFlag, Eigen::VectorXf yMax, Eigen::VectorXf yMin) {
  Clear(); // Clear and set to defaults

  A_ = A;
  B_ = B;
  C_ = C;
  D_ = D;
  SatFlag_ = SatFlag;
  yMax_ = yMax;
  yMin_ = yMin;

  uint8_t numU = B_.cols();
  uint8_t numX = A_.rows();
  uint8_t numY = C_.rows();

  y_.resize(numY);
  yMax_.resize(numY);
  yMin_.resize(numY);
  ySat_.resize(numY);

  Reset(); // Initialize states and output

  // Compute the Inverse of C
  // Pseduo-Inverse using singular value decomposition
  CA_inv_ = (C_ * A_).jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve( Eigen::MatrixXf::Identity(numY, numY)); // Jacobi SVD
  CB_ = C_*B_;
}

void __SSClass::Run(GenericFunction::Mode mode, Eigen::VectorXf u, float dt, Eigen::VectorXf *y, Eigen::VectorXi *ySat) {
  mode_ = mode;

  switch(mode_) {
    case GenericFunction::Mode::kStandby: {
      break;
    }
    case GenericFunction::Mode::kArm: {
      Reset();
      InitializeState(u, dt);
      UpdateState(u, dt);
      OutputEquation(u, dt);
      break;
    }
    case GenericFunction::Mode::kHold: {
      OutputEquation(u, dt);
      break;
    }
    case GenericFunction::Mode::kEngage: {
      UpdateState(u, dt);
      OutputEquation(u, dt);
      break;
    }
  }
  *y = y_;
  *ySat = ySat_;
}

void __SSClass::InitializeState(Eigen::VectorXf u, float dt) {
  x_ = (1.0f/dt) * CA_inv_ * (y_ - (CB_*dt + D_) * u);
}

void __SSClass::UpdateState(Eigen::VectorXf u, float dt) {
  x_ = dt * (A_*x_ + B_*u);
}

void __SSClass::OutputEquation(Eigen::VectorXf u, float dt) {
  y_ = C_*x_ + D_*u;

  // saturate output
  if (SatFlag_ == true){
    for (size_t i=0; i < y_.size(); i++) {
      if (y_(i) <= yMin_(i)) {
        y_(i) = yMin_(i);
        ySat_(i) = -1;
      } else if (y_(i) >= yMax_(i)) {
        y_(i) = yMax_(i);
        ySat_(i) = 1;
      } else {
        ySat_(i) = 0;
      }
    }

    if (ySat_.cwiseAbs().any()) { // check if any of the outputs are saturated
      InitializeState(u, dt); // Re-initialize the states with saturated outputs
    }
  }
}

void __SSClass::Reset() {
  x_.Zero(A_.rows()); // Reset to Zero
  y_.Zero(C_.cols()); // Reset to Zero
  ySat_.Zero(C_.cols()); // Reset to Zero

  mode_ = GenericFunction::Mode::kStandby;
}

void __SSClass::Clear() {
  A_.resize(0,0);
  B_.resize(0,0);
  C_.resize(0,0);
  D_.resize(0,0);

  yMin_.resize(0);
  yMax_.resize(0);

  SatFlag_ = false;

  CA_inv_.resize(0,0);
  CB_.resize(0,0);

  Reset();
}
