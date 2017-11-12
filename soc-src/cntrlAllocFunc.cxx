/*
Control Allocation and Mixer Functions
Chris Regan
rega0051@umn.edu

  purpose: functions available for the Controller's Allocation
*/


#include "cntrlAllocFunc.hxx"


void ControlAlloc(const MatCntrlEff &cntrlEff, const VecObj &vObj, const VecEff &uMin, const VecEff &uMax, const MatObj &wObj, const MatEff &wEff, const VecEff &uPref, VecEff &uCmd, int method)
{

  switch(method) {
  case 60:
    //ContAllocSQP(cntrlEff, vObj, uMin, uMax, wObj, wEff, uPref, uCmd);
    break;
  case 50:
    //ContAllocMOQP(cntrlEff, vObj, uMin, uMax, wObj, wEff, uPref, uCmd);
    break;
  case 40:
    //ContAllocFxp(cntrlEff, vObj, uMin, uMax, wObj, wEff, uPref, uCmd);
    break;
  case 30:
    //ContAllocMOLP(cntrlEff, vObj, uMin, uMax, wObj, wEff, uPref, uCmd);
    break;
  case 25:
    //ContAllocPseudoRedisScale(cntrlEff, vObj, uMin, uMax, wObj, wEff, uPref, uCmd);
    break;
  case 20:
    //ContAllocPseudoRedis(cntrlEff, vObj, uMin, uMax, wObj, wEff, uPref, uCmd);
    break;
  case 10:
    ContAllocPseudoWeighted(cntrlEff, vObj, wObj, wEff, uPref, uCmd);
    break;
  default:
    ContAllocPseudo(cntrlEff, vObj, uPref, uCmd);
    break;
  }
  
  // Saturate to control constraints
    Saturate(uMin, uMax, uCmd);
}

// Control Allocation Methods
// inputs: cntrlEff - surface effectiveness matrix
//         vObj - desired objective
//         uMin - lower bound for effectors
//         uMax - upper bound for effectors
//         wObj - objective weighting matrix
//         wEff - effector weighting matrix
//         uPref - "prefered" effector allocation
//         numRow - number of rows in cntrlEff
//         numCol - number of columns in cntrlEff   
// outputs: uCmd - effector commands signal

void ContAllocPseudo(const MatCntrlEff &cntrlEff, const VecObj &vObj, const VecEff &uPref, VecEff &uCmd)
{
  // Pseudo-Inverse control allocation
  VecObj b;
  VecEff x;
  
  // b = (vObj - cntrlEff * uPref)
  b = (vObj - cntrlEff * uPref);
  
  // Complete the Weighted Pseudo-Inverse
  SolvPinvSVD(cntrlEff, b, x);

  // uCmd = pinv_sol(cntrlEff, b) + uPref
  uCmd = x + uPref;
}

void ContAllocPseudoWeighted(const MatCntrlEff &cntrlEff, const VecObj &vObj, const MatObj &wObj, const MatEff &wEff, const VecEff &uPref, VecEff &uCmd)
{
  // Pseudo-Inverse control allocation method with Weightings
  MatCntrlEff A;
  VecObj b;
  VecEff x;
  
  // A = wObj * cntrlEff / wEff
  A = wObj * cntrlEff * wEff.inverse();

  // b = wObj * (vObj - cntrlEff * uPref)
  b = wObj * (vObj - cntrlEff * uPref);
  
  // Complete the Weighted Pseudo-Inverse
  SolvPinvSVD(A, b, x);

  // uCmd = inv(wEff) * pinv_sol(A, b) + uPref
  uCmd = (wEff.inverse() * x) + uPref;
}

void ContAllocFxp(const MatCntrlEff &cntrlEff, const VecObj &vObj, const VecEff &uMin, const VecEff &uMax, const MatObj &wObj, const MatEff &wEff, const VecEff &uPref, VecEff &uCmd, float gammaEff, int numIter)
{
  // fixed-point optimization control allocation method
  MatEff H, G, I;
  VecEff Fv;
  MatCntrlEffT BzT;
  
  VecEff xMin, xMax;

  // Resize and initialize Vectors and Matrices
  int numObj = cntrlEff.rows();
  int numEff = cntrlEff.cols();

  H.conservativeResize(numEff, numEff);
  G.conservativeResize(numEff, numEff);
  I.setIdentity(numEff, numEff); // Create a constant Identity matrix

  Fv.conservativeResize(numEff);
  BzT.conservativeResize(numEff, numObj);

  xMin.conservativeResize(numEff);
  xMax.conservativeResize(numEff);

  // Change variables, shift datum to uPref = 0;
  uCmd = uCmd - uPref;
  xMin = uMin - uPref;
  xMax = uMax - uPref;
  
  // Common Parameter, temp
  BzT = cntrlEff.transpose() * wObj.transpose() * wObj;
  
  // H = (1 - gammaEff) * (cntrlEff' * wObj * cntrlEff) + gammaEff * wEff
  H = (1 - gammaEff) * (BzT * cntrlEff) + gammaEff * wEff.transpose() * wEff;
  
  // w = 1/norm(H) <- frobenious norm
  float w = 1.0 / H.norm();
  
  // Fv = (1 - gammaEff) * (w * cntrlEff' * wObj) * vObj
  Fv = (1 - gammaEff) * (w * BzT * vObj);

  // G = w*H - I(numEff,numEff)
  G = (w * H) - I;
  
  // Loop a set number of times
  for(int iIter=0 ; iIter < numIter ; iIter++) {
    // Compute next point without considering the constraints.
    uCmd = Fv - G * uCmd;
    
    // Saturate to control constraints.
    Saturate(xMin, xMax, uCmd);
  }

  // Change variable back, uCmd = uCmd + uPref
  uCmd = uCmd + uPref;
}

/*void ContAllocPseudoRedis(const MatCntrlEff &cntrlEff, const VecObj &vObj, const VecEff &uMin, const VecEff &uMax, const MatObj &wObj, const MatEff &wEff, const VecEff &uPref, VecEff &uCmd)
{
  // Redistributed pseduo-inverse control allocation method
  int iObj, iEff, iIter;
  float tmp;
  int inadm;
  int numEffFree;
  
  MatCntrlEff BFree;
  VecObj vObjResid;
  VecEff pFree;
  VecEffInt iEffSat, iEffFree;
  
  // Resize and initialize Vectors and Matrices
  int numObj = cntrlEff.rows();
  int numEff = cntrlEff.cols();

  BFree.conservativeResize(numObj, numEff);

  vObjResid.conservativeResize(numObj);
  pFree.conservativeResize(numEff);

  iEffSat.conservativeResize(numEff);
  iEffFree.conservativeResize(numEff);

  // Initialize iEffSat = 0
  //VecEff iEffSate.Zero(numEff);

  int numIter = numEff;
  for(iIter = 0 ; iIter < numIter ; iIter++){
    
    // Find the "Free" Effectors
    // iEffFree = iEffSat==0
    // numEffFree = length(iEffFree)
    numEffFree = FindFree(iEffSat, iEffFree);
    
    // Create 'BFree', using only the 'free' columns of 'cntrlEff'
    ShrinkMatColumns(cntrlEff, iEffFree, BFree);
    
    // Create 'vObjResid', the remaining objective
    vObjResid = vObj - cntrlEff * uCmd ;
    
    // Solve the Pseudo Inverse
    SolvPinvSVD(BFree, vObjResid, pFree);
    

    ExpandVecElem(pFree, iEffFree, uCmd);

    int numSat = SaturateIndex(uMin, uMax, uCmd, iEffSat);

    if((numEffFree < numObj) & (numSat == 0)) return;
    
  }
}*/

/*void ContAllocPseudoRedisScale(const MatCntrlEff &cntrlEff, const VecObj &vObj, const VecEff &uMin, const VecEff &uMax, const MatObj &wObj, const MatEff &wEff, const VecEff &uPref, VecEff &uCmd)
{
  // redistributed pseduo-inverse control allocation method with control surface scaling
  int iObj, iEff, iIter;
  float tmp;
  float clip;
  int indx_clip;
  int numEffFree;
  
  VecEff iEffSat;
  VecEff iEffFree;
  MatCntrlEff BFree;
  
  VecObj vObjResid;
  VecObj pFree;
  VecEff uCmdRng;
  VecEff uCmdCntr;
  float uCmdCntr;
  
  // Size of cntrlEff
  int numEff = cntrlEff.rows();
  int numObj = cntrlEff.cols();

  // Initialize iEffSat, Define uCmdRng and uCmdCntr
  for(iEff=0 ; iEff<numEff ; iEff++){
    iEffSat[iEff] = 0 ;
    uCmdRng[iEff] = uMax[iEff] - uMin[iEff] ;
    uCmdCntr[iEff] = uMin[iEff] + uCmdRng[iEff]/2.0 ;
  }
  
  for(iIter=1 ; iIter<=numEff ; iIter++){
    // Define iEffFree from iEffSat
    numEffFree = 0 ;
    for(iEff=0 ; iEff<numEff ; iEff++) {
      if(iEffSat[iEff]==0) {
	iEffFree[numEffFree] = iEff ;
	numEffFree++;
      }
    }
    
    if(numEffFree < numObj) return;
    
    // Create 'BFree', a suBrix of 'free' columns of 'cntrlEff'
    for(iObj=0 ; iObj<numObj ; iObj++){
      for(iEff=0 ; iEff<numEffFree ; iEff++){
	BFree[iObj][iEff] = cntrlEff[iObj][iEffFree[iEff]];
      }
    }
    
    // Create 'vObjResid', the remaining objective
    for(iObj=0 ; iObj<numObj ; iObj++){
      tmp = 0.0 ;
      for(iEff=0 ; iEff<numEff ; iEff++) tmp += cntrlEff[iObj][iEff]*uCmd[iEff] ;
      
      vObjResid[iObj] = vObj[iObj] - tmp ;
    }
    
    
    SolvPinvSVD(BFree, vObjResidFree, pFree);
    
    
    for(iEff=0 ; iEff<numEffFree ; iEff++) {
      // update 'uCmd'
      uCmd[iEffFree[iEff]] = uCmd[iEffFree[iEff]] + pFree[iEff] ;
    }
    
    for(iEff=0 ; iEff<numEff ; iEff++) {
      // change variables to normalized
      uCmdCntr[iEff] = (uCmd[iEff] - uCmdCntr[iEff]) / uCmdRng[iEff] ;
    }
    
    // [clip, iEffFree_clip] = max(abs(uCmdCntr))
    clip = fabs(uCmdCntr[0]);
    for(iEff=0 ; iEff<numEff ; iEff++) {
      if(fabs(uCmdCntr[iEff]) > clip){
	indx_clip = iEff ;
	clip = fabs(uCmdCntr[indx_clip]) ;
      }
    }
    
    // Scale uCmdCntr by clip such that it limits all controls
    if (clip > .5) {
      // clip and change variable back to uCmd
      for(iEff=0 ; iEff<numEffFree ; iEff++){
	uCmd[iEffFree[iEff]] = ((uCmdCntr[iEffFree[iEff]]/(2*clip)) * uCmdRng[iEffFree[iEff]]) + uCmdCntr[iEffFree[iEff]] ; 
      }
      
      // Remove clipped control from free variables.
      iEffSat[indx_clip] = 1 ;
    }
    else {
      return;
    }
  }
}*/

/*void ContAllocMOLP(const MatCntrlEff &cntrlEff, const VecObj &vObj, const VecEff &uMin, const VecEff &uMax, const MatObj &wObj, const MatEff &wEff, const VecEff &uPref, VecEff &uCmd, float gammaEff)
{
  // linear mixed optimization control allocation method
  int iObj, iEff;
  float errMax ;
  
  // Size of cntrlEff
  int numEff = cntrlEff.rows();
  int numObj = cntrlEff.cols();
  int lenNew = 2*(numObj + numEff);

  Eigen::Matrix<float, numObj, lenNew> A;
  Eigen::Vector<float, numObj> b;
  Eigen::Vector<float, lenNew> c, x, xMin, xMax;
  
  Eigen::Vector<int, lenNew> wInit ;

  float tmp = 0.0 ;


  // Helper variables
  zerosObj = Eigen::Vector<float, numObj>::Zeros();
  zerosEff = Eigen::Vector<float, numEff>::Zeros();
  onesObj = Eigen::Vector<float, numObj>::Ones();
  onesEff = Eigen::Vector<float, numEff>::Ones();
  eyeObj = Eigen::Matrix<float, numObj, numObj>::Identity();


  // Change of variables, setup MOLP optimization problem
  // c = [ones(2*numObj, 1) ; gammaEff * ones(2*numEff, 1)]
  c << onesObj,
        onesObj,
        gammaEff * onesEff,
        gammaEff * onesEff;
  
  // A = [eye(numObj) , -eye(numObj) , -cntrlEff , cntrlEff]
  A << eyeObj, -eyeObj, -cntrlEff, cntrlEff;
  
  // b = cntrlEff * uPref - vObj
  b = cntrlEff * uPref - vObj;
  

  // Maximum moment error scalar
  errMax = b.abs().sum() ;
  
  // xMin = [zeros(2*(numObj+numEff))] 
  xMin << zerosObj, 
          zerosObj,
          zerosEff,
          zerosEff;
  
  // xMax = [errMax * ones(2*numObj, 1) , uMin , -uMin] 
  xMax << errMax * onesObj,
          errMax * onesObj,
          uMax - uPref,
          uPref - uMin;
  
  // Initialize x, xInit = [max(b, zeros(numObj,1)); max(-b, zeros(numObj,1)); zeros(numEff,1); zeros(numEff,1)]
  x << b.cwiseMax(zerosObj),
       zerosObj.cwiseMax(-b),
       zerosEff,
       zerosEff;
  
  // solve with LP solver
  LinProgSvd(c, A, b, xMin, xMax, wInit, &x);
  
  // Construct uCmd from x
  uPlus = x.segment(2*numObj, numEff);
  uMinus = x.segment(2*numObj+numEff, numEff);

  uCmd = uPlus - uMinus + uPref;
  
  // Construct error from x
  //ePlus = x.segment(0, numObj);
  //eMinus = x.segment(numObj, numObj);

  //e = ePlus - eMinus;
}*/

/*void ContAllocMOQP(const MatCntrlEff &cntrlEff, const VecObj &vObj, const VecEff &uMin, const VecEff &uMax, const MatObj &wObj, const MatEff &wEff, const VecEff &uPref, VecEff &uCmd, float gammaEff)
{
  // quadratic mixed optimization control allocation method
  // Size of cntrlEff
  int numEff = cntrlEff.rows();
  int numObj = cntrlEff.cols();
  int lenNew = (numObj + numEff);

  // Variables for change of variables
  Eigen::Matrix<float, numObj, lenNew> A;
  Eigen::Vector<float, lenNew> b;
  Eigen::Vector<int, numEff> iEffSat;

  // Weighting Factor between two objectives
  float gammaEffSqrt = gammaEff.sqrt();

  // Change Variables
  A << wObj * cntrlEff,
       gammaEffSqrt * wEff; // A = [wObj * cntrlEff ; gammaEffSqrt * wEff]

  b << wObj * vObj,
       gammaEffSqrt * wEff * uPref; // b = [wObj * vObj ; gammaEffSqrt * wEff * uPref]

  // Initialize iEffSat to zeros
  iEffSat.setZero();

  // Initialize uCmd to zeros
  uCmd.setZero();

  // Solve the QP with the QP Phase1 Solver
  QuadProgPhase1(A, b, uMin, uMax, &iEffSat, &uCmd);
}*/


/*void ContAllocSQP(const MatCntrlEff &cntrlEff, const VecObj &vObj, const VecEff &uMin, const VecEff &uMax, const MatObj &wObj, const MatEff &wEff, const VecEff &uPref, VecEff &uCmd)
{
  // quadratic sequential optimization control allocation method
  // Size of cntrlEff
  int numEff = cntrlEff.rows();
  int numObj = cntrlEff.cols();
  int lenNew = (numObj + numEff);
  
  // Variables for change of variables
  MatCntrlEff A1;
  VecObj b1;

  MatEff A2;
  VecEff b2;

  VecEff iEffSat;

  // Setup QP Phase1
  // A1 = wObj * cntrlEff
  A1 = wObj * cntrlEff;

  // b1 = wObj * vObj
  b1 = wObj * vObj;

  // Initialize iEffSat to zeros
  iEffSat.setZero();

  // Initialize uCmd to zeros
  uCmd.setZero();
  
  // Solve Phase1
  QuadProgPhase1(A1, b1, uMin, uMax, &iEffSat, &uCmd);
  
  // Number of Free effectors after phase 1
  numEff_B = (iEffSat==0).sum() ;
  
  // Setup QP Phase2
  if(numEff_B >= numObj) {
    // A2 = wEff
    A2 = wEff;
    
    // b2 = wEff * uPref
    b2 = wEff * uPref;
    
    // Solve Phase2
    QuadProgPhase2(A1, b1, A2, b2, uMin, uMax, &iEffSat, &uCmd);
  }
}*/

// Pseudo-Inverse Solvers
// inputs: A - matrix
//         b - desired objective     
// outputs: x - solution

void SolvPinv(const MatSolv &A, const VecSolvObj &b, VecSolvEff &x)
{
  // Pseduo-Inverse solver using Normal Equations
  //x = A.transpose() * (A*A.transpose()).inverse() * b; // x = A' * inv(A*A') * b
  x = (A.transpose() * A).inverse() * (A.transpose() * b); // x = inv(A' * A) * (A' * b)
}

void SolvPinvLU(const MatSolv &A, const VecSolvObj &b, VecSolvEff &x)
{
  // Pseduo-Inverse solver using LU decomposition
  // LU Decomposition based linear algebra solver
  //x = A.partialPivLu().solve(b); // Partial Pivot LU, A must be square and Invertable
  x = A.fullPivLu().solve(b); // Full Pivot LU
}

void SolvPinvCholesky(const MatSolv &A, const VecSolvObj &b, VecSolvEff &x)
{
  // Pseduo-Inverse solver using Cholesky decomposition
  // Cholesky Decomposition based linear algebra solver
  x = A.ldlt().solve(b); // , A must be Positive or Negative Semidefinite!!
  x = A.llt().solve(b); // , A must be Positive Definite!!
}

void SolvPinvSVD(const MatSolv &A, const VecSolvObj &b, VecSolvEff &x)
{
  // Pseduo-Inverse solver using singular value decomposition
  // SVD Decomposition based linear algebra solver
  x = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b); // Jacobi SVD solver
}

void SolvPinvQR(const MatSolv &A, const VecSolvObj &b, VecSolvEff &x)
{
  // Pseduo-Inverse solver using QR decomposition
  // QR Decomposition based linear algebra solver
  //x = A.HouseholderQr().solve(b); // Householder QR solver, fast but can be unstable
  x = A.colPivHouseholderQr().solve(b); // Columnwise Householder QR solver, slower but accurate
  //x = A.FullPivHouseholderQr().solve(b); // Householder QR solver, slowest most stable
}


// Quadratic Programming Solvers
// inputs: A - matrix
//         b - desired objective
//         xMin - lower bound for solution
//         xMax - upper bound for solution
// outputs: x - solution

/*void QuadProgPhase1(const MatCntrlEff &A, const VecObj &b, const VecObj &xMin, const VecObj &xMax, const VecEffInt &iEffSat, VecObj &x, float tol)
{
// Quadratic Program solver - phase 1

// iEffSat - working vector indicating bound surface deflections
//                iEffSat = -1 for x == xMin
//                iEffSat =  0 for xMin < x < xMax
//                iEffSat =  1 for x == xMax
  
  int iObj, iEff, iIter, iAlpha, iAlphaFree, lambdaNeg, iObj_neg ;
  float tmp, alpha;
  int numEffFree;
  
  Eigen::Array<bool, numEff> iEffFree;

  MatCntrlEff AFree;
  VecObj bResid;
  VecObj pFree;
  VecObj stepDist;
  VecObj lambda;
  
  int numIter = 2*numEff + 1;
  
  // Initialize solution residual
  bResid = b - A * x;
  
  // Determine indices of free variables
  iEffFree = iEffSat==0;

  // Solve Iteratively
  for(iIter = 0 ; iIter < numIter ; Iter++) {
    // Update 'AFree', AFree = A(:, iEffFree);
    AFree = isolateColumns (A, iEffFree);

    // Number of free variables
    numEffFree = (iEffFree == 1).sum();

    // Solve the P-Inv using only the active set
    //SolvPinvSVD(AFree, bResid, &pFree);
    SolvPinvQR(AFree, bResid, &pFree);

    // Apply the solved pertubations into the full solution vector
    p.setZero();
    ExpandVecElem(&pFree, &iEffFree, *p);

    // Temp update solution
    xOpt = x + p;

    // Optimal Feasible ??
    feas = (xOpt.array() >= xMin.array()) & (xOpt.array() <= xMax.array());

    if (feas.all()) { // Solution is completely within constraints
      // Solution is feasible, check optimality

      // Update solution and residual
      x = xOpt;
      bResid = bResid - AFree * pFree;

      if (numEffFree < numObj) {
        // Compute gradient of criteria
        g = -A.transpose() * bResid; // gradient

        // Compute Lagrange multipliers, for all bounds not just active set
        lambda = -iEffSat.cast<float>().array() .* g.array();

        if (lambda >= -tol) {
          // All Lagrange multipliers are positive, done
          return 1;
        }
        else {
          // Solution is not optimal, remove an active constraint
          // Remove most negative lambda constraint
          lambdaNeg = lambda.minCoef(&iLamdaNeg);
          iEffFree(iLamdaNeg) = 0;
          iEffSat(iLamdaNeg) = 1;
        }
      }
    }
    else {
      // Solution is not feasible
      // Compute distance to boundaries
      distBound.setOnes(); // Initialize to one

      iMin = iEffFree & p < -tol
      iMax = iEffFree & p > tol

      distBoundMin = (xMin - x).array() / p.array();
      distBoundMax = (xMax - x).array() / p.array();

      distBound = (iMin).select(distBoundMin, iMin);
      distBound = (iMax).select(distBoundMax, iMax);

      // Proportion of p to travel
      alpha = dist.minCoeff(&iAlpha);

      // Update the solution point and residual
      x = x + alpha * p;
      bResid = bResid - AFree * alpha * pFree;

      // Add constraint to working set
      iEffSat(iAlpha) = sign(p(iAlpha));
      iEFfFree(iAlpha) = 0;
    }
  }
}*/


/*void QuadProgPhase2(const MatCntrlEff &A1, const VecObj &b1, const MatCntrlEff &A2, const VecObj &b2, const VecObj &xMin, const VecObj &xMax, VecEffInt &iEffSat, VecObj &x, float tol)
{
// Quadratic Program solver - phase 2

// iEffSat - working vector indicating bound surface deflections
//                iEffSat = -1 for x == xMin
//                iEffSat =  0 for xMin < x < xMax
//                iEffSat =  1 for x == xMax

  int iObj, iEff, iIter, iAlpha, iEffFree_alpha, lambdaNeg, iObj_neg;
  float tmp, alpha;
  int numObj_c, numEffFree, numEff_fixed ;
  
  VecObj iEffFree;
  VecObj iEffFixed;
  VecObj bResid;
  VecObj p;
  VecObj zeros;
  VecObj stepDist;
  VecObj g;
  VecObj lambda;
  VecObj Lambda;
  float ET; //ET[MAX_M+MAX_N]
  
  int numIter = 2*numEff + 1;
  
  
  // Initialize solution residual: bResid = b2 - A2 * x
  bResid = b2 - A2 * x
  
  
  for(iIter=1 ; iIter<=1 ; iIter++){
    printf("phase 2 iIter# %d\numEff",iIter);
    // iEffFree from iEffSat
    numEffFree = 0 ;
    numEff_fixed = 0 ;
    for(iEff=0 ; iEff<numEff ; iEff++) {
      if(iEffSat[iEff]==0) {
	iEffFree[numEffFree] = iEff;
	numEffFree++;
      }
      else {
	iEffFixed[numEff_fixed] = iEff;
	numEff_fixed++;
      }
    }
    
    numObj_c = numObj + numEff_fixed;
    
    // ET = E' = [A1 ; CO]'
    for(iObj=0 ; iObj<numObj_c ; iObj++){
      if(iObj<numObj){
	     for(iEff=0 ; iEff<numEff ; iEff++) ET[iEff][iObj] = A1[iObj][iEff];
      }
      else{
	     for(iEff=0 ; iEff<numEff ; iEff++){
	  if((iEff)==(iEffFixed[iObj-numObj])) {
	    ET[iEff][iObj] = -iEffSat[iEffFixed[iObj-numObj]];
	  }
	  else{
	    ET[iEff][iObj] = 0.0;
	  }
	}
      }
    }
    
    // p = 0 , stepDist = 1
    for(iEff=0 ; iEff<numEff ; iEff++){
      p[iEff] = 0.0 ;
      zeros[iEff] = 0.0 ;
      stepDist[iEff] = 1.0 ;
    }
    
    
    SolvPinvSVD(ET, zeros, numEff, numEff, p);
    
	
    // Compute maximimum step length in search direction, alpha
    for(iEff=0 ; iEff<numEffFree ; iEff++){
      if(p[iEffFree[iEff]] > 0.0){
        stepDist[iEffFree[iEff]] = (xMax[iEffFree[iEff]] - x[iEffFree[iEff]]) / p[iEffFree[iEff]] ;
      }
      else if(p[iEffFree[iEff]] < 0.0){
        stepDist[iEffFree[iEff]] = (x[iEffFree[iEff]] - xMin[iEffFree[iEff]]) / -p[iEffFree[iEff]] ;
      }
      else{
        stepDist[iEffFree[iEff]] = 1.0 ;
      }
    }
    
    // get alpha, iAlpha as minimum of stepDist 
    iEffFree_alpha = min_dvec(stepDist, numEffFree) ;
    
    iAlpha = iEffFree[iEffFree_alpha] ;
    alpha = stepDist[iEffFree_alpha] ;
    
    if(alpha > 1.0) alpha = 1.0;
    
    // Update solution: xFree = xFree + alpha*p
    for(iEff=0 ;iEff<numEffFree ; iEff++){
      x[iEffFree[iEff]] += alpha*p[iEff] ;
    }
    
    
    // Update residual: bResid = bResid-A2Free*alpha*p
    for(iObj=0 ; iObj<numEff ; iObj++){
      tmp = 0.0 ;
      for(iEff=0 ; iEff<numEffFree ; iEff++) tmp += A2[iObj][iEffFree[iEff]] * p[iEff];
      
      bResid[iObj] += -alpha*tmp ;
    }
    
    
    if((alpha >= 1.0) | (iIter==1)) { // check for optimality
      
      // g = -A2'*bResid
      for(iObj=0 ; iObj<numEff ; iObj++) {
        tmp = 0.0;
        for(iEff=0 ; iEff<numEff ; iEff++) {
          tmp += A2[iEff][iObj]*bResid[iEff];
        }
	      
        g[iObj] = -tmp;
      }
      
      // Compute Lagrangian multipliers
      SolvPinvSVD(ET, g, numObj+numEff, numEff, Lambda);
            
      // lambda(iEffFixed) = Lambda(numObj+1:end)
      lambdaNeg = 0 ;
      for(iObj=0 ; iObj<numEff ; iObj++) lambda[iObj] = 0.0;
      for(iObj=0 ; iObj<numEff_fixed ; iObj++) {
	      lambda[iEffFixed[iObj]] = Lambda[iObj+numObj] ;
	
	// Test for negative lambda
	if(lambda[iEffFixed[iObj]] <= -EPS_QP2SVD) lambdaNeg = 1 ;
      }
      //printf("lambda = \numEff");
      //print_dvec(lambda, numEff);
      
      // Exit if all lambda are non-negative	
      if(lambdaNeg == 0) return;
      
      // Remove constraint with most negative lambda
      iObj_neg = min_dvec(lambda, numEff) ;
      //printf("iObj_neg = %d\numEff", iObj_neg);
      
      //lambdaNeg = lambda[iObj_neg] ;
      iEffSat[iObj_neg] = 0 ;
    }
    else{ // 0<=alpha<1 
      // Add alpha constraint to working set
      if(p[iEffFree_alpha] > 0){
	iEffSat[iAlpha] = 1 ;
      }
      else{
	iEffSat[iAlpha] = -1 ;
      }
    }
  }
}*/


// General Functions
void Saturate(const VecEff &uMin, const VecEff &uMax, VecEff &uCmd)
{
  // Saturate the uCmd to be between uMin and uMax
  uCmd = uCmd.cwiseMin(uMax).cwiseMax(uMin);
}

int SaturateIndex(const VecEff &uMin, const VecEff &uMax, VecEff &uCmd, VecEffInt &iEffSat)
{
  int len = uCmd.size();
  int numSat = 0;

  for(int i = 0 ; i < len ; i++) {
    if (uCmd[i] >= uMax[i]) {
      uCmd[i] = uMax[i];
      iEffSat[i] = 1;
      numSat++;
    } else if (uCmd[i] <= uMin[i]) {
      uCmd[i] = uMin[i];
      iEffSat[i] = -1;
      numSat++;
    } else {
      iEffSat[i] = 0;
    }
  }

  return numSat;
}

int FindFree(const VecEffInt &iEffSat, VecEffInt &iEffFree)
{
  int len = iEffSat.size();
  int num = 0;
  
  for(int i = 0 ; i < len ; i++){
    if (iEffSat[i] == 0) {
      iEffFree[i] = 1;
    } else {
      iEffFree[i] = 0;
    }

    num++;
  }
}

void ShrinkMatColumns(const MatCntrlEff &M, const VecEffInt &elemKeep, MatCntrlEff &MShrink)
{
  // Copy only colums of a matrix
  int numRows = M.rows();
  int numCols = M.cols();

  int numElemKeep = elemKeep.sum();

  int i = 0;
  int iKeep = 0;

  for (i = 0; i < numCols; i++) {
    if( elemKeep[i] == 1 ) {
      MShrink.col(iKeep) = M.col(i);
      iKeep++;
    }
  }

  MShrink.conservativeResize(numRows, numElemKeep);
}

void ShrinkVecElem(const VecEff &V, const VecEffInt &elemKeep, VecEff &VShrink)
{
  // Copy only elements of vector
  int num = V.size();

  int numKeep = elemKeep.sum();

  int i = 0;
  int iKeep = 0;

  for (i = 0; i < num; i++) {
    if( elemKeep[i] == 1 ) {
      VShrink(iKeep) = V(i);
      iKeep++;
    }
  }

  VShrink.conservativeResize(numKeep);
}

void ExpandMatColumns(const MatCntrlEff &M, const VecEffInt &elemKeep, MatCntrlEff &MExp)
{
  // Copy only colums of a matrix
  int numRows = M.rows();
  int numCols = M.cols();

  MExp.Zero(numRows, numCols);

  int numExp = elemKeep.size();

  int i = 0;

  for (i = 0; i < numExp; i++) {
    if( elemKeep[i] == 1 ) {
      MExp.col(i) = M.col(i);
    } else {
      MExp.col(i).setZero();
    }
  }

  MExp.conservativeResize(numRows, numExp);
}

void ExpandVecElem(const VecEff &V, const VecEffInt &elemKeep, VecEff &VExp)
{
  // Expand a Vector
  int num = V.size();

  VExp.Zero(num);

  int numExp = elemKeep.size();
  int i = 0;
  int iGrow;

  for (i = 0; i < numExp; i++) {
    if( elemKeep[i] == 1 ) {
      VExp(i) = V(i);
    } else {
      VExp(i) = 0;
    }
  }

    VExp.conservativeResize(numExp);
}
