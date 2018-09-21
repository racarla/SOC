#include "filter-algorithms.hxx"
#include <iostream>

using namespace std;

int main(void) {
  __GeneralFilter filter;
  float y;
  vector<float> x{0.20368,0.16302,-0.094846,0.039093,-0.093058,-0.28765,-0.30204,-0.29307,-0.24682,-0.29942,-0.5535,-0.40014,-0.45079,-0.61325,-0.57608,-0.7791,-0.74429,-0.65252,-0.71158,-0.69427,-0.79097,-0.96288,-0.77253,-0.76034,-0.82918,-0.81044,-0.81107,-0.89176,-0.81493,-0.92105,-0.76849,-0.9144,-0.82676,-0.85448,-0.80829,-0.5899,-0.58204,-0.63342,-0.42921,-0.60955,-0.45737,-0.41829,-0.26685,-0.20213,-0.2953,-0.15929,-0.10891,0.0035769,0.082285,0.15694,0.10073,0.26498,0.32178,0.26096,0.31148,0.46661,0.64087,0.54332,0.65999,0.62301,0.80598,0.73054,0.83918,0.93052,1.0185,1.0724,1.0028,0.93065,0.95968,1.0094,1.174,1.0424,1.1934,1.0577,1.2322,1.0864,1.043,1.0476,1.1258,1.0732,1.0221,1.1173,1.0278,0.98716,1.0439,0.84761,0.92389,0.87851,0.7379,0.73486,0.5596,0.49968,0.56249,0.56645,0.54554,0.28362,0.33146,0.24394,0.066399,0.084281};
  vector<float> b{0.065,-0.065};
  vector<float> a{1.0, -0.9608};
  filter.Configure(b, a);
  for (auto val : x) {
    y = filter.Run(val);
    cout << y << endl;
  }
  return 0;
}
