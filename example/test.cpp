#include "dual_robotarm_calibration.h"
#include <unistd.h>
#include <vector>
#include <array>
#include <iostream>
#include <string>			//string lib
#include <cmath>

using namespace std;
using namespace Eigen;

int main()
{

    dual_robotarm_calibration drc;

    vector<Vector3d> m;
    vector<Vector3d> s;
    Isometry3d m2s;
    bool b = drc.Three_points_calibration(m,s,m2s);
	return 0;
}
