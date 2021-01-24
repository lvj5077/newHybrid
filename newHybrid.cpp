#include <sys/types.h>
#include <sys/stat.h>
#include <libgen.h>
#include <stdio.h>
#include <stdlib.h>
#include <cmath>
#include <math.h>
#include <cassert>
#include <cstring>
#include <eigen3/Eigen/Dense>

#include <stdlib.h>

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <cstdlib>
#include <deque>
#include <map>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
using namespace Eigen;
using namespace std;


#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <eigen3/Eigen/Dense>
#include <Eigen/Geometry>


#include <Eigen/Dense>

#include "glog/logging.h"

// #include "sophus/se3.hpp"
// #include "sophus/so3.hpp"


using namespace Eigen;
using namespace std;
using namespace ceres;

struct ReprojectionErrorNum
{
	ReprojectionErrorNum(double img_fix_x, double img_fix_y,double img_mv_x, double img_mv_y)
		:img_fix_x(img_fix_x), img_fix_y(img_fix_y), img_mv_x(img_mv_x), img_mv_y(img_mv_y)
		{}

    template <typename T>
	bool operator()(const T* const camera_R, const T* const camera_T,const T* const depth_fix, T* residuals) const
	{

        T p_mv[3];
        T p_fx[3];

        p_fx[0] = T(img_fix_x)*depth_fix[0];
        p_fx[1] = T(img_fix_y)*depth_fix[0];
        p_fx[2] = depth_fix[0];

        ceres::QuaternionRotatePoint(camera_R, p_fx, p_mv);
        p_mv[0] += camera_T[0]; 
        p_mv[1] += camera_T[1]; 
        p_mv[2] += camera_T[2];
        T xp = p_mv[0] / p_mv[2];
        T yp = p_mv[1] / p_mv[2];

	    residuals[0] = (xp - T(img_mv_x));
	    residuals[1] = (yp - T(img_mv_y));

    	return true;
	}

	static ceres::CostFunction* Create(const double img_fix_x, const double img_fix_y, const double img_mv_x, const double img_mv_y) 
	{
	  return (new ceres::AutoDiffCostFunction<
	          ReprojectionErrorNum, 2, 4, 3, 1>(
	          	new ReprojectionErrorNum(img_fix_x,img_fix_y,img_mv_x, img_mv_y)));
	}

	double img_fix_x;
	double img_fix_y;

    double img_mv_x;
    double img_mv_y;

    // double objectWeight;

};



int main(int argc, char** argv) {


    int kNumObservations = 0;


    ceres::Problem problem;

	ceres::LocalParameterization* local_parameterization = new ceres::QuaternionParameterization();

	double c_rotation[4];
	double c_translation[3];

    // rot      0.2 0.2 0.2
    // trans    0.1 0.2 0.3
    c_rotation[0] =  1.0;
    c_rotation[1] =  0.0;
    c_rotation[2] =  0.0;
    c_rotation[3] =  0.0;

    c_translation[0] = 0.0;
    c_translation[1] = 0.0;
    c_translation[2] = 0.0;

    

    problem.AddParameterBlock(c_rotation, 4, local_parameterization);
    problem.AddParameterBlock(c_translation, 3);



    
    const double ddpairs[] = {0.1641, 0.2466, 1.1733, 0.3529, 0.2316
, 0.0610,-0.1424, 0.9296, 0.3367,-0.0538
,-0.1087,-0.3089, 0.9517, 0.2256,-0.2151
,-0.3950,-0.3724, 1.1099,-0.0007,-0.3342
, 0.5008, 0.4861, 0.5594, 0.5458, 0.5159
,-0.1991,-0.3608, 0.8158, 0.1680,-0.2304
, 0.0848,-0.0878, 1.2727, 0.3476,-0.0580
, 0.2016, 0.0433, 1.1964, 0.4245, 0.0783
,-0.6320,-0.4334, 0.6253,-0.1007,-0.2558
,-0.5905, 0.2252, 0.6302,-0.1335, 0.1655
, 0.0836,-0.0262, 0.5924, 0.3346, 0.1194
,-0.0291,-0.6856, 0.5078, 0.3556,-0.3033
, 0.4230, 0.3054, 0.9231, 0.5530, 0.3480
, 0.2587,-0.3456, 1.1556, 0.5622,-0.2427
, 0.1916,-0.1684, 1.2229, 0.4603,-0.1042
,-0.4351,-0.2547, 1.0312,-0.0395,-0.2278
,-0.7015, 0.0507, 0.6088,-0.1792, 0.0530
,-0.3636,-0.3609, 1.1318, 0.0210,-0.3255
, 0.4762,-0.1511, 0.6265, 0.6621, 0.0833
, 0.6984,-0.6230, 0.6343, 0.9851,-0.2625
, 0.3069,-0.6477, 0.5986, 0.6223,-0.3055
,-0.5731, 0.4430, 0.6420,-0.1434, 0.2905
, 0.3333,-0.3119, 0.6683, 0.5838,-0.0732
,-0.5596, 0.1487, 0.6962,-0.1220, 0.1086
,-0.4679, 0.5681, 0.8175,-0.1211, 0.3577
, 0.1724,-0.0827, 0.8164, 0.4158, 0.0332
,-0.2386, 0.2714, 0.7176, 0.0664, 0.2293
, 0.2048, 0.3437, 0.7510, 0.3654, 0.3484
, 0.1789,-0.0484, 1.3929, 0.4257,-0.0216
, 0.0691, 0.1292, 1.2032, 0.2945, 0.1218
, 0.2274,-0.3696, 1.0557, 0.5331,-0.2486
,-0.3874, 0.6338, 0.6844,-0.0592, 0.4217
, 0.3300,-0.4389, 0.7120, 0.6144,-0.1917
, 0.8151,-0.4050, 0.5773, 1.0083,-0.0348
, 0.2595, 0.2107, 1.4138, 0.4452, 0.2065
,-0.3429,-0.0103, 1.2067,-0.0114,-0.0525
,-0.1263, 0.2543, 1.0578, 0.1243, 0.1931
,-0.1608,-0.1278, 0.8134, 0.1637,-0.0488
, 0.2777,-0.3408, 0.6662, 0.5432,-0.1022
, 0.0873,-0.4123, 1.1225, 0.4137,-0.3186};




    kNumObservations = sizeof(ddpairs)/sizeof(ddpairs[0]) /5;   //40 
    double depth_fix[kNumObservations][1];

    // kNumObservations = 10;

    for (int i = 0; i < kNumObservations; ++i) {
        depth_fix[i][0] = 1;
        problem.AddParameterBlock(depth_fix[i], 1); 
        // if (ddpairs[5*i+2] >0){
        if(i>30){
            depth_fix[i][0] = ddpairs[5*i+2];
            problem.SetParameterBlockConstant(depth_fix[i]);
        }

        ceres::CostFunction* cost_function = ReprojectionErrorNum::Create(ddpairs[5*i],ddpairs[5*i+1],ddpairs[5*i+3],ddpairs[5*i+4]);
        problem.AddResidualBlock(cost_function, NULL, c_rotation, c_translation, depth_fix[i]); 
    }


    // problem.SetParameterBlockConstant(c_rotation);
    ceres::LossFunction *loss_function;
        //loss_function = new ceres::HuberLoss(1.0);
    // loss_function = new ceres::CauchyLoss(1.0);
    ceres::Solver::Options options;
    options.max_num_iterations = 250;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.BriefReport() << "\n";

    std::cout <<c_rotation[0]<<" "<<c_rotation[1]<<" "<<c_rotation[2]<<" "<<c_rotation[3] << "\n";
    std::cout <<c_translation[0] <<" "<<c_translation[1] <<" "<<c_translation[2]<< "\n";

    for (int i = 0; i < kNumObservations; ++i) {
        std::cout <<depth_fix[i][0] - ddpairs[5*i+2] << "\n"; 
    }

    return 0;
}