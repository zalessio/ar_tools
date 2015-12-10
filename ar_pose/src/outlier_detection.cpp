#ifndef OUTLIER_DETECTION_CPP
#define OUTLIER_DETECTION_CPP

#include "ar_pose/outlier_detection.h"

double distance_point_to_point(double *model, double *point)
{
	double alpha = 0.06;
	double dx      = model[0] - point[0];
	double dy      = model[1] - point[1];
	double dz      = model[2] - point[2];
	double position_distance = std::sqrt(dx*dx + dy*dy + dz*dz);
	double dotProd = model[3]*point[3] + model[4]*point[4] + model[5]*point[5] + model[6]*point[6];
	double orientation_distance = alpha*std::acos(2*dotProd*dotProd -1);

	return  position_distance + orientation_distance;
}

bool model_one_point(double *model, double *point)
{
	model[0] = point[0];
	model[1] = point[1];
	model[2] = point[2];
	model[3] = point[3];
	model[4] = point[4];
	model[5] = point[5];
	model[6] = point[6];
	// assert that the line goes through the two points
	return true;
}

int outlier_detection_trial(bool *out_mask, double *data, double *model, double max_error, int datadim, int nDataPoint, double* err_outlier, int modelIndex)
{
	int cx = 0;
	err_outlier[0] = 0;
	for (int i = 0; i < nDataPoint; i++)
	{	
		if(i == modelIndex){
			out_mask[i] = 1;
			cx += 1;
			continue;
		}

		double *datai = data + i*datadim;
		double e;

		e = distance_point_to_point(model, datai);
		
		if (!(e >= 0)){ 
			ROS_DEBUG("WARNING e = %f \n", e);
			ROS_DEBUG("data %f %f %f %f %f %f %f",datai[0],datai[1],datai[2],datai[3],datai[4],datai[5],datai[6]);
		}
		assert(e >= 0);	

		if (e < max_error){
			out_mask[i] = 1;
			cx += 1;
		}else{
			out_mask[i] = 0;
			err_outlier[0] += e;
		}
	}
	return cx;
}

std::vector<geometry_msgs::Pose> outlier_detection(std::vector<geometry_msgs::Pose> inputData, int nTrials, double max_error, int recoursionIndex)
{
	int nDataPoint = inputData.size();
	int datadim = 7;
    int modeldim = 7;

	int best_ninliers = 0;
	double best_error = 1e20;
	bool best_mask[nDataPoint];
    double data[nDataPoint*datadim];

	for(int i = 0; i< nDataPoint; i++){
		data[datadim*i + 0] = inputData[i].position.x;
		data[datadim*i + 1] = inputData[i].position.y;
		data[datadim*i + 2] = inputData[i].position.z;
		data[datadim*i + 3] = inputData[i].orientation.x;
		data[datadim*i + 4] = inputData[i].orientation.y;
		data[datadim*i + 5] = inputData[i].orientation.z;
		data[datadim*i + 6] = inputData[i].orientation.w;
	}

	for (int i = 0; i < nTrials; i++){

        int modelIndex =  i%nDataPoint;
		double model[modeldim];
		double x[datadim];
		for(int k = 0; k < datadim; k++)
			x[k] = data[datadim*modelIndex + k]; 

		bool nm = model_one_point(model,x);
		
		if (nm){
			bool tmp_mask[nDataPoint];
			double tmp_err_outlier[1];
			tmp_err_outlier[0] = 0;
			int tmp_n_inliers = outlier_detection_trial(tmp_mask, data, model, max_error, datadim, nDataPoint, tmp_err_outlier,modelIndex);

			if(tmp_err_outlier[0] < best_error && tmp_n_inliers >= best_ninliers){
				best_ninliers = tmp_n_inliers;
				best_error = tmp_err_outlier[0];
				for(int k = 0; k < nDataPoint; k++)
					best_mask[k] = tmp_mask[k];
			}
		}
	}
	
	//ROS_INFO("data: %d",nDataPoint);
    //ROS_INFO("inliers: %d",best_ninliers);
    //ROS_INFO("error: %f",best_error);

	if(best_ninliers > 1){
		std::vector<geometry_msgs::Pose> returnData;
		for(int k = 0; k < nDataPoint; k++)
			if(best_mask[k])
				returnData.push_back(inputData[k]);	
		return returnData;
	}else{
		if(recoursionIndex < nMaxRecoursion){
			recoursionIndex++;
			ROS_DEBUG("OUTLIER DETECTION RECOURSION %d", recoursionIndex);
			return outlier_detection(inputData, nTrials, best_error/(1.8*nDataPoint),recoursionIndex);
		}
		ROS_DEBUG("OUTLIER DETECTION DIDN'T FIND A MODEL!");
		return inputData;
	}

}

geometry_msgs::Pose averageOnlyInlier(std::vector<geometry_msgs::Pose> inputData, int nTrials, double maxErrToBeInlier)
{

	std::vector<geometry_msgs::Pose> inliersData;
	geometry_msgs::Pose averagePoseInliers;
	int recoursionIndex = 0;

	// read input data size
	int nDataPoint = inputData.size();
	if(nDataPoint <= 2){
		inliersData = inputData;
	}else{
		// call the ransac function to fit a model to data
		inliersData = outlier_detection(inputData, nTrials, maxErrToBeInlier,recoursionIndex);
    }

	double mean_x    = 0;
    double mean_y    = 0;
    double mean_z    = 0;
    double mean_roll_sin = 0;
    double mean_pitch_sin = 0;
    double mean_yaw_sin  = 0;
    double mean_roll_cos = 0;
    double mean_pitch_cos = 0;
    double mean_yaw_cos  = 0;

	for (int i = 0; i < inliersData.size(); i++){
		tf::Quaternion q(inliersData[i].orientation.x,inliersData[i].orientation.y,inliersData[i].orientation.z,inliersData[i].orientation.w);
		tf::Matrix3x3 m(q);
		double roll, pitch, yaw;
		m.getRPY(roll, pitch, yaw);

	    mean_x    += inliersData[i].position.x;
        mean_y    += inliersData[i].position.y;
        mean_z    += inliersData[i].position.z;
        mean_roll_sin  += std::sin(roll);
        mean_pitch_sin += std::sin(pitch);
        mean_yaw_sin   += std::sin(yaw);
        mean_roll_cos  += std::cos(roll);
        mean_pitch_cos += std::cos(pitch);
        mean_yaw_cos   += std::cos(yaw);
	}

    double sizeInliers = (double) inliersData.size();
	averagePoseInliers.position.x = mean_x/sizeInliers;
    averagePoseInliers.position.y = mean_y/sizeInliers;
    averagePoseInliers.position.z = mean_z/sizeInliers;

    tf::Quaternion q = tf::createQuaternionFromRPY(std::atan2(mean_roll_sin,mean_roll_cos),std::atan2(mean_pitch_sin,mean_pitch_cos),std::atan2(mean_yaw_sin,mean_yaw_cos));
    averagePoseInliers.orientation.x = q.x();
    averagePoseInliers.orientation.y = q.y();
    averagePoseInliers.orientation.z = q.z();
    averagePoseInliers.orientation.w = q.w();

	return averagePoseInliers;
}

#endif