#ifndef OUTLIER_DETECTION_H
#define OUTLIER_DETECTION_H

#include <ros/ros.h>
#include <ros/package.h>
#include <ros/console.h>

#include <assert.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>
#include <vector>
#include <stdlib.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_listener.h>

const int nMaxRecoursion = 3;

// instance of "rror_evaluation_function"
static double distance_point_to_point(double *, double *);

// instance of "model_generating_function"
static bool model_one_point(double *, double *);

// evaluate a given model over the data, and fill a mask with the
// inliers (according to the given allowed error).  This function returns the
// number of inliers.
int outlier_detection_trial(
		bool *,        // array mask identifying the inliers
		double *,      // array of input data
		double *,      // parameters of the model
		double ,       // maximum allowed error to be inlier
		int ,          // dimension of each data point
		int ,          // number of data points
		double *,      // final error of the model
		int      	   //index of the point taken as model  
		);
//
// Given a point, it is considered as the model that fits all the dataset
// Several models are tried, and the model with the highest
// number of inliers and le smaller total error is kept.
//
// A basic idea of this kind of algorithm is that a maximum allowed error is fixed
// by hand, and then the inliers of a model are defined as the data points
// which fit the model up to the allowed error.  The algorithm
// tries all the possible models and keeps the one with the largest number of inliers 
// and minimum total error.
// if no model is found then the algorithm is called again with an higher maximu error
// this error is calculated based on the previous best total error
// this recoursion is computed a maximum amount of time, if no model is found then the all
// dataset is returned
std::vector<geometry_msgs::Pose> outlier_detection(
		std::vector<geometry_msgs::Pose> , 	// input data
		int ,       						// number of models to try
		double,      						// maximum allowed error to consider a point inlier
		int 								// the grade of recoursion
		);

//Return the average Pose calculate with only the inliers points calculate with outlier_detection function
geometry_msgs::Pose averageOnlyInlier(
	    std::vector<geometry_msgs::Pose> , // input data
	    int ,                              // number of models to try
	    double                             // maximum allowed error to consider a point inlier
	    );

#endif
