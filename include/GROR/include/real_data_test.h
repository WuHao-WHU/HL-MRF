#pragma once
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>
#include <pcl/visualization/pcl_visualizer.h>

void evaluateTransfromError(Eigen::Matrix4f estamate_transform, Eigen::Matrix4f refrence_transform, double &deltT, double &deltR);

void evaluatePrecisionAndRecall(pcl::Correspondences gt_inliers, pcl::Correspondences est_inliers, pcl::Correspondences &recall_inliers, pcl::Correspondences &recall_outliers, double &precision, double &recall);

int getGroundTruthLiners(pcl::PointCloud<pcl::PointXYZ>::Ptr issS, pcl::PointCloud<pcl::PointXYZ>::Ptr issT, pcl::Correspondences corr, pcl::Correspondences &ground_truth_inliers, Eigen::Matrix4f &truth, double res);

bool getCloudPairAndGroundTruth(std::string dataset, std::string data_name, pcl::PointCloud<pcl::PointXYZ>::Ptr origin_cloudS, pcl::PointCloud<pcl::PointXYZ>::Ptr origin_cloudT, Eigen::Matrix4f &truth);

void saveResult(std::string fnameOut, Eigen::Matrix4f result);