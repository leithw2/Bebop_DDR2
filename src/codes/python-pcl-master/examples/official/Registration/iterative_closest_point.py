# -*- coding: utf-8 -*-
# How to use iterative closest point
# http://pointclouds.org/documentation/tutorials/iterative_closest_point.php#iterative-closest-point

import pcl
import random
import numpy as np

import cv2
import matplotlib.pyplot as plt
from numpy import save
from numpy import load

from mpl_toolkits import mplot3d

# from pcl import icp, gicp, icp_nl


def main():
    cloud_in = pcl.PointCloud()
    cloud_out = pcl.PointCloud()

    my_cloud_points = load('data_points.npy')

    ax = plt.axes(projection='3d')
    ax.set_xlim3d(-1, 1)
    ax.set_ylim3d(-1, 1)
    ax.set_zlim3d(-1, 1)
    ax.scatter(my_cloud_points[:,0], my_cloud_points[:,1], my_cloud_points[:,2], s=0.1)

    # Fill in the CloudIn data
    # cloud_in->width    = 5;
    # cloud_in->height   = 1;
    # cloud_in->is_dense = false;
    # cloud_in->points.resize (cloud_in->width * cloud_in->height);
    # for (size_t i = 0; i < cloud_in->points.size (); ++i)
    # {
    #   cloud_in->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
    #   cloud_in->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
    #   cloud_in->points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
    # }


    points_in = np.zeros((500, 3), dtype=np.float32)
    RAND_MAX = 1024.0
    for i in range(0, 500):
        points_in[i][0] = 1024 * random.random() / RAND_MAX
        points_in[i][1] = 1024 * random.random() / RAND_MAX
        points_in[i][2] = 1024 * random.random() / RAND_MAX

    my_cloud_points= my_cloud_points.astype(np.float32)

    my_cloud_points = np.array([my_cloud_points[:,0], my_cloud_points[:,1], my_cloud_points[:,2]]).T
    print(my_cloud_points.shape)
    cloud_in.from_array(my_cloud_points)

    # std::cout << "Saved " << cloud_in->points.size () << " data points to input:" << std::endl;
    # for (size_t i = 0; i < cloud_in->points.size (); ++i) std::cout << "    " <<
    #   cloud_in->points[i].x << " " << cloud_in->points[i].y << " " <<
    #   cloud_in->points[i].z << std::endl;
    # *cloud_out = *cloud_in;
    print('Saved ' + str(cloud_in.size) + ' data points to input:')
    points_out = np.zeros((8107,3 ), dtype=np.float32)

    # std::cout << "size:" << cloud_out->points.size() << std::endl;
    # for (size_t i = 0; i < cloud_in->points.size (); ++i)
    # cloud_out->points[i].x = cloud_in->points[i].x + 0.7f;

    # print('size:' + str(cloud_out.size))
    # for i in range(0, cloud_in.size):
    print('size:' + str(my_cloud_points.size))
    print(points_out.size)
    for i in range(0, cloud_in.size-1):
        points_out[i][0] = my_cloud_points[i][0]
        points_out[i][1] = my_cloud_points[i][1]
        points_out[i][2] = my_cloud_points[i][2]

    #print(points_out.T)
    points_out = np.concatenate([points_out, np.ones((points_out.shape[0],1),dtype=points_out.dtype)], axis=1)

    points_out = rotZ(points_out, np.pi/5)

    points_out = np.array([points_out[:,0], points_out[:,1], points_out[:,2]]).T

    cloud_out.from_array(points_out.astype(np.float32))

    # std::cout << "Transformed " << cloud_in->points.size () << " data points:" << std::endl;
    print('Transformed ' + str(cloud_in.size) + ' data points:')

    # for (size_t i = 0; i < cloud_out->points.size (); ++i)
    #   std::cout << "    " << cloud_out->points[i].x << " " << cloud_out->points[i].y << " " << cloud_out->points[i].z << std::endl;
    for i in range(0, cloud_out.size):
        print('     ' + str(cloud_out[i][0]) + ' ' + str(cloud_out[i]
                                                         [1]) + ' ' + str(cloud_out[i][2]) + ' data points:')

    # pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    # icp.setInputCloud(cloud_in);
    # icp.setInputTarget(cloud_out);
    # pcl::PointCloud<pcl::PointXYZ> Final;
    # icp.align(Final);
    icp = cloud_in.make_IterativeClosestPoint()
    # Final = icp.align()
    converged, transf, estimate, fitness = icp.icp(cloud_in, cloud_out)

    points_out = np.concatenate([points_out, np.ones((points_out.shape[0],1),dtype=points_out.dtype)], axis=1)
    points_out2 = np.linalg.inv(transf).dot(points_out.T).T
    #points_out2 = points_out.dot(np.linalg.inv(transf))
    print(points_out[0].shape)
    ax.scatter(points_out[:,0], points_out[:,1], points_out[:,2], s=0.1)

    ax.scatter(points_out2[:,0], points_out2[:,1], points_out2[:,2], s=.1)

    plt.show()
    # std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
    # std::cout << icp.getFinalTransformation() << std::endl;
    # print('has converged:' + str(icp.hasConverged()) + ' score: ' + str(icp.getFitnessScore()) )
    # print(str(icp.getFinalTransformation()))
    print('has converged:' + str(converged) + ' score: ' + str(fitness))
    print(str(transf))

def rotZ(cloud, theta):

    cos = np.cos(theta)
    sin = np.sin(theta)

    MatrixZ = np.array([[cos ,-sin,0   ,0],
                        [sin ,cos ,0   ,0],
                        [0   ,0   ,1   ,0],
                        [0   ,0   ,0   ,1]])
    return MatrixZ.dot(cloud.T).T

if __name__ == "__main__":
    # import cProfile
    # cProfile.run('main()', sort='time')
    main()
