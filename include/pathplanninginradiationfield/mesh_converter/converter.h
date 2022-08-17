#pragma once
#ifndef _MESH_CONVERTER_H_
#define _MESH_CONVERTER_H_

#include <ros/ros.h>

// load Polygonfile
#include <pcl/PolygonMesh.h>
#include <pcl/io/vtk_lib_io.h>

// triangulation (fill pointCloud)
#include <pcl/features/normal_3d.h>
#include <vtkTriangle.h>
#include <vtkTriangleFilter.h>
#include <vtkPolyDataMapper.h>
#include <pcl/filters/voxel_grid.h>

// visualizer
#include <pcl/visualization/cloud_viewer.h>

// savePCD
#include <pcl/io/pcd_io.h>


typedef pcl::PointXYZ PointType;

class MeshConverter
{
    public:
        MeshConverter();


        void uniform_sampling(vtkSmartPointer<vtkPolyData> polydata, size_t n_samples, bool calc_normal, pcl::PointCloud<pcl::PointNormal> & cloud_out);

        inline void randPSurface (vtkPolyData * polydata, std::vector<double> * cumulativeAreas, double totalArea, Eigen::Vector4f& p, bool calcNormal, Eigen::Vector3f& n);

        inline double uniform_deviate (int seed);

        inline void randomPointTriangle (float a1, float a2, float a3, float b1, float b2, float b3, float c1, float c2, float c3, Eigen::Vector4f& p);


        void initCloud();

        void ConverterHandle();

        void extractVertex();

        void triangulationCloud();

        void cloudViewer();

        
    private:
        ros::NodeHandle nh;

        std::string meshType;
        std::string loadMeshDirectory;
        std::string meshName;
        std::string savePCDDirectory;

        int number_samples;
        float leaf_size;
        bool write_normals;
        bool enableVisual;

        pcl::PolygonMesh::Ptr mesh;
        pcl::PointCloud<PointType>::Ptr vertexCloud;
        pcl::PointCloud<PointType>::Ptr modelCloud;
};

#endif