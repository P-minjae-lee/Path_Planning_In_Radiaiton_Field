#include <pathplanninginradiationfield/mesh_converter/converter.h>

MeshConverter::MeshConverter()
{
    nh.param<std::string>("mesh_converter/meshType", meshType, "obj");
    nh.param<std::string>("mesh_converter/loadMeshDirectory", loadMeshDirectory, "/catkin_ws_phd/src/pathplanninginradiationfield/meshes/sf03/version_3/");
    nh.param<std::string>("mesh_converter/savePCDDirectory", savePCDDirectory, "/catkin_ws_phd/src/pathplanninginradiationfield/meshes/sf03/version_3/");
    nh.param<std::string>("mesh_converter/meshName", meshName, "sf03");

    nh.param<int>("mesh_converter/number_samples", number_samples, 1000000);
    nh.param<float>("mesh_converter/leaf_size", leaf_size, 0.01);
    nh.param<bool>("mesh_converter/write_normals", write_normals, false);

    nh.param<bool>("mesh_converter/enableVisual", enableVisual, true);

    initCloud();
}





void MeshConverter::uniform_sampling (vtkSmartPointer<vtkPolyData> polydata, size_t n_samples, bool calc_normal, pcl::PointCloud<pcl::PointNormal> & cloud_out)
{
  polydata->BuildCells ();
  vtkSmartPointer<vtkCellArray> cells = polydata->GetPolys ();
 
  double p1[3], p2[3], p3[3], totalArea = 0;
  std::vector<double> cumulativeAreas (cells->GetNumberOfCells (), 0);
  size_t i = 0;
  vtkIdType npts = 0, *ptIds = NULL;
  for (cells->InitTraversal (); cells->GetNextCell (npts, ptIds); i++)
  {
    polydata->GetPoint (ptIds[0], p1);
    polydata->GetPoint (ptIds[1], p2);
    polydata->GetPoint (ptIds[2], p3);
    totalArea += vtkTriangle::TriangleArea (p1, p2, p3);
    cumulativeAreas[i] = totalArea;
  }
 
  cloud_out.points.resize (n_samples);
  cloud_out.width = static_cast<uint32_t> (n_samples);
  cloud_out.height = 1;
 
  for (i = 0; i < n_samples; i++)
  {
    Eigen::Vector4f p;
    Eigen::Vector3f n;
    randPSurface (polydata, &cumulativeAreas, totalArea, p, calc_normal, n);
    cloud_out.points[i].x = p[0];
    cloud_out.points[i].y = p[1];
    cloud_out.points[i].z = p[2];
    if (calc_normal)
    {
      cloud_out.points[i].normal_x = n[0];
      cloud_out.points[i].normal_y = n[1];
      cloud_out.points[i].normal_z = n[2];
    }
  }
}

inline void MeshConverter::randPSurface (vtkPolyData * polydata, std::vector<double> * cumulativeAreas, double totalArea, Eigen::Vector4f& p, bool calcNormal, Eigen::Vector3f& n)
{
  float r = static_cast<float> (uniform_deviate (rand ()) * totalArea);
 
  std::vector<double>::iterator low = std::lower_bound (cumulativeAreas->begin (), cumulativeAreas->end (), r);
  vtkIdType el = vtkIdType (low - cumulativeAreas->begin ());
 
  double A[3], B[3], C[3];
  vtkIdType npts = 0;
  vtkIdType *ptIds = NULL;
  polydata->GetCellPoints (el, npts, ptIds);
  polydata->GetPoint (ptIds[0], A);
  polydata->GetPoint (ptIds[1], B);
  polydata->GetPoint (ptIds[2], C);

  if (calcNormal)
  {
    // OBJ: Vertices are stored in a counter-clockwise order by default
    Eigen::Vector3f v1 = Eigen::Vector3f (A[0], A[1], A[2]) - Eigen::Vector3f (C[0], C[1], C[2]);
    Eigen::Vector3f v2 = Eigen::Vector3f (B[0], B[1], B[2]) - Eigen::Vector3f (C[0], C[1], C[2]);
    n = v1.cross (v2);
    n.normalize ();
  }
  
  randomPointTriangle (float (A[0]), float (A[1]), float (A[2]),
                       float (B[0]), float (B[1]), float (B[2]),
                       float (C[0]), float (C[1]), float (C[2]), p);
}

inline double MeshConverter::uniform_deviate (int seed)
{
  double ran = seed * (1.0 / (RAND_MAX + 1.0));
  return ran;
}

inline void MeshConverter::randomPointTriangle (float a1, float a2, float a3, float b1, float b2, float b3, float c1, float c2, float c3, Eigen::Vector4f& p)
{
  float r1 = static_cast<float> (uniform_deviate (rand ()));
  float r2 = static_cast<float> (uniform_deviate (rand ()));
  float r1sqr = std::sqrt (r1);
  float OneMinR1Sqr = (1 - r1sqr);
  float OneMinR2 = (1 - r2);
  a1 *= OneMinR1Sqr;
  a2 *= OneMinR1Sqr;
  a3 *= OneMinR1Sqr;
  b1 *= OneMinR2;
  b2 *= OneMinR2;
  b3 *= OneMinR2;
  c1 = r1sqr * (r2 * c1 + b1) + a1;
  c2 = r1sqr * (r2 * c2 + b2) + a2;
  c3 = r1sqr * (r2 * c3 + b3) + a3;
  p[0] = c1;
  p[1] = c2;
  p[2] = c3;
  p[3] = 0;
}




void MeshConverter::initCloud()
{
    mesh.reset(new pcl::PolygonMesh());
    vertexCloud.reset(new pcl::PointCloud<PointType>());
    modelCloud.reset(new pcl::PointCloud<PointType>());
}

void MeshConverter::ConverterHandle()
{
    extractVertex();

    triangulationCloud();

    if (enableVisual) cloudViewer();
}

void MeshConverter::extractVertex()
{
    std::string fileDirectory;

    if (meshType == "obj")
    {
        fileDirectory = std::getenv("HOME") + loadMeshDirectory + meshName + ".obj";
        
        if (pcl::io::loadPolygonFileOBJ(fileDirectory, *mesh) != -1)
        {
            pcl::fromPCLPointCloud2(mesh->cloud, *vertexCloud);
        }
        else ROS_ERROR("do not convert mesh to point cloud");
    }
    else if(meshType == "stl")
    {
        fileDirectory = std::getenv("HOME") + loadMeshDirectory + meshName + ".STL";
        if (pcl::io::loadPolygonFileSTL(fileDirectory, *mesh) != -1)
        {
            pcl::fromPCLPointCloud2(mesh->cloud, *vertexCloud);
        }
        else ROS_ERROR("do not convert mesh to point cloud");
    }
    else
    {
        ROS_ERROR("Unkown Mesh Type");
    }
}

void MeshConverter::triangulationCloud()
{
    vtkSmartPointer<vtkPolyData> polydata1 = vtkSmartPointer<vtkPolyData>::New();

    pcl::io::mesh2vtk(*mesh, polydata1);

    vtkSmartPointer<vtkTriangleFilter> triangleFilter = vtkSmartPointer<vtkTriangleFilter>::New();

    triangleFilter->SetInputData (polydata1);

    triangleFilter->Update();

    vtkSmartPointer<vtkPolyDataMapper>triangleMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    triangleMapper->SetInputConnection(triangleFilter->GetOutputPort());
    triangleMapper->Update();
    polydata1 = triangleMapper->GetInput();

    pcl::PointCloud<pcl::PointNormal>::Ptr normalCloud(new pcl::PointCloud<pcl::PointNormal>);
    uniform_sampling (polydata1, number_samples, write_normals, *normalCloud);
    
    pcl::VoxelGrid<pcl::PointNormal> grid_;
    grid_.setInputCloud (normalCloud);
    grid_.setLeafSize (leaf_size, leaf_size, leaf_size);
 
    pcl::PointCloud<pcl::PointNormal>::Ptr voxelCloud(new pcl::PointCloud<pcl::PointNormal>);
    grid_.filter (*voxelCloud);

    pcl::copyPointCloud (*voxelCloud, *modelCloud);   

    std::string savePCDfile = std::getenv("HOME") + savePCDDirectory + meshName + ".pcd";

    if (access(savePCDfile.c_str(), F_OK) != 0)
    {
        pcl::io::savePCDFileBinary(savePCDfile, *modelCloud);
    }
    else
    {
        int unused = system((std::string("exec rm -r ") + savePCDfile).c_str());
        pcl::io::savePCDFileBinary(savePCDfile, *modelCloud);
    }  
}

void MeshConverter::cloudViewer()
{
    pcl::visualization::PCLVisualizer viewer("mesh to cloud");
    viewer.addPointCloud(modelCloud);
    while (!viewer.wasStopped())
    {
        viewer.spinOnce();
        if (!ros::ok()){
            break;
        }
    }
}

int main(int argc, char** argv){
    ros::init(argc, argv, "mesh_converter");
    ROS_INFO_STREAM("Start Mesh Converter");
    MeshConverter meshconverter;

    meshconverter.ConverterHandle();

    return 0;
}