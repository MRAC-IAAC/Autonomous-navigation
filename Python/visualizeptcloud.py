# Shows fast visualization within python Open3D
import numpy
import open3d


# define the input cloud and output cloud directories
#input_path = 'C:/Users/shabe/OneDrive/Documents/GitHub/IAAC_studio_1_2/studio workshop support/STUDIO SUPPORT WORKSHOP -20210302T100019Z-001/STUDIO SUPPORT WORKSHOP/point-cloud/data/fused.ply'
#input_path = "/media/shabelson/BC507DA2507D63D4/Documents and Settings/shabe/OneDrive/Documents/GitHub/IAAC_studio_1_2/studio workshop support/STUDIO SUPPORT WORKSHOP -20210302T100019Z-001/STUDIO SUPPORT WORKSHOP/point-cloud/data/fused.ply"
# Read the input cloud
fname = "./soft2_compiled_1615307126_734546.csv"

xyz = numpy.genfromtxt(fname,dtype=float,delimiter=' ')
locPts = open3d.cpu.pybind.utility.Vector3dVector(xyz)
locPts = open3d.geometry.PointCloud(locPts)
	
# View pointcloud
open3d.visualization.draw_geometries([locPts])