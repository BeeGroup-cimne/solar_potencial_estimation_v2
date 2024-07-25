import os # Interface with system directories
import pandas as pd
import subprocess  # Merge laz files

import sys
sys.path.append("Sample/DEMto3D-QGIS-Plugin-master/model_builder") #("C:/DEMto3D-QGIS-Plugin/model_builder")  # Import stl related modules
import Model_Builder, STL_Builder
from qgis.core import *

from Functions.general_functions import create_output_folder

class STLExporter:
    """
    Used for generating the 3D model of the neighborhood, given the .csv of the LiDAR points
    
    ### Attributes: 
    #### Defined upon initialization:
    - building:  single-row dataframe containing the following fields: identifier, x, y
    - stl_side: length (in meters) of the side of the neighborhood (square)
    - temp_path: path where temporal files will be stored
    - TXT_path: path where the .txt of the neighborhood should be stored
    - LAS_path: path where the .las of the neighborhood should be stored
    - DEM_path: path where the raster image (.asc) of the neighborhood should be stored
    - STL_path: path where the .stl of the neighborhood should be stored
    - LAStoolsPath: path where the LAStools applications are located (needed for txt2las, but general path must be specified)

    ### Public methods:
    - segmentSquare: from all the LiDAR files, merged them (if needed) and then crop to a .txt of a the square of stl_side*stl_side corresponding to the neighborhood
    - txt_to_las: convert the previously generated .txt of the neighborhood to a .las file (point cloud file)
    - las_to_dem: convert the previously generated .las file of the neighborhood to a raster file (middle step needed for 3D generation)
    - dem_to_3d: convert the previously generated raster file of the neighborhood to a .stl file (3D model)
    """
    
    def __init__(self, building, stl_side, squarePaths, temp_path, txt2lasPath, las2demPath):
        """
    #### Inputs:/
    - building:  single-row dataframe containing the following fields: identifier, x, y
    - stl_side: length (in meters) of the side of the neighborhood (square)
    - squarePaths: array containing [TXT_path, LAS_path, DEM_path, STL_path]
    - temp_path: path where temporal files will be stored(
    - txt2lasPath, las2demPath: path where each LAStools applications is located
        """
        self.building = building
        self.stl_side = stl_side
        self.temp_path = temp_path
        self.TXT_path = squarePaths[0]
        self.LAS_path = squarePaths[1]
        self.DEM_path = squarePaths[2]
        self.STL_path = squarePaths[3]
        self.txt2lasPath = txt2lasPath #"C:/LAStools/bin/txt2las.exe"
        self.las2demPath = las2demPath 
    
    def segmentSquare(self, LiDAR_extended):
        """
    #### Inputs:
    - LiDAR_extended: path where a (if needed, merged) file is sotred. This file will be segmented into a square of stl_side

    #### Exports:
    - .csv with all the points inside a square of stl_side centered in the building of choice
        """
        
        create_output_folder(self.TXT_path)
        outputFile = self.TXT_path + "/" + self.building.identifier[0] + '.txt'

        centroid = [ self.building.x[0], self.building.y[0]]

        LiDARData = pd.read_csv(LiDAR_extended, sep=" ", header=None)
        #Filter x
        LiDARData =LiDARData[LiDARData[0] >= (centroid[0] - self.stl_side/2)]
        LiDARData =LiDARData[LiDARData[0] <= (centroid[0] + self.stl_side/2)]
        #Filter y
        LiDARData =LiDARData[LiDARData[1] >= (centroid[1] - self.stl_side/2)]
        LiDARData =LiDARData[LiDARData[1] <= (centroid[1] + self.stl_side/2)]
        #Write txt
        LiDARData.to_csv(outputFile, index = False, header = False, sep=" ")  
    
    def txt_to_las(self):
        """
    #### Exports:
    - .laz of the square of stl_side centered in the building of choice. Generated from .csv file
        """
        create_output_folder(self.LAS_path)
        txt_path = self.TXT_path + '/'
        txt_files = []
        for file in os.listdir(txt_path):
            if file.endswith('.txt'):
                txt_files.append("-i")
                txt_files.append(os.path.abspath(txt_path + file))

        outputDir = os.path.abspath(self.LAS_path)

        args = [self.txt2lasPath]
        args.extend(txt_files)
        args.extend(['-odir', outputDir])
        proc = subprocess.Popen(args)#, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True)
        return proc.communicate()
        
    def las_to_dem(self):
        """
    #### Exports:
    - raster file of the square of stl_side centered in the building of choice. Generated from .las file
        """
        create_output_folder(self.DEM_path)
        las_path = self.LAS_path + '/'
        las_files = []
        for file in os.listdir(las_path):
            if file.endswith('.las'):
                las_files.append("-i")
                las_files.append(os.path.abspath(las_path + file))

        outputDir = os.path.abspath(self.DEM_path)

        args = [self.las2demPath]
        args.extend(las_files)
        args.extend(['-odir', outputDir])
        proc = subprocess.Popen(args)#, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True)
        output, error = proc.communicate()

    def dem_to_3d(self):
        """
    #### Exports:
    - .stl (3d model) files of the square of stl_side centered in the building of choice. Generated from raster file.
        """
        create_output_folder(self.STL_path)
    
        name = self.building.identifier[0]
        raster_directory = self.DEM_path + '/' + name + '.asc'
        raster_file = QgsRasterLayer(raster_directory, "raster_neighborhood")
        raster_extent = raster_file.extent()
        min_height = raster_file.dataProvider().bandStatistics(1, QgsRasterBandStats.All, raster_extent, 0).minimumValue
            
        parameters = {
        "layer": raster_directory,
        "roi_x_max": raster_extent.xMaximum(),
        "roi_x_min": raster_extent.xMinimum(),
        "roi_y_max": raster_extent.yMaximum(),
        "roi_y_min": raster_extent.yMinimum(),
        "roi_rect_Param": {
            "center": [
                raster_extent.center().x(),
                raster_extent.center().y()
            ],
            "width": abs(raster_extent.xMaximum() - raster_extent.xMinimum()),
            "height": abs(raster_extent.yMaximum() - raster_extent.yMinimum()),
            "rotation": 0
        },
        "spacing_mm": 0.5, # The smaller, the more detailed, but takes much longer to run
        "height": abs(raster_extent.yMaximum() - raster_extent.yMinimum()),
        "width": abs(raster_extent.xMaximum() - raster_extent.xMinimum()),
        "z_scale": 1.0,
        "scale": 1000,
        "scale_h": 1000,
        "scale_w": 1000,
        "z_inv": False,
        "z_base": min_height,
        "divideRow": 1,
        "divideCols": 1,
        "projected": False,
        "crs_layer": "",
        "crs_map": "",
        "baseModel": 1, #offset 1 mm, so that the lowest height is not at 0 (this way we guarantee that there is "something" at every height)
        "trimmed": False,
        }

        # STL conversion
        model = Model_Builder.Model(parameters)
        model.run()
        matrix_dem = model.get_model()

        stl_file = self.STL_path + '/' + name + '.stl'
        stl = STL_Builder.STL(parameters, stl_file, matrix_dem)
        stl.run()