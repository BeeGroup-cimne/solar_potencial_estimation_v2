import os # Interface with system directories
import pandas as pd
import subprocess  # Merge laz files

import sys
sys.path.append("C:/DEMto3D-QGIS-Plugin/model_builder")  # Import stl related modules
import Model_Builder, STL_Builder
from qgis.core import *
from qgis.analysis import QgsNativeAlgorithms
from qgis import processing

from Functions.general_functions import create_output_folder

class STLExporter:
    """
    ### Attributes:
    -

    ### Public methods:
    -
    """
    
    def __init__(self, building, stl_side, squarePaths, temp_path, LAStoolsPath):
        self.building = building
        self.stl_side = stl_side
        self.temp_path = temp_path
        self.TXT_path = squarePaths[0]
        self.LAS_path = squarePaths[1]
        self.DEM_path = squarePaths[2]
        self.STL_path = squarePaths[3]
        self.LAStoolsPath = LAStoolsPath # "C:/LAStools/bin/txt2las.exe"
    
    def segmentSquare(self, LiDAR_extended):
        """
    #### Inputs:
    - LiDAR_extended: path where a (if needed, merged) file is sotred. This file will be segmented into a square of stl_side

    #### Exports:
    - .csv with all the points inside a square of stl_side centered in the building of choice
        """
        
        create_output_folder(self.TXT_path)
        outputFile = self.TXT_path + "\\" + self.building.identifier[0] + '.txt'

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
                txt_files.append(txt_path + file)

        outputDir = self.LAS_path

        args = [self.LAStoolsPath + "/bin/txt2las.exe"]
        args.extend(txt_files)
        args.extend(['-odir', outputDir])
        proc = subprocess.Popen(args, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True)
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
                las_files.append(las_path + file)

        outputDir = self.DEM_path

        args = [self.LAStoolsPath + "/bin/las2dem.exe"]
        args.extend(las_files)
        args.extend(['-odir', outputDir])
        proc = subprocess.Popen(args, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True)
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