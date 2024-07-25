from Functions.general_functions import create_output_folder, delete_folder, merge_txt
import os
import pandas as pd

# # Import qgis modules

# QGIS and processing related-modules
from qgis.core import *
from qgis import processing

class LidarSegmentator:
    """
    Used for segmentating the LiDAR files into the builing and its neighborhood
    
    ### Attributes: 
    #### Defined upon initialization:
    - building: single-row dataframe containing the following fields: identifier, x, y
    - temp_path: path to store temporal files
    #### Self-generated:
    - cadaster_files: list of cadaster files in which the building could be (i.e., the building is within the limits of eachcadaster files)
    - found_polygon_layer: QGIS vector layer containing the polygon corresponding to the building

    ### Public methods:
    - findFilesForSquare: knowing the building centroid, finds which LiDAR files are required to obtain the neighborhood square
    - find_potential_cadaster: updates the cadaster_files attribute with a list with all the cadaster files in which the building centroid can be
    - polygon_cadaster: obtains a QGIS vector Layer containing only the polygon for the given building (updates the found_polygon_layer attribute)
    - export_geopackage:  exports the found_polygon_layer to a .gpkg file
    - LiDAR_polygon_segmentation: exports a .csv file of the LiDAR points that are inside the building (with a buffer applied to the building surroundings).
    """

    def __init__(self, building, temp_path="Temp"):
        """
    #### Inputs:
    - building: single-row dataframe containing the following fields: identifier, x, y
    - temp_path: path to store temporal files
        """
        self.building = building
        self.temp_path = temp_path

    @staticmethod
    def __find_LiDAR(LiDAR_limits, x, y):
        """
    #### Inputs:
    - A dataframe containing the LiDAR files names and limits
    - x and y coordinates of the point

    #### Outputs: 
    - The filename of the file that contains the given point. If the file is not found, it returns "Not found"
        """
        for i in range(len(LiDAR_limits)):
            LiDAR_file = LiDAR_limits.iloc[i]
            if((x >= LiDAR_file['xMin']) and (x <= LiDAR_file['xMax'])):
                if((y >= LiDAR_file['yMin']) and (y <= LiDAR_file['yMax'])):
                    return LiDAR_file['file_names']
        return "Not found"

    def findFilesForSquare(self, LiDAR_limits, size):
        """
    #### Inputs:
    - A dataframe containing the LiDAR files names and limits
    - The side of the square to segment

    #### Outputs: 
    - List of all the LiDAR files needed to generate a square of sizeXsize centered in a given point. 
    If no file is found or there are missing files for that specific building, returns incomplete or empty list.
        """
        
        x = self.building.x[0]
        y = self.building.y[0]

        withinLimits = []
        withinLimits.append(self.__find_LiDAR(LiDAR_limits, x, y))

        # If the building is not found, there is no point in checking surroundings
        notFound = "Not found"
        if(withinLimits[0] == notFound):
            return withinLimits
        else:
            # But if there is a building, we search for the corners of square
            withinLimits.append(LidarSegmentator.__find_LiDAR(LiDAR_limits, x+size/2, y+size/2))
            withinLimits.append(LidarSegmentator.__find_LiDAR(LiDAR_limits, x+size/2, y-size/2))
            withinLimits.append(LidarSegmentator.__find_LiDAR(LiDAR_limits, x-size/2, y-size/2))
            withinLimits.append(LidarSegmentator.__find_LiDAR(LiDAR_limits, x-size/2, y+size/2))

        # We delete duplicates
        withinLimits = list(set(withinLimits))

        # If there is a not found file, but it is not the one containing the building center, we won't care
        if(notFound in withinLimits):
            withinLimits.remove(notFound)

        return withinLimits
    
    @staticmethod
    def __inside_rect(point, minCorner, maxCorner):
        """
    #### Inputs:
    - 2-element arrays (x and y coordinates) for the point to check, and the minimum and maximum corner of the rectangle to check

    #### Outputs: 
    - Returns True or False if the given point is inside a rectangle limited by its bottom-left and top-right corners
        """
        if((point[0] >= minCorner[0]) and (point[1] >= minCorner[1])):
            if((point[0] <= maxCorner[0]) and (point[1] <= maxCorner[1])):
                return True
        return False
    
    def find_potential_cadaster(self,cadaster_limits):
        """
    Generates a list with all the cadaster files in which the building centroid can be (cadaster_files attribute)
    
    #### Inputs:
    - A dataframe of the cadaster limits of each file

    #### Outputs: 
    - None
        """
        self.cadaster_files = []
        for i in range(len(cadaster_limits)):
            current_file = cadaster_limits.iloc[i]

            cornerMin = [current_file['minx'], current_file['miny']]
            cornerMax = [current_file['maxx'], current_file['maxy']]
            
            if(LidarSegmentator.__inside_rect([self.building.x[0], self.building.y[0]], cornerMin, cornerMax)):
                self.cadaster_files.append(current_file['file'])
    
    def polygon_cadaster(self, cadaster_files_path, srcLiDAR):
        """
    Obtains a QGIS vector Layer containing only the polygon for the given building

    #### Inputs:
    - The path where are the cadaster files are (all the files, not only those that have been confirmed to be within limits)
    - The src used in the point cloud file

    #### Outputs: 
    - Returns True or False, whether a polygon has been found
        """

        # Exports building info to (temporal) csv and imports it as a layer 
        tempExport = os.path.abspath(self.temp_path+ "/Isolated_Building.csv")
        self.building.to_csv(tempExport, index=False)
        uri = 'file:///%s?crs=%s&xField=%s&yField=%s' % (tempExport, 'epsg:'+str(srcLiDAR), 'x','y')
        Building_Centroid_Layer=QgsVectorLayer(uri,"Building_Centroid_Layer","delimitedtext")
        # Finds the polygon in the cadaster file(s) that contains the centroid 
        for cadaster_file in self.cadaster_files:
            # Loads cadaster Layer
            directory = os.path.abspath(cadaster_files_path + "/" + cadaster_file)
            Cadaster_Layer = QgsVectorLayer(directory,"Cadaster_Layer","ogr")

            # Finds cadaster polygon that contains building centroid
            selected = processing.run("native:selectbylocation", {
                'INPUT':Cadaster_Layer,
                'PREDICATE':[1], # contains
                'INTERSECT':Building_Centroid_Layer,
                'METHOD':0,
                'OUTPUT': 'TEMPORARY_OUTPUT',
                }
            )
            
            self.found_polygon_Layer = Cadaster_Layer.materialize(QgsFeatureRequest().setFilterFids(Cadaster_Layer.selectedFeatureIds()))
            self.found_polygon_Layer.setName("found_polygon_Layer")
            
            if(self.found_polygon_Layer.featureCount() >= 1):
                return True
            
        return False
    
    def export_geopackage(self, export_path):
        """
    #### Inputs:
    - The path where the geopackage containing the building has to be exported

    #### Outputs: 
    - None
    
    #### Exports:
    - Saves the found_polygon_layer to a .gpkg file
        """

        options = QgsVectorFileWriter.SaveVectorOptions()
        options.driverName = "GPKG"
        QgsVectorFileWriter.writeAsVectorFormatV3(self.found_polygon_Layer, export_path, QgsCoordinateTransformContext(), options)

    def LiDAR_polygon_segmentation(self, LiDAR_file, export_path, offset, srcLiDAR):
        """
    #### Inputs:
    - Path of the file where the LiDAR points of the building are (could be an original file or it could be a merged file)
    - The path where the geopackage containing the building has to be exported
    - The offset desired to apply to the cadaster polygon
    - The src used in the point cloud file

    #### Outputs: 
    - Error message (was used for debugging, it is now ignored).
    
    #### Exports:
    - A .csv file containing only the LiDAR points that are inside the cadaster polygon (with the necessary buffer).
        """

        uri = 'file:///%s?crs=%s&xField=%s&yField=%s&zField=%s&delimiter=%s' % (os.path.abspath(LiDAR_file), 'epsg:'+str(srcLiDAR), 'field_1','field_2', 'field_3', ' ')
        Layer_LIDAR=QgsVectorLayer(uri,"Layer_LIDAR","delimitedtext")

        offset_polygon_layer = processing.run("native:buffer", {
            'INPUT':self.found_polygon_Layer,
            'DISTANCE':offset,# /(40075*1000)*360, # Converts from meters to arc degrees
            'SEGMENTS':5,
            'END_CAP_STYLE':2,
            'JOIN_STYLE':2,
            'MITER_LIMIT':2,
            'DISSOLVE':False,
            'OUTPUT': 'TEMPORARY_OUTPUT',
        })

        selected = processing.run("native:selectbylocation", {
            'INPUT':Layer_LIDAR,
            'PREDICATE':[6], # inside
            'INTERSECT':offset_polygon_layer['OUTPUT'],
            # 'INTERSECT':self.found_polygon_Layer,
            'METHOD':0,
            }
        )

        #points_buildings_Layer = selected['OUTPUT']
        points_buildings_Layer = Layer_LIDAR.materialize(QgsFeatureRequest().setFilterFids(Layer_LIDAR.selectedFeatureIds()))
        points_buildings_Layer.setName("points_buildings_Layer")

        options = QgsVectorFileWriter.SaveVectorOptions()
        options.driverName = "CSV"
        options.fileEncoding = "UTF-8"
        options.ct = QgsCoordinateTransform(points_buildings_Layer.crs(), QgsCoordinateReferenceSystem.fromEpsgId(srcLiDAR),QgsProject.instance())
        error = QgsVectorFileWriter.writeAsVectorFormatV3(points_buildings_Layer, export_path, QgsCoordinateTransformContext(), options)
    
        # This is done to get rid of the first row, that contains a random point mistakenly assumed to be the headers
        read_path = export_path + ".csv"
        df = pd.read_csv(read_path)
        #df = df.set_axis(["x","y","z"], axis=1)
        df.to_csv(read_path, index=False, header = False)
        return error