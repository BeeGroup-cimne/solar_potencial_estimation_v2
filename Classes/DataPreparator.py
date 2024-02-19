from Functions.general_functions import create_output_folder
import pandas as pd # Handle dataframes
import pyproj # Convert building and cadastre to new coordinate system
import os # Get list of files within a directory
import subprocess  # To work with lastools .exe files

# QGIS and processing related-modules
from qgis.core import *
from qgis.analysis import QgsNativeAlgorithms
from qgis import processing

from processing.core.Processing import Processing
import processing

class DataPreparator:
    """
    Used for preparing the data files of the building list, cadastre and LiDAR
    
    ### Attributes:
    #### Defined upon initialization:
    - buildings_path: .csv or .txt containing the building info. The following 3 fields are required (with these names): identifier, lat, long. Files must be prepared to have this 3 columns or the function must be redefined
    - cadastre_path: Directory containing all the cadastre geopackages, AND ONLY THE GEOPACKAGES (extensions: .gpkg or .zip containing a .gpkg)
    - LiDAR_path: Directory containing all the LiDAR files, AND ONLY THE LiDAR FILES (extensions: .laz, or .txt/.csv)
    - output_path: Directory where exports need to be saved (if the folder does not exist, it will be created)

    ### Public methods:
    - prepare_buildings: converts a list of buildings to the specified crs (i.e. that of LiDAR data). It also filters for buildings with the same coordinates
    - prepare_cadastre: exports a list with all the cadastre files in a given directory with the min and max corners of each file, in the specified crs (i.e., that of the LiDAR data)
    - prepare_LiDAR: exports a .csv with the list of all the LiDAR files in a directory and their limits. It also needs the paht of LAStools, because if there is a .laz file, it is first converted to .csv
    """
            
    def __init__(self, buildings_path, cadastre_path, LiDAR_path, output_path):
        """
    #### Inputs:
    - buildings_path: .csv or .txt containing the building info. The following 3 fields are required (with these names): identifier, lat, long. Files must be prepared to have this 3 columns or the function must be redefined
    - cadastre_path: Directory containing all the cadastre geopackages, AND ONLY THE GEOPACKAGES (extensions: .gpkg or .zip containing a .gpkg)
    - LiDAR_path: Directory containing all the LiDAR files, AND ONLY THE LiDAR FILES (extensions: .laz, or .txt/.csv)
    - output_path: Directory where exports need to be saved (if the folder does not exist, it will be created)
        """
        self.buildings_path = buildings_path
        self.cadastre_path = cadastre_path
        self.LiDAR_path = LiDAR_path
        self.output_path = output_path

    @staticmethod
    def __findRepeats(list, position):
        """
    #### Inputs:
    - A list 
    - The position of an element inside the list
    
    #### Outputs:
    - Returns the position of the first element that matches the same value as the one in the given position. If no element that matches is found, it returns -1
        """
        iterable = list[0:position]
        for i in range(len(iterable)):
            if(list[i] == list[position]):
                return i
        return (-1) 

    @staticmethod
    def __signFromDirection(value):
        """
    #### Inputs:
    - A coordinate (latitude/longitude), that can either be a string or a float

    #### Outputs:
    - If it is a string and it ends in N/S/E/W, returns the string as a signed float. If it was already a float, returns the same value without change
        """
        if(type(value) ==type("Hello")):
            direction = value[-1]
            number = value[:-1]
            if direction == 'E' or direction == 'N':    return float(number)
            elif direction == 'W' or direction == 'S':    return -float(number)
            else:   return float(value)

        else:
            return value

    @staticmethod
    def __las2txt(LiDAR_path, LAStoolsPath):
        """
    #### Inputs:
    - Directory containing all the LiDAR files (extensions: .laz)
    - Directory where las2txt.exe is stored

    #### Outputs:
    - proc.communicate(), to know whether the conversion was successful or not. This return was only used for debugging and then was ignored.

    #### Exports:
    - If the input files are in .laz extension, they are converted to .csv and exported in a new folder
        """
        laz_files = []
        for file in os.listdir(LiDAR_path):
            if file.endswith('.laz'):
                
                laz_files.append("-i")
                laz_files.append(LiDAR_path + "/" + file)

        if(len(laz_files) > 0):
            args = [LAStoolsPath + "/bin/las2txt.exe"]
            args.extend(laz_files)
            # args.extend(['-odir', LiDAR_path])
            args.extend(['-parse', 'xyz'])
            proc = subprocess.Popen(args, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True)
            return proc.communicate()

    def prepare_buildings(self, source, target):
        """
    #### Inputs:
    - (int) source crs in which lat and long is stored
    - (int) target crs to which transform and get x and y

    #### Outputs:
    - None

    #### Exports:
    - .csv with the filtered buildings and the reprojected coordinates
    - .csv with the duplicate buildings and which building they are equal to
        """
        buildings = pd.read_csv(self.buildings_path, sep=',')
        buildings = buildings[['identifier', 'lat', 'long']]

        # Find duplicates and stores them in a df
        toDrop = []
        sameAs = []

        for i in range(len(buildings)):
            repeatsX = DataPreparator.__findRepeats(buildings['lat'], i) 
            if(repeatsX != -1):
                if(DataPreparator.__findRepeats(buildings['long'], i) == repeatsX):
                    toDrop.append(buildings['identifier'][i])
                    sameAs.append(buildings['identifier'][repeatsX])
        
        dupes = pd.DataFrame({'duplicate': toDrop, 'equal_to': sameAs})

        # Delete duplicates
        buildings = buildings[~buildings['identifier'].isin(toDrop)]
        buildings = buildings.reset_index(drop=True)

        # Get the sign from the direction
        buildings.long = buildings.long.apply(DataPreparator.__signFromDirection)
        buildings.lat = buildings.lat.apply(DataPreparator.__signFromDirection)
        
        # Reproject the remaining buildings
        source_crs = 'epsg:' + str(source) # Coordinate system of the buildings list
        target_crs = 'epsg:' + str(target) # Coordinate system of LiDAR data

        global_to_local = pyproj.Transformer.from_crs(source_crs, target_crs)
        buildings['x'] = global_to_local.transform(buildings.lat, buildings.long)[0]
        buildings['y'] = global_to_local.transform(buildings.lat, buildings.long)[1]

        save_path = self.output_path + "/Buildings"

        # Export results and duplicates
        create_output_folder(save_path)
        buildings.to_csv((save_path + "/Buildings_filtered.csv"), index=False) # Export buildings
        dupes.to_csv((save_path + "/Duplicate buildings.csv"), index=False) # Export duplicates

    def prepare_cadastre(self, source, target):
        """
    #### Inputs:
    - (int) source crs with which the cadastre is defined
    - (int) target crs to which transform and get the corners

    #### Outputs:
    - None

    #### Exports:
    - .csv with the min and max corners for each cadastre file
        """
        
        # Initialize QGIS Application and Processing framework
        QgsApplication.setPrefixPath("C:\\OSGeo4W64\\apps\\qgis", True)
        app = QgsApplication([], True)
        QgsApplication.initQgis()
        Processing.initialize()

        # Declare transformer
        source_crs = 'epsg:' + str(source) # Coordinate system of the cadastre files
        target_crs = 'epsg:' + str(target) # Coordinate system of LiDAR data
        global_to_local = pyproj.Transformer.from_crs(source_crs, target_crs)
        
        # Get list of cadastre files
        cadastre_files = []
        for file in os.listdir(self.cadastre_path):
            if (file.endswith('.gpkg') or file.endswith('.gpkg.zip')):
                cadastre_files.append(file) #.split('.')[0]

        # Iterate each file to find the minimum and maximum corners and store them
        filename = []
        minx = []
        miny = []
        maxx = []
        maxy = []

        for file in cadastre_files:
            directory = self.cadastre_path + "\\" + file
            Cadastre = QgsVectorLayer(directory,"Cadastre","ogr")
            
            parameters = {
                'INPUT':Cadastre,
                'ROUND_TO':0,
                'OUTPUT':'TEMPORARY_OUTPUT'
            }

            extension = processing.run("native:polygonfromlayerextent", parameters)['OUTPUT']
            
            fields = []
            for field in extension.fields():
                fields.append(field.name())  
            
            for feature in extension.getFeatures():
                filename.append(file)
                minx.append(global_to_local.transform(feature['MINY'], feature['MINX'])[0])
                miny.append(global_to_local.transform(feature['MINY'], feature['MINX'])[1])
                maxx.append(global_to_local.transform(feature['MAXY'], feature['MAXX'])[0])
                maxy.append(global_to_local.transform(feature['MAXY'], feature['MAXX'])[1])

        # Convert stored info to dataframe and export it to .csv 
        cadastre_limits = pd.DataFrame({
            'file': filename,
            'minx': minx, 'miny': miny, 
            'maxx':maxx, 'maxy':maxy
            })

        save_path = self.output_path + "/Cadastre"
        create_output_folder(save_path)
        cadastre_limits.to_csv((save_path + "/Cadastre_Limits.csv"), index=False)

    def prepare_LiDAR(self, las2txtPath="C:/LAStools/bin/las2txt.exe"):
        """
    #### Inputs:
    - Directory where las2txt.exe is stored

    #### Outputs:
    - None

    #### Exports:
    - If the input files are in .laz extension, they are converted to .csv and exported in the same folder
    - .csv with the limits each LiDAR file
        """

        convertedFiles = DataPreparator.__las2txt(self.LiDAR_path,las2txtPath)

        file_names = []
        xMin = []
        yMin = []
        xMax = []
        yMax = []

        for file in os.listdir(self.LiDAR_path):
            if file.endswith('.txt') or file.endswith('.csv'):
                file_names.append(str(file).split('\\')[-1])

        for i in range(len(file_names)):
            temp = pd.read_csv((self.LiDAR_path + "/" + file_names[i]), sep=" ", header = None)
            xMin.append(temp[0].min())
            xMax.append(temp[0].max())
            yMin.append(temp[1].min())
            yMax.append(temp[1].max())

        del(temp)

        LiDAR_files = pd.DataFrame(list(zip(file_names, xMin, yMin, xMax, yMax)), columns =['file_names', 'xMin', 'yMin', 'xMax', 'yMax'])

        save_path = self.output_path + "/LiDAR"
        create_output_folder(save_path)
        LiDAR_files.to_csv((save_path + "/LiDAR_Limits.csv"), index=False)