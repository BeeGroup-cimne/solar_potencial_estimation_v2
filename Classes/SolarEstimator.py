from Classes.LidarSegmentator import LidarSegmentator
from Classes.STLExporter import STLExporter
from Classes.PlaneDetector import PlaneDetector
from Classes.PlaneProcessor import PlaneProcessor
from Classes.Shader import Shader
from Classes.PySAMSimulator import PySAMSimulator
from Classes.Plotter import Plotter

import warnings
warnings.filterwarnings('ignore')

from Functions.general_functions import create_output_folder, merge_txt

import pandas as pd

class SolarEstimator:
    """
    ### Attributes:
    #### Defined upon initialization:
    - building: single-row dataframe of the building containing, at least the following fields: x, y, identifier
    - output_path: path to store the results. A folder specific for the current building is created inside that path and multiple folders will be created inside that.
    - crsLiDAR: cordinate reference system used in the LiDAR files
    - temp_path: THIS FOLDER AND ITS CONTENTS ARE DELETED ONCE THE CODE IS FINISHED. It is used to store temporal results. 
    ##### Attributes with default values
    - square_side: size of the neighborhood to export the 3d model 
    
    #### Self-generated:
    - #### All instances of the imported class are made attributes to allow the user to check them from the outside. These are: segmentator, stlGenerator, planeDetector, planeProcessor, shader and pysam_simulator
    - All the specific paths where to results will be exported are also generated, check the __register_folders method for further details
    ##### Generated from loadData
    - LiDAR_limits: dataframe containing the name of each LiDAR file and its x,y minimum and maximum values
    - cadaster_limits: dataframe containing the name of each cadaster file and its x,y minimum and maximum values (converted to the same coordinates as LiDAR files)
    - LiDAR_path: directory where all the LiDAR files are
    - cadaster_path: directory where all the cadaster files are
    ##### Generated from segmentLiDAR
    - LiDAR_extended: path containing the LiDAR file from which the neighborhood will be obtained
    ##### Generated from simulatePySAM
    - pySAMResults: dataframe containing a summary of the simulation results for each sampled point

    ### Public methods:
    - loadData: registers the path of all the external information needed into the SolarEstimator object
    - segmentLiDAR: generates a .csv file with the LiDAR points inside the building limits (with a buffer)
    - createNeighborhood: generates the point cloud/3D files of the neighborhood
    - identifyPlanes: identifies the planes of the class's building
    - processPlanes: processes the planes found in the previous step
    - computeShading: generates the shading matrices of the planes
    - simulatePySAM: obtains the PV generation of the points sampled and shaded in the previous step
    - plotEnergyMap: plots an energy map, after simulating the solar generation

    """

    def __init__(self, building, output_path, crsLiDAR, square_side=200, temp_path="Results/_Temp"):
        """
    This function registers the path of all the external information needed into the SolarEstimator object

    #### Inputs:
    - building: single-row dataframe of the building containing, at least the following fields: x, y, identifier
    - output_path: path to store the results. A folder specific for the current building is created inside that path and multiple folders will be created inside that.
    - crsLiDAR: cordinate reference system used in the LiDAR files
    - square_side: size of the neighborhood to export the 3d model 
    - temp_path: THIS FOLDER AND ITS CONTENTS ARE DELETED ONCE THE CODE IS FINISHED. It is used to store temporal results. 
        """
        self.building = pd.DataFrame(building).reset_index(drop=True)
        self.output_path = output_path + "/" + self.building.identifier[0]
        self.temp_path = temp_path
        self.crsLiDAR = crsLiDAR
        self.square_side = square_side

        create_output_folder(self.temp_path)

        self.__register_folders()

    def __register_folders(self):
        """
    Updates all attributes of the class that contain the paths to export the different results. These paths are later used in the different methods.

    This method does not have any inputs or outputs, it's just an auxiliar function
        """
        self.segmented_path = self.output_path + "/01 - Segmented Buildings"
        self.square_path = self.output_path + "/02 - Segmented Squares"
        baseOutputPath_TXT = self.square_path + "/01 - Point Cloud - txt"
        baseOutputPath_LAS = self.square_path + "/02 - LiDAR files - las"
        baseOutputPath_DEM = self.square_path + "/03 - Raster images - dem"
        baseOutputPath_STL = self.square_path + "/04 - 3D models - stl"
        self.squarePaths = [baseOutputPath_TXT, baseOutputPath_LAS, baseOutputPath_DEM, baseOutputPath_STL]

        self.identifiedPath = self.output_path + "/03 - Plane Identification"

        planeListPath = self.identifiedPath + "/01 - Plane Lists"
        planePointsPath = self.identifiedPath + "/02 - Points from planes"
        imagesPlanePath = self.identifiedPath + "/03 - Images"
        self.identifiedPaths = [planeListPath, planePointsPath, imagesPlanePath]

        self.processedPath = self.output_path + "/04 - Plane Processing"
        processedResultsPath = self.processedPath + "/01 - Results/"
        processedImagesPath = self.processedPath + "/02 - Images/"
        self.processedPaths = [processedResultsPath, processedImagesPath]
        

        self.shadingPath = self.output_path + "/05 - Shading Matrices"
        self.pysamResultsPath = self.output_path + "/06 - PySAM Simulation"

    # Step 0 - load the Data
    def loadData(self, LiDAR_info_path, cadaster_info_path, LiDAR_files_path, cadaster_files_path):
        """
    This function registers the path of all the external information needed into the SolarEstimator object

    #### Inputs:
    - LiDAR_info_path: file with all the LiDAR limits (the one generated with the dataPreparator class) 
    - cadaster_info_path: file with all the cadaster limits (the one generated with the dataPreparator class) 
    - LiDAR_files_path: path where the LiDAR files are
    - cadaster_files_path: path where the cadaster files are
    
    #### Outputs:
    - None
        """
        self.LiDAR_limits = pd.read_csv(LiDAR_info_path) 
        self.cadaster_limits = pd.read_csv(cadaster_info_path)
        self.LiDAR_path = LiDAR_files_path
        self.cadaster_path = cadaster_files_path

    # Step 1 - segment LiDAR
    def segmentLiDAR(self, offset=1, square_side=None):
        """
    This function generates a .csv file with the LiDAR points inside the building limits (with a buffer)

    #### Inputs:
    - offset: the offset/buffer (in m) that should be applied around the geometry lmits of the building
    - square_side: the length (in m) of the side of the square of the 3D model that is generated and will later be used for shading
    
    #### Outputs:
    - None

    #### Exports:
    - .csv file of the LiDAR points inside the polygon of the building with the given buffer
        """

        # Store attributes
        if(square_side != None):
            self.square_side = square_side
        LiDAR_offset = offset
        
        # Prepare output
        create_output_folder(self.segmented_path)
        
        self.segmentator = LidarSegmentator(self.building, self.temp_path)

        # Gets all LiDAR files within a "side" square and merge the files
        fileList = self.segmentator.findFilesForSquare(self.LiDAR_limits, self.square_side)
        if(len(fileList) < 1):
            with open(self.output_path + "/log.csv", 'w') as f:
                f.write("Building ", self.building.identifier[0], " has no LiDAR data available")
        else:
            filenames = [self.LiDAR_path + "/" + fileList[x] for x in range(len(fileList))]
            destination = self.temp_path + "/temporalMerged.txt"
            self.LiDAR_extended = merge_txt(filenames, destination)
            
            # Checks each cadaster limits and looks for polygon only in those files where it is indicated it could be in 
            self.segmentator.find_potential_cadaster(self.cadaster_limits)
            foundcadaster = self.segmentator.polygon_cadaster(self.cadaster_path, self.crsLiDAR)
            if(foundcadaster):
                export_path = self.segmented_path + "/" + self.building.identifier[0]
                self.segmentator.export_geopackage(export_path)
                self.segmentator.LiDAR_polygon_segmentation(self.LiDAR_extended, export_path, LiDAR_offset, self.crsLiDAR)
            else:
                with open(self.output_path + "/log.csv", 'w') as f:
                    f.write("Building " + self.building.identifier[0] + " does not have cadaster info")
                print("Building " + self.building.identifier[0] + " does not have cadaster info")

    # Step 2 - Export the .stl file
    def createNeighborhood(self, LAStoolsPath, square_side=None, export3D=True):
        """
    This function generates the point cloud/3D files of the neighborhood

    #### Inputs:
    - LAStoolsPath: directory where txt2las.exe is stored
    - square_side: the length (in m) of the side of the square of the 3D model that is generated and will later be used for shading
    - export3D: if False, it ony  export .csv and .laz files. If True, it exports also the raster and .stl file 
    
    #### Outputs:
    - None

    #### Exports:
    - .csv file of the LiDAR points within the neighborhood
    - .laz file of the neighborhood
    - (if export3D) .asc file of the neighborhood
    - (if export3D) .stl file (3D model) of the neighborhood
        """
        if(square_side != None):
            self.square_side = square_side

        self.stlGenerator = STLExporter(self.building, self.square_side, self.squarePaths, self.temp_path, LAStoolsPath=LAStoolsPath)
        self.stlGenerator.segmentSquare(self.LiDAR_extended)
        self.stlGenerator.txt_to_las()
        if(export3D):
            self.stlGenerator.las_to_dem()
            self.stlGenerator.dem_to_3d()

    # Step 3 - Plane identification with RANSAC
    def identifyPlanes(self, generateFigures=True, **kwargs):
        """
    This function identifies the planes of the class's building

    #### Inputs:
    - generateFigures: whether or not to export figures of the identified planes
    - **kwargs: see the PlaneDetector documentation (default attributes)
    
    #### Outputs:
    - None

    #### Exports:
    - if generateFigures, it exports two images: one corresponding to the point height histogram (to "see" where the heightgroups should be split), and another of the data cloud colorcoded by heightgroups.
    - For each heightgroup, a .csv file containing all the planes (parameters a,b,c,d) found in that heightgroup
    - For each plane found, a .csv file containing all the points (x,y,z) that belong to that plane
    - if generateFigures: it exports images of the step by step solution, as well as 3d scatters of each level once it is finished
        """
        self.planeDetector = PlaneDetector(self.building, self.segmented_path, self.identifiedPaths, generateFigures, **kwargs)
        self.planeDetector.detectPlanes()

    # Step 4 - Plane processing (with multiple criteria)    
    def processPlanes(self, crsCadaster=4326, generateFigures=True, **kwargs):
        """
    This function processes the planes found in the previous step:
    - Merges planes that are too similar.
    - Splits planes if there are density discontinuities
    - Deletes planes of bad quality.
    - Deletes overlaps.
    - Pierces holes.
    - Trims according to cadaster limits

    #### Inputs:
    - crsCadaster: coordinate reference system of the cadaster files (used for trimming)
    - generateFigures: whether or not to export figures of the processed planes (currently, this variable is not used)
    - **kwargs: see the PlaneDetector documentation (default attributes)
    
    #### Outputs:
    - None

    #### Exports:
    - .csv of all the parameters of each plane (as well as area, tilt, azimuth and polygon definition)
    - .png image of the 3D scatter plot
    - .png image of the 2D scatter plot + regions plot
        """
        cadasterPath = self.segmented_path + "/" + self.building.identifier[0] + ".gpkg"
        self.planeProcessor = PlaneProcessor(self.building, self.segmented_path, self.identifiedPaths, self.processedPaths, cadasterPath, generateFigures, **kwargs)
        self.planeProcessor.plotPlanes("From RANSAC_" + self.building.identifier[0])

        i = 0
        # print("Iteration: ", i)

        previousSplits = []
        self.planeProcessor.merge()
        previousSplits.append(self.planeProcessor.splitDelete())
        
        while(previousSplits[-1] > 1):
            i = i + 1
            # print("Iteration: ", i, ". Previous had ", previousSplits[-1], " splits")
            self.planeProcessor.merge()
            previousSplits.append(self.planeProcessor.splitDelete())

            if(len(previousSplits) > 3): # If we are stuck on a loop, we must break it
                if(previousSplits[-1] == previousSplits[-2] and previousSplits[-1] == previousSplits[-3] and previousSplits[-1] == previousSplits[-4]):
                    break
        
        self.planeProcessor.deleteOverlaps()
        self.planeProcessor.pierce(cadaster=False)
        self.planeProcessor.cadasterTrim(source=crsCadaster, target=self.crsLiDAR)
        self.planeProcessor.exportResults()

    # Step 5 -Shading calculation
    def computeShading(self, generateFigures=True, **kwargs):
        """
    This function generates the shading matrices of the planes

    #### Inputs:
    - generateFigures: whether or not to export figures for each sampled point (and raySending). An average matrix plot for each plane will always be generated
    - **kwargs: see the Shader documentation (default attributes)

    #### Outputs:
    - None

    #### Exports:
    - .csv file of the generated matrices (average, and for each sampled point)
    - if desired, individual matrix images and ray sending plot.
        """

        if(generateFigures):
            self.shadingPath = self.shadingPath + " with figures"
        create_output_folder(self.shadingPath, deleteFolder=True)

        planeListFile = self.processedPaths[0] + "PlaneList_" + self.building.identifier[0] + ".csv"
        planedf = pd.read_csv(planeListFile) # Yes, I'm doing this only to get the number of planes
        
        for planeIDShading in range(len(planedf)):
            self.shader = Shader(self.building, planeIDShading, self.squarePaths[0], self.squarePaths[3], self.processedPaths[0], self.shadingPath, **kwargs)
            self.shader.prepareDataShading()
            self.shader.sampleRoof()
            self.shader.shadingCalculation()
            self.shader.plotShadingMatrix(plotAll=False)
            if(generateFigures):
                self.shader.plotShadingMatrix(plotAll=True)
    
    # Step 6 -PySAM simulation
    def simulatePySAM(self, tmyfile, generateFigures=False, ratio=float(0.450/2)):
        """
    This function obtains the PV generation of the points sampled and shaded in the previous step

    #### Inputs:
    - tmyfile: .csv of the TMYfile, must be otained from NREL (https://nsrdb.nrel.gov/data-viewer) or converted to the same format
    - generateFigures: whether or not to export figures for each sampled point (and raySending). An average matrix plot for each plane will always be generated
    - ratio: watts per square meter ratio. This is used because the simulator needs, as an input, a solar capactiy, not an area.

    #### Outputs:
    - None

    #### Exports:
    - .csv file containing the hourly generation in a yera of each sampled point solar simulation and a .csv of the summary
    - if generateFigures, a heatmap for the building generation is generated.
        """

        planeListFile = self.processedPaths[0] + "PlaneList_" + self.building.identifier[0] + ".csv"
        planedf = pd.read_csv(planeListFile)
        create_output_folder(self.pysamResultsPath, deleteFolder=True)
        create_output_folder(self.pysamResultsPath + "/Yearly Results/", deleteFolder=True)


        roofIds = []
        tilts = []
        azimuths = []
        areas = []
        pointsX = []
        pointsY = []
        acAnnuals = []
        radiationAnnuals = []

        for planeIDMatrix in range(len(planedf)):
            sampledPointsPath = self.shadingPath + "/" + str(planeIDMatrix) + "/Points sampled.csv"
            sampledPoints = pd.read_csv(sampledPointsPath, header = None)
            sampledPoints = sampledPoints.rename(columns={0: "x", 1: "y", 2:"z"})

            for i in range(len(sampledPoints)):
                roofIds.append(planeIDMatrix)
                tilts.append(planedf.tilt[planeIDMatrix])
                azimuths.append(planedf.azimuth[planeIDMatrix])
                areas.append(planedf.area[planeIDMatrix])
                pointsX.append(sampledPoints.x[i])
                pointsY.append(sampledPoints.y[i])

                shadingMatrixPath = self.shadingPath + "/" + str(planeIDMatrix) + "/Individual Matrices/" + str(i).zfill(2) + ".csv"
                resultsPath = self.pysamResultsPath + "/Yearly Results/" + str(planeIDMatrix) + "_"  + str(i).zfill(2)
                
                self.pysam_simulator = PySAMSimulator(shadingMatrixPath, resultsPath, planedf, planeIDMatrix, ratio, tmyfile)
                simulationResults = self.pysam_simulator.runPySAMSimulation()
                acAnnuals.append(simulationResults["ac_annual"])
                radiationAnnuals.append(simulationResults["solrad_annual"])
                if(generateFigures):
                    self.pysam_simulator.plotGenerationHeatmap()


        self.pySAMResults = pd.DataFrame({"roofID": roofIds, 
                                          "tilt": tilts, 
                                          "azimuth": azimuths,
                                          "area": areas,
                                          "x": pointsX,
                                          "y": pointsY,
                                          "ac_annual": acAnnuals,
                                          "solarIrradiance": radiationAnnuals})
        
        self.pySAMResults.to_csv(self.pysamResultsPath + "/Summary.csv", index=False)

    def plotEnergyMap(self):
        """
    This function plots an energy map, after simulating the solar generation

    #### Inputs:
    - None 
    
    #### Outputs:
    - None

    #### Exports:
    - .png file, corresponding to the total generation heatmap
    """
        Plotter.plotEnergyMap(self.building, self.processedPaths[0], self.segmented_path, self.pysamResultsPath)