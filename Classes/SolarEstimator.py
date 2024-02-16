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

    def register_folders(self):
        self.segmented_path = self.output_path + "/01 - Segmented Buildings"
        self.square_path = self.output_path + "/02 - Segmented Squares"
        baseOutputPath_TXT = self.square_path + "/01 - Point Cloud - txt"
        baseOutputPath_LAS = self.square_path + "/02 - LiDAR files - las"
        baseOutputPath_DEM = self.square_path + "/03 - Raster images - dem"
        baseOutputPath_STL = self.square_path + "/04 - 3D models - stl"
        self.squarePaths = [baseOutputPath_TXT, baseOutputPath_LAS, baseOutputPath_DEM, baseOutputPath_STL]

        # self.identifiedPath = self.output_path + "/03 - Plane Identification"

        # planeListPath = self.identifiedPath + "/01 - Plane Lists/"
        # planePointsPath = self.identifiedPath + "/02 - Points from planes/"
        # imagesPlanePath = self.identifiedPath + "/03 - Images/"
        # self.identifiedPaths = [planeListPath, planePointsPath, imagesPlanePath]

        # self.processedPath = self.output_path + "/04 - Plane Processing"
        # processedResultsPath = self.processedPath + "/01 - Results/"
        # processedImagesPath = self.processedPath + "/02 - Images/"
        # self.processedPaths = [processedResultsPath, processedImagesPath]

        # self.shadingPath = self.output_path + "/05 - Shading Matrices"
        # self.pysamResultsPath = self.output_path + "/06 - PySAM Simulation"

    def __init__(self, building, output_path, srcLiDAR, temp_path="Results/_Temp"):
        self.building = pd.DataFrame(building).reset_index(drop=True)
        self.output_path = output_path + "/" + self.building.identifier[0]
        self.temp_path = temp_path
        self.srcLiDAR = srcLiDAR
        create_output_folder(self.temp_path)

        self.register_folders()

    # Step 0 - load the Data
    def loadData(self, LiDAR_info_path, cadastre_info_path, LiDAR_files_path, cadastre_files_path):
        self.LiDAR_limits = pd.read_csv(LiDAR_info_path) 
        self.cadastre_limits = pd.read_csv(cadastre_info_path)
        self.LiDAR_path = LiDAR_files_path
        self.cadastre_path = cadastre_files_path

    # Step 1 - segment LiDAR
    def segmentLiDAR(self, offset=1, stl_side=500):
        # Store data
        self.stl_side = stl_side
        self.LiDAR_offset = offset
        
        # Prepare output
        create_output_folder(self.segmented_path)
        
        self.segmentator = LidarSegmentator(self.building, self.temp_path)

        # Gets all LiDAR files within a "side" square and merge the files
        fileList = self.segmentator.findFilesForSquare(self.LiDAR_limits, self.stl_side)
        if(len(fileList) < 1):
            with open(self.output_path + "/log.csv", 'w') as f:
                f.write("Building ", self.building.identifier[0], " has no LiDAR data available")
        else:
            filenames = [self.LiDAR_path + "/" + fileList[x] for x in range(len(fileList))]
            destination = self.temp_path + "/temporalMerged.txt"
            self.LiDAR_extended = merge_txt(filenames, destination)
            
            # Checks each cadastre limits and looks for polygon only in those files where it is indicated it could be in 
            self.segmentator.find_potential_cadastre(self.cadastre_limits)
            self.foundCadastre = self.segmentator.poligon_cadastre(self.cadastre_path, self.srcLiDAR)
            if(self.foundCadastre):
                export_path = self.segmented_path + "/" + self.building.identifier[0]
                self.segmentator.export_geopackage(export_path)
                self.segmentator.LiDAR_poligon_segmentation(self.LiDAR_extended, export_path, self.LiDAR_offset, self.srcLiDAR)
            else:
                with open(self.output_path + "/log.csv", 'w') as f:
                    f.write("Building " + self.building.identifier[0] + " does not have cadastre info")
                print("Building " + self.building.identifier[0] + " does not have cadastre info")

    # Step 2 - Export the .stl file
    def createNeighborhood(self, export3D=True):
        self.stlGenerator = STLExporter(self.building, self.stl_side, self.squarePaths, self.temp_path, LAStoolsPath="C:/LAStools")
        self.stlGenerator.segmentSquare(self.LiDAR_extended)
        self.stlGenerator.txt_to_las()
        if(export3D):
            self.stlGenerator.las_to_dem()
            self.stlGenerator.dem_to_3d()

    # Step 3 - Plane identification with RANSAC
    def identifyPlanes(self, generateFigures=True, **kwargs):
        self.generateFigures = generateFigures
        self.planeDetector = PlaneDetector(self.building, self.segmented_path, self.identifiedPaths, self.generateFigures, **kwargs)
        self.planeDetector.detectPlanes()

    # Step 4 - Plane processing (with multiple criteria)    
    def processPlanes(self, generateFigures=True, **kwargs):
        cadastrePath = self.segmented_path + "/" + self.building.identifier[0] + ".gpkg"
        self.generateFigures = generateFigures
        self.planeProcessor = PlaneProcessor(self.building, self.segmented_path, self.identifiedPaths, self.processedPaths, cadastrePath, self.generateFigures, **kwargs)
        self.planeProcessor.loadIdentifiedData()
        self.planeProcessor.plotPlanes("From RANSAC" + self.building.identifier[0])

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
        self.planeProcessor.pierce(cadastre=False)
        self.planeProcessor.cadastreTrim()
        self.planeProcessor.exportResults()

    # Step 5 -Shading calculation
    def computeShading(self, generateFigures=True, **kwargs):
        if(generateFigures):
            self.shadingPath = self.shadingPath + " with figures"
        create_output_folder(self.shadingPath, deleteFolder=True)
        self.generateFigures = generateFigures

        planeListFile = self.processedPaths[0] + "PlaneList_" + self.building.identifier[0] + ".csv"
        self.planedf = pd.read_csv(planeListFile) # Yes, I'm doing this only to get the number of planes
        
        for planeIDShading in range(len(self.planedf)):
            self.shader = Shader(self.building, planeIDShading, self.squarePaths[0], self.squarePaths[3], self.processedPaths[0], self.shadingPath, **kwargs)
            self.shader.prepareDataShading()
            self.shader.sampleRoof()
            self.shader.shadingCalculation()
            if(self.generateFigures):
                self.shader.plotShadingMatrix(plotAll=True)
    
    # Step 6 -PySAM simulation
    def simulatePySAM(self, tmyfile, multipleTilts=True, ratio=float(0.450/2), **kwargs):
        # planeListFile = self.processedPaths[0] + "PlaneList_" + self.building.identifier[0] + ".csv"
        # self.planedf = pd.read_csv(planeListFile)
        # create_output_folder(self.pysamResultsPath, deleteFolder=True)

        # for planeIDMatrix in range(len(self.planedf)):
        #     self.pysam_simulator = PySAMSimulator(self.shadingPath, self.pysamResultsPath, self.planedf, planeIDMatrix, ratio, tmyfile, **kwargs)
        #     self.pysam_simulator.runPySAMSimulation()
        #     self.pysam_simulator.plotGenerationHeatmap()

        #     if(multipleTilts and (self.planedf.tilt[planeIDMatrix] < 10)):
        #         tilts = [0, 5, 10, 15, 20, 25, 30, 35, 40]
        #         azimuths = [90, 135, 180, 215, 270]
        #         for ti in tilts:
        #             for az in azimuths:
        #                 self.pysam_simulator.azimuth = az
        #                 self.pysam_simulator.tilt = ti
        #                 self.pysam_simulator.runPySAMSimulation()
        #                 self.pysam_simulator.plotGenerationHeatmap()


        planeListFile = self.processedPaths[0] + "PlaneList_" + self.building.identifier[0] + ".csv"
        self.planedf = pd.read_csv(planeListFile)
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

        for planeIDMatrix in range(len(self.planedf)):
            sampledPointsPath = self.shadingPath + "/" + str(planeIDMatrix) + "/Points sampled.csv"
            sampledPoints = pd.read_csv(sampledPointsPath, header = None)
            sampledPoints = sampledPoints.rename(columns={0: "x", 1: "y", 2:"z"})

            for i in range(len(sampledPoints)):
                roofIds.append(planeIDMatrix)
                tilts.append(self.planedf.tilt[planeIDMatrix])
                azimuths.append(self.planedf.azimuth[planeIDMatrix])
                areas.append(self.planedf.area[planeIDMatrix])
                pointsX.append(sampledPoints.x[i])
                pointsY.append(sampledPoints.y[i])

                shadingMatrixPath = self.shadingPath + "/" + str(planeIDMatrix) + "/Individual Matrices/" + str(i).zfill(2) + ".csv"
                resultsPath = self.pysamResultsPath + "/Yearly Results/" + str(planeIDMatrix) + "_"  + str(i).zfill(2)
                
                self.pysam_simulator = PySAMSimulator(shadingMatrixPath, resultsPath, self.planedf, planeIDMatrix, ratio, tmyfile, **kwargs)
                simulationResults = self.pysam_simulator.runPySAMSimulation()
                acAnnuals.append(simulationResults["ac_annual"])
                radiationAnnuals.append(simulationResults["solrad_annual"])
                # self.pysam_simulator.plotGenerationHeatmap()


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
        Plotter.plotEnergyMap(self.building, self.processedPaths[0], self.segmented_path, self.pysamResultsPath)
    
    def plotRadiationMap(self):
        Plotter.plotRadiationMap(self.building, self.processedPaths[0], self.segmented_path, self.pysamResultsPath)