# QGIS and processing related-modules
from qgis.core import *
QgsApplication.setPrefixPath("", True)
from processing.core.Processing import Processing

# Initialize QGIS Application and Processing framework
app = QgsApplication([], True)
QgsApplication.initQgis()
Processing.initialize()

import pandas as pd
import warnings
warnings.filterwarnings('ignore')

def prepare_alldata(data_prepPaths, LiDAR_target, las2txtPath="C:/LAStools/bin/las2txt.exe", buildings_source=4326, cadastre_source=4326):
    from Classes.DataPreparator import DataPreparator
    dataprep = DataPreparator(data_prepPaths[0], data_prepPaths[1], data_prepPaths[2], data_prepPaths[3])
    # dataprep.prepare_buildings(source=buildings_source, target=LiDAR_target)
    dataprep.prepare_cadaster(source=cadastre_source, target=LiDAR_target)
    # dataprep.prepare_LiDAR(las2txtPath=las2txtPath)
    print("Data preparation done")

def simulateBuilding(building, simulationPaths, crsCadaster, crsLiDAR):
    from Classes.SolarEstimator import SolarEstimator
    print(building.identifier.values[0])
    solarestimator = SolarEstimator(building, simulationPaths[0], crsLiDAR=crsLiDAR, square_side=500, temp_path="./Sample/Results/_Temp")
    solarestimator.loadData(simulationPaths[1], simulationPaths[2], simulationPaths[3], simulationPaths[4])
    print("\t Starting simulation")

    solarestimator.segmentLiDAR(square_side=200)   
    # solarestimator.createNeighborhood(txt2lasPath="/home/jaumeasensio/lastools/bin/txt2las64", las2demPath="/home/jaumeasensio/lastools/bin/las2dem64", export3D=False, square_side=200) 
    # print("\t Square neighborhood generation done")

    solarestimator.identifyPlanes(minGlobalPercentage=0.1, minPartialPercentage=0.2, heightThreshold=0.5, distanceThreshold=0.2, 
                                  pdfExponent=2, ransacIterations=250, densityMultiplier=0.75, stoppingPercentage=None, generateFigures=True)
    print("\t Plane identification done")

    solarestimator.processPlanes(crsCadaster=crsCadaster, generateFigures=True, slidingHole=0.75, minHoleSide = 2.5)
    print("\t Plane processing done")

    # solarestimator.computeShading(generateFigures=False, Nsamples=25, div=2, bufferSize=1)
    # print("\t Shading done")

    # solarestimator.simulatePySAM(simulationPaths[5], generateFigures=False)
    # solarestimator.plotEnergyMap()
    # print("\t PV simulation finished")

if __name__ =="__main__":
    # # For data preparation
    #     # To modify
    # buildings_path = "./Sample/Data/Buildings lists/Buildings List.csv"
    # cadaster_path = "./Sample/Data/Cadaster files"
    # LiDAR_path = "./Sample/Data/LiDAR files/DVDs"
    # output_path = "./Sample/Results/Data Preparation"
    #     # Do not touch
    # data_prepPaths = [buildings_path, cadaster_path, LiDAR_path, output_path]
    # las2txtPath = "/home/jaumeasensio/Software/LAStools/bin/las2txt64"
    # prepare_alldata(data_prepPaths, LiDAR_target=5514, las2txtPath=las2txtPath, cadastre_source=3035)

    # For PV simulation
        # To modify
    buildings_info_path = "./Sample/Results/Data Preparation/Buildings/Buildings_filtered.csv"
    # buildingsID = ['eP-EAZK-029', 'eP-EAZK-051', 'eP-EAZK-084', 'eP-EAZK-099', 'eP-EAZK-184', 'eP-EAZK-276', 'eP-EAZK-277', 'eP-EAZK-280', 'eP-EAZK-281', 'eP-EAZK-305']
    buildingsID = ['eP-EAZK-029', 'eP-EAZK-276', 'eP-EAZK-277']

    LiDAR_info_path = "./Sample/Results/Data Preparation/LiDAR/LiDAR_Limits.csv"
    cadaster_info_path = "./Sample/Results/Data Preparation/Cadaster/Cadaster_Limits.csv"
    LiDAR_path = "./Sample/Data/LiDAR files/DVDs"
    cadaster_path = "./Sample/Data/Cadaster files"
    tmyfile = "./Sample/Data/TMY_Zlin-2018.csv"
    output_path = "./Sample/Results"
    
    crsCadaster=3035
    crsLiDAR=5514

        # Do not touch
    simulationPaths = [output_path, LiDAR_info_path, cadaster_info_path, LiDAR_path, cadaster_path, tmyfile]
    buildings = pd.read_csv(buildings_info_path) 
    for buildingID in buildingsID:
        building = buildings[buildings.identifier == buildingID].reset_index(drop=True)
        simulateBuilding(building, simulationPaths, crsCadaster, crsLiDAR)
    
    print("Simulation done")