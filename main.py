from Classes.DataPreparator import DataPreparator
from Classes.SolarEstimator import SolarEstimator
import pandas as pd

buildings_path = "Sample/Data/Buildings lists/Buildings Campus Terrassa.csv"
cadastre_path = "Sample/Data/Cadastre files"
LiDAR_path = "Sample/Data/LiDAR files"
output_path = "Sample/Results/Data Preparation"

def prepare_alldata(data_prepPaths, LiDAR_target, buildings_source=4326, cadstre_source=4326, las2txtPath="C:/LAStools"):
    dataprep = DataPreparator(data_prepPaths[0], data_prepPaths[1], data_prepPaths[2], data_prepPaths[3])
    dataprep.prepare_buildings(source=buildings_source, target=LiDAR_target)
    dataprep.prepare_cadastre(source=cadstre_source, target=LiDAR_target)
    dataprep.prepare_LiDAR(las2txtPath=las2txtPath)
    print("\t Data preparation done")

def simulateBuilding(building, simulationPaths, crsCadaster, srcLiDAR):
    solarestimator = SolarEstimator(building, simulationPaths[0], srcLiDAR=srcLiDAR, square_side=500, temp_path="Sample/Results/_Temp")
    solarestimator.loadData(simulationPaths[1], simulationPaths[2], simulationPaths[3], simulationPaths[4])
    print("\t Starting simulation")

    solarestimator.segmentLiDAR(square_side=500)   
    solarestimator.createNeighborhood(LAStoolsPath="C:/LAStools", export3D=True) 
    print("\t Square neighborhood generation done")

    solarestimator.identifyPlanes(minGlobalPercentage=0.05, minPartialPercentage=0.5, heightThreshold=0.5, distanceThreshold=0.1, ransacIterations=100, densityMultiplier=0.5, stoppingPercentage=0.2, pdfExponent=2)
    print("\t Plane identification done")

    solarestimator.processPlanes(crsCadaster=crsCadaster, generateFigures=True, slidingHole=0.75, minHoleSide = 2.5)
    print("\t Plane processing done")

    solarestimator.computeShading(generateFigures=False, Nsamples=25, div=2, bufferSize=1, shadeInside=True)
    print("\t Shading done")

    solarestimator.simulatePySAM(simulationPaths[5], generateFigures=False)
    solarestimator.plotEnergyMap()
    print("\t PV simulation finished")

if __name__ =="__main__":
    # For data preparation
        # To modify
    buildings_path = "Sample/Data/Buildings lists/Buildings Campus Terrassa.csv"
    cadastre_path = "Sample/Data/Cadastre files"
    LiDAR_path = "Sample/Data/LiDAR files"
    output_path = "Sample/Results/Data Preparation"
        # Do not touch
    data_prepPaths = [buildings_path, cadastre_path, LiDAR_path, output_path]
    prepare_alldata(data_prepPaths, LiDAR_target=25831)

    # For PV simulation
        # To modify
    buildings_info_path = r"C:\Users\jaasb\INVESTIGO\BEE Group\eplanet shared\Programa Final\Results\Data Preparation\Buildings\Buildings_filtered.csv"
    # buildingsID = ['eP-EAZK-050', 'eP-EAZK-076', 'eP-EAZK-085', 'eP-EAZK-162', 'eP-EAZK-163', 'eP-EAZK-165']
    buildingsID = ['eP-EAZK-076']

    LiDAR_info_path = "Sample/Results/Data Preparation/LiDAR/LiDAR_Limits.csv"
    cadastre_info_path = "Sample/Results/Data Preparation/Cadastre/Cadastre_Limits.csv"
    LiDAR_path = "Sample/Data/LiDAR files"
    cadastre_path = "Sample/Data/Cadastre files"
    tmyfile = "Sample/Data/TMY_Terrassa-2018.csv"
    output_path = "Sample/Results"

    crsCadaster=3035
    crsLiDAR=5514

        # Do not touch
    simulationPaths = [output_path, LiDAR_info_path, cadastre_info_path, LiDAR_path, cadastre_path, tmyfile]
    buildings = pd.read_csv(buildings_info_path) 
    for buildingID in buildingsID:
        building = buildings[buildings.identifier == buildingID].reset_index(drop=True)
        simulateBuilding(building, simulationPaths, crsCadaster, crsLiDAR)
    
    print("Done")


