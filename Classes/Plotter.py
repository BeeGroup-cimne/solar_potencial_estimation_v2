import pandas as pd
import matplotlib.pyplot as plt
import matplotlib as mpl
import numpy as np
from shapely import wkt

class Plotter:
    @staticmethod
    def plotMap(building, processedResultsPath, segmented_path, pysamResultsPath, magnitude):
        
        centerx = building.x[0]
        centery = building.y[0]

        planedf = pd.read_csv(processedResultsPath + "PlaneList_" + building.identifier[0] + ".csv")

        buildingPoints = pd.read_csv(segmented_path + "/" + building.identifier[0] + ".csv", header=None)
        buildingPoints = buildingPoints.rename(columns={0:"x", 1:"y", 2:"z"})
        buildingPoints.x = buildingPoints.x - centerx
        buildingPoints.y = buildingPoints.y - centery
        
        pysam_df = pd.read_csv(pysamResultsPath + "/Summary.csv")

        if(magnitude == "ac_annual"):
            plottingMagnitude = pysam_df.ac_annual/pysam_df.area
            savename = pysamResultsPath + "/" + building.identifier[0] + " - EnergyDensity.png"
            cmap = mpl.cm.plasma #coolwarm, plasma, rainbow, cividis, YlOrRd

        elif(magnitude == "daily_radiation"):
            plottingMagnitude = pysam_df.solarIrradiance
            savename = pysamResultsPath + "/" + building.identifier[0] + " - Daily radiation.png"
            cmap = mpl.cm.summer #coolwarm, plasma, rainbow, cividis, YlOrRd

        
        _min, _max = np.amin(plottingMagnitude), np.amax(plottingMagnitude)
        
        norm = mpl.colors.Normalize(vmin=_min, vmax=_max)
        sm = plt.cm.ScalarMappable(cmap=cmap, norm=norm) 
        sm.set_array([]) 

        fig, ax = plt.subplots()

        for i in range(len(planedf)):
            currentdf = pysam_df[pysam_df.roofID==i]
            x = currentdf.x
            y = currentdf.y
            if(magnitude == "ac_annual"):
                z = currentdf.ac_annual/currentdf.area
            elif(magnitude == "daily_radiation"):
                z = currentdf.solarIrradiance
            
            if(len(x.unique()) > 1 and len(y.unique()) > 1):
                cntr1 = ax.tricontourf(x, y, z, cmap=cmap, norm=norm, alpha=1)

            polygon = wkt.loads(planedf.trimmedPolygon[i])
            exterior_x, exterior_y = polygon.exterior.xy
            exterior_x = exterior_x - centerx
            exterior_y = exterior_y - centery
            plt.plot(exterior_x, exterior_y, "black", linewidth=0.5)


            # Create a list of interior rings
            interior_x, interior_y = [], []
            for interior in polygon.interiors:
                x, y = interior.xy
                interior_x.append(x)
                interior_y.append(y)

            for j in range(len(interior_x)):
                hole_x = interior_x[j] - centerx
                hole_y = interior_y[j] - centery
                plt.fill(hole_x, hole_y, c="white")
                plt.plot(hole_x, hole_y, "black", linewidth=0.5)
                
        ax.scatter(buildingPoints.x, buildingPoints.y, c="gray", alpha=0.05)

        fig.colorbar(sm,  ax=ax)

        # plt.subplots_adjust(hspace=0.5)
        ax.set_aspect('equal', adjustable='box')

        ax.grid(alpha=0.25)
        fig.savefig(savename)

    @staticmethod
    def plotEnergyMap(building, processedResultsPath, segmented_path, pysamResultsPath):
        Plotter.plotMap(building, processedResultsPath, segmented_path, pysamResultsPath, "ac_annual")