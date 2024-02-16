import json
import pandas as pd
import numpy as np
import PySAM.Pvwattsv8 as PVWatts
import PySAM.Grid as Grid
import csv
import matplotlib.pyplot as plt
import math
from datetime import datetime
from matplotlib.dates import DateFormatter

from Functions.general_functions import create_output_folder

class PySAMSimulator:
    def __init__(self, shadingPath, exportPath, planedf, planeID, ratio, tmyfile):
        self.file_names = ["./Classes/pysam_template_pvwattsv8", "./Classes/pysam_template_grid"]
        # self.shadingMatrixPath = shadingPath + "/" + str(planeID) + "/Average_" + str(planeID) + ".csv"
        self.shadingMatrixPath = shadingPath
        self.planeID = planeID
        self.area = planedf.area[planeID]
        self.ratio = ratio
        self.tilt = planedf.tilt[planeID]
        self.azimuth = planedf.azimuth[planeID]
        self.tmyfile = tmyfile
        self.exportPath = exportPath

        # create_output_folder(self.exportPath)
        
    
    def loadModules(self):
        """
        From the default directory, loads the json files containing the necessary info and returns it into two modules: pv and grid
        """
        self.pv = PVWatts.new()
        self.grid = Grid.from_existing(self.pv)

        self.modules = [self.pv, self.grid]

        for f, m in zip(self.file_names, self.modules):
            with open(f + ".json", 'r') as file:
                data = json.load(file)
                # loop through each key-value pair
                for k, v in data.items():
                        m.value(k, v)

        
    def readShadingMatrix(self):
        """
        Returns the shadingMatrix from the given matrixFile (csv)
        """
        self.shadingMatrix = []

        with open(self.shadingMatrixPath) as csvfile:
            reader = csv.reader(csvfile)
            for row in reader:
                singleRow = []
                for element in row:
                    if(element == ''):
                        singleRow.append(0.0)
                    else:    
                        singleRow.append(float(element))
                self.shadingMatrix.append(singleRow)

    
    def modifyParameters(self):
        """
        Returns a json containing the different parameters that can be tuned: 
        - shadingMatrix (read in the previous function) 
        - area and ratio, for kW calculation
        - tilt and azimuth
        - tmyfile 
        """
        self.modifiedParams = {"shading_azal": self.shadingMatrix,
                    "system_capacity": self.area*self.ratio, #*self.pv.value("gcr"), #We don't need the area by the ground coverage ratio
                    "tilt": self.tilt,
                    "azimuth": self.azimuth,
                    "solar_resource_file": self.tmyfile}
        
    def runPySAMSimulation(self):
        """
    #### Inputs:
    - matrixFile (str): directory where the shading matrix is
    - area (float): rooftop area
    - ratio (float): in kW/m^2 (a good value is 0.450/2)
    - tilt, azimuth (float): tilt and azimuth from plane processing
    - tmyfile (str): path where the TMY file is

    #### Outputs: 
    - pv.export(): a json file containing all the parameters from the simulation, as well as its outputs (with hourly generation included)

    #### Exports: 
    None
        """

        self.loadModules()
        self.readShadingMatrix()
        self.modifyParameters()

        for i in range(len(self.modifiedParams)): 
            self.pv.value(list(self.modifiedParams.keys())[i], list(self.modifiedParams.values())[i])

        self.modules = [self.pv, self.grid]
        
        for m in self.modules:
            m.execute()

        self.generation = self.pv.export()["Outputs"]["ac"]
        self.generation = np.array(self.generation).reshape(365, 24)
        generation_df = pd.DataFrame(self.generation)
        
        generation_df.to_csv((self.exportPath + '.csv'), index=False)
        return self.pv.export()["Outputs"]
    
    
    def plotGenerationHeatmap(self):

        fig, ax = plt.subplots(figsize=(12, 6))

        ax.set_title('Generation with tilt of ' + str(round(self.tilt, 1)) + ' and azimuth of ' + str(round(self.azimuth, 1)) + ' degrees: ' + str(round(self.pv.export()["Outputs"]["ac_annual"], 0)) + " kW")

        # Create a heatmap
        sc = ax.imshow(np.matrix.transpose(self.generation), cmap='rainbow', aspect='auto')  # Use a colormap of your choice

        plt.colorbar(sc)  # Add a colorbar with a label

        # Set axis labels and title
        ax.set_xlabel('Day of the Year')
        ax.set_ylabel('Hour of the Day')

        myDates = [datetime(2018,i+1,1) for i in range(12)]

        days = [31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31]

        # Format the x-axis ticks as dates
        ax.set_xticks([1 + sum(days[0:i]) for i in range(len(days))])  # Adjust the ticks as needed
        ax.set_xticklabels(myDates)
        ax.xaxis.set_major_formatter(DateFormatter("%b"))

        savename = self.exportPath + '/' + str(self.planeID) + "_Generation_heatmap_Tilt" + str(round(self.tilt, 1)) + "_Azimuth" + str(round(self.azimuth, 1)) + ".png"

        fig.savefig(savename)
        plt.close()