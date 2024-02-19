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
    """
    Used for computing several shading matrices of the planes of the given plane 
    
    ### Attributes: 
    #### Defined upon initialization:
    - file_names: THIS MUST NOT BE MODIFIED. Path to pysam_templates (relative path is already defined)
    - shadingMatrixPath: .csv containing the shading path, as obtained from the Shader module
    - planeID: (integer) id of the plane to simulate (id obtained from plane identification, the same id as the one used for shading)
    - area: plane area, in m^2 
    - ratio: watts per square meter ratio. This is used because the simulator needs, as an input, a solar capactiy, not an area.
    - tilt: plane tilt, in degrees
    - azimuth: plane azimuth, in degrees
    - tmyfile: .csv of the TMYfile, must be otained from NREL (https://nsrdb.nrel.gov/data-viewer) or converted to the same format
    - exportPath: path to export the simulation results.

    #### Self-generated:
    - pv: module (json) containing the PVWatts simulation parameters. Once simulated, it also contains the results
    - grid: module (json) containing the grid simulation parameters.

    - shadingMatrix: shading matrix of the current point, in the format needed for PySAM

    - generation: 24x365 matrix containing the hourly generation

    ### Public methods:
    - runPySAMSimulation: runs the PV simulation for each sampled point, with their respective shading matrix. Updates the generation attribute
    - plotGenerationHeatmap: plots the yearly PV generation (hour of day vs. day of the year) as a heatmap
    """

    def __init__(self, shadingPath, exportPath, planedf, planeID, ratio, tmyfile):
        """ 
     #### Inputs:
    - shadingMatrixPath: .csv containing the shading path, as obtained from the Shader module
    - exportPath: path to export the simulation results.
    - planedf: dataframe containing the area, tilt and azimuth of each plane found in the building. Simulation is performed only on one plane at a time 
    - planeID: (integer) id of the plane to simulate (id obtained from plane identification, the same id as the one used for shading)
    - ratio: watts per square meter ratio. This is used because the simulator needs, as an input, a solar capactiy, not an area.
    - tmyfile: .csv of the TMYfile, must be otained from NREL (https://nsrdb.nrel.gov/data-viewer) or converted to the same format
       """
        self.file_names = ["./Classes/pysam_template_pvwattsv8", "./Classes/pysam_template_grid"]
        self.shadingMatrixPath = shadingPath
        self.planeID = planeID
        self.area = planedf.area[planeID]
        self.ratio = ratio
        self.tilt = planedf.tilt[planeID]
        self.azimuth = planedf.azimuth[planeID]
        self.tmyfile = tmyfile
        self.exportPath = exportPath

        # create_output_folder(self.exportPath)
        
    
    def __loadModules(self):
        """ 
    From the default directory, loads the json files containing the necessary info and returns it into two modules: pv and grid

    #### Inputs:
    - None

    #### Outputs:
    - None (updates the pv, grid and modules attributes)
        """
        self.pv = PVWatts.new()
        self.grid = Grid.from_existing(self.pv)

        modules = [self.pv, self.grid]

        for f, m in zip(self.file_names, modules):
            with open(f + ".json", 'r') as file:
                data = json.load(file)
                # loop through each key-value pair
                for k, v in data.items():
                        m.value(k, v)

        
    def __readShadingMatrix(self):
        """ 
    Returns the shadingMatrix from the given matrixFile (csv)

    #### Inputs:
    - None (knows shading matrix path from object initialization)

    #### Outputs:
    - None (updates the shadingMatrix attriibute)
        
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

        
    def runPySAMSimulation(self):
        """ 
    Runs the PV simulation for each sampled point, with their respective shading matrix. Updates the generation, pv, grid and modules attributes

    #### Inputs:
    - None

    #### Outputs: 
    - pv.export(): a json file containing all the parameters from the simulation, as well as its outputs (with hourly generation included)

    #### Exports: 
    - None
        """

        self.__loadModules()
        self.__readShadingMatrix()
        
        modifiedParams = {"shading_azal": self.shadingMatrix,
            "system_capacity": self.area*self.ratio, #*self.pv.value("gcr"), #We don't need the area by the ground coverage ratio
            "tilt": self.tilt,
            "azimuth": self.azimuth,
            "solar_resource_file": self.tmyfile}

        for i in range(len(modifiedParams)): 
            self.pv.value(list(modifiedParams.keys())[i], list(modifiedParams.values())[i])

        modules = [self.pv, self.grid]
        
        for m in modules:
            m.execute()

        self.generation = self.pv.export()["Outputs"]["ac"]
        self.generation = np.array(self.generation).reshape(365, 24)
        generation_df = pd.DataFrame(self.generation)
        
        generation_df.to_csv((self.exportPath + '.csv'), index=False)
        return self.pv.export()["Outputs"]
    
    
    def plotGenerationHeatmap(self):
        """ 
    Plots the yearly PV generation (hour of day vs. day of the year) as a heatmap

    #### Inputs:
    - None

    #### Outputs:
    - None

    #### Exports:
    - .png image of the yearly heatmap plot
        """

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

        savename = self.exportPath + "_Generation_heatmap_Tilt" + str(round(self.tilt, 1)) + "_Azimuth" + str(round(self.azimuth, 1)) + ".png"

        fig.savefig(savename)
        plt.close()