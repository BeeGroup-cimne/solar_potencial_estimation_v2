# Solar Potential Estimaton Tool
## Objective

This program was developed to assess the photovoltaic potential on solar rooftops of a building based on LiDAR data. LiDAR is a technique used to  detect distance to objects and, in the cases of buildings and geographical regions, LiDAR data is a sample of points with an x, y and z coordinates. To help with the delimiting of each building, cadastre info is used to obtain the polygon outline.

![Goal](https://github.com/BeeGroup-cimne/solar_potencial_estimation_v2/assets/108261022/1f012108-87f7-44fb-80ad-5d55184c340f)

## Requirements
The following external apps are required:
- **LAStools** (available at https://lastools.github.io/download/): for las to txt conversion and viceversa.
- **QGIS** (available at https://qgis.org/en/site/forusers/download.html): for map operation.
    -   **DEMto3D**: QGIS plugin (can be installed within the program) to generate STL files

This was tested on a custom anaconda environment using a Jupyter Notebook. Check the *environment.yml* file to see the required packages (with the version that was proved to work).

## Program structure

This code is designed to work on multiple buildings. So it is expected to handle a list of buildings, more than one cadaster file (or one single cadaster file with multiple buildings in it), and all the LiDAR files available.

To do so, the program contains two main modules: **Data preparation** and **Solar Estimation**. The file *main.py* contains a use example for both of them (however, data needed for the program to run is not provided in this repository).

Since the program creates multiple middle-step results, it has all been encapsulated in different classes that comunicate with each other, so that the user of this program only needs to specify the input data paths and the path for the desired output.


## Data Preparation

For the program to work three elements of information are required:
- A list of buildings with an identifier and their coordinates.
- The cadaster file(s) containing the polygon/outline of each building.
- The LiDAR file(s) containing the points from the buildings as well as their neighborhods.

The *DataPreparator* class in this program is responsible of preparing these file to compatible formats and to, in case of having multiple buildings or a large amount of data, pre-scan the data to summarize the most important information to make quicker searches afterwords.

The class contains the following methods:
- **prepare_buildings:** converts the buildings list to the same coordiantes as the LiDAR data, and also deletes duplicates in the list.
- **prepare_cadaster:** scans each cadaster file to know its span (minimum and maximum corners of the file) and export that information to a list in the same coordinates as the LiDAR data.
- **prepare_LiDAR:** scans each LiDAR file and exports a list with the limits (minimum and maximum corners) of each file, to simplify searches in the future. In the case the LiDAR data is in .laz file, it also converts them to .txt, which is the extension that will be used in the program.

## Solar Estimation

Onces the data is prepared, the solar simulation can begin. This is done with the help of the *SolarEstimator* class, that handles of the steps in the simulation process, as well as managing the directories of each input and output.

Once the class is instantiated, it needs to be updated with the paths of all the data information (LiDAR and cadastre files) and, then, it can start all the process.

The whole program is designed in submodules, as shown in the following figure and as explained below. 

![SimulationProcess](https://github.com/BeeGroup-cimne/solar_potencial_estimation_v2/assets/108261022/2a94e86b-91cd-45df-9a3b-f6032a0fa63b)

### 1. LiDAR segmentation
This uses the *LidarSegmentator* class to generate a .csv file with the LiDAR points inside the building limits. A buffer to get some area outside the building can also be applied.
<p align="center">
<img src="https://github.com/BeeGroup-cimne/solar_potencial_estimation_v2/assets/108261022/f940ef56-8eae-4c72-b6b0-30cd410fdfd7" alt="LiDAR data" width="400"/>
</p>

### 2. STL Generation
Here, the *STLExporter* class is used to obtain the 3D model of the neighborhood (a square of the given side, defauls is 200m), that will later be used in the shading module. LASTools are required here for the 3D exportation.
<p align="center">
<img src="https://github.com/BeeGroup-cimne/solar_potencial_estimation_v2/assets/108261022/87bcc430-b5a4-4684-b86d-9362b19393d5" alt="3D model" width="600"/>
</p>


### 3. Plane identification
This part of the program calls the *PlaneDetector* class to identify the planes in the building rootop. Given the LiDAR file of only the building, the planes are identified by applying the RANSAC algorithm, which tries to fit different planes (stochasticly genreated) to the given data, and returns the ones that had a better fit. To get faster and better results, the data is first split by height discontinuities and the selection is not entirely random, but instead points with a most common gradient are more likely to be sampled.
<p align="center">
<img src="https://github.com/BeeGroup-cimne/solar_potencial_estimation_v2/assets/108261022/43edf422-d2a1-4af6-b106-cab69e08853c" alt="Plane Identification" width="400"/>
</p>

### 4. Plane processing
Once the planes have been succesfully identified, some postProcessing techniques are applied, with the *planeProcessor* class. This postprocessing techinques are:
- Merges planes that are too similar.
- Splits planes if there are discontinuities
- Deletes planes of bad quality (too small, too few data points or density is too low).
- Deletes overlaps.
- Pierces holes (areas in which there are not points).
- Trim according to cadaster limits
<p align="center">
<img src="https://github.com/BeeGroup-cimne/solar_potencial_estimation_v2/assets/108261022/bf221b27-0b95-42ac-a69a-e0805cac953d" alt="Plane Process" width="400"/>
</p>

### 5. Shading computation
With the definitive planes of the building, the shading calculation can begin (using the *Shader* module). To shade a rooftop, some points are sampled and, for each point, the shading matrix is calculated.

To calculate the shading matrix of a point, the 3D model previously generated is loaded into the program and, from the sampled point, rays are sent in all the directions and, in those directions that the ray collides with the 3d model, that direction (altitude and azimuth) is registered to be shaded. This is done for all the points sampled t ogather different areas of the same rooftop (specially for larger rooftops)

A matrix like the following (where the horizontal axis is the ray azimtuh and the vertical axis is their altitude) is generated for each sampled point.
<p align="center">
<img src="https://github.com/BeeGroup-cimne/solar_potencial_estimation_v2/assets/108261022/f466e6ae-db83-4e30-a050-972a71820b93" alt="Shading" width="400"/>
</p>

### 6. PySAM simulation
Here, the *PySAMSimulator* class is applied to obtain the PV generation of the points sampled and shaded in the previous step. 

This simulation is done using the PySAM library, that integrates the System Advisor Model (SAM, from NREL) capabilities in a Python module. To perform the simulation, a typical meteorological year (TMY) file of the region of interest is needed, which can be downloaded at (https://nsrdb.nrel.gov/data-viewer).

The final results are the yearly generation of all the sampled points in the rooftop. These results can be used to estimate the PV potential of a building but beware that the final results are an estimation **considering that the whole rooftop was covered in solar panels**, which might not be always feasible. The more realistic results are the density of yearly energy generation per square meterm which can be used to determine if it is worth to install panels in a particular rooftop.
<p align="center">
<img src="https://github.com/BeeGroup-cimne/solar_potencial_estimation_v2/assets/108261022/6b0947f4-c353-42bd-b3be-11544bdc21ce" alt="Solar estimation" width="400"/>
</p>


## Use Case example

This code was used in the ePLANET H2020 European project to analyze the PV potential for public buildings in the Zlín Region (Czech Republic). The results are available at https://zenodo.org/records/12705189.

## Authors
- Killian Premel
- Jaume Asensio - jaume.asensio.upc.edu
- Maria Maiques - maria.maiques@estudiantat.upc.edu
- Gerard Laguna - glaguna@cimne.upc.edu
- Jordi Cipriano - cipriano@cimne.upc.edu
- Gerard Mor gmor@cimne.upc.edu
- Jose Manuel Broto - jmbroto@cimne.upc.edu
- Francesc Contreras - fcontreras@cimne.upc.edu

Copyright (c) 2024 Jaume Asensio, Gerard Laguna, Jordi Cipriano, Killian Premel, Maria Maiques, Gerard Mor, Jose Manuel Broto, Francesc Contreras
