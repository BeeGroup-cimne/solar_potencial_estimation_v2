import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import trimesh # For stl stuff
from shapely import Polygon # For polygon
from shapely.geometry import Point # Points
from scipy.spatial import ConvexHull # Create polygon
import math # Trigonometric functions
from Functions.general_functions import create_output_folder
import shapely.wkt, shapely.affinity
from sklearn.decomposition import PCA

class Shader:
    """
    Used for computing several shading matrices of the planes of the given plane 
    
    ### Attributes: 
    #### Defined upon initialization:
    - building: single-row dataframe containing x, y and identifier of the desired building
    - planeID: (integer) id of the it is desired to shade
    - pointCloud_path: path with the .txt file of the neighborhood (square of stl_side by stl_side). This is needed so that the minimum height of the building and the minimum height of the 3d model can be equalized
    - STL_path: path with the .stl file of the neighborhood (to shade)
    - planes_path: path where the files relative to the plane (plane list and points within plane) are
    - shadingResultsPath: path to store the shading results
    ##### Attributes with default values
    - Nsamples: maximum number of points to be sampled from a plane. This is orientative, as it does not count sampling the plane outline, and is not correct for planes with irregular shape
    - div: minimum distance between sampled points. This ensures that, for smaller planes, (if Nsamples is too high) the sampling does not become redundant 
    - anglesAzimuth: array with the minimum and maximum angles to sweep [angleMin, angleMax]. Azimuth begins at North=0º and positive goes clockwise
    - stepAzimuth: resolution of the angle step while looping in azimuth sweep
    - anglesTilt: array with the minimum and maximum angles to sweep [angleMin, angleMax]. Tilt begins at horizontal=0º and positive goes upwise (until max 90º, although it can be stopped before)
    - stepTilt: resolution of the angle step while looping in tilt sweep
    - bufferSize: buffer distance (in meters) around plane to avoid being shaded by itself

    #### Self-generated:
    - planeParams: dataframe containing a, b, c, d parameters of the plane equation
    - planedf: dataframe containing all x, y, z points (from raw LiDAR data) belonging to the current plane
    - trimmedPolygon: shapely Polygon object, containing the trimmed polygon of the plane (to know where to sample)
    - mesh: mesh item (containing points and triangles) loaded from the stl file
    - planeResultsPath: path where individual matrices will be stored for the current plane

    - points: list containing 3 arrays (x, y and z coordinates) for the points sampled from the current plane
    
    - matrix: list with all shading matrices, for all sampled points
    - intersedctedList: list containing, for each sampled point, sets of 2 arrays, containing the data of intersections (intersected x,y coordinates and elevation of the ray that caused the intersection)
    - averageMatrix: average matrix, considering all the sampled points

    ### Public methods:
    - prepareDataShading: loads into the class all the data needed for shading. Updates the planeParam, planedf, trimmedPolygon, mesh, planeResultsPath attributes.
    - sampleRoof: knowing the plane (attribute from the object), obtains an array of points to shade
    - shadingCalculation: computes the shading matrix of the current plane. Updates the matrix, intersectedList, averageMatrix attributes.
    - plotShadingMatrix: given the shading matrices, exports the average matrix as an image and, if desired, the individual matrices and the images of rays sending
    """

    def __init__(self, building, planeID, pointCloud_path, STL_path, planes_path, shadingResultsPath, 
                 Nsamples=50, div=2, anglesAzimuth=[60, 300], stepAzimuth=3, anglesTilt=[0, 87], stepTilt=3, bufferSize=1):
        """ 
    #### Inputs:
    - building: single-row dataframe containing x, y and identifier of the desired building
    - planeID: (integer) id of the it is desired to shade
    - pointCloud_path: path with the .txt file of the neighborhood (square of stl_side by stl_side). This is needed so that the minimum height of the building and the minimum height of the 3d model can be equalized
    - STL_path: path with the .stl file of the neighborhood (to shade)
    - planes_path: path where the files relative to the plane (plane list and points within plane) are
    - shadingResultsPath: path to store the shading results
    - Nsamples: maximum number of points to be sampled from a plane. This is orientative, as it does not count sampling the plane outline, and is not correct for planes with irregular shape
    - div: minimum distance between sampled points. This ensures that, for smaller planes, (if Nsamples is too high) the sampling does not become redundant 
    - anglesAzimuth: array with the minimum and maximum angles to sweep [angleMin, angleMax]. Azimuth begins at North=0º and positive goes clockwise
    - stepAzimuth: resolution of the angle step while looping in azimuth sweep
    - anglesTilt: array with the minimum and maximum angles to sweep [angleMin, angleMax]. Tilt begins at horizontal=0º and positive goes upwise (until max 90º, although it can be stopped before)
    - stepTilt: resolution of the angle step while looping in tilt sweep
    - bufferSize: buffer distance (in meters) around plane to avoid being shaded by itself
        """
        self.building = building
        self.planeID = planeID
        self.pointCloud_path = pointCloud_path + "/" + self.building.identifier[0] + ".txt"
        self.STL_path = STL_path  + "/" + self.building.identifier[0] + ".stl"
        self.planes_path = planes_path
        self.shadingResultsPath = shadingResultsPath
    
        self.Nsamples = Nsamples
        self.div = div
        self.anglesAzimuth = anglesAzimuth
        self.stepAzimuth = stepAzimuth
        self.anglesTilt = anglesTilt
        self.stepTilt = stepTilt
        self.bufferSize = bufferSize

    def prepareDataShading(self):
        """ 
    Loads into the class all the data needed for shading. Updates the planeParam, planedf, trimmedPolygon, mesh, planeResultsPath attributes.

    #### Inputs:
    - None

    #### Outputs:
    - None
        """
        # Get point cloud center
        centerX, centerY = self.building.x[0], self.building.y[0]

        # Load point cloud and change to same coordinate system as 3d model
        pointsNeighborhood = pd.read_csv(self.pointCloud_path, header=None, sep = " ")
        pointsNeighborhood = pointsNeighborhood.rename(columns={0: "x", 1: "y", 2:"z"})

        minz = pointsNeighborhood["z"].min()

        # Get plane equation (d is fixed so that we can recalculate points later)
        planeParamsPath = self.planes_path + "/PlaneList_" + self.building.identifier[0] + ".csv"
        self.planeParams = pd.read_csv(planeParamsPath)
        self.planeParams = self.planeParams.iloc[self.planeID]
        self.planeParams.d = self.planeParams.d + (self.planeParams.a*centerX + self.planeParams.b*centerY + self.planeParams.c*minz) 

        # Get plane points and polygon
        readPlanePath = self.planes_path + self.building.identifier[0] + "_" + str(self.planeID) + ".csv"
        self.planedf = pd.read_csv(readPlanePath, header=None)
        self.planedf = self.planedf.rename(columns={0: "x", 1: "y", 2:"z"})
        self.planedf = self.planedf[["x","y","z"]]

        self.planedf["x"] = self.planedf["x"] - centerX
        self.planedf["y"] = self.planedf["y"] - centerY
        self.planedf["z"] = self.planedf["z"] - minz # !The min has to be pointsdf.min(), so that heights are the same

        # Load trimmed polygon
        self.trimmedPolygon = shapely.wkt.loads(self.planeParams.trimmedPolygon)
        self.trimmedPolygon = shapely.affinity.translate(self.trimmedPolygon, xoff=-centerX, yoff=-centerY)

        # Load stl
        self.mesh = trimesh.load(self.STL_path)
        self.mesh.vertices[:] = self.mesh.vertices[:] - [(self.mesh.bounds[1][0])/2, (self.mesh.bounds[1][1])/2, 0] # Must be centered arround building

        # Prepare output folder
        self.planeResultsPath = self.shadingResultsPath + "/" + str(self.planeID) + "/" 
        create_output_folder(self.planeResultsPath, deleteFolder=True)

    @staticmethod
    def __computePCA(planedf):
        """ 
    Given the plane, computes the PCA to get the ideal directions to sample

    #### Inputs:
    - planedf: containing "x" and "y" of the desired plane

    #### Outputs:
    - pca: pca object (will be used to obtain principal directions)
        """
        X = planedf[["x", "y"]]
        pca = PCA(n_components=2)
        pca.fit(X)
        return pca

    def sampleRoof(self):
        """ 
    Knowing the plane (attribute from the object), obtains an array of points to shade. Updates the points attribute

    #### Inputs:
    - None (used some of the object's attributes)

    #### Outputs:
    - None (the attribute points is update to contain 3 array of the X, Y and Z coordinates of teh sampled points)

    ### Exports:
    - .csv containing the sampled points (x, y and z coordinates), in the system of reference of the mesh (that is, the building centroid is 0,0)
        """
        pca = Shader.__computePCA(self.planedf)
        
        newData = pca.transform(self.planedf[["x", "y"]])
        newX = []
        newY = []
        for newPoint in newData:
            newX.append(newPoint[0])
            newY.append(newPoint[1])
        minX, minY, maxX, maxY = min(newX), min(newY), max(newX), max(newY)
        
        deltaX = maxX - minX
        deltaY = maxY - minY
        div = max(math.sqrt(deltaX*deltaY/self.Nsamples), self.div)

        xPCA = [minX + i * div for i in range(math.floor((maxX - minX) / div) + 1)] # +1 because we add one for the minimum and one for the maximmum
        yPCA = [minY + i * div for i in range(math.floor((maxY - minY) / div) + 1)]
        xvPCA, yvPCA = np.meshgrid(xPCA, yPCA)

        xv = xvPCA.copy()
        yv = yvPCA.copy()

        for i in range(len(xv[0])):
            for j in range(len(yv)):
                xv[j][i], yv[j][i] = np.dot([xvPCA[0][i], yvPCA[j][0]], pca.components_) + pca.mean_
        
        pointsX = []
        pointsY = []
        pointsZ = []

        for i in range(len(xv[0])):
            for j in range(len(yv)):
                point = Point(xv[j][i], yv[j][i])
                if (point.within(self.trimmedPolygon) == True):
                    pointsX.append(xv[j][i])
                    pointsY.append(yv[j][i])
                    pointsZ.append(-1/self.planeParams.c * (self.planeParams.a*xv[0][i]+ self.planeParams.b*yv[j][0]+ self.planeParams.d))
        
        xarray= np.array(self.trimmedPolygon.exterior.xy[0])
        yarray = np.array(self.trimmedPolygon.exterior.xy[1])

        pointsX.extend(xarray)
        pointsY.extend(yarray)
        pointsZ.extend(-1/self.planeParams.c * (self.planeParams.a*xarray+ self.planeParams.b*yarray+ self.planeParams.d))

        self.points = [pointsX, pointsY, pointsZ]

        df = pd.DataFrame({"x": pointsX, "y": pointsY, "z": pointsZ})
        df.to_csv(self.planeResultsPath + "Points sampled.csv", header = None, index=False)
        

    @staticmethod    
    def __vector(azimuth, tilt):
        """ 
    Given the azimuth and tilt of a plane, returns the coordinates of the unit vector

    #### Inputs:
    - azimuth: azimuth in degrees. For reference, North is azimuth=0, and East is azimuth=90
    - tilt: tilt in degrees

    #### Outputs: array with the x,y,z components of the unit vector normal to the plane
        """
        x = math.sin(math.radians(azimuth)) * math.cos(math.radians(tilt))
        y = math.cos(math.radians(azimuth)) * math.cos(math.radians(tilt))
        z = math.sin(math.radians(tilt))
        return np.array([[x, y, z]])
    
        
    def shadingCalculation(self):
        """ 
    Computes the shading matrix of the current plane. Updates the matrix, intersectedList, averageMatrix attributes.

    #### Inputs:
    - None

    #### Outputs:
    - None

    #### Exports:
    - .csv containing the average shading matrix of the whole plane
    - Folder (Individual Matrices) containing a .csv of the shading matrix for each sampled_point  
        """
        self.matrix = []

        minAngleAzimuth = self.anglesAzimuth[0]
        maxAngleAzimuth = self.anglesAzimuth[1]
        stepAngleAzimuth = self.stepAzimuth

        minAngleTilt = self.anglesTilt[0]
        maxAngleTilt = self.anglesTilt[1]
        stepAngleTilt = self.stepTilt

        self.intersectedList = []

        normal = np.array([self.planeParams.a, self.planeParams.b, self.planeParams.c])
        if(normal[2] < 0):
            normal = -normal # Normal vector must be pointing upwards

        for i in range(len(self.points[0])): # Ideally its len(planedf)), but points need to be sampled for time of computing reasons
            #print(i)
            intersectedPoints = []
            intersectedTilts = []
            matr_point_3by3 = np.zeros((int((maxAngleTilt-minAngleTilt)/stepAngleTilt) + 1, int((maxAngleAzimuth-minAngleAzimuth)/stepAngleAzimuth) + 1))
            
            ray_origins = np.array([[self.points[0][i], self.points[1][i], self.points[2][i]]])
            for azimuth in range(minAngleAzimuth, maxAngleAzimuth+stepAngleAzimuth, stepAngleAzimuth): # From 60 to 303 degrees in 3 deg steps reduces computation calculation
                for tilt in range(minAngleTilt, maxAngleTilt+stepAngleTilt, stepAngleTilt):
                    ray_directions = Shader.__vector(azimuth, tilt)

                    # Only send rays if the ray doesn't intersect the plane

                    if(abs(Shader.__anglesVectors(ray_directions[0], normal)) < 90):

                        loc_intersects, n_intersection, _ = self.mesh.ray.intersects_location(ray_origins, ray_directions)

                        if len(n_intersection) > 0:
                            
                            for j in range(len(n_intersection)):
                                p = Point(loc_intersects[j][0], loc_intersects[j][1])
                                if(p.within(self.trimmedPolygon.buffer(self.bufferSize)) == False):
                                    matr_point_3by3[0:(tilt-minAngleTilt)//stepAngleTilt, (azimuth-minAngleAzimuth)//stepAngleAzimuth] = 100
                                    intersectedPoints.append(p)
                                    intersectedTilts.append(tilt)

                    else: # Shade Inside
                        matr_point_3by3[(tilt-minAngleTilt)//stepAngleTilt, (azimuth-minAngleAzimuth)//stepAngleAzimuth] = 100
                    

            self.intersectedList.append([intersectedPoints, intersectedTilts])
            self.matrix.append(matr_point_3by3)

        if(len(self.matrix) > 0):

            mat_stl_mean = sum(self.matrix)/len(self.matrix)

            # Export individual matrices

            az = [i for i in range(minAngleAzimuth, maxAngleAzimuth+stepAngleAzimuth, stepAngleAzimuth)]
            ti = [i for i in range(minAngleTilt, maxAngleTilt+stepAngleTilt, stepAngleTilt)]
            
            individualResultsPath = self.planeResultsPath + "Individual Matrices/" 
            create_output_folder(individualResultsPath)

            for i in range(len(self.points[0])):
                df_mat = pd.DataFrame(self.matrix[i], index=ti, columns=az)
                savename = individualResultsPath + str(i).zfill(2) + ".csv"
                df_mat.to_csv(savename)

            # Export average matrix
            self.averageMatrix = pd.DataFrame(mat_stl_mean, index=ti, columns=az)
            savename = self.planeResultsPath + "Average_" +  str(self.planeID) + ".csv"
            self.averageMatrix.to_csv(savename)

    @staticmethod
    def __anglesVectors(u, v):
        """
        Given two vectors (for this project, they are two planes normals), it computes the angle (in degrees) between them
        #### Inputs:
        - u, v: two vector (each vector is a 3-element array containing the x,y,z components)

        #### Outputs: 
        - Angle between the two vectors (in degrees)
        """
        moduleU = math.sqrt(u[0]**2 + u[1]**2 + u[2]**2)
        moduleV = math.sqrt(v[0]**2 + v[1]**2 + v[2]**2)
        dotProduct = u[0]*v[0] + u[1]*v[1] + u[2]*v[2]
        cosTheta = dotProduct/(moduleU*moduleV)
        if(cosTheta > 1): # Yep, this can happen
            cosTheta = 1
        elif(cosTheta < -1):
            cosTheta = -1
        return math.acos(cosTheta)*180/math.pi

    def plotShadingMatrix(self, plotAll=False):
        """
    Given the shading matrices, exports the average matrix as an image and, if desired, the individual matrices and the images of rays sending
    #### Inputs:
    - plotAll: whether to plot all intermediate steps (that is, ALL individual matrices and ALL ray sending images). This is very time-consuming, so it is advised to be kept as False

    #### Outputs: None

    #### Exports: 
    - Figure with the average shading matrix 
    - (if plotAll): figures with the individual matrices and ray sendings
        """

        plt.rcParams['figure.figsize'] = [12,6]
        fig, ax = plt.subplots()
        df_mat = self.averageMatrix.iloc[::-1]
        sc = ax.matshow(df_mat, cmap='viridis_r')

        # Set the x-axis tick labels to be the column names of df_mat
        x_ticks = np.arange(0, len(df_mat.columns), 5)  # Adjust the step as needed
        ax.set_xticks(x_ticks)
        ax.set_xticklabels(df_mat.columns[x_ticks])

        # Set the y-axis tick labels to be the index values of df_mat
        y_ticks = np.arange(0, len(df_mat.index), 3)  # Adjust the step as needed
        ax.set_yticks(y_ticks)
        ax.set_yticklabels(df_mat.index[y_ticks])
        
        plt.colorbar(sc)
        
        savename = self.planeResultsPath  + "Average_" +  str(self.planeID) + ".png"

        fig.savefig(savename)
        plt.close()

        if(plotAll):

            meshX = []
            meshY = []
            meshZ = []

            for i in range(len(self.mesh.vertices)):
                meshX.append(self.mesh.vertices[i][0])
                meshY.append(self.mesh.vertices[i][1])
                meshZ.append(self.mesh.vertices[i][2])

            if(len(self.matrix) > 0):
                MatrixImagesPath = self.shadingResultsPath + "/" + str(self.planeID) + "/Matrix Images/"
                create_output_folder(MatrixImagesPath)
                RaysPath = self.shadingResultsPath + "/" + str(self.planeID) + "/Rays Sending/"
                create_output_folder(RaysPath)


                for i in range(len(self.matrix)):
                    # Plot Matrices -----------------------------------------------------------------------------------------
                    minAngleAzimuth = self.anglesAzimuth[0]
                    maxAngleAzimuth = self.anglesAzimuth[1]
                    stepAngleAzimuth = self.stepAzimuth

                    minAngleTilt = self.anglesTilt[0]
                    maxAngleTilt = self.anglesTilt[1]
                    stepAngleTilt = self.stepTilt

                    az = [i for i in range(minAngleAzimuth, maxAngleAzimuth+stepAngleAzimuth, stepAngleAzimuth)]
                    ti = [i for i in range(minAngleTilt, maxAngleTilt+stepAngleTilt, stepAngleTilt)]
                    
                    df_mat = pd.DataFrame(self.matrix[i], index=ti, columns=az)
                    
                    plt.rcParams['figure.figsize'] = [12,6]
                    fig, ax = plt.subplots()
                    df_mat = df_mat.iloc[::-1]
                    sc = ax.matshow(df_mat, cmap='viridis_r')

                    # Set the x-axis tick labels to be the column names of df_mat
                    x_ticks = np.arange(0, len(df_mat.columns), 5)  # Adjust the step as needed
                    ax.set_xticks(x_ticks)
                    ax.set_xticklabels(df_mat.columns[x_ticks])

                    # Set the y-axis tick labels to be the index values of df_mat
                    y_ticks = np.arange(0, len(df_mat.index), 3)  # Adjust the step as needed
                    ax.set_yticks(y_ticks)
                    ax.set_yticklabels(df_mat.index[y_ticks])

                    plt.colorbar(sc)
                    
                    savename = MatrixImagesPath  + str(i).zfill(2) + ".png"

                    fig.savefig(savename)
                    plt.close()

                    # Plot ray sendings -----------------------------------------------------------------------------------------
                    fig, ax = plt.subplots()
                    ax.scatter(meshX, meshY, c=meshZ)
                    ax.scatter(self.points[0], self.points[1], c="gray", marker = ".")

                    intersectedX = []
                    intersectedY = []


                    pointToCheck = i

                    norm = plt.Normalize()
                    colormap = plt.cm.Reds_r
                    colors = colormap(norm(self.intersectedList[pointToCheck][1]))

                    for j in range(len(self.intersectedList[pointToCheck][0])):
                        intersectedX.append(self.intersectedList[pointToCheck][0][j].xy[0])
                        intersectedY.append(self.intersectedList[pointToCheck][0][j].xy[1])
                        ax.plot([self.points[0][pointToCheck], intersectedX[j][0]], [self.points[1][pointToCheck], intersectedY[j][0]], c=colors[j], zorder=1, linestyle=":") #linewidth=5
                    ax.scatter(intersectedX, intersectedY, c="black", s=25, zorder=2)
                    ax.scatter(self.points[0][pointToCheck], self.points[1][pointToCheck], c="red", s=50, zorder=2)

                    ax.set_xlim(-25,25) 
                    ax.set_ylim(-25,25)
                    # plt.show()
                    ax.set_aspect('equal', adjustable='box')

                    # Create a ScalarMappable for the colorbar
                    sm = plt.cm.ScalarMappable(cmap=colormap, norm=norm)
                    sm.set_array([])

                    ax.plot(self.trimmedPolygon.buffer(self.bufferSize).exterior.xy[0], self.trimmedPolygon.buffer(self.bufferSize).exterior.xy[1])
                    ax.set_aspect('equal', adjustable='box')

                    savename = RaysPath  + str(i).zfill(2) + ".png"
                    fig.savefig(savename)
                    plt.close()


