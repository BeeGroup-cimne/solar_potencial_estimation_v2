import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import trimesh # For stl stuff
from shapely import Polygon # For polygon
from shapely.geometry import Point # Points
from scipy.spatial import ConvexHull # Create polygon
import math # Trigonometric functions
from Functions.general_functions import create_output_folder
from Classes.PlaneProcessor import PlaneProcessor
import shapely.wkt, shapely.affinity
from sklearn.decomposition import PCA

class Shader:
    def __init__(self, building, planeID, pointCloud_path, STL_path, planes_path, shadingResultsPath, 
                 Nsamples=50, div=2, anglesAzimuth=[60, 300], stepAzimuth=3, anglesTilt=[0, 87], stepTilt=3, bufferSize=1, shadeInside=True):
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
        self.shadeInside = shadeInside
        self.bufferSize = bufferSize

    def prepareDataShading(self):
        # Get point cloud center
        centerX, centerY = self.building.x[0], self.building.y[0]

        # Load point cloud and change to same coordinate system as 3d model
        pointsNeighborhood = pd.read_csv(self.pointCloud_path, header=None, sep = " ")
        pointsNeighborhood = pointsNeighborhood.rename(columns={0: "x", 1: "y", 2:"z"})

        minz = pointsNeighborhood["z"].min()

        # self.pointsNeighborhood["x"] = self.pointsNeighborhood["x"] - centerX
        # self.pointsNeighborhood["y"] = self.pointsNeighborhood["y"] - centerY
        # self.pointsNeighborhood["z"] = self.pointsNeighborhood["z"] - minz

        # Get plane equation (fix d so that we can recalculate points later)
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

        # x = self.planedf["x"]
        # y = self.planedf["y"]
        # p = []

        # for j in range(len(x)):
        #     p.append((x[j], y[j]))
        # coords = PlaneProcessor.convexhull(p)
        # self.polygon = Polygon(coords)
        self.polygon = shapely.wkt.loads(self.planeParams.polygon)
        self.polygon = shapely.affinity.translate(self.polygon, xoff=-centerX, yoff=-centerY)
        # Load trimmed polygon
        self.trimmedPolygon = shapely.wkt.loads(self.planeParams.trimmedPolygon)
        self.trimmedPolygon = shapely.affinity.translate(self.trimmedPolygon, xoff=-centerX, yoff=-centerY)

        # Load stl

        self.mesh = trimesh.load(self.STL_path)
        self.mesh.vertices[:] = self.mesh.vertices[:] - [(self.mesh.bounds[1][0])/2, (self.mesh.bounds[1][1])/2, 0] # Must be centered arround building

        # Prepare output folder
        self.planeResultsPath = self.shadingResultsPath + "/" + str(self.planeID) + "/" 
        create_output_folder(self.planeResultsPath, deleteFolder=True)

    def computePCA(self):
        X = self.planedf[["x", "y"]]
        self.pca = PCA(n_components=2)
        self.pca.fit(X)

    def sampleRoof(self):
        self.computePCA()
        
        newData = self.pca.transform(self.planedf[["x", "y"]])
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
                xv[j][i], yv[j][i] = np.dot([xvPCA[0][i], yvPCA[j][0]], self.pca.components_) + self.pca.mean_
        
        self.pointsX = []
        self.pointsY = []
        self.pointsZ = []

        for i in range(len(xv[0])):
            for j in range(len(yv)):
                point = Point(xv[j][i], yv[j][i])
                if (point.within(self.trimmedPolygon) == True):
                    self.pointsX.append(xv[j][i])
                    self.pointsY.append(yv[j][i])
                    self.pointsZ.append(-1/self.planeParams.c * (self.planeParams.a*xv[0][i]+ self.planeParams.b*yv[j][0]+ self.planeParams.d))
        
        xarray= np.array(self.trimmedPolygon.exterior.xy[0])
        yarray = np.array(self.trimmedPolygon.exterior.xy[1])

        self.pointsX.extend(xarray)
        self.pointsY.extend(yarray)
        self.pointsZ.extend(-1/self.planeParams.c * (self.planeParams.a*xarray+ self.planeParams.b*yarray+ self.planeParams.d))

        df = pd.DataFrame({"x": self.pointsX, "y": self.pointsY, "z": self.pointsZ})
        df.to_csv(self.planeResultsPath + "Points sampled.csv", header = None, index=False)
        

    @staticmethod    
    def vector(azimuth, tilt):
        """ 
        Given the azimuth and tilt, returns the coordinates of the unit vector
        """
        x = math.sin(math.radians(azimuth)) * math.cos(math.radians(tilt))
        y = math.cos(math.radians(azimuth)) * math.cos(math.radians(tilt))
        z = math.sin(math.radians(tilt))
        return np.array([[x, y, z]])
    
        
    def shadingCalculation(self):
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

        for i in range(len(self.pointsX)): # Ideally its len(planedf)), but points need to be sampled for time of computing reasons
            #print(i)
            intersectedPoints = []
            intersectedTilts = []
            matr_point_3by3 = np.zeros((int((maxAngleTilt-minAngleTilt)/stepAngleTilt) + 1, int((maxAngleAzimuth-minAngleAzimuth)/stepAngleAzimuth) + 1))
            
            ray_origins = np.array([[self.pointsX[i], self.pointsY[i], self.pointsZ[i]]])
            for azimuth in range(minAngleAzimuth, maxAngleAzimuth+stepAngleAzimuth, stepAngleAzimuth): # From 60 to 303 degrees in 3 deg steps reduces computation calculation
                for tilt in range(minAngleTilt, maxAngleTilt+stepAngleTilt, stepAngleTilt):
                    ray_directions = Shader.vector(azimuth, tilt)

                    # Only send rays if the ray doesn't intersect the plane

                    if(abs(PlaneProcessor.anglesVectors(ray_directions[0], normal)) < 90):

                        loc_intersects, n_intersection, _ = self.mesh.ray.intersects_location(ray_origins, ray_directions)

                        if len(n_intersection) > 0:
                            
                            for j in range(len(n_intersection)):
                                p = Point(loc_intersects[j][0], loc_intersects[j][1])
                                if(p.within(self.polygon.buffer(self.bufferSize)) == False):
                                    matr_point_3by3[0:(tilt-minAngleTilt)//stepAngleTilt, (azimuth-minAngleAzimuth)//stepAngleAzimuth] = 100
                                    intersectedPoints.append(p)
                                    intersectedTilts.append(tilt)

                    else:
                        if(self.shadeInside):
                            matr_point_3by3[(tilt-minAngleTilt)//stepAngleTilt, (azimuth-minAngleAzimuth)//stepAngleAzimuth] = 100
                        else:
                            matr_point_3by3[(tilt-minAngleTilt)//stepAngleTilt, (azimuth-minAngleAzimuth)//stepAngleAzimuth] = 0
            
                    

            self.intersectedList.append([intersectedPoints, intersectedTilts])
            self.matrix.append(matr_point_3by3)

        if(len(self.matrix) > 0):

            self.mat_stl_mean = sum(self.matrix)/len(self.matrix)

            # Export individual matrices

            az = [i for i in range(minAngleAzimuth, maxAngleAzimuth+stepAngleAzimuth, stepAngleAzimuth)]
            ti = [i for i in range(minAngleTilt, maxAngleTilt+stepAngleTilt, stepAngleTilt)]
            
            self.individualResultsPath = self.planeResultsPath + "Individual Matrices/" 
            create_output_folder(self.individualResultsPath)

            for i in range(len(self.pointsX)):
                self.df_mat = pd.DataFrame(self.matrix[i], index=ti, columns=az)
                savename = self.individualResultsPath + str(i).zfill(2) + ".csv"
                self.df_mat.to_csv(savename)

            # Export average matrix
            self.df_mat = pd.DataFrame(self.mat_stl_mean, index=ti, columns=az)
            #df_mat = df_mat.iloc[::-1]
            savename = self.planeResultsPath + "Average_" +  str(self.planeID) + ".csv"
            self.df_mat.to_csv(savename)


    def plotShadingMatrix(self, plotAll=False):
        """
    #### Inputs:
    - df_mat: shading matrix, previously calculated
    - exportPath (string): base path where exported files will be saved 
    - building (string): building name 
    - planeID (int): position of the plane within the planeParams .csv

    #### Outputs: None

    #### Exports: 
    - Figure with the average shading matrix 
        """
        self.meshX = []
        self.meshY = []
        self.meshZ = []

        for i in range(len(self.mesh.vertices)):
            self.meshX.append(self.mesh.vertices[i][0])
            self.meshY.append(self.mesh.vertices[i][1])
            self.meshZ.append(self.mesh.vertices[i][2])

        if(len(self.matrix) > 0):
            self.MatrixImagesPath = self.shadingResultsPath + "/" + str(self.planeID) + "/Matrix Images/"
            create_output_folder(self.MatrixImagesPath)
            self.RaysPath = self.shadingResultsPath + "/" + str(self.planeID) + "/Rays Sending/"
            create_output_folder(self.RaysPath)

            plt.rcParams['figure.figsize'] = [12,6]
            fig, ax = plt.subplots()
            df_mat = self.df_mat.iloc[::-1]
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
                for i in range(len(self.matrix)):
                    # Plot Matrix -----------------------------------------------------------------------------------------
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
                    
                    savename = self.MatrixImagesPath  + str(i).zfill(2) + ".png"

                    fig.savefig(savename)
                    plt.close()

                    # Plot ray sendings -----------------------------------------------------------------------------------------
                    fig, ax = plt.subplots()
                    ax.scatter(self.meshX, self.meshY, c=self.meshZ)
                    ax.scatter(self.pointsX, self.pointsY, c="gray", marker = ".")

                    intersectedX = []
                    intersectedY = []


                    pointToCheck = i

                    norm = plt.Normalize()
                    colormap = plt.cm.Reds_r
                    colors = colormap(norm(self.intersectedList[pointToCheck][1]))

                    for j in range(len(self.intersectedList[pointToCheck][0])):
                        intersectedX.append(self.intersectedList[pointToCheck][0][j].xy[0])
                        intersectedY.append(self.intersectedList[pointToCheck][0][j].xy[1])
                        ax.plot([self.pointsX[pointToCheck], intersectedX[j][0]], [self.pointsY[pointToCheck], intersectedY[j][0]], c=colors[j], zorder=1, linestyle=":") #linewidth=5
                    ax.scatter(intersectedX, intersectedY, c="black", s=25, zorder=2)
                    ax.scatter(self.pointsX[pointToCheck], self.pointsY[pointToCheck], c="red", s=50, zorder=2)

                    ax.set_xlim(-25,25) 
                    ax.set_ylim(-25,25)
                    # plt.show()
                    ax.set_aspect('equal', adjustable='box')

                    # Create a ScalarMappable for the colorbar
                    sm = plt.cm.ScalarMappable(cmap=colormap, norm=norm)
                    sm.set_array([])

                    ax.plot(self.trimmedPolygon.buffer(self.bufferSize).exterior.xy[0], self.trimmedPolygon.buffer(self.bufferSize).exterior.xy[1])
                    ax.set_aspect('equal', adjustable='box')

                    savename = self.RaysPath  + str(i).zfill(2) + ".png"
                    fig.savefig(savename)
                    plt.close()


