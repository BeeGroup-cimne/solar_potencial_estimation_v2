import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import pathlib
import math #sqrt, pi and trigonometric functions
from scipy.spatial import ConvexHull
from shapely import Polygon, Point
import shapely.wkt, shapely.affinity
from sklearn import linear_model
import geopandas as gpd
import pyproj # Convert building and cadastre to new coordinate system
from matplotlib.path import Path
from matplotlib.patches import PathPatch
from matplotlib.collections import PatchCollection
from sklearn.decomposition import PCA
from Functions.general_functions import create_output_folder

class PlaneProcessor:
    """
    This class is used to process the planes found in the previous step:
    - Merges planes that are too similar.
    - Splits planes if there are density discontinuities
    - Deletes planes of bad quality.
    - Deletes overlaps.
    - Pierces holes.
    - Trims according to cadastre limits

    
    ### Attributes: 
    #### Defined upon initialization:
    - 
    ##### Attributes with default values
    - 
    #### Self-generated:
    - 

    ### Public methods:
    - 
    """
    def __init__(self, building, segmentedPath, identifiedPaths, processedPaths, cadastrePath, generateFigures=True,
                 planeDistanceThreshold=0.5, angleThreshold=10, mergeReiterations=10, splitDistanceThreshold = 2, angleSplitIncrement=5,
                 minPointsDelete=6, minAreaDelete=5, minDensityDelete=0.25, convexHullHorizon = 5, parallelismAngleThreshold=5, slidingHole=0.75, minHoleSide = 3):
        self.building = building 
        self.segmentedPath = segmentedPath
        self.planeListPath = identifiedPaths[0]
        self.planePointsPath = identifiedPaths[1]
        self.processedResultsPath = processedPaths[0]
        self.processedImagesPath = processedPaths[1]
        self.cadastrePath = cadastrePath
        self.generateFigures = generateFigures

        self.planeDistanceThreshold = planeDistanceThreshold
        self.angleThreshold = angleThreshold
        self.mergeReiterations = mergeReiterations
        self.splitDistanceThreshold = splitDistanceThreshold
        self.angleSplitIncrement = angleSplitIncrement

        self.minPointsDelete = minPointsDelete
        self.minAreaDelete = minAreaDelete
        self.minDensityDelete = minDensityDelete
        self.parallelismAngleThreshold = parallelismAngleThreshold
        self.convexHullHorizon = convexHullHorizon
        self.slidingHole = slidingHole
        self.minHoleSide = minHoleSide

        self.__loadIdentifiedData()

    def __loadIdentifiedData(self):
        """
        Updates the following object's attributes: 
        planedf: dataframe containing all the planes information (a,b,c,d, Origin, tilt, azimuth for each plane)
        planePointList: list of dataframes of points belonging to each plane (one df for each plane)
        buildingPoints: dataframe containing all points belonging to the building

        #### Inputs:
        - None
        
        #### Outputs:
        -  None

        #### Exports:
        - Folders to store the results and exported images are created (if there were already those folders, they are deleted and created again, empty)
        """ 
        create_output_folder(self.processedResultsPath, deleteFolder=True)
        create_output_folder(self.processedImagesPath, deleteFolder=True)
    
        planeList = []
        for i, item in enumerate(pathlib.Path((self.planeListPath)).iterdir()):
            if item.is_file():
                if(item.stat().st_size != 0):
                    df = pd.read_csv(item, header=None)
                    df = df.rename(columns={0: "a", 1: "b", 2:"c", 3:"d"})
                    df["Origin"] = str(i)
                    planeList.append(df)

        self.planedf = pd.concat(planeList).reset_index(drop=True)
        # Get tilt and azimuth for all planes

        tilts = []
        azimuths = []

        for index,plane in self.planedf.iterrows():
            normal = plane[["a", "b", "c"]].values
            tilt, azimuth = PlaneProcessor.__getTiltAzimuth(normal)
            tilts.append(tilt)
            azimuths.append(azimuth)

        self.planedf["tilt"] = tilts
        self.planedf["azimuth"] = azimuths

        # Get the plane points

        buildingFiles = []
        for item in pathlib.Path((self.planePointsPath)).iterdir():
            if item.is_file():
                buildingFiles.append(str(item))

        self.planePointList = []

        for i in range(len(buildingFiles)):
            df = pd.read_csv(buildingFiles[i], header=None)
            df = df.rename(columns={0: "x", 1: "y", 2:"z"})
            self.planePointList.append(df)

        # Get the building points  
        self.buildingPoints = pd.read_csv((self.segmentedPath + "/" + self.building.identifier[0] + ".csv"), header=None)
        self.buildingPoints = self.buildingPoints.rename(columns={0: "x", 1: "y", 2:"z"})

    @staticmethod    
    def __getTiltAzimuth(normal):
        """
        Given the normal vector of a plane, returns the tilt and azimuth of that plane.
        Tilt goes from 0 (horizontal) to 90 degrees (vertical) whereas azimuth goes from 0 (pointing north) to 360 degrees, growing clockwise

        #### Inputs:
        - normal: 3 element array (x,y,z from the normal vector, i.e, a,b,c parameters of the plane equation)
        
        #### Outputs:
        -  tilt, azimuth: in degreees
        """

        # Check if z is negative, plane normal must be pointing upwards

        if(normal[2] < 0):
            normal = -normal
        
        # Azimuth
        alpha = math.degrees(math.atan2(normal[1], normal[0]))
        azimuth = 90 - alpha
        if azimuth >= 360.0:
            azimuth -= 360.0
        elif azimuth < 0.0:
            azimuth += 360.0
        
        # Tilt
        t = math.sqrt(normal[0] ** 2 + normal[1] ** 2)
        if t == 0:
            tilt = 0.0
        else:
            tilt = 90 - math.degrees(math.atan(normal[2] / t)) # 0 for flat roof, 90 for wall/vertical roof
        tilt = round(tilt, 3)

        return (tilt, azimuth)

    def plotPlanes(self, name, showTrimmed=False):
        """
        Plots the points and a polygon for each plane (colorcoded)  
        Name is the title (and part of the image name)
        Generated plot is stored in imagesPath path

        #### Inputs:
        - name: fileName and title of the plot
        - showTrimmed: to plot the polygons of each plane (before cadastreTrim) or the trimmedPolygon (after cadastreTrim)
        
        #### Outputs:
        -  None

        #### Exports:
        - .png of a plot with the current state of the plane processing. Contains, for all planes, their point scatter and their current polygon 
        """
        plt.rcParams['figure.figsize'] = [6,6]
        fig, ax = plt.subplots()
        color = iter(plt.cm.rainbow(np.linspace(0, 1, len(self.planePointList))))
        _ = ax.scatter(self.buildingPoints.x, self.buildingPoints.y, c="Gray", marker=".", alpha = 0.1)
       
        for i in range(len(self.planePointList)):
            # df = self.planePointList[i].copy()

            # x = df["x"]
            # y = df["y"]

            # p = []

            # for j in range(len(x)):
            #     p.append((x[j], y[j]))
            
            if(len(self.planePointList[i]) >= self.minPointsDelete):
                ax.set_title(name)
                c = next(color)

                if(not showTrimmed):
                    poly = plt.Polygon(PlaneProcessor.__convexhull(self.planePointList[i]), color=c, alpha=0.2) #poly = plt.Polygon(convexhull(p), ec="k", alpha=0.2)
                    ax.add_patch(poly)    
                else:
                    if(type(self.exportPlanedf.trimmedPolygon[i]) == type(Polygon())):
                        plt.fill(*self.exportPlanedf.trimmedPolygon[i].exterior.xy, color=c, alpha=0.2)

                        for interior in self.exportPlanedf.trimmedPolygon[i].interiors:
                            plt.plot(*interior.xy, color="k", alpha=0.5)

                    else:
                        for geom in self.exportPlanedf.trimmedPolygon[i].geoms:
                            plt.fill(*geom.exterior.xy, color=c, alpha=0.2)

                            for interior in geom.interiors:
                                plt.plot(*interior.xy, color="k", alpha=0.5) #plt.fill(*interior.xy, color=c, alpha=0.2)

                ax.scatter(self.planePointList[i].x, self.planePointList[i].y, color=c, marker='.',  label=i)
        
        ax.legend(loc="lower left")
        ax.set_aspect('equal', adjustable='box')
        
        filename = self.processedImagesPath + name + ".png"
        #plt.show()
        fig.savefig(filename)
        plt.close()

    @staticmethod 
    def __convexhull(df):
        """
        Given a list of points (x and y coordinates), returns the smallest convex polygon that contains all the points

        #### Inputs:
        - df: dataframe of points, containing x and y fields 

        #### Outputs:
        -  array of points (each point is a 2D array, [x, y]) of the polygon convex hull

       """
        x = df["x"]
        y = df["y"]

        p = []

        for j in range(len(x)):
            p.append((x[j], y[j]))

        p = np.array(p)
        hull = ConvexHull(p)
        return p[hull.vertices,:]

    ###########################################################################################################################################
    # The following code contains all functions needed for the merging step. 
    # Some of these auxiliar functions may be used  in other steps of plane processing

    def merge(self):
        """
        Merges are the planes that are similar (that means, that they are close to each other and have small angle of difference)
        
        #### Inputs:
        - None

        #### Outputs: 
        - None. The attributes planedf and planePointList  are updated

        #### Exports: 
        - images for the initial situation, the middle steps, and the final planes found (merged)
        """

        # Merge all planes that are similar and plot and export results
        indexesToMerge = self.__getClosestPlanes()

        i = 0

        while(indexesToMerge[0] > -1):
            i = i +1
            self.__joinPlanes(indexesToMerge) 
            indexesToMerge = self.__getClosestPlanes()

    def __getClosestPlanes(self):
        """
        Given some planes (the planes equations are in a df and the points for each plane are in a list of dfs), returns the indexes of those planes that are close enough.
        Planes are close if their angle is below a threshold (they are somewhat parallel) and their (pseudo)distance is below another threshold.
        Only planes from the same height group can be merged  
        This does not return all the closest planes, but the first ones that fulfill the criteria 
                
        #### Inputs:
        - None. It uses the planedf attribute.

        #### Outputs: 
        - 2-element array containing the first two indexes of the planes to merge.
        """
        for i in range(len(self.planedf)):
            for j in range(i+1, len(self.planedf)):
                if(self.planedf["Origin"].iloc[i] == self.planedf["Origin"].iloc[j]): # Only merge planes from same height level
                    u = self.planedf[["a", "b", "c",  "d"]].iloc[i].values
                    v = self.planedf[["a", "b", "c",  "d"]].iloc[j].values
                    angle = PlaneProcessor.__anglesVectors(u, v)
                    if(angle < self.angleThreshold):
                        if(PlaneProcessor.__pseudoDistance(u, self.planePointList[j]) < self.planeDistanceThreshold):
                            return[i, j]
                        
        # If anything is found, returns negative indexes
        return[-1, -1]
    
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

    @staticmethod
    def __pseudoDistance(plane, pointsOtherPlane):
        """
        Given a plane defined by its equation, and a list of points belonging to another plane, it computes the average distance of the points to the plane.
        This is meant to be the distance between non-parallel planes (I feel the hatred of a lot of mathematicians right now).
        
        #### Inputs:
        - plane: 4-element array (a, b, c, d)
        - pointsOtherPlane: dataframe containing all the points (with x,y,z keys)

        #### Outputs: 
        - Absolute value of the average distance of the points to the plane
        """
        dist = 0
        for i in range(len(pointsOtherPlane)):
            point = pointsOtherPlane[["x", "y", "z"]].iloc[i].values
            dist = dist + PlaneProcessor.__distancePlane(point, plane)
        return abs(dist/len(pointsOtherPlane))

    @staticmethod
    def __distancePlane(point, planeParams):
        """
    #### Inputs:
    - point: 3-element array (x, y, z)
    - planeParams: 4-element array (a, b, c, d)

    #### Outputs: 
    - dis: signed distance bettween the point and the plane
        """
        a, b, c, d = planeParams[0], planeParams[1], planeParams[2], planeParams[3]
        x, y, z = point[0], point[1], point[2]
        dis = a*x + b*y + c*z+ d
        return dis

    def __joinPlanes(self, indexesToMerge):
        """
        Given two indexes and all the needed planes info, merges two planes.
        The merging process consists of refitting a plane containing points from both planes.
        After the merging, if reiterations > 1, the plane looks for inliers (closer than distance threshold) in the whole building df and refits as many times as specified.
            
        #### Inputs:
        - indexesToMerge: array containing the index of the two planes to be merged

        #### Outputs: 
        - None. The attributes planedf and planePointList  are updated
        """
        # Copy plane df and delete planes to merge
        newplanedf = self.planedf.copy()
        origin = self.planedf.Origin[indexesToMerge[0]]
        newplanedf = newplanedf.drop(index = indexesToMerge).reset_index(drop=True)

        # Get all points from both planes in a single df
        planePoints = [self.planePointList[indexesToMerge[0]], self.planePointList[indexesToMerge[1]]]
        planePointsMerged = pd.concat(planePoints).reset_index(drop=True)

        # Fit a plane
        X = planePointsMerged[["x", "y"]].values
        y = planePointsMerged[["z"]].values
        model = linear_model.LinearRegression().fit(X, y)
        plane = [model.coef_[0][0], model.coef_[0][1], -1, model.intercept_[0]]
        
        inliers = []
        for i in range(len(planePointsMerged)): # We could sub planePointsMerged for self.buildingPoints
            point = planePointsMerged[["x", "y", "z"]].iloc[i]
            if(abs(PlaneProcessor.__distancePlane(point, plane)) < self.planeDistanceThreshold):
                inliers.append(i)

        planeMergedInliers = planePointsMerged[["x", "y", "z"]].iloc[inliers].reset_index(drop=True) #pointsdf or planePointsMerged

        #######################################################
        # The following part performs the regression multiple times to see if the merged plane gets updated
        # I am not entirely sure why I did this or if it is even necessary, but I have not deleted it (yet)

        for k in range(self.mergeReiterations):
            # Find inliers and add them to the new plane
            inliers = []
            for i in range(len(planePointsMerged)): # We could sub planePointsMerged for pointsdf
                point = planePointsMerged[["x", "y", "z"]].iloc[i]
                if(abs(PlaneProcessor.__distancePlane(point, plane)) < self.planeDistanceThreshold):
                    inliers.append(i)

            planeMergedInliers = planePointsMerged[["x", "y", "z"]].iloc[inliers].reset_index(drop=True) #pointsdf or planePointsMerged

            X = planeMergedInliers[["x", "y"]].values
            y = planeMergedInliers[["z"]].values
            model = linear_model.LinearRegression().fit(X, y)
            plane = [model.coef_[0][0], model.coef_[0][1], -1, model.intercept_[0]]
        
        # End of dubious code
        #######################################################
            
        tilt, azimuth = PlaneProcessor.__getTiltAzimuth(np.array(plane))
        self.planePointList.append(planeMergedInliers) 

        # Delete old planes from planePointList 
        del self.planePointList[indexesToMerge[1]] # ! Must delete indexes1 first, or else it changes order
        del self.planePointList[indexesToMerge[0]]

        # Add the newplane
        newplane = plane
        newplane.append(origin)
        newplane.append(tilt)
        newplane.append(azimuth)
        newplanedf.loc[len(newplanedf)] = newplane

        self.planedf = newplanedf
                
    # End of merging functions
    ###########################################################################################################################################
    ###########################################################################################################################################
    # The following code contains all functions needed for the splitDeleting step. 
    # Some of these auxiliar functions may be used  in other steps of plane processing

    def splitDelete(self):
            """
        Splits all planes in regions of points that are too far apart. 
        Checks if the resulting planes are of good quality and, if not, it deletes them 

        #### Inputs:
        - None. Uses the following attributes: planePointList, planedf

        #### Outputs: 
        - howManySplits: number of how many planes were split (this is used for iteration control in the SolarEstimator class)
        The attribute exportplanedf is generated/updated. The whole code should be checked because planedf and export planedf do basicly the same thing 
        (the program works, but this variable duplication is weird) 
            """
            howManySplits = 0

            splitPlanes = []
            splitPlaneParams = []

            for i in range(len(self.planedf)):
                originaldf = pd.DataFrame(self.planePointList[i])
                dividedPlane = self.__lineSplit(originaldf)
                splitPlanes = splitPlanes + dividedPlane

                # Recalculate plane parameters, only if the plane was split
                if(len(dividedPlane) > 1):
                    howManySplits = howManySplits + 1
                    for j in range(len(dividedPlane)):
                        X = dividedPlane[j][["x", "y"]].values
                        y = dividedPlane[j][["z"]].values
                        model = linear_model.LinearRegression().fit(X, y)
                        plane = [model.coef_[0][0], model.coef_[0][1], -1, model.intercept_[0]]
                        tilt, azimuth = PlaneProcessor.__getTiltAzimuth(np.array(plane))

                        newplane = plane
                        newplane.append(self.planedf.Origin[i]) # Get the original height level
                        newplane.append(tilt)
                        newplane.append(azimuth)
                        splitPlaneParams.append(newplane)

                else:
                    splitPlaneParams.append(self.planedf.iloc[i].values)

            # Deletes deletable plane
            planesToDelete = []

            for i in range(len(splitPlanes)):
                if(self.__isPlaneDeletable(splitPlanes[i])):
                    planesToDelete.append(i)

            planesToDelete.reverse() # List reversed to not mess up with array positions
            for x in planesToDelete:
                del splitPlanes[x]  
                del splitPlaneParams[x]

            # Saves plane parameters

            splitPlaneParams = pd.DataFrame(splitPlaneParams)
            splitPlaneParams.columns = self.planedf.columns[0:len(splitPlaneParams.columns)]
            self.planedf = splitPlaneParams
            self.planePointList = splitPlanes

            # Get all results in a dataframe
            areas = []
            polygons = []

            for i in range(len(self.planedf)):
                x =  self.planePointList[i]["x"]
                y =  self.planePointList[i]["y"]
                polygon = Polygon(PlaneProcessor.__convexhull(self.planePointList[i]))
                areas.append(polygon.area*1/math.cos(self.planedf.tilt[i]*math.pi/180))
                polygons.append(polygon)

            self.exportPlanedf = self.planedf.copy()
            self.exportPlanedf["area"] = areas
            self.exportPlanedf["polygon"] = polygons

            return howManySplits

    def __lineSplit(self, pointsToSplitdf, startingAngle=0):
        """
        Given a point dataframe, it splits it in the directions it detects discontinuities above thresholdDistance.
        It tests all cases (does the full lap) in a resolution of increment.
        This is recursive: once a plane is split, the "swept" half is scanned for more discontinuities and, once it is finished, the algorithm returns to the "unswept" half 
       
        #### Inputs:
        - originaldf: dataframe containing all the points of a plane
        - startingAngle: angle in which to start sweeping (at the moment, it is always left at 0)

        #### Outputs: 
        - splitPlanes: list of all the regions in which the plane was split. Each list contains a dataframe of the points (x, y, z) from that region
       """ 
        pointsToSplitdf = pd.DataFrame(pointsToSplitdf)
        splitPlanes = []

        for angle in range(startingAngle, 360, self.angleSplitIncrement):
            currentSlope = -math.tan(angle*math.pi/180)
            
            # Computes distance for all points
            for index,point in pointsToSplitdf.iterrows():
                pointsToSplitdf.loc[index,"distance"] = PlaneProcessor.__distPointLine(point.x, point.y, currentSlope)
            
            # Orders points by distance and calculates difference
            pointsToSplitdf = pointsToSplitdf.sort_values("distance").reset_index(drop=True)

            pointsToSplitdf["deltaDist"] = np.concatenate(([0], pointsToSplitdf.distance[1:len(pointsToSplitdf)].values - pointsToSplitdf.distance[0:len(pointsToSplitdf)-1].values))

            lastSplit = 0

            for i in range(len(pointsToSplitdf)):
                if(pointsToSplitdf.deltaDist[i] > self.splitDistanceThreshold):
                    subplane = pd.DataFrame(pointsToSplitdf.iloc[lastSplit:i].reset_index(drop=True))
                    subsplitPlanes = self.__lineSplit(subplane, startingAngle=0)

                    if(len(subsplitPlanes) <= 1):
                        splitPlanes = splitPlanes + [subplane]

                    else:
                        splitPlanes = splitPlanes + subsplitPlanes
                    lastSplit = i
            
            pointsToSplitdf = pointsToSplitdf.iloc[lastSplit:len(pointsToSplitdf)].reset_index(drop=True)

        # Append "final group"
        if(lastSplit != len(pointsToSplitdf)-1):
            splitPlanes.append(pointsToSplitdf.iloc[lastSplit:len(pointsToSplitdf)])

        return splitPlanes

    @staticmethod    
    def __distPointLine(x, y, A, B=1, C=0):
        """
        Computes the distance between a point and the straight line Ax+By+C=0
        #### Inputs:
        - x,y: coordiantes of the point
        - A,B,C: parameters of a straight line. If B=1 and C=0, A is the negative of the slope of a line that crosses (0,0)

        #### Outputs: 
        - Absolute value of the distance between point and line
        """   
        return abs(A*x+B*y+C)/math.sqrt(A**2+B**2)

    def __isPlaneDeletable(self, planePointdf):
        """
        Tests for all the criteria a plane should be deleted:
        - The plane has too few points
        - The plane area is too small
        - The point density is too small

        #### Inputs:
        - planePointdf: dataframe containing all the points (x, y, z) in a region 
        
        #### Outputs: 
        - True or False: whether the plane should be deleted or not 
        """ 
        if(len(planePointdf) < self.minPointsDelete):
            return True
        
        coords = PlaneProcessor.__convexhull(planePointdf)
        polygon = Polygon(coords)
        if(polygon.area < self.minAreaDelete):
            return True

        if(len(planePointdf)/polygon.area < self.minDensityDelete):
            return True

        return False    

    # End of splitting functions
    ###########################################################################################################################################
    ###########################################################################################################################################
    # The following code contains all functions needed for the deleting overlaps step. 
    # Some of these auxiliar functions may be used  in other steps of plane processing

    def deleteOverlaps(self):
        """
        Deletes areas from one plane that overlap in the other. A higher level overlap will always be deleting on the lower plane
        For planes in the same height group, the highest plane will make a hole on the lowest plane.

        #### Inputs:
        - None: uses the exportPlanedf attribute
        
        #### Outputs: 
        - None: updates the exportPlanedf and the planePointList attribute
        """ 
        for i in range(len(self.exportPlanedf)):
            for j in range(i+1, len(self.exportPlanedf)):
                if(self.exportPlanedf.polygon[i].intersects(self.exportPlanedf.polygon[j])):
                    if(self.exportPlanedf.Origin[i] != self.exportPlanedf.Origin[j]):
                        if(self.exportPlanedf.Origin[i] < self.exportPlanedf.Origin[j]): #If i is below j, i must be bitten by j
                            self.exportPlanedf.polygon[i] = self.exportPlanedf.polygon[i].difference(self.exportPlanedf.polygon[j])
                        else:
                            self.exportPlanedf.polygon[j] = self.exportPlanedf.polygon[j].difference(self.exportPlanedf.polygon[i])
                    else:
                        # First check: "parallel" planes that SOMEHOW got on the same height level
                        vectorI = self.planedf[["a", "b", "c", "d"]].iloc[i].values
                        vectorJ = self.planedf[["a", "b", "c", "d"]].iloc[j].values
                        if(abs(PlaneProcessor.__anglesVectors(vectorI, vectorJ)) < self.parallelismAngleThreshold):
                            zI = -1/vectorI[2]*(vectorI[0]*self.building.x + vectorI[1]*self.building.y + vectorI[3])
                            zJ = -1/vectorJ[2]*(vectorJ[0]*self.building.x + vectorJ[1]*self.building.y + vectorJ[3])
                            
                            if(zI[0] < zJ[0]):
                                self.exportPlanedf.polygon[i] = self.exportPlanedf.polygon[i].difference(self.exportPlanedf.polygon[j])
                            else:
                                self.exportPlanedf.polygon[j] = self.exportPlanedf.polygon[j].difference(self.exportPlanedf.polygon[i])
                        else:
                            # Something should be done here. This is fore intersections on non-parallel planes
                            print(i, j)

        # Delete planes that have empty polygons
        toDelete = []
        for i in range(len(self.exportPlanedf)):
            if(self.exportPlanedf.polygon[i].is_empty):
                toDelete.append(i)
        
        if(len(toDelete) > 0):
            self.exportPlanedf = self.exportPlanedf.drop(toDelete).reset_index(drop=True)
            toDelete.reverse()
            for i in toDelete:
                del self.planePointList[i]

    # End of overlap deleting functions
    ###########################################################################################################################################
    ###########################################################################################################################################
    # The following code contains all functions needed for the piercing polygons step. 
    # Some of these auxiliar functions may be used  in other steps of plane processing

    def pierce(self, cadastre=True):
        """
        Checks for empty areas inside every region and makes a square hole in them

        #### Inputs: (it also uses the exportPlanedf and the planePointList attributes)
        - cadastre: whether or not to consider the trimmedPolygon. If false, it will consider the polygon element from exportPlanedf
        
        #### Outputs: 
        - None: updates the exportPlanedf attribute
        """
        for planeID in range(len(self.exportPlanedf)):
            pointsDF = pd.DataFrame(self.planePointList[planeID])
            
            if(cadastre):
                outline = self.exportPlanedf.trimmedPolygon[planeID].buffer(self.minHoleSide)
            else:
                outline = self.exportPlanedf.polygon[planeID]

            # Uses PCA to determine square orientation
            X = pointsDF[["x", "y"]]
            pca = PCA(n_components=2)
            pca.fit(X)

            newData = pca.transform(pointsDF[["x", "y"]])
            newX = []
            newY = []
            for newPoint in newData:
                newX.append(newPoint[0])
                newY.append(newPoint[1])
            minX, minY, maxX, maxY = min(newX)-self.minHoleSide, min(newY)-self.minHoleSide, max(newX)+self.minHoleSide, max(newY)+self.minHoleSide

            xmin, ymin = np.dot([minX, minY], pca.components_) + pca.mean_
            xmax, ymax = np.dot([maxX, maxY], pca.components_) + pca.mean_

            density = len(pointsDF)/outline.area
            size = max(math.sqrt(self.minHoleSide/density), self.minHoleSide)            
            increment = size*self.slidingHole

            center = [xmin, ymin]
            vectorX = pca.components_[0]
            vectorY = pca.components_[1]
            basesquare = Polygon([center - vectorX*size/2 - vectorY*size/2,
                          center - vectorX*size/2 + vectorY*size/2,
                          center + vectorX*size/2 + vectorY*size/2,
                          center + vectorX*size/2 - vectorY*size/2])
            
            holes = []

            for i in range(int((maxX-minX)/increment+1)):
                for j in range(int((maxY-minY)/increment+1)):
                    offset = vectorX*increment*i + vectorY*increment*j           
                    square = shapely.affinity.translate(basesquare, xoff=offset[0], yoff=offset[1])
                    
                    nPoints = 0

                    # print(i,j, square.intersects(outline))
                    if(outline.intersects(square) or outline.contains(square)): #outline.intersects(square) or outline.contains(square)
                        for k in range(len(pointsDF)):
                            p = Point(pointsDF.x[k], pointsDF.y[k])
                            if(p.within(square)):
                                nPoints = nPoints + 1
                        if(nPoints == 0):
                            holes.append(square)

            if(len(holes) > 0):
                baseHole = holes[0]
                
                for i in range(1, len(holes)):
                    baseHole = baseHole.union(holes[i])

                if(cadastre):
                    self.exportPlanedf.trimmedPolygon[planeID] = outline.difference(baseHole)
                    self.__deleteMultipolygons(trimmed=True)
                else:
                    self.exportPlanedf.polygon[planeID] = outline.difference(baseHole)
                    self.__deleteMultipolygons(trimmed=False)



    def __deleteMultipolygons(self, trimmed=True):
        """
        Checks all the polygons in every region of exportPlanedf.
        If the found region is a multipolygon, it converts it to a single polygon.
        This is achieved by just keeping the first polygon of the list, but a more sophisticated method should be developed.

        #### Inputs: (it also uses the exportPlanedf attribute)
        - trimmed: whether or not to consider the trimmedPolygon. If false, it will consider the polygon element from exportPlanedf
        
        #### Outputs: 
        - None: updates the exportPlanedf attribute
        """
        if(trimmed):
            for i in range(len(self.exportPlanedf)):
                piercedItem = self.exportPlanedf.trimmedPolygon[i]
                if(type(piercedItem) != type(Polygon())):
                    self.exportPlanedf.trimmedPolygon[i] = self.exportPlanedf.trimmedPolygon[i].geoms[0]

        else:
            for i in range(len(self.exportPlanedf)):
                piercedItem = self.exportPlanedf.polygon[i]
                if(type(piercedItem) != type(Polygon())):
                    self.exportPlanedf.polygon[i] = self.exportPlanedf.polygon[i].geoms[0]


    # End of piercing functions
    ###########################################################################################################################################
    ###########################################################################################################################################
    # The following code contains all functions needed for the cadaster trimming step. 
    # Some of these auxiliar functions may be used  in other steps of plane processing

    def cadasterTrim(self, source, target):
        """
        Trims all polygons to be only inside the limits of cadaster.
        The new polygons are stored in the trimmedPolygon column.

        #### Inputs:
        - source: (int) coordinate reference system of the cadaster file
        - target: (int) coordinate reference system of the lidar file

        #### Outputs:
        - None. The exportPlanedf attribute gets updated
        """
        polygonData = gpd.read_file(self.cadastrePath)
        cadastreLimits = polygonData.geometry[0]
        if(type(cadastreLimits) != type(Polygon())):
            cadastreLimits = cadastreLimits.geoms[0]

        source_crs = 'epsg:' + str(source) # Coordinate system of the buildings (default=?)
        target_crs = 'epsg:' + str(target) # Coordinate system of LiDAR data (default=Czech)
        global_to_czech = pyproj.Transformer.from_crs(source_crs, target_crs)

        cadastreNewCoords = global_to_czech.transform(cadastreLimits.exterior.coords.xy[1], cadastreLimits.exterior.coords.xy[0])

        p = []

        for j in range(len(cadastreNewCoords[0])):
            p.append((cadastreNewCoords[0][j], cadastreNewCoords[1][j]))

        self.actualLimitsCadastre = Polygon(p)
        
        cadastreTrimmed = []
        
        for i in range(len(self.exportPlanedf)):
            cadastreTrimmed.append(self.exportPlanedf.polygon[i].intersection(self.actualLimitsCadastre))
            self.exportPlanedf.area[i] = cadastreTrimmed[i].area*1/math.cos(self.exportPlanedf.tilt[i]*math.pi/180)
        
        self.exportPlanedf["trimmedPolygon"] = cadastreTrimmed

        self.__deleteMultipolygons(trimmed=True)

    # End of cadaster trimming functions
    ###########################################################################################################################################
       
    def exportResults(self):
        """
        Exports the final results in a .csv file. It also generates a final plot and 3d scatter
        The exported resulted are ordered by height
        
        #### Inputs:
        - None 

        #### Outputs:
        - None

        #### Exports:
        - .csv of all the parameters of each plane (as well as area, tilt, azimuth and polygon definition)
        - .png image of the 3D scatter plot
        - .png image of the 2D scatter plot + regions plot
        """
        # Delete planes that are empty
        toDelete = []
        for i in range(len(self.exportPlanedf)):
            if(self.exportPlanedf.trimmedPolygon[i].is_empty):
                toDelete.append(i)
            elif(self.exportPlanedf.trimmedPolygon[i].area < self.minAreaDelete):
                toDelete.append(i)
            elif(len(self.planePointList[i])/self.exportPlanedf.trimmedPolygon[i].area < self.minDensityDelete):
                toDelete.append(i)
        
        if(len(toDelete) > 0):
            self.exportPlanedf = self.exportPlanedf.drop(toDelete).reset_index(drop=True)
            toDelete.reverse()
            for i in toDelete:
                del self.planePointList[i]

        # Sort results by Origin and center height
        self.exportPlanedf["zcenter"] = 0
        self.exportPlanedf.zcenter = -1/self.exportPlanedf.c*(self.exportPlanedf.a*self.building.x[0] + self.exportPlanedf.b*self.building.y[0] + self.exportPlanedf.d)

        self.exportPlanedf = self.exportPlanedf.sort_values(["Origin", "zcenter"])

        self.planePointList = [self.planePointList[i] for i in self.exportPlanedf.index]

        self.exportPlanedf = self.exportPlanedf.drop("zcenter", axis=1).reset_index(drop=True)

        # Export results
        for i in range(len(self.planePointList)):
            planePointsFile = self.processedResultsPath + self.building.identifier[0] + "_" + str(i) + '.csv'
            self.planePointList[i].to_csv(planePointsFile, header=False, index=False)

        planeFile = self.processedResultsPath + 'PlaneList_' + self.building.identifier[0]  + '.csv'
        name = "4 - Cadastre Trimmed - " + self.building.identifier[0]
        self.plotPlanes(name, showTrimmed=True)
        self.plot3d(name)
        self.exportPlanedf.to_csv(planeFile, header=True, index=False)  

    def plot3d(self, name):
        """
        Plots a 3D scatter of the points for each plane (colorcoded) 
        Name is the title (and part of the image name)
        Generated plot is stored in imagesPath path

        #### Inputs:
        - name: fileName and title of the plot
        
        #### Outputs:
        - None

        #### Exports:
        - .png of a plot with the current state of the plane processing. Contains, for all planes, their 3d point scatter
        """

        fig = plt.figure()
        ax = fig.add_subplot(projection='3d')
        color = iter(plt.cm.rainbow(np.linspace(0, 1, len(self.planePointList))))
        _ = ax.scatter(self.buildingPoints.x, self.buildingPoints.y, self.buildingPoints.z, c="Gray", marker=".", alpha = 0.1)
        
        for k in range(len(self.planePointList)):
            c = next(color)
            _ = ax.scatter(self.planePointList[k].x, self.planePointList[k].y, self.planePointList[k].z, color=c, label=k, marker=".")

        ax.legend(loc="lower left")
        ax.set_aspect('equal', adjustable='box')
        filenameImage = self.processedImagesPath + name + '_Plane3D_' + ".png"
        plt.savefig(filenameImage)
        plt.close()