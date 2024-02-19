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
from Classes.PlaneDetector import PlaneDetector

class PlaneProcessor:
    def __init__(self, building, segmentedPath, identifiedPaths, processedPaths, cadastrePath, generateFigures=True,
                 planeDistanceThreshold=0.5, angleThreshold=10, mergeReiterations=10, splitDistanceThreshold = 2, angleSplitIncrement=5,
                 minPointsDelete=3, minAreaDelete=5, minDensityDelete=0.25, convexHullHorizon = 5, parallelismAngleThreshold=5, slidingHole=0.75, minHoleSide = 3):
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
                
    @staticmethod    
    def getAngles(normal):
        """
        Given the normal vector of a plane, returns the tilt and azimuth of that plane.
        Tilt goes from 0 (horizontal) to 90 degrees (vertical) whereas azimuth goes from 0 (pointing north) to 360 degrees, growing clockwise
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

    @staticmethod
    def anglesVectors(u, v):
        """
        Given two vectors (for this project, they are two planes normals), it computes the angle (in degrees) between them
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
    def pseudoDistance(plane, pointsOtherPlane):
        """
        Given a plane defined by its equation, and a list of points belonging to another plane, it computes the average distance of the points to the plane.
        This is meant to be the distance between non-parallel planes (I feel the hatred of a lot of mathematicians right now).
        """
        dist = 0
        for i in range(len(pointsOtherPlane)):
            point = pointsOtherPlane[["x", "y", "z"]].iloc[i].values
            dist = dist + PlaneDetector.distancePlane(point, plane)
        return dist/len(pointsOtherPlane)

    def getClosestPlanes(self):
        """
        Given some planes (the planes equations are in a df and the points for each plane are in a list of dfs), returns the indexes of those planes that are close enough.
        Planes are close if their angle is below a threshold (they are somewhat parallel) and their (pseudo)distance is below another threshold.
        Only planes from the same height split can be merged  
        This does not reutrn the closest planes, but the first ones that fulfill the criteria 
        """
        for i in range(len(self.planedf)):
            for j in range(i+1, len(self.planedf)):
                if(self.planedf["Origin"].iloc[i] == self.planedf["Origin"].iloc[j]): # Only merge planes from same height level
                    u = self.planedf[["a", "b", "c",  "d"]].iloc[i].values
                    v = self.planedf[["a", "b", "c",  "d"]].iloc[j].values
                    angle = PlaneProcessor.anglesVectors(u, v)
                    if(angle < self.angleThreshold):
                        if(PlaneProcessor.pseudoDistance(u, self.planePointList[j]) < self.planeDistanceThreshold):
                            return[i, j]

        return[-1, -1]
    # For plotting purposes
    @staticmethod 
    def convexhull(p):
        """
        Given a list of points (x and y coordinates), returns a convex polygon
        """
        p = np.array(p)
        hull = ConvexHull(p)
        return p[hull.vertices,:]


    @staticmethod
    def computeAngle(currentPoint, finalPoint):
        if(finalPoint.name == currentPoint.name):  return np.inf
        else:
            deltaX = finalPoint.x - currentPoint.x
            deltaY = finalPoint.y - currentPoint.y
            angle = math.atan2(deltaX, deltaY)*180/math.pi
            if(angle < 0): angle = angle + 360
            return angle
    @staticmethod
    def getDistanceSquare(currentPoint, finalPoint):
        deltaX = finalPoint.x - currentPoint.x
        deltaY = finalPoint.y - currentPoint.y
        return deltaX**2 + deltaY**2
    @staticmethod
    def shortSightedConvexHull(planePoints, thresholdDistance):
        squareDistThreshold = thresholdDistance**2
        pointsDF = planePoints.copy().reset_index(drop=True)

        pointList = []
        # Step 1: append left-most point
        previousIndex = []
        previousIndex.append(pointsDF.x.idxmin())
        currentPoint = pointsDF.iloc[pointsDF.x.idxmin()]
        pointList.append([currentPoint.x, currentPoint.y])

        lastAngle = -np.inf

        while(currentPoint.name != previousIndex or len(pointList) <= 1):
        # for j in range(13):   

            minAngle = np.inf
            minIndex = -1
            for i in range(len(pointsDF)):
                currentAngle = PlaneProcessor.computeAngle(currentPoint, pointsDF.iloc[i])

                if(currentAngle < minAngle and currentAngle > lastAngle):
                    minAngle = currentAngle
                    minIndex = i

            foundInsideThreshold = False
            minIndexConvex = minIndex # We save this to recover the point later
            minAngleConvex = minAngle

            if(PlaneProcessor.getDistanceSquare(currentPoint, pointsDF.iloc[minIndex]) > squareDistThreshold): #If point is too far we must look for closer points
                # print("Point ", [pointsDF.iloc[minIndex].x, pointsDF.iloc[minIndex].y], " is too far from ", [currentPoint.x, currentPoint.y])
                # We repeat the process, but now we only selcet points below the distance threshold
                minAngle = np.inf
                minIndex = -1
                for i in range(len(pointsDF)):
                    currentAngle = PlaneProcessor.computeAngle(currentPoint, pointsDF.iloc[i])
                    if(currentAngle < minAngle and currentAngle > lastAngle and PlaneProcessor.getDistanceSquare(currentPoint, pointsDF.iloc[i]) < squareDistThreshold):
                        minAngle = currentAngle
                        minIndex = i
                        foundInsideThreshold = True

            if(minIndex == -1 and minIndexConvex == -1):
                break

            if(foundInsideThreshold):
                lastAngle = PlaneProcessor.computeAngle(pointsDF.iloc[minIndex], pointsDF.iloc[minIndexConvex])
                currentPoint = pointsDF.iloc[minIndex]
                previousIndex.append(minIndex)
            else:
                lastAngle = minAngleConvex
                currentPoint = pointsDF.iloc[minIndexConvex]
                previousIndex.append(minIndexConvex)
            
            pointList.append([currentPoint.x, currentPoint.y])

        if(previousIndex[-1] != previousIndex[0]):
            del pointList[0]

        return pointList

    @staticmethod
    def plot_polygon(ax, poly, **kwargs):
        path = Path.make_compound_path(
            Path(np.asarray(poly.exterior.coords)[:, :2]),
            *[Path(np.asarray(ring.coords)[:, :2]) for ring in poly.interiors])

        patch = PathPatch(path, **kwargs)
        collection = PatchCollection([patch], **kwargs)
        
        ax.add_collection(collection, autolim=True)
        ax.autoscale_view()
        return collection

    def plotPlanes(self, name, showTrimmed=False):
        """
        Given a list of dfs (each df is made of points belonging to a certain plane), plots the points and a polygon for each plane (colorcoded)  
        Name is the title (and part of the image name)
        Generated plot is stored in file path
        """
        if(self.generateFigures):
            # plt.rcParams['figure.figsize'] = [6,6]
            fig, ax = plt.subplots()
            color = iter(plt.cm.rainbow(np.linspace(0, 1, len(self.planePointList))))
            _ = ax.scatter(self.buildingPoints.x, self.buildingPoints.y, c="Gray", marker=".", alpha = 0.1)
            if(not showTrimmed):
                for i in range(len(self.planePointList)):
                    df = self.planePointList[i].copy()

                    x = df["x"]
                    y = df["y"]

                    p = []

                    for j in range(len(x)):
                        p.append((x[j], y[j]))
                    
                    if(len(p) > 2):
                        ax.set_title(name)
                        c = next(color)

                        poly = plt.Polygon(PlaneProcessor.convexhull(p), color=c, alpha=0.2) #poly = plt.Polygon(convexhull(p), ec="k", alpha=0.2)
                        ax.add_patch(poly)    
                        
                        ax.scatter(x, y, color=c, marker='.',  label=i)
                
                ax.legend(loc="lower left")
                ax.set_aspect('equal', adjustable='box')
                
                filename = self.processedImagesPath + name + ".png"
                #plt.show()
                fig.savefig(filename)
                plt.close()

            else:
                for i in range(len(self.planePointList)):
                    df = self.planePointList[i].copy()

                    x = df["x"]
                    y = df["y"]

                    p = []

                    for j in range(len(x)):
                        p.append((x[j], y[j]))
                    
                    if(len(p) > 2):
                        ax.set_title(name)
                        c = next(color)

                        # PlaneProcessor.plot_polygon(ax, self.exportPlanedf.trimmedPolygon[i], color=c,alpha=0.25)
                        if(type(self.exportPlanedf.trimmedPolygon[i]) == type(Polygon())):
                            plt.fill(*self.exportPlanedf.trimmedPolygon[i].exterior.xy, color=c, alpha=0.2)

                            for interior in self.exportPlanedf.trimmedPolygon[i].interiors:
                                plt.plot(*interior.xy, color="k", alpha=0.5)

                        else:
                            for geom in self.exportPlanedf.trimmedPolygon[i].geoms:
                                plt.fill(*geom.exterior.xy, color=c, alpha=0.2)

                                for interior in geom.interiors:
                                    plt.plot(*interior.xy, color="k", alpha=0.5) #plt.fill(*interior.xy, color=c, alpha=0.2)

                        ax.scatter(x, y, color=c, marker='.',  label=i)
                
                ax.legend(loc="lower left")
                ax.set_aspect('equal', adjustable='box')
                
                filename = self.processedImagesPath + name + ".png"
                #plt.show()
                fig.savefig(filename)
                plt.close()
                
    

            
       
    def mergePlanes(self):
        """
        Given two indexes and all the needed planes info, merges two planes.
        The merging process consists of refitting a plane containing points from both planes.
        After the merging, if reiterations > 1, the plane looks for inliers (closer than distance threshold) in the whole building df and refits as many times as specified.
        """
        # Copy plane df and delete planes to merge
        newplanedf = self.planedf.copy()
        origin = self.planedf.Origin[self.indexesToMerge[0]]
        newplanedf = newplanedf.drop(index = self.indexesToMerge).reset_index(drop=True)

        # Get all points from both planes in a single df
        planePoints = [self.planePointList[self.indexesToMerge[0]], self.planePointList[self.indexesToMerge[1]]]
        planePointsMerged = pd.concat(planePoints).reset_index(drop=True)

        # Fit a plane
        X = planePointsMerged[["x", "y"]].values
        y = planePointsMerged[["z"]].values
        model = linear_model.LinearRegression().fit(X, y)
        plane = [model.coef_[0][0], model.coef_[0][1], -1, model.intercept_[0]]
        tilt, azimuth = PlaneProcessor.getAngles(np.array(plane))
        
        inliers = []
        for i in range(len(planePointsMerged)): # We could sub planePointsMerged for self.buildingPoints
            point = planePointsMerged[["x", "y", "z"]].iloc[i]
            if(PlaneDetector.distancePlane(point, plane) < self.planeDistanceThreshold):
                inliers.append(i)

        planeMergedInliers = planePointsMerged[["x", "y", "z"]].iloc[inliers].reset_index(drop=True) #pointsdf or planePointsMerged


        for k in range(self.mergeReiterations):

            # Find inliers and add them to the new plane
            inliers = []
            for i in range(len(planePointsMerged)): # We could sub planePointsMerged for pointsdf
                point = planePointsMerged[["x", "y", "z"]].iloc[i]
                if(PlaneDetector.distancePlane(point, plane) < self.planeDistanceThreshold):
                    inliers.append(i)

            planeMergedInliers = planePointsMerged[["x", "y", "z"]].iloc[inliers].reset_index(drop=True) #pointsdf or planePointsMerged

            X = planeMergedInliers[["x", "y"]].values
            y = planeMergedInliers[["z"]].values
            model = linear_model.LinearRegression().fit(X, y)
            plane = [model.coef_[0][0], model.coef_[0][1], -1, model.intercept_[0]]
            tilt, azimuth = PlaneProcessor.getAngles(np.array(plane))

        self.planePointList.append(planeMergedInliers) 

            
        # # This line must be toggled with the previous paragraf
        # planePointList.append(planePointsMerged)        

        # Delete old planes from planePointList 
        del self.planePointList[self.indexesToMerge[1]] # ! Must delete indexes1 first, or else it changes order
        del self.planePointList[self.indexesToMerge[0]]

        # Add newplane
        newplane = plane
        newplane.append(origin)
        newplane.append(tilt)
        newplane.append(azimuth)
        # newplane.append(0) #Area = 0
        # newplane.append("0") #Polygon not yet defined
        newplanedf.loc[len(newplanedf)] = newplane

        self.planedf = newplanedf

    def loadIdentifiedData(self):
        create_output_folder(self.processedResultsPath, deleteFolder=True)
        create_output_folder(self.processedImagesPath, deleteFolder=True)
    
        planeList = []
        for i, item in enumerate(pathlib.Path((self.planeListPath)).iterdir()):
            if item.is_file():
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
            tilt, azimuth = PlaneProcessor.getAngles(normal)
            tilts.append(tilt)
            azimuths.append(azimuth)

        self.planedf["tilt"] = tilts
        self.planedf["azimuth"] = azimuths

        # Get the plane points

        self.buildingFiles = []
        for item in pathlib.Path((self.planePointsPath)).iterdir():
            if item.is_file():
                self.buildingFiles.append(str(item))

        self.planePointList = []

        for i in range(len(self.buildingFiles)):
            df = pd.read_csv(self.buildingFiles[i], header=None)
            df = df.rename(columns={0: "x", 1: "y", 2:"z"})
            self.planePointList.append(df)

        # Get the building points
        # print(len(self.planedf))
        # print(len(self.planePointList))
    
        self.buildingPoints = pd.read_csv((self.segmentedPath + "/" + self.building.identifier[0] + ".csv"), header=None)
        self.buildingPoints = self.buildingPoints.rename(columns={0: "x", 1: "y", 2:"z"})
        
    @staticmethod    
    def distPointPlane(x, y, A, B=1, C=0):
        """
        Computes the distance between a point and the straight line Ax+By+C=0
        """   
        #return (A*x+B*y+C)**2/(A**2+B**2)
        return abs(A*x+B*y+C)/math.sqrt(A**2+B**2)

    def lineSplit(self, originaldf, startingAngle=0):
        """
        Given a point dataframe, it splits it in the directions it detects discontinuities above thresholdDistance.
        It tests all cases (does the full lap) in a resolution of increment.
        This is iterative: once a plane is split, the "swept" half is scanned for more discontinuities and, once it is finished, the algorithm returns to the "unswept" half 
        """ 
        planedf = pd.DataFrame(originaldf)
        splittedPlanes = []
        #cutoffAngles = []

        for angle in range(startingAngle, 360, self.angleSplitIncrement):
            A = -math.tan(angle*math.pi/180)
            
            # Computes distance for all points
            for index,point in planedf.iterrows():
                planedf.loc[index,"distance"] = PlaneProcessor.distPointPlane(point.x, point.y, A)
            
            # Orders points by distance and calculates difference
            planedf = planedf.sort_values("distance").reset_index(drop=True)

            planedf["deltaDist"] = np.concatenate(([0], planedf.distance[1:len(planedf)].values - planedf.distance[0:len(planedf)-1].values))

            lastSplit = 0

            for i in range(len(planedf)): #range(1,len(planedf))
                if(planedf.deltaDist[i] > self.splitDistanceThreshold):
                    subplane = pd.DataFrame(planedf.iloc[lastSplit:i].reset_index(drop=True))
                    subsplittedPlanes = self.lineSplit(subplane, startingAngle=0) #startingAngle=angle

                    if(len(subsplittedPlanes) <= 1):
                        splittedPlanes = splittedPlanes + [subplane]
                        #cutoffAngles = cutoffAngles + [angle]

                    else:
                        splittedPlanes = splittedPlanes + subsplittedPlanes
                        #cutoffAngles = cutoffAngles + subsplitAngles
                    lastSplit = i
            
            planedf = planedf.iloc[lastSplit:len(planedf)].reset_index(drop=True)

        # Append "final group"
        if(lastSplit != len(planedf)-1):
            splittedPlanes.append(planedf.iloc[lastSplit:len(planedf)])

        return splittedPlanes#, cutoffAngles

    def isPlaneDeletable(self, planePointdf):
        """
        Tests for all the criteria a plane should be deleted:
        - The plane has too few points
        - The plane area is too small
        - The point density is too small
        """ 
        if(len(planePointdf) < self.minPointsDelete):
            return True
        
        x = planePointdf["x"]
        y = planePointdf["y"]
        p = []
        for j in range(len(x)):
            p.append((x[j], y[j]))
        coords = PlaneProcessor.convexhull(p)
        polygon = Polygon(coords)
        if(polygon.area < self.minAreaDelete):
            return True

        if(len(planePointdf)/polygon.area < self.minDensityDelete):
            return True

        return False    

    def merge(self):
        """
    #### Inputs:
    - building (str): building name to look for
    - planeListBasePath (str): path where the plane list is.
    - planePointsBasePath (str): path where the poinnts for different planes are. We will read only those belonging to building
    - buildingPointsBasePath (str): path where the points for all the building are.
    - outputFilesBasePath(str): were to export the files

    #### Outputs: 
    - planedf: dataframe with the merged planes info. This includes plane parameters, level origin (from split heights) and tilt and azimuth 
    - planePointList: list of df containing the points for each plane (in the same order as planedf)

    #### Exports: 
    - images for the initial situation, the middle steps, and the final planes found (merged)
        """
        # self.loadIdentifiedData()

        # Merge all planes that are similar and plot and export results
        self.indexesToMerge = self.getClosestPlanes()

        i = 0
        name = "0 - Original - " + self.building.identifier[0]
        self.plotPlanes(name)
        
        while(self.indexesToMerge[0] > -1):
            i = i +1
            # print(indexes)
            self.mergePlanes()

            name = self.building.identifier[0] + "_Merging iteration " + str(i) + " - Merged " + str(self.indexesToMerge [0]) + " and " + str(self.indexesToMerge [1])  
            self.plotPlanes(name)  
            self.indexesToMerge = self.getClosestPlanes()

        name = "1 - Merging - " + self.building.identifier[0] + "- merged after " + str(i) + " stages"
        self.plotPlanes(name)

    def splitDelete(self):
        """
    #### Inputs:
    - building (str): building name to look for
    - outputFilesBasePath(str): were to export the files.
    - planedf: dataframe with the merged planes info. This includes plane parameters, level origin (from split heights) and tilt and azimuth 
    - planePointList: list of df containing the points for each plane (in the same order as planedf)
    - minPoints: minimum number a plane is required to have to be valid. If minimum cirteria is not fulfilled, the plane will be discarded
    - minArea: minimum area a plane is required to have to be valid
    - minDensity: minimum poinnt density a point is required to have to be valid 

    #### Outputs: 
    - splitPlaneParamsdf: df containing the plane info (parameters, plane of origin, tilt and azimuth) for the planes obtained after splitting

    #### Exports: 
    - splitPlaneParamsdf: the same df that it returns is exported to a csv
    - each individual plane points is also exported in a csv
        """
        howManySplits = 0

        # Splits all planes in regions of points that are too far apart

        self.splitPlanes = []
        self.splitPlaneParams = []

        for i in range(len(self.planePointList)):
            originaldf = pd.DataFrame(self.planePointList[i])
            dividedPlane = self.lineSplit(originaldf)
            self.splitPlanes = self.splitPlanes + dividedPlane

            # Recalculate plane parameters, only if the plane was split
            if(len(dividedPlane) > 1):
                howManySplits = howManySplits + 1
                for j in range(len(dividedPlane)):
                    X = dividedPlane[j][["x", "y"]].values
                    y = dividedPlane[j][["z"]].values
                    model = linear_model.LinearRegression().fit(X, y)
                    plane = [model.coef_[0][0], model.coef_[0][1], -1, model.intercept_[0]]
                    tilt, azimuth = PlaneProcessor.getAngles(np.array(plane))

                    newplane = plane
                    newplane.append(self.planedf.Origin[i]) # Get the original height level
                    newplane.append(tilt)
                    newplane.append(azimuth)
                    self.splitPlaneParams.append(newplane)

            else:
                self.splitPlaneParams.append(self.planedf.iloc[i].values)
                


        # Export split planes
        self.planePointList = self.splitPlanes
        name = "2 - Split Final - " + self.building.identifier[0]
        self.plotPlanes(name)

        # Deletes deletable plane
        
        self.planesToDelete = []

        for i in range(len(self.splitPlanes)):
            if(self.isPlaneDeletable(self.splitPlanes[i])):
                self.planesToDelete.append(i)

        self.planesToDelete.reverse()
        for x in self.planesToDelete:
            del self.splitPlanes[x]  
            del self.splitPlaneParams[x]

        # Saves plane parameters

        self.splitPlaneParams = pd.DataFrame(self.splitPlaneParams)
        self.splitPlaneParams.columns = self.planedf.columns
        self.planedf = self.splitPlaneParams

        # Export final result planes
        self.planePointList = self.splitPlanes
        name = "3 - Deleted Bad planes - " + self.building.identifier[0] + " - " + str(len(self.planesToDelete)) + " planes deleted"
        self.plotPlanes(name)

        # Get all results in a dataframe

        areas = []
        polygons = []

        for i in range(len(self.planedf)):
            x =  self.planePointList[i]["x"]
            y =  self.planePointList[i]["y"]
            polygon = PlaneProcessor.generatePolygon(x, y)
            # coords = PlaneProcessor.shortSightedConvexHull(self.planePointList[i], self.convexHullHorizon)
            # polygon = Polygon(coords)
            areas.append(polygon.area*1/math.cos(self.planedf.tilt[i]*math.pi/180))
            polygons.append(polygon)

        self.exportPlanedf = self.planedf.copy()
        self.exportPlanedf["area"] = areas
        self.exportPlanedf["polygon"] = polygons
        return howManySplits
        
    @staticmethod
    def generatePolygon(x, y):
        p = []
        for j in range(len(x)):
            p.append((x[j], y[j]))
        coords = PlaneProcessor.convexhull(p)
        polygon = Polygon(coords)
        return polygon

    def cadastreTrim(self, source, target):
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

        self.deleteMultipolygons(trimmed=True)

    def deleteOverlaps(self):
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
                        if(abs(PlaneProcessor.anglesVectors(vectorI, vectorJ)) < self.parallelismAngleThreshold):
                            zI = -1/vectorI[2]*(vectorI[0]*self.building.x + vectorI[1]*self.building.y + vectorI[3])
                            zJ = -1/vectorJ[2]*(vectorJ[0]*self.building.x + vectorJ[1]*self.building.y + vectorJ[3])
                            
                            if(zI[0] < zJ[0]):
                                self.exportPlanedf.polygon[i] = self.exportPlanedf.polygon[i].difference(self.exportPlanedf.polygon[j])
                            else:
                                self.exportPlanedf.polygon[j] = self.exportPlanedf.polygon[j].difference(self.exportPlanedf.polygon[i])
                        else:
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

    def deleteMultipolygons(self, trimmed=True):
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

    def computePCA(self):
        X = self.buildingPoints[["x", "y"]]
        self.pca = PCA(n_components=2)
        self.pca.fit(X)

    def pierce(self, cadastre=True):
        self.computePCA()
        for planeID in range(len(self.exportPlanedf)):
            # X = self.buildingPoints[["x", "y"]]
            # self.pca = PCA(n_components=2)
            # self.pca.fit(X)
            
            pointsDF = pd.DataFrame(self.planePointList[planeID])
            if(cadastre):
                outline = self.exportPlanedf.trimmedPolygon[planeID]
            else:
                outline = self.exportPlanedf.polygon[planeID]

            X = pointsDF[["x", "y"]]
            self.pca = PCA(n_components=2)
            self.pca.fit(X)

            newData = self.pca.transform(pointsDF[["x", "y"]])
            newX = []
            newY = []
            for newPoint in newData:
                newX.append(newPoint[0])
                newY.append(newPoint[1])
            minX, minY, maxX, maxY = min(newX), min(newY), max(newX), max(newY)

            xmin, ymin = np.dot([minX, minY], self.pca.components_) + self.pca.mean_
            xmax, ymax = np.dot([maxX, maxY], self.pca.components_) + self.pca.mean_

            density = len(pointsDF)/outline.area
            size = max(math.sqrt(self.minHoleSide/density), self.minHoleSide)            
            increment = size*self.slidingHole

            center = [xmin, ymin]
            vectorX = self.pca.components_[0]
            vectorY = self.pca.components_[1]
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
                    if(outline.contains(square)): #outline.intersects(square) or outline.contains(square)
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
                    self.deleteMultipolygons(trimmed=True)
                else:
                    self.exportPlanedf.polygon[planeID] = outline.difference(baseHole)
                    self.deleteMultipolygons(trimmed=False)

    def exportResults(self):
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

        # Delete planes by criteria
        

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
        # self.splitPlaneParams = pd.DataFrame(self.splitPlaneParams)
        # self.splitPlaneParams.columns = self.planedf.columns
        name = "4 - Cadastre Trimmed - " + self.building.identifier[0]
        self.plotPlanes(name, showTrimmed=True)
        self.exportPlanedf.to_csv(planeFile, header=True, index=False)  