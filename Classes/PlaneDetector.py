import numpy as np
import pandas as pd
import math #pi, sqrt
from sklearn import linear_model
from matplotlib import pyplot as plt
import random
import math #ceil
from Functions.general_functions import create_output_folder
import cv2 
from shapely import Polygon, Point
from shapely import wkt
from scipy.spatial import ConvexHull


class PlaneDetector:
    """
    Used for 
    
    ### Attributes: 
    #### Defined upon initialization:
    - 
    #### Self-generated:
    - 

    ### Public methods:
    - 
    """

    def __init__(self, building, segmented_LiDAR, savePaths, generateFigures=True, 
                 heightThreshold=1, nGradient=5, ransacIterations=20, distanceThreshold=0.2, 
                 minGlobalPercentage=0.1, minPartialPercentage=0.4, stoppingPercentage=0.1, pdfExponent=2, densityMultiplier=0.5,
                 deleteFirst=True):
        
        self.building = building
        self.segmented_LiDAR = segmented_LiDAR

        self.planeListPath = savePaths[0] + "/"
        self.planePointsPath = savePaths[1] + "/"
        self.imagesPath = savePaths[2] + "/"

        self.heightThreshold = heightThreshold
        self.nGradient = nGradient
        self.ransacIterations = ransacIterations
        self.distanceThreshold = distanceThreshold
        self.minGlobalPercentage = minGlobalPercentage
        self.minPartialPercentage = minPartialPercentage
        self.stoppingPercentage = stoppingPercentage 
        self.generateFigures = generateFigures 
        self.pdfExponent = pdfExponent
        self.deleteFirst = deleteFirst
        self.densityMultiplier = densityMultiplier

    def __heightSplit(self, df, filter=True):
        """
        Given a dataframe, sorts points by height and splits it in groups separated by a threshold height difference
        If filter = True, deletes first group (ground points) and groups made of 3 points or less 
        """

        # Now filter

        newdf = df.sort_values("z").copy().reset_index(drop=True)
        newdf["deltaZ"] = np.concatenate(([0], newdf.z[1:len(df)].values - newdf.z[0:len(df)-1].values))

        heightGroups = []
        lastSplit = 0

        for i in range(1,len(newdf)):
            if(newdf.deltaZ[i] > self.heightThreshold):
                heightGroups.append(newdf.iloc[lastSplit:i])
                lastSplit = i

        # Append "final group"
        if(lastSplit != i):
            heightGroups.append(newdf.iloc[lastSplit:i])
        
        # Filter, if wanted
        if(not filter): 
            return heightGroups

        else:
            filteredHeightGroups = heightGroups.copy()
            if(self.deleteFirst):
                if(len(filteredHeightGroups) > 1):
                    # We delete floor
                    del filteredHeightGroups[0]

            toDelete = []

            # We delete all those with 3 points or less
            for x in range(len(filteredHeightGroups)):
                if(len(filteredHeightGroups[x]) <= 3):
                    toDelete.append(x)

            toDelete.reverse() #Change order so it deltes last groups first (thus indexes are not modified during the process)
            for x in toDelete:
                del filteredHeightGroups[x]   

        # Generate figures
        if(self.generateFigures):
            fig = plt.figure()
            ax = fig.add_subplot()
            nbins = int((math.ceil(df.z.max() - df.z.min()))/self.heightThreshold)
            # ax.hist(x=df.z, bins='auto', color='#0504aa', alpha=0.7, rwidth=0.85)
            ax.hist(x=df.z, bins=nbins, color='#0504aa', alpha=0.7, rwidth=0.85)
            filenameImage = self.imagesPath + self.building.identifier[0] + "_HeightHistogram.png"
            plt.savefig(filenameImage)
            plt.close()

            fig = plt.figure()
            ax = fig.add_subplot()
            color = iter(plt.cm.rainbow(np.linspace(0, 1, len(filteredHeightGroups))))
            _ = ax.scatter(df.x, df.y, c="Gray", marker=".", alpha = 0.1)
            
            for k in range(len(filteredHeightGroups)):
                c = next(color)
                _ = ax.scatter(filteredHeightGroups[k].x, filteredHeightGroups[k].y, color=c, label=k, marker=".")

            ax.legend(loc="lower left")
            ax.set_aspect('equal', adjustable='box')
            filenameImage = self.imagesPath + self.building.identifier[0] + "_HeightGroups.png"
            plt.savefig(filenameImage)
            plt.close()

            return filteredHeightGroups
          
    def __closestPoints(self, point,):
        """
        Returns the nGradient closests points to the given point
        """
        byDistance = self.currentGroup.copy()
        byDistance["distance"] = np.sqrt((byDistance.x - point.x)**2 + (byDistance.y - point.y)**2)
        return byDistance.sort_values("distance").head(self.nGradient+1)[1:].reset_index(drop=True)
        #return byDistance.sort_values("distance").head(nGradient+1)[1:]

    def __partialGradient(self, point, neighbors):
        """
        Calculates the gradient of a given point given a list of its closest neighbors
        """
        grad = [0,0]
        for i in range(len(neighbors)):
            pt = neighbors.iloc[[i]]
            if(pt.x.values - point.x != 0):
                grad[0] = grad[0] + (pt.z.values - point.z)/(pt.x.values - point.x)
            if(pt.y.values - point.y != 0):
                grad[1] = grad[1] + (pt.z.values - point.z)/(pt.y.values - point.y)
        return grad 

    def __gradient(self, point):
        """
        Given a point and a dataframe, obtains its closest neighbors and calculates the gradient (merges __closestPoints() and __partialGradient())
        """
        return self.__partialGradient(point, self.__closestPoints(point))

    def __computeGradient(self):
        """
        Calculates the gradient of all points in a given dataframe
        """
        for index,point in self.currentGroup.iterrows():
            self.currentGroup.loc[index,"gradientX"],self.currentGroup.loc[index,"gradientY"] = self.__gradient(point)
        self.currentGroup["absGradient"] = np.sqrt(self.currentGroup["gradientX"]**2 + self.currentGroup["gradientY"]**2)
        self.currentGroup["angleGradientDeg"] = np.arctan2(self.currentGroup.gradientY,self.currentGroup.gradientX)*360/(2*math.pi)
        self.currentGroup["angleGradientDeg"][self.currentGroup["angleGradientDeg"] < 0 ] = self.currentGroup["angleGradientDeg"] + 360

        return self.currentGroup

    @staticmethod
    def distancePlane(point, planeParams):
        """
    #### Inputs:
    - point: 3-element array (x, y, z)
    - planeParams: 4-element array (a, b, c, d)

    #### Outputs: 
    - dis: absolute value of the distance bettween the point and the plane
        """
        a, b, c, d = planeParams[0], planeParams[1], planeParams[2], planeParams[3]
        x, y, z = point[0], point[1], point[2]
        dis = a*x + b*y + c*z+ d
        return abs(dis)
    
    @staticmethod
    def __planeDensity(planePoints):
        """
        Given a list of points (x and y coordinates), returns the plane density
        """

        x = planePoints["x"].values
        y = planePoints["y"].values

        p = []

        for j in range(len(x)):
            p.append((x[j], y[j]))

        p = np.array(p)
        hull = ConvexHull(p)
        limits = p[hull.vertices,:]
        polygon = Polygon(limits)
        area = polygon.area
        density = len(planePoints)/area
        return density


    def __ransac_ordered(self, pointsdf, minPoints): #n = 3
        """
        Given a dataframe, find a plane by applying RANSAC (with a given number of iterations).
        Points are inliers if the distance is above the thresholdDistance
        Planes are valid if they have more than minPoints inliers
        """
        
        # Returns the best point only for that data
        
        bestPlane = []
        bestStd = np.inf
        bestScore = 0
        bestPointsInPlane = []

        if(minPoints < len(pointsdf)):

            for i in range(self.ransacIterations):
                angles = pointsdf.angleGradientDeg
                hist, bins = np.histogram(angles, bins=int(360/5))
                
                probabilities = []

                for j in range(len(bins)-1):
                    for k in range(hist[j]):
                        probabilities.append(hist[j])
                
                probabilities = np.power(probabilities, self.pdfExponent) # This is done to further differences
                # probabilities = np.where(probabilities > np.mean(probabilities), 1, 0)
                probabilities = probabilities/sum(probabilities)


                cumulativeProbability = []
                for j in range(len(probabilities)):
                    cumulativeProbability.append(sum(probabilities[0:j]))
                
                pointsdf["cumulativeProbability"] = cumulativeProbability

     ########## This part is modified #############################

                # target 1: selected based on gradient
                target1 = np.random.random()
                while(target1 > pointsdf.cumulativeProbability.iloc[-1]):    target1 = np.random.random()
                rand1 = next(x for x, val in enumerate(pointsdf.cumulativeProbability) if val > target1)
                basePoint = pointsdf.loc[rand1]
                # print(basePoint)
                # targets 2 and 3: selected based on distance to target 1
                pointsdf["distance"] = np.sqrt((pointsdf.x - basePoint.x)**2 + (pointsdf.y - basePoint.y)**2)
                newpointsdf = pointsdf.copy()
                newpointsdf = newpointsdf[newpointsdf["distance"] != 0]

                probabilities = [1/x for x in newpointsdf.distance]
                
                probabilities = np.power(probabilities, self.pdfExponent) # This is done to further differences
                # probabilities = np.where(probabilities > np.mean(probabilities), 1, 0)
                probabilities = probabilities/sum(probabilities)

                cumulativeProbability = []
                for j in range(len(probabilities)):
                    cumulativeProbability.append(sum(probabilities[0:j]))
                
                newpointsdf["cumulativeProbability"] = cumulativeProbability

                target2, target3 = np.random.random(), np.random.random()
                while(target2 > newpointsdf.cumulativeProbability.iloc[-1]):    target2 = np.random.random()
                while(target3 > newpointsdf.cumulativeProbability.iloc[-1]):    target3 = np.random.random()
                rand2 = next(x for x, val in enumerate(newpointsdf.cumulativeProbability) if val > target2)
                rand3 = next(x for x, val in enumerate(newpointsdf.cumulativeProbability) if val > target3)
                
                while rand2 == rand3:
                    target2, target3 = np.random.random(), np.random.random()
                    while(target2 > newpointsdf.cumulativeProbability.iloc[-1]):    target2 = np.random.random()
                    while(target3 > newpointsdf.cumulativeProbability.iloc[-1]):    target3 = np.random.random()
                    rand2 = next(x for x, val in enumerate(newpointsdf.cumulativeProbability) if val > target2)
                    rand3 = next(x for x, val in enumerate(newpointsdf.cumulativeProbability) if val > target3)

                toFit = pd.concat([basePoint.to_frame().T, newpointsdf.iloc[[rand2, rand3]]])
                # print(toFit)

     ########## This part is the original from before #############################

                # target1, target2, target3 = np.random.random(), np.random.random(), np.random.random()
                # while(target1 > pointsdf.cumulativeProbability.iloc[-1]):    target1 = np.random.random()
                # while(target2 > pointsdf.cumulativeProbability.iloc[-1]):    target2 = np.random.random()
                # while(target3 > pointsdf.cumulativeProbability.iloc[-1]):    target3 = np.random.random()

                # # print("\t Targets:", target1, target2, target3)

                # rand1 = next(x for x, val in enumerate(pointsdf.cumulativeProbability) if val > target1)
                # rand2 = next(x for x, val in enumerate(pointsdf.cumulativeProbability) if val > target2)
                # rand3 = next(x for x, val in enumerate(pointsdf.cumulativeProbability) if val > target3)
                
                # while rand1 == rand2 or rand1 == rand3 or rand2 == rand3:
                #     target1, target2, target3 = np.random.random(), np.random.random(), np.random.random()
                #     while(target1 > pointsdf.cumulativeProbability.iloc[-1]):    target1 = np.random.random()
                #     while(target2 > pointsdf.cumulativeProbability.iloc[-1]):    target2 = np.random.random()
                #     while(target3 > pointsdf.cumulativeProbability.iloc[-1]):    target3 = np.random.random()
                #     rand1 = next(x for x, val in enumerate(pointsdf.cumulativeProbability) if val > target1)
                #     rand2 = next(x for x, val in enumerate(pointsdf.cumulativeProbability) if val > target2)
                #     rand3 = next(x for x, val in enumerate(pointsdf.cumulativeProbability) if val > target3)
                
                # toFit = pointsdf.iloc[[rand1, rand2, rand3]]
                # print(toFit)
     
     ########## This continues as usual #############################
                
                # Fit a plane with these n random points 
                X = toFit[["x", "y"]].values
                y = toFit[["z"]].values
                
                model = linear_model.LinearRegression().fit(X, y)
                plane = model.coef_[0][0], model.coef_[0][1], -1, model.intercept_[0]
                
                pointsInPlane = []
                # Check for all points in the dataset that can belong to this plane (distance below threshold)
                for j in range(len(pointsdf)):
                    point = pointsdf[["x", "y", "z"]].iloc[j]
                    pointsdf.loc[j, "dist"] = PlaneDetector.distancePlane(point, plane)
                    if(pointsdf.loc[j, "dist"] < self.distanceThreshold):
                        pointsInPlane.append(j)

                planePoints = pointsdf[["x", "y", "z", "dist"]].iloc[pointsInPlane]
                notPlanePoints = pointsdf[["x", "y", "z"]].copy().drop(pointsInPlane, axis = 0)

     ########## This part is modified #############################
                if(len(planePoints) > minPoints):
                    size = len(pointsInPlane)
                    std = np.sum(planePoints.dist)**2 / (len(planePoints) - 1)
                    density = PlaneDetector.__planeDensity(planePoints)
                    if(density > self.densityMultiplier*self.buildingDensity):
                        # If a good plane, check for score to decide if it is the best plane
                        newScore = size*std*density**2
                        if(newScore > bestScore):
                            bestPointsInPlane = pointsInPlane
                            bestPlane = plane
                            bestScore = newScore


     ########## This part is the original from before #############################
                # # Check the size of the pointInPlane df (and compare it to the min percentage required)
                # if(len(planePoints) > minPoints):
                #     # print("\t Fulfills minimum points criteria")
                #     newStd = np.sum(planePoints.dist)**2 / (len(planePoints) - 1)
                #     # If a good plane, check for std to decide if it is the best plane
                #     if(newStd < bestStd):
                #         bestPointsInPlane = pointsInPlane
                #         bestPlane = plane
                #         bestStd = newStd
        
        planePoints = pointsdf[["x", "y", "z", "dist"]].iloc[bestPointsInPlane].reset_index(drop=True)
        notPlanePoints = pointsdf[["x", "y", "z","angleGradientDeg"]].copy().drop(bestPointsInPlane, axis = 0).sort_values(by=["angleGradientDeg"]).reset_index(drop=True)
        # print("\t", bestPlane)
        # Return plane parameters and points inside roof
        return bestPlane, planePoints, notPlanePoints

    def detectPlanes(self):
        create_output_folder(self.planePointsPath, deleteFolder=True)
        create_output_folder(self.planeListPath, deleteFolder=True)
        create_output_folder(self.imagesPath, deleteFolder=True)

        df = pd.read_csv((self.segmented_LiDAR + "/" + self.building.identifier[0] + ".csv"), header=None)
        df = df.rename(columns={0: "x", 1: "y", 2:"z"})

        heightGroups = self.__heightSplit(df)

        self.buildingDensity = PlaneDetector.__planeDensity(df)

        for j in range(len(heightGroups)):
            print("Group:", j)
            self.currentGroup = heightGroups[j].copy().reset_index(drop=True)

            # Compute gradient
            self.currentGroup = self.__computeGradient()
            self.currentGroup = self.currentGroup.sort_values(by=["angleGradientDeg"])

            self.planeList = []
            self.planePointsList = []
            notPlanePoints = self.currentGroup.copy()

            looping = True

            while(looping): #Need to redefine the stopping criteria         # looping = len(notPlanePoints) > self.stoppingPercentage*len(self.currentGroup)
                # print("Another iteration")
                bestPlane, planePoints, notPlanePoints = self.__ransac_ordered(notPlanePoints, min(self.minGlobalPercentage*len(self.currentGroup), self.minPartialPercentage*len(notPlanePoints)))
                if(len(bestPlane) > 0):
                    self.planeList.append(bestPlane)
                    self.planePointsList.append(planePoints[["x","y","z"]])
                    print("\t Found plane. Percentatge remaining:", len(notPlanePoints)/len(heightGroups[j]))
                   
                    for k in range(len(self.planePointsList)):
                        filenamePoints =  self.planePointsPath + self.building.identifier[0] + "_Div_" + str(j) + "_plane " + str(k) + ".csv"
                        pd.DataFrame(self.planePointsList[k]).to_csv(filenamePoints, header=False, index=False) 
                    filenameList = self.planeListPath + self.building.identifier[0] + '_PlaneList_' + str(j)  + '.csv'
                    pd.DataFrame(self.planeList).to_csv(filenameList, header=False, index=False)   

                    ### Generate image as planes are found
                    if(self.generateFigures):
                        fig = plt.figure()
                        ax = fig.add_subplot()
                        color = iter(plt.cm.rainbow(np.linspace(0, 1, len(self.planePointsList))))
                        _ = ax.scatter(df.x, df.y, c="Gray", marker=".", alpha = 0.1)
                        
                        for k in range(len(self.planePointsList)):
                            filenamePoints =  self.planePointsPath + self.building.identifier[0] + "_Div_" + str(j) + "_plane " + str(k) + ".csv"
                            pd.DataFrame(self.planePointsList[k]).to_csv(filenamePoints, header=False, index=False) 
                            c = next(color)
                            _ = ax.scatter(self.planePointsList[k].x, self.planePointsList[k].y, color=c, label=k, marker=".")

                        ax.legend(loc="lower left")
                        ax.set_aspect('equal', adjustable='box')
                        filenameImage = self.imagesPath + self.building.identifier[0] + '_PlaneList_' + str(j) + "_Step_" + str(len(self.planePointsList)) + ".png"
                        plt.savefig(filenameImage)
                        plt.close()

                    else:
                        for k in range(len(self.planePointsList)):
                            filename =  self.planePointsPath  + self.building.identifier[0] + "_Div_" + str(j) + "_plane " + str(k) + ".csv"
                            pd.DataFrame(self.planePointsList[k]).to_csv(filename, header=False, index=False)

                # looping = len(notPlanePoints) > self.stoppingPercentage*len(self.currentGroup)
                # looping = (len(bestPlane) > 0)
                minPoints = np.inf
                for i in range(len(self.planePointsList)):
                    minPoints = min(minPoints, len(self.planePointsList[i]))
                
                looping = ((len(notPlanePoints) > max(self.stoppingPercentage*len(self.currentGroup), minPoints)) or (len(bestPlane) > 0)) and (len(notPlanePoints) > 3) 
                # if(len(notPlanePoints) < self.stoppingPercentage*len(self.currentGroup)): looping = False

            print("\t Run out of iterations")         

            # if(self.generateFigures):
            #     fig = plt.figure()
            #     ax = fig.add_subplot()
            #     color = iter(plt.cm.rainbow(np.linspace(0, 1, len(self.planePointsList))))
            #     _ = ax.scatter(df.x, df.y, c="Gray", marker=".", alpha = 0.1)
                
            #     for k in range(len(self.planePointsList)):
            #         filenamePoints =  self.planePointsPath + self.building.identifier[0] + "_Div_" + str(j) + "_plane " + str(k) + ".csv"
            #         pd.DataFrame(self.planePointsList[k]).to_csv(filenamePoints, header=False, index=False) 
            #         c = next(color)
            #         _ = ax.scatter(self.planePointsList[k].x, self.planePointsList[k].y, color=c, label=k, marker=".")

            #     ax.legend(loc="lower left")
            #     ax.set_aspect('equal', adjustable='box')
            #     filenameImage = self.imagesPath + self.building.identifier[0] + '_PlaneList_' + str(j) + ".png"
            #     plt.savefig(filenameImage)
            #     plt.close()

            # else:
            #     for k in range(len(self.planePointsList)):
            #         filename =  self.planePointsPath  + self.building.identifier[0] + "_Div_" + str(j) + "_plane " + str(k) + ".csv"
            #         pd.DataFrame(self.planePointsList[k]).to_csv(filename, header=False, index=False)

            filenameList = self.planeListPath + self.building.identifier[0] + '_PlaneList_' + str(j)  + '.csv'
            pd.DataFrame(self.planeList).to_csv(filenameList, header=False, index=False)     