import numpy as np
import pandas as pd
import math #pi, sqrt
from sklearn import linear_model
from matplotlib import pyplot as plt
import math #ceil
from Functions.general_functions import create_output_folder
from shapely import Polygon, Point
from shapely import wkt
from scipy.spatial import ConvexHull


class PlaneDetector:
    """
    Used for clustering the point cloud into different planes, appling RANSAC
    
    ### Attributes: 
    #### Defined upon initialization:
    - building: single row dataframe containing at least the following fields: identifier, x, y
    - segmented_LiDAR: file (.csv or .txt) of the building point cloud
    - planeListPath: where to store the lists of the found planes
    - planePointsPath: where to store the lists of the points belonging to the found planes
    - imagesPath: where to export the generated figures
    ##### Attributes with default values
    - generateFigures: whether or not to export figures of the whole process
    - heightThreshold: minimum vertical distance between consecutive points (when ordered by altitude) to consider that they belong to different clusters
    - deleteFirst: whether or not to delete the first group when spliting by height (if there is only one group, it is never deleted)
    - nGradient: how many neighbors to consider when computing the gradient
    - ransacIterations: how many iterations will perform the RANSAC algorithm
    - distanceThreshold: maximum distance a point can be from a plane to be considered an inlier
    - minGlobalPercentage: percentage of point needed for a plane to be valid, relative to the whole heightgroup
    - minPartialPercentage: percentage OF THE REMAINING POINTS needed for a plane to be valid. When in conflict with minGlobalPercentage, the one that implies less points prevails.
    - stoppingPercentage (currently not used): maximum percentatge of remaining
    - pdfExponent: exponent applied to the probability density functions to the gradient and distance distribution for RANSAC starting points sampling
    - densityMultiplier: for density filtering, ratio (regarding the density of the current Heightgroup) a plane must have to be valid

    ### Public methods:
    - score: function to asses how goood a plane is, based on (size, std, density). Thi method is thought to be redefined from outside this class
    - detectPlanes: given the building points, find the planes
    """

    def __init__(self, building, segmented_LiDAR, savePaths, generateFigures=True, 
                 heightThreshold=1, deleteFirst=True, 
                 nGradient=5, ransacIterations=20, distanceThreshold=0.2, 
                 minGlobalPercentage=0.1, minPartialPercentage=0.4, stoppingPercentage=None, pdfExponent=2, densityMultiplier=0.5):
        """
    #### Inputs:
    - building: single row dataframe containing at least the following fields: identifier, x, y
    - segmented_LiDAR: file (.csv or .txt) of the building point cloud
    - savePaths, array containing planeListPath, planePointsPath and imagesPath
    ##### with default values
    - generateFigures: whether or not to export figures of the whole process
    - heightThreshold: minimum vertical distance between consecutive points (when ordered by altitude) to consider that they belong to different clusters
    - deleteFirst: whether or not to delete the first group when spliting by height (if there is only one group, it is never deleted)
    - nGradient: how many neighbors to consider when computing the gradient
    - ransacIterations: how many iterations will perform the RANSAC algorithm
    - distanceThreshold: maximum distance a point can be from a plane to be considered an inlier
    - minGlobalPercentage: percentage of point needed for a plane to be valid, relative to the whole heightgroup
    - minPartialPercentage: percentage OF THE REMAINING POINTS needed for a plane to be valid. When in conflict with minGlobalPercentage, the one that implies less points prevails.
    - stoppingPercentage (currently not used): maximum percentatge of remaining
    - pdfExponent: exponent applied to the probability density functions to the gradient and distance distribution for RANSAC starting points sampling
    - densityMultiplier: for density filtering, ratio (regarding the density of the current Heightgroup) a plane must have to be valid
        """
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

    def __heightSplit(self, buildingPoints, filter=True):
        """
        Given a dataframe, sorts points by height and splits it in groups separated by a threshold height difference (defined in the class initialization)
       
        #### Inputs:
        - buildingPoints: dataframe containing x, y, z fields corresponding to the point cloud of the building
        - filter: If filter = True, deletes first group (ground points) and groups made of 3 points or less 

        #### Outputs:
        - filteredHeightGroups: list containing the points in the building (without the filtered ones, if it applies) in their respective heightgroup (ordered by height).

        #### Exports:
        - if generateFigures, it exports two images: one corresponding to the point height histogram (to "see" where the heightgroups should be split), and another of the data cloud colorcoded by heightgroups.
        """

        # Prepare dataframe and split

        newdf = buildingPoints.copy().sort_values("z").reset_index(drop=True)
        newdf["deltaZ"] = np.concatenate(([0], newdf.z[1:len(buildingPoints)].values - newdf.z[0:len(buildingPoints)-1].values))

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
            nbins = int((math.ceil(buildingPoints.z.max() - buildingPoints.z.min()))/self.heightThreshold)
            # ax.hist(x=df.z, bins='auto', color='#0504aa', alpha=0.7, rwidth=0.85)
            ax.hist(x=buildingPoints.z, bins=nbins, color='#0504aa', alpha=0.7, rwidth=0.85)
            filenameImage = self.imagesPath + self.building.identifier[0] + "_HeightHistogram.png"
            plt.savefig(filenameImage)
            plt.close()

            fig = plt.figure()
            ax = fig.add_subplot()
            color = iter(plt.cm.rainbow(np.linspace(0, 1, len(filteredHeightGroups))))
            _ = ax.scatter(buildingPoints.x, buildingPoints.y, c="Gray", marker=".", alpha = 0.1)
            
            for k in range(len(filteredHeightGroups)):
                c = next(color)
                _ = ax.scatter(filteredHeightGroups[k].x, filteredHeightGroups[k].y, color=c, label=k, marker=".")

            ax.legend(loc="lower left")
            ax.set_aspect('equal', adjustable='box')
            filenameImage = self.imagesPath + self.building.identifier[0] + "_HeightGroups.png"
            plt.savefig(filenameImage)
            plt.close()

        return filteredHeightGroups

    def __gradient(self, point, currentGroup):
        """
        Given a point and a dataframe, obtains its closest neighbors and computes the gradient. It needs the nGradient attribute

        #### Inputs:
        - point: df containing x, y and z fields
        - currentGroup: dataframe containing all the points in the heightgroup

        #### Outputs:
        - grad: two-dimensoinal array corresponding to the gradient
        """
        
        byDistance = currentGroup.copy()
        byDistance["distance"] = np.sqrt((byDistance.x - point.x)**2 + (byDistance.y - point.y)**2)
        neighbors = byDistance.sort_values("distance").head(self.nGradient+1)[1:].reset_index(drop=True)

        grad = [0,0]
        for i in range(len(neighbors)):
            pt = neighbors.iloc[[i]]
            if(pt.x.values - point.x != 0):
                grad[0] = grad[0] + (pt.z.values - point.z)/(pt.x.values - point.x)
            if(pt.y.values - point.y != 0):
                grad[1] = grad[1] + (pt.z.values - point.z)/(pt.y.values - point.y)
        return grad 

    def __computeGradient(self, currentGroup):
        """
        Calculates the gradient of all points in a given dataframe

        #### Inputs:
        - currentGroup: dataframe containing all the points in the heightgroup

        #### Outputs:
        - currentGroup: the same dataframe, with an updated column, corresponding to the gradient of each point
        """
        for index,point in currentGroup.iterrows():
            currentGroup.loc[index,"gradientX"],currentGroup.loc[index,"gradientY"] = self.__gradient(point, currentGroup)
        currentGroup["absGradient"] = np.sqrt(currentGroup["gradientX"]**2 + currentGroup["gradientY"]**2)
        currentGroup["angleGradientDeg"] = np.arctan2(currentGroup.gradientY,currentGroup.gradientX)*360/(2*math.pi)
        currentGroup["angleGradientDeg"][currentGroup["angleGradientDeg"] < 0 ] = currentGroup["angleGradientDeg"] + 360

        return currentGroup

    @staticmethod
    def __distancePlane(point, planeParams):
        """
        Given a point, and the parameters of a plane, computes the distance of the point to the plane
        
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
        Given a dataframe of points (x and y coordinates), returns the plane density (it computes the plane polygon via convexHull)

        #### Inputs:
        - planePoints: dataframe containing x and y fields

        #### Outputs:
        - density: density (in points per square meter)
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
    
    @staticmethod
    def score(size, std, density):
        """
        Custom-definable function to decide how to weight size, std and density to assess how good a found plane is.
        This method is thought to be redefined outside this class to fit each user's case.

        #### Inputs:
        - size: how many points are found in the plane (i.e, how many there are within the distanceThreshold)
        - std: average standard deviation of all the inliers
        - density: density in points per square meter of the plane found

        #### Outputs:
        - score: a single value metric to decide which plane is best
        """
        return size*std*density**4
    
    def __samplePoints(self, pointsToIdentifydf):
        """
        Given a dataframe, it chooses 3 points for RANSAC regression.
        One point is chosen randomly but conditioned by gradient, the other two are chosen randomly but likeliness is based on proximity to first chosen point.

        #### Inputs:
        - pointsToIdentifydf: dataframe containing the points to identify (x, y and z fields are required)

        #### Outputs:
        - toFit: dataframe containing the 3 chosen points (with x, y, z fields)
        """
        pointsdf = pointsToIdentifydf.copy()
        
        angles = pointsdf.angleGradientDeg
        hist, bins = np.histogram(angles, bins=int(360/5))
        
        probabilities = []

        for j in range(len(bins)-1):
            for k in range(hist[j]):
                probabilities.append(hist[j])
        
        probabilities = np.power(probabilities, self.pdfExponent)
        probabilities = probabilities/sum(probabilities)

        cumulativeProbability = []
        for j in range(len(probabilities)):
            cumulativeProbability.append(sum(probabilities[0:j]))
        
        pointsdf["cumulativeProbability"] = cumulativeProbability

        # target 1: selected based on gradient
        target1 = np.random.random()
        while(target1 > pointsdf.cumulativeProbability.iloc[-1]):    target1 = np.random.random()
        rand1 = next(x for x, val in enumerate(pointsdf.cumulativeProbability) if val > target1)
        basePoint = pointsdf.loc[rand1]
        # targets 2 and 3: selected based on distance to target 1
        pointsdf["distance"] = np.sqrt((pointsdf.x - basePoint.x)**2 + (pointsdf.y - basePoint.y)**2)
        newpointsdf = pointsdf.copy()
        newpointsdf = newpointsdf[newpointsdf["distance"] != 0]
        probabilities = [1/x for x in newpointsdf.distance]
        
        probabilities = np.power(probabilities, self.pdfExponent) # This is done to further differences
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

        return toFit

    def __ransac_ordered(self, pointsToIdentifydf, minPoints, heightGroupDensity): #n = 3
        """
        Given a dataframe, it finds a plane by applying RANSAC (with a given number of iterations).
        Points are inliers if the distance is above the thresholdDistance
        Planes are valid if they have more than minPoints inliers

        #### Inputs:
        - pointsToIdentifydf: dataframe containing the points to identify (x, y and z fields are required)
        - minPoints: how many points a plane is needed to have for it to be valid
        - heightGroupDensity: point density of the currentHeightGroup, for density comparison

        #### Outputs:
        - bestPlane: array containing the parameters (a, b, c, d) corresponding to the best plane found
        - planePoints: all the points that are inliers to the found plane
        - notPlanePoints: all the points that do not belong to the selected plane
        """
        
        pointsdf = pointsToIdentifydf.copy()
        # Returns the best point only for that data
        
        bestPlane = []
        bestStd = np.inf
        bestScore = 0
        bestPointsInPlane = []
        

        if(minPoints < len(pointsdf)):

            for i in range(self.ransacIterations):
                toFit = self.__samplePoints(pointsdf)

                # Fit a plane with these n random points 
                X = toFit[["x", "y"]].values
                y = toFit[["z"]].values
                
                model = linear_model.LinearRegression().fit(X, y)
                plane = model.coef_[0][0], model.coef_[0][1], -1, model.intercept_[0]
                
                pointsInPlane = []
                # Check for all points in the dataset that can belong to this plane (distance below threshold)
                for j in range(len(pointsdf)):
                    point = pointsdf[["x", "y", "z"]].iloc[j]
                    pointsdf.loc[j, "dist"] = PlaneDetector.__distancePlane(point, plane)
                    if(pointsdf.loc[j, "dist"] < self.distanceThreshold):
                        pointsInPlane.append(j)

                planePoints = pointsdf[["x", "y", "z", "dist"]].iloc[pointsInPlane]

                if(len(planePoints) > minPoints):
                    size = len(pointsInPlane)
                    std = np.sum(planePoints.dist)**2 / (len(planePoints) - 1)
                    density = PlaneDetector.__planeDensity(planePoints)
                    if(density > self.densityMultiplier*heightGroupDensity):
                        # If a good plane, check for score to decide if it is the best plane
                        newScore = PlaneDetector.score(size, std, density,)
                        if(newScore > bestScore):
                            bestPointsInPlane = pointsInPlane
                            bestPlane = plane
                            bestScore = newScore

        planePoints = pointsdf.copy().loc[pointsdf.index.isin(bestPointsInPlane)].reset_index(drop=True)
        planePoints = planePoints[["x", "y", "z", "dist"]]

        notPlanePoints = pointsdf.copy().loc[~pointsdf.index.isin(bestPointsInPlane)].sort_values(by=["angleGradientDeg"]).reset_index(drop=True)
        notPlanePoints = notPlanePoints[["x", "y", "z","angleGradientDeg"]]

        return bestPlane, planePoints, notPlanePoints

    def detectPlanes(self):
        """
        Applies RANSAC to find the planes corresponding to the building rooftops

        #### Inputs:
        - None

        #### Outputs:
        - None

        #### Exports:
        - For each heightgroup, a .csv file containing all the planes (parameters a,b,c,d) found in that heightgroup
        - For each plane found, a .csv file containing all the points (x,y,z) that belong to that plane
        - if generateFigures: it exports images of the step by step solution, as well as 3d scatters of each level once it is finished
        """
        create_output_folder(self.planePointsPath, deleteFolder=True)
        create_output_folder(self.planeListPath, deleteFolder=True)
        create_output_folder(self.imagesPath, deleteFolder=True)

        buildingPoints = pd.read_csv((self.segmented_LiDAR + "/" + self.building.identifier[0] + ".csv"), header=None)
        buildingPoints = buildingPoints.rename(columns={0: "x", 1: "y", 2:"z"})

        heightGroups = self.__heightSplit(buildingPoints)
        print("Split into", len(heightGroups), "height groups")

        for j in range(len(heightGroups)):
            heightGroupDensity = PlaneDetector.__planeDensity(heightGroups[j])

            print("Group:", j)
            currentGroup = heightGroups[j].copy().reset_index(drop=True)

            # Compute gradient
            currentGroup = self.__computeGradient(currentGroup)
            currentGroup = currentGroup.sort_values(by=["angleGradientDeg"])

            planeList = []
            planePointsList = []
            notPlanePoints = currentGroup.copy()
            print('\t Starting with', len(notPlanePoints), 'points')
            
            looping = True
            previousEmpty = False

            while(looping): #Need to redefine the stopping criteria         # looping = len(notPlanePoints) > self.stoppingPercentage*len(currentGroup)
                
                bestPlane, planePoints, notPlanePoints = self.__ransac_ordered(notPlanePoints, min(self.minGlobalPercentage*len(currentGroup), self.minPartialPercentage*len(notPlanePoints)), heightGroupDensity)
                if(len(bestPlane) > 0):
                    planeList.append(bestPlane)
                    planePointsList.append(planePoints[["x","y","z"]])
                    print('\t\t', len(planePoints), 'points identified,', len(notPlanePoints), 'points remaining')
                    print("\t\t Found plane. Percentatge remaining:", len(notPlanePoints)/len(heightGroups[j]))
                   
                    for k in range(len(planePointsList)):
                        filenamePoints =  self.planePointsPath + self.building.identifier[0] + "_Div_" + str(j) + "_plane " + str(k) + ".csv"
                        pd.DataFrame(planePointsList[k]).to_csv(filenamePoints, header=False, index=False) 
                    filenameList = self.planeListPath + self.building.identifier[0] + '_PlaneList_' + str(j)  + '.csv'
                    pd.DataFrame(planeList).to_csv(filenameList, header=False, index=False)   

                    # Generate image as planes are found
                    if(self.generateFigures):
                        fig = plt.figure()
                        ax = fig.add_subplot()
                        color = iter(plt.cm.rainbow(np.linspace(0, 1, len(planePointsList))))
                        _ = ax.scatter(buildingPoints.x, buildingPoints.y, c="Gray", marker=".", alpha = 0.1)

                        for k in range(len(planePointsList)):
                            filenamePoints =  self.planePointsPath + self.building.identifier[0] + "_Div_" + str(j) + "_plane " + str(k) + ".csv"
                            pd.DataFrame(planePointsList[k]).to_csv(filenamePoints, header=False, index=False) 
                            c = next(color)
                            try:
                                _ = ax.scatter(planePointsList[k].x, planePointsList[k].y, color=c, label=k, marker=".")
                            except:
                                pass
                        ax.legend(loc="lower left")
                        ax.set_aspect('equal', adjustable='box')
                        filenameImage = self.imagesPath + self.building.identifier[0] + '_PlaneList_' + str(j) + "_Step_" + str(len(planePointsList)) + ".png"
                        plt.savefig(filenameImage)
                        plt.close()

                    else:
                        for k in range(len(planePointsList)):
                            filename =  self.planePointsPath  + self.building.identifier[0] + "_Div_" + str(j) + "_plane " + str(k) + ".csv"
                            pd.DataFrame(planePointsList[k]).to_csv(filename, header=False, index=False)

                minPoints = np.inf
                for i in range(len(planePointsList)):
                    minPoints = min(minPoints, len(planePointsList[i]))
                
                looping = (not previousEmpty or (len(bestPlane) > 0)) and (len(notPlanePoints) > 3)
                previousEmpty = len(bestPlane) == 0

                if self.stoppingPercentage != None:
                    looping = (looping or (len(notPlanePoints) > self.stoppingPercentage*len(currentGroup)))and (len(notPlanePoints) > 3)

            print("\t Run out of iterations.", len(notPlanePoints), 'points left')   

            # Generate 3d scatter
            if(self.generateFigures):

                fig = plt.figure()
                ax = fig.add_subplot(projection='3d')
                color = iter(plt.cm.rainbow(np.linspace(0, 1, len(planePointsList))))
                _ = ax.scatter(buildingPoints.x, buildingPoints.y, buildingPoints.z, c="Gray", marker=".", alpha = 0.1)
                
                for k in range(len(planePointsList)):
                    filenamePoints =  self.planePointsPath + self.building.identifier[0] + "_Div_" + str(j) + "_plane " + str(k) + ".csv"
                    pd.DataFrame(planePointsList[k]).to_csv(filenamePoints, header=False, index=False) 
                    try:
                        _ = ax.scatter(planePointsList[k].x, planePointsList[k].y, planePointsList[k].z, c=next(color), label=k, marker=".")
                    except:
                        pass
                    
                ax.legend(loc="lower left")
                ax.set_aspect('equal', adjustable='box')
                filenameImage = self.imagesPath + self.building.identifier[0] + '_Plane3D_' + str(j) + ".png"
                plt.savefig(filenameImage)
                plt.close()
                
            filenameList = self.planeListPath + self.building.identifier[0] + '_PlaneList_' + str(j)  + '.csv'
            pd.DataFrame(planeList).to_csv(filenameList, header=False, index=False)     