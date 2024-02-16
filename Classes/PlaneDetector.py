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

class PlaneDetector:
    def __init__(self, building, segmented_LiDAR, savePaths, generateFigures=True, 
                 heightThreshold=1, nGradient=5, ransacIterations=20, distanceThreshold=0.2, 
                 minGlobalPercentage=0.1, minPartialPercentage=0.4, stoppingPercentage=0.1, pdfExponent=2, 
                 deleteFirst=True, interactive=False, readFromFile=False, fileSplit=None):
        
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
        self.readFromFile = readFromFile
        self.interactive = interactive
        self.fileSplit = fileSplit

    def __heightSplit(self, df, filter=True):
        """
        Given a dataframe, sorts points by height and splits it in groups separated by a threshold height difference
        If filter = True, deletes first group (ground points) and groups made of 3 points or less 
        """

        # Generate figures
        if(self.generateFigures):
            fig = plt.figure()
            ax = fig.add_subplot()
            nbins = int(math.ceil(df.z.max() - df.z.min()))
            # ax.hist(x=df.z, bins='auto', color='#0504aa', alpha=0.7, rwidth=0.85)
            ax.hist(x=df.z, bins=nbins, color='#0504aa', alpha=0.7, rwidth=0.85)
            filenameImage = self.imagesPath + self.building.identifier[0] + "_HeightHistogram.png"
            plt.savefig(filenameImage)
            plt.close()

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

    def __ransac_ordered(self, pointsdf, minPoints): #n = 3
        """
        Given a dataframe, find a plane by applying RANSAC (with a given number of iterations).
        Points are inliers if the distance is above the thresholdDistance
        Planes are valid if they have more than minPoints inliers
        """
        
        # Returns the best point only for that data
        
        bestPlane = []
        bestStd = np.inf
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
                
                target1, target2, target3 = np.random.random(), np.random.random(), np.random.random()
                while(target1 > pointsdf.cumulativeProbability.iloc[-1]):    target1 = np.random.random()
                while(target2 > pointsdf.cumulativeProbability.iloc[-1]):    target2 = np.random.random()
                while(target3 > pointsdf.cumulativeProbability.iloc[-1]):    target3 = np.random.random()

                # print("\t Targets:", target1, target2, target3)

                rand1 = next(x for x, val in enumerate(pointsdf.cumulativeProbability) if val > target1)
                rand2 = next(x for x, val in enumerate(pointsdf.cumulativeProbability) if val > target2)
                rand3 = next(x for x, val in enumerate(pointsdf.cumulativeProbability) if val > target3)
                
                while rand1 == rand2 or rand1 == rand3 or rand2 == rand3:
                    target1, target2, target3 = np.random.random(), np.random.random(), np.random.random()
                    while(target1 > pointsdf.cumulativeProbability.iloc[-1]):    target1 = np.random.random()
                    while(target2 > pointsdf.cumulativeProbability.iloc[-1]):    target2 = np.random.random()
                    while(target3 > pointsdf.cumulativeProbability.iloc[-1]):    target3 = np.random.random()
                    rand1 = next(x for x, val in enumerate(pointsdf.cumulativeProbability) if val > target1)
                    rand2 = next(x for x, val in enumerate(pointsdf.cumulativeProbability) if val > target2)
                    rand3 = next(x for x, val in enumerate(pointsdf.cumulativeProbability) if val > target3)
                
                # What if we chose the most frequent angle? 

                # lowerbound = max(0, (hist.argmax())*5 )
                # upperbound = min(360, (hist.argmax()+1)*5 )
                
                # anglesFiltered = pointsdf[(pointsdf.angleGradientDeg > lowerbound) & (pointsdf.angleGradientDeg < upperbound)].angleGradientDeg
                # if(len(anglesFiltered) < 3):
                #     anglesFiltered = pointsdf[(pointsdf.angleGradientDeg > lowerbound)].angleGradientDeg
                # if(len(anglesFiltered) < 3):
                #     anglesFiltered = pointsdf[(pointsdf.angleGradientDeg < upperbound)].angleGradientDeg
                # if(len(anglesFiltered) < 3):
                #     anglesFiltered = pointsdf.angleGradientDeg

                

                # rand1, rand2, rand3 = random.choice(anglesFiltered.index), random.choice(anglesFiltered.index), random.choice(anglesFiltered.index)
                # while rand1 == rand2 or rand1 == rand3 or rand2 == rand3:
                #     rand1, rand2, rand3 = random.choice(anglesFiltered.index), random.choice(anglesFiltered.index), random.choice(anglesFiltered.index)

                # targetAngle = anglesFiltered.mean()
                # rand1 = np.array(pointsdf[pointsdf.angleGradientDeg.gt(targetAngle)].index)[0]
                # rand2 = rand1 - 1
                # if(rand2 < 0):  rand2 = rand1+2
                # rand3 = rand1 + 1
                # if(rand3 >= len(pointsdf)): rand3 = rand1-2

                # print("\t Points:", rand1, rand2, rand3)

                # # Select 3 consecutive random points
                # rand1 = random.randint(0, len(pointsdf)-3)
                # rand2 = rand1 + 1 
                # rand3 = rand1 + 2

                # # What if it was entirely random?
                # rand1 = random.randint(0, len(pointsdf)-1)
                # rand2 = random.randint(0, len(pointsdf)-1)
                # rand3 = random.randint(0, len(pointsdf)-1)

                # while rand1 == rand2 or rand1 == rand3 or rand2 == rand3:
                #     rand1 = random.randint(0, len(pointsdf)-1)
                #     rand2 = random.randint(0, len(pointsdf)-1)
                #     rand3 = random.randint(0, len(pointsdf)-1)

                toFit = pointsdf.iloc[[rand1, rand2, rand3]]
                # print(toFit)

                # Fit a plane with these n random points 
                X = toFit[["x", "y"]].values
                y = toFit[["z"]].values
                
                for k in range(2):
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
                    X = planePoints[["x", "y"]].values
                    y = planePoints[["z"]].values

                # Check the size of the pointInPlane df (and compare it to the min percentage required)
                if(len(planePoints) > minPoints):
                    # print("\t Fulfills minimum points criteria")
                    newStd = np.sum(planePoints.dist)**2 / (len(planePoints) - 1)
                    # If a good plane, check for std to decide if it is the best plane
                    if(newStd < bestStd):
                        bestPointsInPlane = pointsInPlane
                        bestPlane = plane
                        bestStd = newStd
                        # print("\t", i, ":", bestPlane, ". Ratio:", len(planePoints)/len(pointsdf))
                # else:
                    # print("\t Does not fulfill minimum points criteria")
        
        planePoints = pointsdf[["x", "y", "z", "dist"]].iloc[bestPointsInPlane].reset_index(drop=True)
        notPlanePoints = pointsdf[["x", "y", "z","angleGradientDeg"]].copy().drop(bestPointsInPlane, axis = 0).sort_values(by=["angleGradientDeg"]).reset_index(drop=True)
        # print("\t", bestPlane)
        # Return plane parameters and points inside roof
        return bestPlane, planePoints, notPlanePoints
    
    

    def __interactiveSplit(self, df):
        points = []
        choords = []
        polygonList = []

        xmin = df.x.min()
        ymin = df.y.min()
        offset = max((df.x.max() - df.x.min()), (df.y.max() - df.y.min()))/2 * 2

        if(not self.readFromFile):

            def click_event(event, x, y, flags, params): 

                # checking for left mouse clicks 
                if event == cv2.EVENT_LBUTTONDOWN:                 
                    points.append((x, y))

                    if len(points) > 1:
                        cv2.line(img, points[-2], points[-1], (255, 0, 0), 4)

                    choordsX = self.building.x.values[0]-offset + x/1000*2*offset
                    choordsY = self.building.y.values[0]+offset - y/1000*2*offset
                    choords.append((choordsX, choordsY))

                    cv2.circle(img, (x,y), radius=10, color=(255, 0, 0), thickness=-10) 
                    cv2.imshow(self.building.identifier.values[0], img) 

                # checking for right mouse clicks      
                if event==cv2.EVENT_RBUTTONDOWN: 
            
                    if len(points) > 1:
                        cv2.line(img, points[0], points[-1], (255, 0, 0), 4)
                        polygonList.append(Polygon(choords))
                        # print("New Polygon", Polygon(choords))
                        points.clear()
                        choords.clear()
                    cv2.imshow(self.building.identifier.values[0], img) 

            # imgPath = r"C:\Users\jaasb\INVESTIGO\BEE Group\eplanet shared\Programa Final\Samples and Test\Interactive App test\MapResults\eP-EAZK-001_ZoomIn.png"
            imgPath = self.segmented_LiDAR + "/" + self.building.identifier[0] + "_ZoomIn.png"
            img = cv2.imread(imgPath, 1) 
            cv2.imshow(self.building.identifier.values[0], img) 

            arrayx = ((df.x.values - self.building.x.values[0] + offset)*1000/(2*offset)).astype(int)
            arrayy = 1000 - ((df.y.values - self.building.y.values[0] + offset)*1000/(2*offset)).astype(int)
            arrayz = (255*(df.z.values-df.z.min())/(df.z.max()-df.z.min())).astype(int)

            scatter = cv2.imread(imgPath, 1) 
            for i in range(len(arrayx)):
                color = cv2.applyColorMap(np.array([[arrayz[i]]], dtype=np.uint8), cv2.COLORMAP_VIRIDIS)[0][0]
                cv2.circle(scatter, tuple([arrayx[i],arrayy[i]]), radius=5, color=(int(color[0]),int(color[1]),int(color[2])), thickness=-2) 

            alpha = 0
            img = cv2.addWeighted(img, alpha, scatter, 1-alpha, 0)

            cv2.imshow(self.building.identifier.values[0], img) 
            cv2.setMouseCallback(self.building.identifier.values[0], click_event) 
            cv2.waitKey(0) 
            cv2.destroyAllWindows() 

            filename = self.imagesPath + self.building.identifier[0] + '_Polygons' + '.txt'
            file = open(filename,'w')
            for polygon in polygonList:
                file.write(str(polygon)+"\n")
            file.close()

            print(len(polygonList), " areas drawn")

            def points_inside_polygon(df, polygon):
                def point_inside_polygon(row):
                    point = Point(row.x, row.y)
                    return point.within(polygon)
                return df[df.apply(point_inside_polygon, axis=1)]

            dividedGroups = []
            for i in range(len(polygonList)):
                subset = points_inside_polygon(df, polygonList[i])
                subset = self.__heightSplit(subset)
                dividedGroups.extend(subset)
                # dividedGroups.append(subset)
            return dividedGroups

        else:
            filename = self.fileSplit
            outlines_df = pd.read_csv(filename, sep=";", header=None)
            polygonList = wkt.loads(outlines_df[0])

            def points_inside_polygon(df, polygon):
                def point_inside_polygon(row):
                    point = Point(row.x, row.y)
                    return point.within(polygon)
                return df[df.apply(point_inside_polygon, axis=1)]

            dividedGroups = []
            for i in range(len(polygonList)):
                subset = points_inside_polygon(df, polygonList[i])
                subset = self.__heightSplit(subset)
                dividedGroups.extend(subset)
                # dividedGroups.append(subset)
            return dividedGroups

    def detectPlanes(self):
        create_output_folder(self.planePointsPath, deleteFolder=True)
        create_output_folder(self.planeListPath, deleteFolder=True)
        create_output_folder(self.imagesPath, deleteFolder=True)

        df = pd.read_csv((self.segmented_LiDAR + "/" + self.building.identifier[0] + ".csv"), header=None)
        df = df.rename(columns={0: "x", 1: "y", 2:"z"})

        if(self.interactive):
            heightGroups = self.__heightSplit(df)
        else:
            heightGroups = self.__interactiveSplit(df)

        for j in range(len(heightGroups)):
            print("Group:", j)
            self.currentGroup = heightGroups[j].copy().reset_index(drop=True)

            # Compute gradient
            self.currentGroup = self.__computeGradient()
            self.currentGroup = self.currentGroup.sort_values(by=["angleGradientDeg"])

            self.planeList = []
            self.planePointsList = []
            notPlanePoints = self.currentGroup.copy()

            while(len(notPlanePoints) > self.stoppingPercentage*len(self.currentGroup)): #Need to redefine the stopping criteria
                # print("Another iteration")
                bestPlane, planePoints, notPlanePoints = self.__ransac_ordered(notPlanePoints, min(self.minGlobalPercentage*len(self.currentGroup), self.minPartialPercentage*len(notPlanePoints)))
                if(len(bestPlane) > 0):
                    self.planeList.append(bestPlane)
                    self.planePointsList.append(planePoints[["x","y","z"]])
                    print("\t Found plane. Percentatge remaining:", len(notPlanePoints)/len(heightGroups[j]))

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
                filenameImage = self.imagesPath + self.building.identifier[0] + '_PlaneList_' + str(j) + ".png"
                plt.savefig(filenameImage)
                plt.close()

            else:
                for k in range(len(self.planePointsList)):
                    filename =  self.planePointsPath  + self.building.identifier[0] + "_Div_" + str(j) + "_plane " + str(k) + ".csv"
                    pd.DataFrame(self.planePointsList[k]).to_csv(filename, header=False, index=False)

            filenameList = self.planeListPath + self.building.identifier[0] + '_PlaneList_' + str(j)  + '.csv'
            pd.DataFrame(self.planeList).to_csv(filenameList, header=False, index=False)     