import os
import sys
import array
import numpy as np
 
classIds = []
classCount = 24;

def count_grasps(setText, fileName, orderFile = None):
   
   d = './allData'
   dOrg = './allDataOrg'
   scenes = 0
   classSuccess = []
   classTotal = []
   graspSuccess = []
   graspTotal = []
   firstClassSuccess = []
   firstClassTotal = []
    
   # Initialize arrays
   for i in range(0,100):
      if i<classCount:
         classSuccess.append(0)
         classTotal.append(0)
         firstClassSuccess.append(0)
         firstClassTotal.append(0)
      graspSuccess.append(0)
      graspTotal.append(0)
 
   successful = 0
   collide = 0
   total = 0
   topSuccess = 0
   returnArr = []
   folderList = filter(os.path.isdir, [os.path.join(d,f) for f in os.listdir(d)])

   changes = [[0, 0, 0], [0, 0, 0], [0,0,0]]

   # Keep track of changes.
   maxScene = 0
   maxSceneCount = -1
   maxScenePath = []

   for item in folderList:
      setName = os.path.basename(os.path.normpath(item))
      folderList2 = filter(os.path.isdir, [os.path.join(item,f) for f in os.listdir(item)])
      if not setName == setText:
          continue
      # Get object id
      for item2 in folderList2:
 
          # Get object id
          objectId = os.path.basename(os.path.normpath(item2))
          objectId = int(objectId) - 1
          classId = classIds[objectId] - 1
 
          folderListScenes = filter(os.path.isdir, [os.path.join(item2,f) for f in os.listdir(item2)])
          for itemScene in folderListScenes:
              filePath = itemScene + "/" +  fileName
              scenes = scenes + 1
              sceneId = os.path.basename(os.path.normpath(itemScene))

              # Read grasp data.
              floatSize = os.path.getsize(filePath)/4
              graspData = array.array('f')
              f = open(filePath, 'rb')
              graspData.fromfile(f, floatSize)
              f.close()

              # Read original file here.
              orgGraspData = array.array('f')
              orgFilePath = dOrg + '/' + setName + '/' + str(objectId+1) + '/' + sceneId + "/data.bin"
              f = open(orgFilePath, 'rb')
              orgGraspData.fromfile(f, floatSize)
              f.close()

              # Get desired order
              topRank = 0
              if not orderFile == None:
                  orderFilePath = './orderFiles/' + setName + '/' + str(objectId + 1) + '/' + os.path.basename(os.path.normpath(itemScene)) + '/' + orderFile
                  order = np.load(orderFilePath)
                  topRank = order[0] - 1
              
              # IGNORE SETS HAVING LESS THAN 10 GRASPS
              if (floatSize/285) < 10:
                  continue;
              for itr in range(0, (floatSize/285)):
                 total = total + 1
                 
                 # Record-keeping about the grasps in general.
     #            print(str(int(-orgGraspData[itr * 285] + 1)) + ':' + str(int(-graspData[itr * 285] + 1)))
                 changes[int(-orgGraspData[itr * 285] + 1)][int(-graspData[itr * 285] + 1)] = \
                     changes[int(-orgGraspData[itr * 285] + 1)][int(-graspData[itr * 285] + 1)] + 1

                 # if graspData[itr * 285] > -0.5:
                 graspTotal[itr] = graspTotal[itr] + 1
                 classTotal[classId] = classTotal[classId] + 1
                 if itr==topRank:
                     firstClassTotal[classId] = firstClassTotal[classId] + 1
                 if graspData[itr * 285] >= 0.5:
                      successful = successful + 1
                      graspSuccess[itr] = graspSuccess[itr] + 1
                      classSuccess[classId] = classSuccess[classId] + 1
                      if itr==topRank:
                         topSuccess = topSuccess + 1
                         firstClassSuccess[classId] = firstClassSuccess[classId] + 1
                 elif graspData[itr*285] < -0.5:
                      collide = collide + 1
   returnArr.append(scenes)
   returnArr.append(successful)
   returnArr.append(total)
   returnArr.append(collide)
 
   print "CLASS PERFORMANCES"
   for i in range(0, classCount):
      if classTotal[i] > 0:
         print str(float(classSuccess[i])/classTotal[i])
      else:
         print "0"
 
   print "FIRST CLASS PERFORMANCES"
   for i in range(0, classCount):
      if firstClassTotal[i] > 0:
         print str(float(firstClassSuccess[i])/firstClassTotal[i])
      else:
         print "0"
 
   print "GRASP PER POSITION"
   for i in range(0, 100):
      if graspTotal[i] > 0:
         print str(float(graspSuccess[i])/graspTotal[i])
      else:
         print "0"
   totalFirstGrasps = 0
   for itr in range(0, classCount):
      totalFirstGrasps = totalFirstGrasps + firstClassTotal[itr]
   print 'Successful: ' + str(successful) + ' Unsuccessful: ' + str(total - (collide + successful)) + ' Colliding: ' + str(collide)
   print "OVERALL SUCCESS:" + str(float(successful)/(total - collide))
   print "TOP SUCCESS:" + str(float(topSuccess)/totalFirstGrasps)
   print topSuccess
   print totalFirstGrasps
   print(changes) 

   return returnArr
 
classFile = open("classIds.txt")
for val in classFile.read().split():
    classIds.append(int(val))
classFile.close()
if len(sys.argv) < 4:
    orderFile = None
else:
    orderFile = sys.argv[3]
count_grasps(sys.argv[1], sys.argv[2], orderFile)
