import os
import sys
import array
 
classIds = []
classCount = 24;
 
def count_grasps(setText, fileName):
   d = './allData'
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
              floatSize = os.path.getsize(filePath)/4
              graspData = array.array('f')
              f = open(filePath, 'rb')
              graspData.fromfile(f, floatSize)
              f.close()
              # IGNORE SETS HAVING LESS THAN 10 GRASPS
              if (floatSize/285) < 10:
                  continue;
              for itr in range(0, (floatSize/285)):
                  total = total + 1
                 # if graspData[itr * 285] > -0.5:
                  graspTotal[itr] = graspTotal[itr] + 1
                  classTotal[classId] = classTotal[classId] + 1
                  if itr==0 and graspData[itr * 285] > -0.5:
                     firstClassTotal[classId] = firstClassTotal[classId] + 1
                  if graspData[itr * 285] >= 0.5:
                      successful = successful + 1
                      graspSuccess[itr] = graspSuccess[itr] + 1
                      classSuccess[classId] = classSuccess[classId] + 1
                      if itr==0:
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
 
   return returnArr
 
classFile = open("classIds.txt")
for val in classFile.read().split():
    classIds.append(int(val))
classFile.close()
count_grasps(sys.argv[1], sys.argv[2])
