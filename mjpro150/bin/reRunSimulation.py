import os
import sys
import shutil
import array
import multiprocessing
import time
from threading import Lock

folders = {}
dataFile = "data.bin"
dropboxFolder = "~/Dropbox"


def get_scenes(setText):
   d = './allData'
   counter = 0

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

          folderListScenes = filter(os.path.isdir, [os.path.join(item2,f) for f in os.listdir(item2)])
          for itemScene in folderListScenes:
              folders[counter] = itemScene
              out = open(itemScene + "/objectId.txt", "w")
              out.write(objectId)
              out.close()
              counter = counter + 1
   return


def f(x):
    itemScene = folders[x]
    newName = ''
    for fil in os.listdir(itemScene):
        if fil.endswith(".pcd"):
            newName = os.path.splitext(fil)[0]
            print('Processing ' + newName)
    newFolder = './tmp/data/' + newName
    shutil.move(itemScene, newFolder)
  #  command = "./basicGrasp " + newName + " " + dataFile + " ../model/BHAM " + dropboxFolder + " visualOff 0 0 > /dev/null"
    command = "./basicGrasp " + newName + " " + dataFile + " ../model/BHAM " + dropboxFolder + " visualOff 0 0"
#    print(command)
    os.system(command)

    # DO SOME PROCESS
    shutil.move(newFolder, itemScene)

numberOfThreads = int(sys.argv[2])
get_scenes(sys.argv[1])
numberOfScenes = len(folders)
dataFile = sys.argv[3]
dropboxFolder = sys.argv[4]
pool = multiprocessing.Pool(int(sys.argv[2]))
pool.map(f, range(0, len(folders)))
