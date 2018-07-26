import os
import time, stat
import sys
import shutil
import array
import multiprocessing
import time
from threading import Lock

dropboxFolder = "~/Dropbox"

def get_scenes(setText, dataFile):
   totalScenes = 0
   processedScenes = 0
   d = './allData'

   folderList = filter(os.path.isdir, [os.path.join(d,f) for f in os.listdir(d)])
   for item in folderList:
      setName = os.path.basename(os.path.normpath(item))
      folderList2 = filter(os.path.isdir, [os.path.join(item,f) for f in os.listdir(item)])
      if not setName == setText:
          continue
      # Get object id
      for item2 in folderList2:

          # Get object id
          folderListScenes = filter(os.path.isdir, [os.path.join(item2,f) for f in os.listdir(item2)])
          for itemScene in folderListScenes:
              fileName = itemScene + "/" + dataFile
              mtime = os.stat(fileName)[stat.ST_MTIME]
              fileName2 = itemScene + "/stable.data"
              mtime2 = os.stat(fileName2)[stat.ST_MTIME]
              if (mtime > mtime2 + 1200):
                  processedScenes = processedScenes + 1
              totalScenes = totalScenes + 1
   print (str(processedScenes) + "/" + str(totalScenes) + " scenes have been processed in " + setText)
   return


get_scenes(sys.argv[1], sys.argv[2])
