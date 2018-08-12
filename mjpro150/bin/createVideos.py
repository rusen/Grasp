import os
import sys
import time
import shutil
d = './tmp/data'
fileName2 = "data_2epoch_0.01_last3"

folderList = filter(os.path.isdir, [os.path.join(d,f) for f in os.listdir(d)])
for item in folderList:
	try:
		setName = os.path.basename(os.path.normpath(item))
		# Copy model files
		d2 = item+"/model"
		print(d2)
		for modelItem in os.listdir(d2):
			print(modelItem)
			shutil.copy2(item + "/model/" + modelItem, "../model/BHAM") 
		command1 = "./playlog ../model/BHAM/" + setName + "_Test.xml " + setName + " re-sim"
		os.system(command1)
		command12 = "ffmpeg -f rawvideo -pixel_format rgb24 -video_size 512x768 -framerate 30 -i " + item + "/video/data/re-sim/0/rgb.out -vf 'vflip' " + item + "/video/data/re-sim/0/video.mp4"
		video1 = item + "/video/data/re-sim/0/video.mp4"
		os.system(command12)
		os.remove(item + "/video/data/re-sim/0/rgb.out")
		command2 = "./playlog ../model/BHAM/" + setName + "_Test.xml " + setName + " re-sim" + " " + fileName2
		os.system(command2)
		command22 = "ffmpeg -f rawvideo -pixel_format rgb24 -video_size 512x768 -framerate 30 -i " + item + "/video/" + fileName2 + "/re-sim/0/rgb.out -vf 'vflip' " + item + "/video/" + fileName2 + "/re-sim/0/video.mp4"
		os.system(command22)
		os.remove(item + "/video/" + fileName2 + "/re-sim/0/rgb.out")
		video2 = item + "/video/" + fileName2 + "/re-sim/0/video.mp4"
		command3 = "ffmpeg -i " + video1 + " -i " + video2 + " -filter_complex '[0:v]pad=iw*2:ih[int];[int][1:v]overlay=W/2:0[vid]' -map [vid] -c:v libx264 -crf 23 -preset veryfast ./videos/" + setName + ".mp4" 
		os.system(command3)
	except:
		print(item + " has not been processed")
