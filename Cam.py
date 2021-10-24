import subprocess
import os
import shutil
import time
oPath = "./"
nPath = "./picture"

def movefile():
    # oPath Read
    file_list = os.listdir(oPath)

    # make array for move files
    mov_files = []

    # Files List Create - JPG
    for file in file_list:
        if(file.endswith(".JPG")):
            mov_files.append(file)

    # Move Files
    for movfi in mov_files:
        img_open_command = "sudo eog " + movfi
        subprocess.call(img_open_command, shell=True)
        shutil.move(oPath + '//' + movfi, nPath + '//' + movfi)
"""
def takePicture():
    print("here")
    subprocess.call("ptpcam -c", shell=True)

    # find the last picture taken. Modify to parse for date or other
    time.sleep(3)
    files = []
    listFiles = subprocess.Popen(["ptpcam", "-L"], stdout=subprocess.PIPE)

    for line in listFiles.stdout.readlines():
        files.append(line.rstrip())

    lastLine = str(files[len(files) - 2],'utf-8').split(" ")
    lastPicture = lastLine[0][:-1]

    # download the picture
    ptpcommand = "ptpcam --get-file=" + lastPicture
    subprocess.call(ptpcommand, shell=True)

    # move image
    movefile()
"""
def takePicture():
    subprocess.call("gphoto2 --capture-image-and-download", shell=True)

    # move image
    movefile()


#takePicture()
