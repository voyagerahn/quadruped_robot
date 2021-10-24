import subprocess
import os
import shutil
oPath = './'
nPath = '../picture'

def movefile():
    #oPath 파일 읽기
    file_list = os.listdir(oPath)

    #이동할 파일들을 담을 배열 객체
    mov_files = []
    
    # Files List Create - TEXT 파일을 제외한 파일들이 이동대상 파일
    for file in file_list:
        if(file.endswith(".JPG")):
            mov_files.append(file)

    # Move Files
    for movfi in mov_files:
        shutil.move(oPath + '//' + movfi, nPath + '//' + movfi)

def takepicture():
    subprocess.call("gphoto2 --capture-image-and-download", shell=True)
    movefile()


takepicture()


import subprocess
import os
import shutil
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
        shutil.move(oPat + '//' + movfi, nPath + '//' movfi)

## example of taking a picture
def takePicture():
    subprocess.call("ptpcam -c", shell=True)

    # example of grabbing device info and using it in your python program.
    ptpinfo = subprocess.Popen(["ptpcam", "--info"], stdout=subprocess.PIPE)

    # although this simply prints to stdout, you can parse

    # the response for your program
    for line in ptpinfo.stdout.readlines():
        print(line.rstrip())

    # find the last picture taken. Modify to parse for date or other
    files = []
    listFiles = subprocess.Popen(["ptpcam", "-L"], stdout=subprocess.PIPE)

    for line in listFiles.stdout.readlines():
        files.append(line.rstrip())

    lastLine = files[len(files) - 2].split(" ")
    lastPicture = lastLine[0][:-1]

    print("The handle for the last picture taken is " + lastPicture)

    # download the picture
    ptpcommand = "ptpcam --get-file=" + lastPicture
    subprocess.call(ptpcommand, shell=True)

    # move image
    movefile()

takePicture()
