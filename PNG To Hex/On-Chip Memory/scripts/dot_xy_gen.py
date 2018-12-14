from PIL import Image
from collections import Counter
from scipy.spatial import KDTree
import numpy as np

from skimage.io import imread
# filename = raw_input("What's the image name? ")
outFile = open("../script_out/"+ 'dots.sv', 'w')
img = imread("../sprite_converted/" +"pacbg_256x256.png");
n_rows, n_cols, n_chan = img.shape;
def BFS_count(x, y, img):
    count = 0
    if(x >= n_cols-1 or y >= n_rows-1 or x < 0 or y < 0):
        return 0
    if(img[x][y][0] == 250):
        count+=1

        # print("hi", img[x][y])
        img[x][y] = [0, 0, 0]
    if(isPink(x-1,y,img)):
        count+=BFS_count(x-1,y,img)

    if(isPink(x,y+1,img)):
        count+=BFS_count(x,y+1,img)


    if(isPink(x+1,y,img)):
        count+=BFS_count(x+1,y,img)

    if(isPink(x,y-1,img)):
        count+=BFS_count(x,y-1,img)
    return count
    # else:
    #     return BFS_count(x-1,y,img) + BFS_count(x,y+1,img) + BFS_count(x+1,y,img) + BFS_count(x,y-1,img) + count
def isPink(x,y,img):
    if(img[x][y][0] == 250):
        return True;

dotCount = 0
for x in range(1,n_cols):
    #j is row index
    for y in range(1,n_rows):
        count = BFS_count(x,y,img)
        if(count >=2 and count < 7):
            dotCount+=1
            outFile.write("Dot dot"+str(dotCount)+" #(parameter x="+str(y)+", parameter y="+str(x)+")(.Clk(CLK), .Reset(Reset_h),.DrawX(DrawX), .DrawY(DrawY), .pac_X_Pos(pac_X_Pos), .pac_Y_Pos(pac_Y_Pos), .isEaten(isEaten"+str(dotCount)+"));\n")
        # outFile.write(pixel)
        # if pink pixel
        # if(img[i][j][0] != 33 and img[i][j][0] != 0):
        #     print("i: ",i, "j ", j, "pixel ",img[i][j])
print(dotCount)
outFile.close()
# outImg.save("../sprite_converted/" + filename + ".png" )
