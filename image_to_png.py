import cv2
import matplotlib.pyplot as plt
import numpy as np
import time

class DaveImage:

    def __init__(self):
        pass

    def Load_CSV(self, filename, bits_Per_Pixel=10):
        self.bits_Per_Pixel = bits_Per_Pixel
        self.pixels = np.loadtxt(filename, delimiter=",", dtype=np.int)

    def Load_DAT(self, filename, width, height, bits_Per_Pixel):
        self.bits_Per_Pixel = bits_Per_Pixel
        img_Data = np.fromfile(filename, dtype=np.uint8).reshape((-1, 2))
        msbs, lsbs = np.hsplit(img_Data, 2)

        self.pixels = ((msbs.astype(np.uint16) << 8) + lsbs.astype(np.uint16))
        self.pixels = self.pixels.reshape((height, width))

    def __Scale_To_8bit(self):
        if (self.bits_Per_Pixel > 8):
            return self.pixels // (2**(self.bits_Per_Pixel - 8))
        else:
            return self.pixels

    def Histogram(self, show_Plot=False):
        hist, bins = np.histogram(self.pixels,
                                  bins=np.arange(0 ,1025))
        if show_Plot:
            plt.plot(hist)
        return hist, bins

    def Show_Image(self):
        cv2.imshow("image window", self.__Scale_To_8bit().astype(np.uint8))
        cv2.waitKey(1)

    def Save_Image(self, filename):
        self.__Scale_To_8bit()
        cv2.imwrite(filename, self.__Scale_To_8bit().astype(np.uint8))


def Multi_Test():
    myImage = DaveImage()
    folder = "images"
    for i in range(10):
        print(i)
        myImage.Load_DAT(f"{folder}/image{i}.dat", 1280, 1024, 8)
        hist, bins = myImage.Histogram()
        myImage.Show_Image()
        myImage.Save_Image(f"{folder}/frame{i}.png")
        time.sleep(0.1)


def AOI_Test():
    myImage = DaveImage()
    myImage.Load_DAT("images/full.dat", 1280, 1024, 10)
    myImage.Save_Image("images/full.png")

    myImage.Load_DAT("images/aoi.dat", 256, 256, 10)
    myImage.Save_Image("images/aoi.png")


if __name__ == "__main__":
    Multi_Test()