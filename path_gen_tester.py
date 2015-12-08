import sys
import path_generator as PG
import Tkinter as tk
import copy

COLORS = ["red","orange","yellow","green","blue","violet"]

def main(argv=None):
    filename = str(raw_input("What image would you like to test? "))
    if (len(filename) == 0):
        filename = "hello.png"
    smoothFactor = raw_input("What smoothFactor? (Default 2) ")
    if (len(str(smoothFactor)) == 0):
        smoothFactor = 2
    smoothFactor = int(smoothFactor)
    array = PG.getPixelArrayForFilename(filename)
    # PG.printBlackEnoughArray(array)
    array = PG.getBoolArray(array)
    original = copy.deepcopy(array)
    # PG.printPixelArray(array)
    paths = PG.getRawPaths(array)
    paths = PG.combinePaths(paths, smoothFactor)
    # mask = PG.getInversionMask(vec, array)
    # PG.printPixelArray(mask)
    # PG.invertForMask(array, mask)
    # PG.print01Array(array)

    gFrame = tk.Tk()
    gFrame.geometry('800x600')

    w = tk.Canvas(gFrame, width=800, height=600)
    w.pack()
    w.configure(background="gray")

    w.create_rectangle(0, 0, len(array[0])*20, len(array)*20,)

    scale = 600/len(array)

    height = len(array)
    width = len(array[0])

    PG.saveToFile("momosavestheday.txt", paths, width, height)

    for r in range(0, height):
        for c in range(0, width):
            if original[r][c]:
                w.create_rectangle(c*scale, r*scale, (c+1)*scale, (r+1)*scale, fill="black")

    i = 0
    for vec in paths:
        for edge in vec:
            w.create_line(edge[0][1]*scale, edge[0][0]*scale, edge[1][1]*scale, edge[1][0]*scale, fill=COLORS[i%6])
            i = i + 1


    gFrame.mainloop()

if __name__ == "__main__":
    sys.exit(main())
