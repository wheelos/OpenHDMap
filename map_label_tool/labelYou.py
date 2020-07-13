#!/usr/bin/python3
 
from tkinter import *

root = Tk()

cv = Canvas(root, bg = 'white')

cv.create_rectangle(10, 10, 100, 100)

cv.pack()

root.mainloop()

#top.mainloop()