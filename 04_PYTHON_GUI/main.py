#!/usr/bin/env python
## Reconbot's Linux GUI:
## Jorge De La Cruz
## e-mail: jorge.cruz.arcia@rwth-aachen.de
## IGM - Institut fur Getriebetechnik und Maschinendynamik

import os
import shlex, subprocess
from Tkinter import *

class App:
    def __init__(self, master):
        self.master = master
        fm = Frame(self.master)
        menu = Menu(master)
        master.config(menu=menu)
        filemenu = Menu(menu)
        menu.add_cascade(label="File", menu=filemenu)
        filemenu.add_command(label="Execute", command=self.Execute)
        filemenu.add_command(label="PID...", command=self.PID)
        filemenu.add_separator()
        filemenu.add_command(label="Exit", command=self.master.quit)

        helpmenu = Menu(menu)
        menu.add_cascade(label="Help", menu=helpmenu)
        helpmenu.add_command(label="About...", command=self.About)
        self.titlei = self.master.title("Reconbot GUI")
        self.logoi = PhotoImage(file="/home/jdelacruz/Pictures/igm.png")
        self.reconbotfig = PhotoImage(file="/home/jdelacruz/Pictures/igm.png")
        self.w1i = Label(fm, image=self.logoi)
        self.reconbot = Label(fm, image=self.reconbotfig)
        self.explanationi = """
Welcome to the Reconbot User Interface
Copyright (c) 2017, IGM-RWTH Aachen University
All rights reserved."""
        self.w2i = Label(fm,
                        justify=LEFT,
                        padx = 10,
                        text=self.explanationi, fg="blue")

        self.w2i.grid(row=0, sticky=W)
        self.w1i.grid(row=0, column = 1, columnspan=2)
        self.reconbot.grid(row=1, column = 0, columnspan=3)
        #master.geometry("1000x800")
        fm.pack()

    def PID(self):
        fm3 = Frame(Toplevel(self.master))
        self.explanation3 = """
Welcome to the Reconbot User Interface"""
        self.w3 = Label(fm3,
                        justify=LEFT,
                        padx = 10,
                        text=self.explanation3)

        self.w3.grid(row=0, column = 1, columnspan=2)
        fm3.pack()

    def Execute(self):
        fm1 = Frame(Toplevel(self.master))
        #self.title = master.title("Reconbot GUI")
        self.logo = PhotoImage(file="/home/jdelacruz/Pictures/igm.png")
        self.w1 = Label(fm1, image=self.logo)
        self.explanation = """
Welcome to the Reconbot User Interface
Copyright (c) 2017, IGM-RWTH Aachen University
All rights reserved."""
        self.w2 = Label(fm1,
                        justify=LEFT,
                        padx = 10,
                        text=self.explanation)

        self.w2.grid(row=0, sticky=W)
        self.w1.grid(row=0, column = 1, columnspan=2)
        #fm1.pack(side="top")
        #fm2 = Frame(master)
        self.button1 = Checkbutton(fm1,text='RCB_full_mode_controller')
        self.button2 = Checkbutton(fm1,text='RCB_3T2R_controller')
        self.button3 = Checkbutton(fm1,text='RCB_3T1R_controller')
        self.button4 = Checkbutton(fm1,text='RCB_1T1RA1C1_controller')
        self.button5 = Checkbutton(fm1,text='RCB_1T1RA2C2_controller')
        self.button6 = Checkbutton(fm1,text='RCB_2T2R6B_controller')
        self.buttonRec = Checkbutton(fm1,text='Record Kinematics variables and Torque')


        self.button1.grid(row=1,sticky = W)
        self.button2.grid(row = 2,sticky = W)
        self.button3.grid(row=3,sticky = W)
        self.button4.grid(row=4,sticky = W)
        self.button5.grid(row=5,sticky = W)
        self.button6.grid(row=6,sticky = W)
        self.buttonRec.grid(row=1,column=1,sticky = W)

        #fm2.pack(side="ce,anchor=W, padx=10)

        #fm3 = Frame(master)
        self.buttonR = Button(fm1,text='Activate all motors to be passive')
        self.buttonRun = Button(fm1,text='Run')
        self.buttonCancel = Button(fm1,text='Cancel', command = self.master.quit)

        self.buttonR.grid(row=3,column=1, sticky = W)
        self.buttonRun.grid(row=6,column=2, sticky = W)
        self.buttonCancel.grid(row=6,column=1, sticky = E)





        #fm3.pack(side="bottom")
        fm1.pack()

    def About(self):
        print "This is a simple example of a menu"


if __name__ == "__main__":
    root = Tk()
    main = App(root)
    a = 'roslaunch igm_collision main.launch sim:=true limited:=true'
    args = shlex.split(a)
    #p=subprocess.Popen(args)
    root.mainloop()
