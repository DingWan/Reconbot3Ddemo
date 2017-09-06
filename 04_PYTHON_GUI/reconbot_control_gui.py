#!/usr/bin/env python
## Reconbot's Linux GUI:
## Jorge De La Cruz
## e-mail: jorge.cruz.arcia@rwth-aachen.de
## IGM - Institut fur Getriebetechnik und Maschinendynamik

import os
import shlex, subprocess, sys
from Tkinter import *
from tkMessageBox import *


class App:
    def __init__(self, master):
        self.master = master
        #self.master.geometry('800x600')
        fm = Frame(self.master)
        menu = Menu(master)
        master.config(menu=menu)
        filemenu = Menu(menu)
        menu.add_cascade(label="File", menu=filemenu)
        filemenu.add_command(label="Execute", command=self.ExecuteWindow)
        filemenu.add_command(label="PID...", command=self.PID)
        filemenu.add_separator()
        filemenu.add_command(label="Exit", command=self.master.quit)

        helpmenu = Menu(menu)
        menu.add_cascade(label="Help", menu=helpmenu)
        helpmenu.add_command(label="About...", command=self.About)

        ##################################################################################
        ##                              Defining Widgets                             ##
        ##################################################################################
        self.titlei = self.master.title("reconbot_control GUI")
        self.logoi = PhotoImage(file="~/catkin_ws/src/reconbot/04_PYTHON_GUI/pictures/igm.png")
        self.reconbotfig = PhotoImage(file="~/catkin_ws/src/reconbot/04_PYTHON_GUI/pictures/reconbot.png")
        self.w1i = Label(fm, image=self.logoi)
        self.reconbot = Label(fm, image=self.reconbotfig)
        self.expli = """
        Welcome to reconbot_control User Interface
        Copyright (c) 2017, IGM-RWTH Aachen University
        All rights reserved."""
        self.w2i = Label(fm,
                        justify=LEFT,
                        padx = 10,
                        text=self.expli, fg="blue",
                        font = "Times 12 bold")

        ##################################################################################
        ##                              Positioning Widgets                            ##
        ##################################################################################
        self.w2i.grid(row=0, sticky=W)
        self.w1i.grid(row=0, column = 1, columnspan=2)
        self.reconbot.grid(row=1, column = 0, columnspan=3)
        #master.geometry("1000x800")
        fm.pack()

    def PID(self):
        fm3 = Frame(Toplevel(self.master))
        self.explanation3 = """
        under construction"""
        self.w3 = Label(fm3,
                        justify=LEFT,
                        padx = 10,
                        text=self.explanation3)

        self.w3.grid(row=0, column = 1, columnspan=2)
        fm3.pack()

    def ExecuteWindow(self):
        #fm1 = Frame(Toplevel(), width=800, height=600)
        fm1 = Toplevel(self.master, relief = RIDGE, bd = 5, bg = "white")
        fm1.geometry("1175x540")
        #canvas = Canvas(fm1)
        #can = Canvas(fm1)
        #can.pack()
        #can.create_line(0, 0, 725, 100)
        #can.pack()
        #self.master.geometry("800x600")
        #fm1.pack()
        #self.title = master.title("Reconbot GUI")
        ##################################################################################
        ##                              Defining Widgets                             ##
        ##################################################################################

        self.logo = PhotoImage(file="~/catkin_ws/src/reconbot/04_PYTHON_GUI/pictures/igm.png")
        self.expl1 = """
        Welcome to reconbot_control User Interface
        Copyright (c) 2017, IGM-RWTH Aachen University
        All rights reserved."""
        self.expl2 = """
        Please select ONE of the following
        kinematic modes (double clicking):"""

        self.expl3 = "Motors mode:"
        self.expl4 = "Record positions, velocities, \n accelerations and torque:"
        self.passiveExpl = 'Passive'
        self.activeExpl = 'Active'
        self.w1 = Label(fm1,
                        justify=LEFT,
                        padx = 10,
                        text=self.expl1,
                        font = "Times 12 bold",
                        fg = "blue",
                        bg = "white")
        self.w2 = Label(fm1, image=self.logo, bg = "white")

        self.w3 = Label(fm1,
                        justify=LEFT,
                        padx = 10,
                        text=self.expl2,
                        font = "Times 11 bold", bg = "white")
        self.labelRec = Label (fm1,
                               justify = LEFT,
                               padx = 10,
                               text = self.expl4,
                               font = "Times 11 bold", bg = "White")
        self.labelMotors = Label (fm1,
                               justify = LEFT,
                               padx = 10,
                               text = self.expl3,
                               font = "Times 11 bold", bg = "white")
        self.buttonRecStart = Button(fm1,text='Start Rec', command = self.StartRecording, width = 7, bg = "green")
        self.buttonRecStop = Button(fm1,text='Stop Rec', command = self.StopRecording, width = 7, bg = "red")
        self.buttonPassive = Button(fm1,text= self.passiveExpl, command = self.PassiveMotors, width = 7, bg= "yellow")
        self.buttonActive = Button(fm1,text= self.activeExpl, command = self.ActiveMotors, width = 7, bg = "green")
        #fm1.pack(side="top")
        #fm2 = Frame(master)
        self.modes = Listbox(fm1)
        for option in ['RCB_full_mode_controller', 'RCB_3T2R_controller',
                        'RCB_3T1R_controller', 'RCB_1T1RA1C1_controller',
                        'RCB_1T1RA2C2_controller', 'RCB_1T1RA1C1A2C2_controller',
                        'RCB_2T2R6B_controller','RCB_2T2R6BX0Y0_controller',
                        'RCB_2T2R5B_controller', 'RCB_2T2R3B_controller',
                        'RCB_2RA1C1_controller', 'RCB_2RA2C2_controller',
                        'RCB_FIXED2U_controller'
                        ]:
                        self.modes.insert(END, option)
        self.modes.bind("<Button-1>", self.ModeState)
        self.buttonCancel = Button(fm1,text='Stop', command = self.StopReconbot, width = 7, bg = "red")
        self.buttonRun = Button(fm1,text='Run',command = self.Execute, width = 7, bg = "green")
        self.buttonConnect = Button(fm1,text='Connect',command = self.Connect, width = 7, bg = "yellow")

        lcc = "Launch roscore and all necessary nodes for controlling the Reconbot."
        lsc = "Stop all ROS processes invoking kill rosmaster."
        lrc = "Execute the trajectory into the ~/catkin_ws/src/reconbot/01_ROS_Code/trajectory/trajectory.txt file."

        labelNote = Label(fm1, justify = LEFT, text = "Note:", font = "Times 12 bold",width = 9)
        labelRun = Label(fm1, justify = LEFT, text = "Run:", bg = "green", width = 9)
        labelConnect = Label(fm1, justify = LEFT, text = "Connect:", bg = "yellow", width = 9)
        labelStop = Label(fm1, justify = LEFT, text = "Stop:", bg = "red", width = 9)
        labelConnectCon = Label(fm1, text = lcc, width=80, anchor = W)
        labelStopCon = Label(fm1, anchor = W, text = lsc, width=80)
        labelRunCon = Label(fm1, anchor = W, text = lrc, width=80)



        ##################################################################################
        ##                              Positioning Widgets                            ##
        ##################################################################################
        self.w1.grid(row=0, column = 0, sticky=W)
        self.w2.grid(row=0, column = 1,columnspan = 3, sticky = E)
        self.w3.grid(row=2, column = 0, sticky = W)
        self.modes.grid(row=4,rowspan=2, padx= 40,sticky = W)
        self.labelRec.grid(row = 4, column = 1, sticky = NE)
        self.labelMotors.grid(row = 3, column = 1, sticky = SE)
        self.buttonRecStart.grid(row=4,column=2,sticky = NE)
        self.buttonRecStop.grid(row=4,column=3,sticky = NW)
        self.buttonPassive.grid(row=3,column=3, sticky = SW)
        self.buttonActive.grid(row=3,column=2, sticky = SE)
        self.buttonRun.grid(row=7,column=3, sticky = W)
        self.buttonConnect.grid(row=6, column=3, sticky = W)
        self.buttonCancel.grid(row=7,column=2, sticky = E)
        labelNote.grid(row = 9 , column = 0, sticky = E)
        labelConnect.grid(row = 10 , column = 0, sticky = E)
        labelStop.grid(row = 11 , column = 0, sticky = E)
        labelRun.grid(row = 12 , column = 0, sticky = E)
        labelConnectCon.grid(row = 10, column=1,columnspan = 2, sticky=W)
        labelStopCon.grid(row = 11, column=1, columnspan = 2, sticky=W)
        labelRunCon.grid(row = 12, column=1, columnspan = 2, sticky=W)
        #fm3.pack(side="bottom")
        #fm1.pack()

    def StopReconbot(self):
            a = 'killall -9 rosmaster'
            #b = 'kill'+' '+self.pidp
            #print b
            args = shlex.split(a)
            #args2 = shlex.split(b)
            #p = subprocess.Popen(args)
            q = subprocess.Popen(args)

    def Execute(self):
        try:
            a = 'rosrun reconbot_control trajectory_publisher'
            args = shlex.split(a)
            p = subprocess.Popen(args)
            #p = subprocess.Popen(args, stdout=subprocess.PIPE,stderr=subprocess.PIPE)
            #output,error = p.communicate()
        except:
            showerror("Exception", "Please, select one of the available modes before connecting")


    def Connect(self):
        try:
            imode = self.selectedMode[0]
            if  imode == 0 or imode == 9 or imode == 12:
                a = 'roslaunch reconbot_control main.launch full_mode:=true'
                args1 = shlex.split(a)
                p=subprocess.Popen(args1)

            if  imode == 1 or imode == 6 or imode == 7:
                a = 'roslaunch reconbot_control main.launch full_mode:=true'
                b = 'rosrun'
                args = shlex.split(a)
                p=subprocess.Popen(args)


                b = 'rosrun reconbot_control torque_enable_client 1'
                args2 = shlex.split(b)
                q=subprocess.Popen(args2)

            if  imode == 2 or imode == 3 or imode == 4 or imode == 5:
                a = 'roslaunch reconbot_control main.launch full_mode:=true'
                args = shlex.split(a)
                p=subprocess.Popen(args)


                b = 'rosrun reconbot_control torque_enable_client 2'
                args2 = shlex.split(b)
                q=subprocess.Popen(args2)

            if  imode == 8:
                a = 'roslaunch reconbot_control main.launch full_mode:=true'
                args = shlex.split(a)
                p=subprocess.Popen(args)


                b = 'rosrun reconbot_control torque_enable_client 8'
                args2 = shlex.split(b)
                q=subprocess.Popen(args2)

            if  imode == 10:
                a = 'roslaunch reconbot_control main.launch full_mode:=true'
                args = shlex.split(a)
                p=subprocess.Popen(args)


                b = 'rosrun reconbot_control torque_enable_client 10'
                args2 = shlex.split(b)
                q=subprocess.Popen(args2)

            if  imode == 11:
                a = 'roslaunch reconbot_control main.launch full_mode:=true'
                args = shlex.split(a)
                p=subprocess.Popen(args)


                b = 'rosrun reconbot_control torque_enable_client 11'
                args2 = shlex.split(b)
                q=subprocess.Popen(args2)

        except:
            #fmexce = Frame(Toplevel(self.master))
            showerror("Exception", "Please, select one of the available modes before connecting")

    def ModeState(self,event):
        self.selectedMode = self.modes.curselection()

    def PassiveMotors(self):
        a = 'rosrun reconbot_control torque_enable_client 15'
        args = shlex.split(a)
        p=subprocess.Popen(args)

    def ActiveMotors(self):
        a = 'rosrun reconbot_control torque_enable_client 0'
        args = shlex.split(a)
        p=subprocess.Popen(args)

    def StartRecording(self):
        a = 'rosrun reconbot_control sensor_data_capture'
        b = 'rosrun reconbot_control torque_data_capture'
        args1 = shlex.split(a)
        args2 = shlex.split(b)
        p=subprocess.Popen(args1)
        q=subprocess.Popen(args2)

    def StopRecording(self):
        a = 'rosnode kill sensor_data_capture'
        b = 'rosnode kill torque_data_capture'
        args1 = shlex.split(a)
        args2 = shlex.split(b)
        p=subprocess.Popen(args1)
        q=subprocess.Popen(args2)


    def About(self):
        aboutfn = Toplevel(bg = "white")
        msg1 = "reconbot_control User Interface developed in Python \n for executing the reconbot_control package"
        msg2 = "Developed by Jorge De La Cruz \n E-mail: delacruz@igm.rwth-aachen.de \n September 2017."
        labelmsg1 = Label(aboutfn, text = msg1, justify = LEFT, font = "Times 11 bold", bg = "white")
        labelmsg2 = Label(aboutfn, text = msg2, justify = LEFT, font = "Times 10 bold", bg = "white")
        labelfig = Label(aboutfn, image = self.logoi, bg = "white")

        labelfig.grid(row = 0, sticky = W)
        labelmsg1.grid(row = 1, sticky = W)
        labelmsg2.grid(row = 2, sticky = W)
        #aboutfn.pack()


if __name__ == "__main__":
    root = Tk()
    main = App(root)
    a = 'roslaunch igm_collision main.launch sim:=true limited:=true'
    args = shlex.split(a)
    #p=subprocess.Popen(args)
    root.mainloop()
