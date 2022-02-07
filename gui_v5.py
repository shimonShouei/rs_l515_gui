#!/usr/bin/env python3
# license removed for brevity
import subprocess
import tkinter
from tkinter import ttk, messagebox
from tkinter import *
import numpy as np
import rospy
from std_msgs.msg import Float32MultiArray, Int32MultiArray
from functools import partial
import cv2
from PIL import Image, ImageTk
import threading
import pyrealsense2 as rs
from tkinter.scrolledtext import ScrolledText
from datetime import datetime

# from std_msgs.msg import Float32MultiArray, Int32MultiArray

from enum import Enum


class thread_state(Enum):
    off = 0
    on = 1
    stopped = -1


max_PWM = 255
max_JOINT = 30
N_links = 10
N_joints = N_links
N_motor_each = 2
N_motor = N_links * N_motor_each
slider_dPWM = 1
N_string = N_joints * 2

videoloop_stop = [0, 0]
started = [False, False]
pipe = rs.pipeline()
threads = [[], []]
No = 0
# pipe = rs.pipeline()
cap1 = cv2.VideoCapture(No)
cap1.set(cv2.CAP_PROP_FRAME_WIDTH, 800)
cap1.set(cv2.CAP_PROP_FRAME_HEIGHT, 600)

colorizer = rs.colorizer()
cfg = rs.config()


def _on_mousewheel(self, event):
    print("mw")
    self.canvas.yview_scroll(-1 * (event.delta / 120), "units")


class APP:

    def init_tabs_dict(self):
        self.tabs_dict["motors_control"] = self.create_tab()
        self.tabs_dict["joints_control"] = self.create_tab()
        self.tabs_dict["plans_control"] = self.create_tab()

    def __init__(self):
        # ROS
        rospy.init_node("GUI", anonymous=True)
        self.motor_pwm_pub = rospy.Publisher("/robot_snake_4/motor_cmd", Int32MultiArray, queue_size=10)
        self.joint_cmd_pub = rospy.Publisher("/robot_snake_10/joint_cmd", Float32MultiArray, queue_size=10)
        rospy.Subscriber("/robot_snake_1/tension_val", Float32MultiArray, self.tension_val_update)
        rospy.Subscriber("/robot_snake_10/joint_val", Float32MultiArray, self.joint_val_update)

        # tkinter
        self.root = Tk()
        self.root.geometry('1400x1000')
        self.root.resizable(1, 1)
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        self.root.title("Tab Widget")
        self.tabControl = ttk.Notebook(self.root)
        self.tabs_dict = {}
        self.tab1 = self.create_tab()
        self.tab2 = self.create_tab()
        self.tab3 = self.create_tab()
        self.tab4 = self.create_tab()
        self.tab5 = self.create_tab()
        self.tab5.columnconfigure(0, weight=1)
        self.tab5.rowconfigure(0, weight=1)
        self.tabControl.add(self.tab1, text='Tab 1')
        self.tabControl.add(self.tab2, text='Tab 2')
        self.tabControl.add(self.tab3, text='Tab 3')
        self.tabControl.add(self.tab4, text='Tab 4')
        self.tabControl.add(self.tab5, text='Tab 5')
        self.tabControl.pack(expand=1, fill="both")

        self.my_canvas_tab1 = Canvas(self.tab1)

        self.my_canvas_tab2 = Canvas(self.tab2)
        self.my_canvas_tab3 = Canvas(self.tab3)
        self.my_canvas_tab4 = Canvas(self.tab4)
        self.my_canvas_tab5 = Canvas(self.tab5)

        self.my_canvas_tab1.pack(side=TOP, fill=BOTH, expand=1)
        self.my_canvas_tab2.pack(side=TOP, fill=BOTH, expand=1)
        self.my_canvas_tab3.pack(side=TOP, fill=BOTH, expand=1)
        self.my_canvas_tab4.pack(side=TOP, fill=BOTH, expand=1)
        # self.my_canvas_tab5.pack(side=TOP, fill=BOTH, expand=1)
        # self.my_canvas_tab5.columnconfigure(10, weight=1)
        # self.my_canvas_tab5.rowconfigure(10, weight=1)

        # y_scrollbar.pack(side=RIGHT, fill=Y)

        self.frame_tab1 = Frame(self.my_canvas_tab1, width=1000, height=10000)
        self.frame_tab2 = Frame(self.my_canvas_tab2, width=1000, height=10000)
        self.frame_tab3 = Frame(self.my_canvas_tab3, width=1000, height=10000)
        self.frame_tab4 = Frame(self.my_canvas_tab4, width=1000, height=10000)
        self.frame_tab5 = Frame(self.tab5, width=1000, height=10000)

        self.frame_tab5.grid(row=0, column=0, sticky="nsew")
        self.frame_tab5.columnconfigure(0, weight=1)
        self.frame_tab5.rowconfigure(0, weight=1)

        self.my_canvas_tab1.create_window(500, 38 * N_motor, window=self.frame_tab1)
        self.my_canvas_tab2.create_window(700, 34 * N_joints, window=self.frame_tab2)
        self.my_canvas_tab3.create_window(700, 34 * N_joints, window=self.frame_tab3)
        self.my_canvas_tab4.create_window(700, 34 * N_joints, window=self.frame_tab4)
        # self.my_canvas_tab5.create_window(700, 34 * N_joints, window=self.frame_tab5)

        self.y_scrollbar_tab1 = ttk.Scrollbar(self.my_canvas_tab1, orient=VERTICAL, command=self.my_canvas_tab1.yview)
        self.y_scrollbar_tab2 = ttk.Scrollbar(self.my_canvas_tab2, orient=VERTICAL, command=self.my_canvas_tab2.yview)
        self.y_scrollbar_tab3 = ttk.Scrollbar(self.my_canvas_tab3, orient=VERTICAL, command=self.my_canvas_tab3.yview)
        self.y_scrollbar_tab4 = ttk.Scrollbar(self.my_canvas_tab4, orient=VERTICAL, command=self.my_canvas_tab4.yview)
        # self.y_scrollbar_tab5 = ttk.Scrollbar(self.my_canvas_tab5, orient=VERTICAL, command=self.my_canvas_tab5.yview)

        self.my_canvas_tab1.config(yscrollcommand=self.y_scrollbar_tab1.set, scrollregion=(0, 0, 1000, 2000))
        self.my_canvas_tab2.config(yscrollcommand=self.y_scrollbar_tab2.set, scrollregion=(0, 0, 1000, 2000))
        self.my_canvas_tab3.config(yscrollcommand=self.y_scrollbar_tab3.set, scrollregion=(0, 0, 1000, 2000))
        self.my_canvas_tab4.config(yscrollcommand=self.y_scrollbar_tab4.set, scrollregion=(0, 0, 1000, 2000))
        # self.my_canvas_tab5.config(yscrollcommand=self.y_scrollbar_tab5.set, scrollregion=(0, 0, 1000, 2000))

        self.y_scrollbar_tab1.pack(side=RIGHT, fill=Y)
        self.y_scrollbar_tab2.pack(side=RIGHT, fill=Y)
        self.y_scrollbar_tab3.pack(side=RIGHT, fill=Y)
        self.y_scrollbar_tab4.pack(side=RIGHT, fill=Y)
        # self.y_scrollbar_tab5.pack(side=RIGHT, fill=Y)
        # mouse wheel scroll
        self.my_canvas_tab1.bind('<4>', lambda event: self.my_canvas_tab1.yview('scroll', -1, 'units'))
        self.my_canvas_tab1.bind('<5>', lambda event: self.my_canvas_tab1.yview('scroll', 1, 'units'))
        self.my_canvas_tab2.bind('<4>', lambda event: self.my_canvas_tab2.yview('scroll', -1, 'units'))
        self.my_canvas_tab2.bind('<5>', lambda event: self.my_canvas_tab2.yview('scroll', 1, 'units'))
        self.my_canvas_tab3.bind('<4>', lambda event: self.my_canvas_tab3.yview('scroll', -1, 'units'))
        self.my_canvas_tab3.bind('<5>', lambda event: self.my_canvas_tab3.yview('scroll', 1, 'units'))
        self.my_canvas_tab4.bind('<4>', lambda event: self.my_canvas_tab4.yview('scroll', -1, 'units'))
        self.my_canvas_tab4.bind('<5>', lambda event: self.my_canvas_tab4.yview('scroll', 1, 'units'))
        # self.my_canvas_tab5.bind('<4>', lambda event: self.my_canvas_tab5.yview('scroll', -1, 'units'))
        # self.my_canvas_tab5.bind('<5>', lambda event: self.my_canvas_tab5.yview('scroll', 1, 'units'))

        self.panel3 = Label(self.my_canvas_tab3)
        self.panel3.place(x=50, y=50)
        self.panel4 = Label(self.my_canvas_tab4)
        self.panel4.place(x=50, y=50)

        self.tab3_button1 = Button(
            self.my_canvas_tab3, text="start", bg="#fff", font=("", 20),
            command=lambda: self.button1_clicked())
        self.tab3_button1.place(x=350, y=850, width=200, height=100)
        self.tab3_button2 = Button(
            self.my_canvas_tab3, text="stop", bg="#fff", font=("", 20),
            command=lambda: self.button2_clicked())
        self.tab3_button2.place(x=150, y=850, width=200, height=100)

        self.tab4_button3 = Button(
            self.my_canvas_tab4, text="start", bg="#fff", font=("", 20),
            command=lambda: self.button3_clicked())
        self.tab4_button3.place(x=350, y=750, width=200, height=100)
        self.tab4_button4 = Button(
            self.my_canvas_tab4, text="stop", bg="#fff", font=("", 20),
            command=lambda: self.button4_clicked())
        self.tab4_button4.place(x=150, y=750, width=200, height=100)

        self.joint_val_text_tab1 = [0 for x in range(N_joints)]
        self.joint_val_tab1 = [0 for x in range(N_joints)]
        self.joint_val_text_tab2 = [0 for x in range(N_joints)]
        self.joint_val_tab2 = [0 for x in range(N_joints)]
        self.tension_label_tab1 = [[0 for i in range(N_motor_each)] for j in range(N_string)]
        self.tension_label_tab2 = [[0 for i in range(N_motor_each)] for j in range(N_string)]
        self.tension_text = [[0 for i in range(N_motor_each)] for j in range(N_string)]
        self.motor_label_tab1 = [[0 for i in range(N_motor_each)] for j in range(N_string)]
        self.motor_text = [[0 for i in range(N_motor_each)] for j in range(N_string)]
        self.sliders_tab1 = [0 for x in range(N_motor)]
        self.sliders_tab2 = [0 for x in range(N_joints)]
        self.buttons_up_tab2 = [0 for x in range(N_joints)]
        self.buttons_down_tab2 = [0 for x in range(N_joints)]
        self.buttons_reset_tab2 = [0 for x in range(N_joints)]
        self.joint_cmd = [0 for x in range(N_joints)]
        j_row = 1
        for i in range(N_links):
            V_i = 0.123
            V_cmd = 0.124
            # joint for tab2
            self.joint_val_text_tab1[i] = StringVar()
            self.joint_val_text_tab1[i].set("Joint {}: {} deg".format(i + 1, V_i))
            self.joint_val_tab1[i] = ttk.Label(self.frame_tab1, textvariable=self.joint_val_text_tab1[i])
            self.joint_val_tab1[i].grid(column=0, row=j_row, padx=0)
            # joint for tab2
            self.joint_val_text_tab2[i] = StringVar()

            self.joint_val_text_tab2[i].set("Joint {}: {} deg/{}".format(i + 1, V_i, V_cmd))
            self.joint_val_tab2[i] = ttk.Label(self.frame_tab2, textvariable=self.joint_val_text_tab2[i])
            self.joint_val_tab2[i].grid(column=0, row=i, padx=30)

            self.sliders_tab2[i] = Scale(self.frame_tab2, length=300, from_=-max_JOINT, to=max_JOINT, orient=HORIZONTAL,
                                         tickinterval=max_JOINT / 2)
            self.sliders_tab2[i].grid(column=3, row=i)
            self.sliders_tab2[i].bind("<B1-Motion>",
                                      lambda event, arg1=i: self.slider_joint_change(event, arg1))
            self.buttons_down_tab2[i] = Button(self.frame_tab2, text="A{}1".format(i),
                                               command=partial(self.slider_joint_buttons, -1, i))
            self.buttons_down_tab2[i].grid(column=2, row=i)
            self.buttons_up_tab2[i] = Button(self.frame_tab2, text="A{}2".format(i),
                                             command=partial(self.slider_joint_buttons, 1, i))
            self.buttons_up_tab2[i].grid(column=4, row=i)
            self.buttons_reset_tab2[i] = Button(self.frame_tab2, text="reset",
                                                command=partial(self.slider_joint_buttons, 0, i))
            self.buttons_reset_tab2[i].grid(column=1, row=i)

            for j in range(N_motor_each):
                cnt = i * 2 + j
                self.sliders_tab1[cnt] = Scale(self.frame_tab1, length=300, from_=-max_PWM, to=max_PWM,
                                               orient=HORIZONTAL, tickinterval=max_PWM / 2)
                self.sliders_tab1[cnt].grid(column=2, row=j_row + j + 1)
                self.sliders_tab1[cnt].bind("<ButtonRelease-1>",
                                            lambda event, arg1=cnt: self.slider_pwm_reset(event, arg1))
                self.sliders_tab1[cnt].bind("<B1-Motion>",
                                            lambda event, arg1=cnt: self.slider_pwm_change(event, arg1))
                self.tension_text[i][j] = StringVar()
                self.tension_label_tab1[i][j] = ttk.Label(self.frame_tab1, textvariable=self.tension_text[i][j])
                self.tension_label_tab1[i][j].grid(column=3, row=j_row + j + 1, padx=0)
                self.tension_text[i][j].set("String #{}: Waite for connection".format(j + 1))
                self.motor_text[i][j] = StringVar()
                self.motor_label_tab1[i][j] = ttk.Label(self.frame_tab1, textvariable=self.motor_text[i][j])
                self.motor_label_tab1[i][j].grid(column=1, row=j_row + j + 1, padx=30)
                self.motor_text[i][j].set("Motor {}: {} deg".format(cnt + 1, "pmw val"))
                self.tension_label_tab2[i][j] = ttk.Label(self.frame_tab2, textvariable=self.tension_text[i][j])
                self.tension_label_tab2[i][j].grid(column=5 + j, row=i, padx=30)
                j_row += 3

        self.process_dict = {"circle": "", "square": "", "joystick": "", "homing": "", "serial_joint": ""
            , "serial_motor": "", "serial_tension": "", "serial_linear": ""}
        button_height = 2
        button_width = 8
        self.homing_button = Button(self.frame_tab5, text="Homing", bg="gray", font=("", 20), height=button_height,
                                    width=button_width,
                                    command=partial(self.run_node,
                                                    "rosrun robot_snake_10 homing_controller __name:=homing_controller",
                                                    "homing"))
        # TODO kill homing automatically
        self.circle_button = Button(self.frame_tab5, text="Circle", bg="yellow", font=("", 20), height=button_height,
                                    width=button_width,
                                    command=partial(self.run_node, "rosrun robot_snake_10 circle_move", "circle"))
        self.square_button = Button(self.frame_tab5, text="Square", bg="orange", font=("", 20), height=button_height,
                                    width=button_width,
                                    command=partial(self.run_node, "rosrun robot_snake_10 square_move", "square"))
        self.joystick_button = Button(self.frame_tab5, text="Joystick", bg="blue", font=("", 20), height=button_height,
                                      width=button_width,
                                      command=partial(self.run_node, "?", "joystick"))

        # self.homing_kill_button = Button(self.frame_tab5, text="Kill Homing", bg="#fff", font=("", 20),
        #                                  command=partial(self.kill_homing))
        # self.homing_kill_button.grid(column=2, row=7)
        self.run_button = Button(self.frame_tab5, text="Run\ncontroller", bg="green", font=("", 20),
                                 height=button_height,
                                 width=button_width,
                                 command=partial(self.run_controller))

        self.stop_button = Button(self.frame_tab5, text="Stop\nmotion", bg="red", font=("", 20), height=button_height,
                                  width=button_width,
                                  command=partial(self.stop_motion))
        self.rosbag_val = IntVar()
        self.bag_btn = Checkbutton(self.frame_tab5, text="Record to bag", variable=self.rosbag_val, onvalue=1,
                                   offvalue=0)

        self.run_button.place(x=500, y=150)
        self.joystick_button.place(x=300, y=325)
        self.square_button.place(x=500, y=325)
        self.circle_button.place(x=700, y=325)
        self.homing_button.place(x=500, y=425)
        self.bag_btn.place(x=1000, y=150)
        self.stop_button.place(x=500, y=650)

        self.log_area = tkinter.Text(self.frame_tab5, height=35, width=45)
        self.log_area.place(x=1000, y=180)
        self.x = 0
        self.y = 0
        self.d = 0
        self.home_position()

        # while not rospy.is_shutdown():
        #     rospy.Rate(100).sleep()

    def button1_clicked(self):
        # if not started[0]:
        t = threading.Thread(target=self.videoLoop, args=(videoloop_stop,))
        t.start()
        threads[0].append(t)
        started[0] = True

    # else:
    #     videoloop_stop[0] = False

    def button2_clicked(self):
        videoloop_stop[0] = -1

    def button3_clicked(self):
        # if not started[1]:
        t = threading.Thread(target=self.videoDepthLoop, args=(videoloop_stop,))
        t.start()
        threads[1].append(t)
        started[1] = True

    # else:
    #     videoloop_stop[1] = False
    #

    def button4_clicked(self):
        videoloop_stop[1] = -1

    def videoDepthLoop(self, mirror=False):
        if videoloop_stop[0] == 1:
            # if switcher tells to stop then we switch it again and stop videoloop
            # for t in threads[0]:
            #     t.join()
            videoloop_stop[0] = -1

        profile = pipe.start(cfg)
        # Skip 5 first frames to give the Auto-Exposure time to adjust
        for x in range(5):
            pipe.wait_for_frames()

        while True:
            if videoloop_stop[1] == -1:
                # if switcher tells to stop then we switch it again and stop videoloop
                pipe.stop()
                print("Frames Captured")

                videoloop_stop[1] = 0
                # panel.destroy()
                # cap1.release()

                break
            videoloop_stop[1] = 1
            # Store next frameset for later processing:
            frameset = pipe.wait_for_frames()
            # depth_image = np.asanyarray(frameset.get_depth_frame().get_data())
            # colorized_depth = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
            depth_frame = frameset.get_depth_frame()
            self.d = depth_frame.get_distance(int(self.x), int(self.y))
            colorized_depth = np.asanyarray(colorizer.colorize(depth_frame).get_data())
            img = Image.fromarray(colorized_depth)
            image = ImageTk.PhotoImage(image=img)
            self.panel4.configure(image=image)
            self.panel4.image = image
            self.panel4.bind('<Motion>', self.motion)
            # check switcher value

    def motion(self, event):
        self.x, self.y = event.x, event.y
        print('x: {}, y: {}, d: {}'.format(self.x, self.y, self.d))

    def videoLoop(self, mirror=False):
        if videoloop_stop[1] == 1:
            # if switcher tells to stop then we switch it again and stop videoloop
            # for t in threads[1]:
            #     t.join()
            videoloop_stop[1] = -1
        No = 1
        # pipe = rs.pipeline()
        profile = pipe.start(cfg)
        # Skip 5 first frames to give the Auto-Exposure time to adjust
        for x in range(5):
            pipe.wait_for_frames()

        while True:
            if videoloop_stop[0] == -1:
                # if switcher tells to stop then we switch it again and stop videoloop
                pipe.stop()
                print("Frames Captured")
                videoloop_stop[0] = 0
                cap1.release()

                break
            videoloop_stop[0] = 1
            # Store next frameset for later processing:
            frameset = pipe.wait_for_frames()
            color_frame = frameset.get_color_frame()
            color_image = np.asanyarray(color_frame.get_data())
            # Render images
            img = Image.fromarray(color_image)
            image = ImageTk.PhotoImage(image=img)
            self.panel3.configure(image=image)
            self.panel3.image = image
            # check switcher value

    def create_tab(self):
        tab = ttk.Frame(self.tabControl)
        tab.pack(fill=BOTH, expand=1)
        return tab

    def stop_motion(self):
        if self.rosbag_val.get():
            self.process_dict["rosbag"].kill()
            self.process_dict["rosbag"].terminate()
            subprocess.Popen(
                "rosnode kill /my_bag".split(),
                shell=False)

        for proc in self.process_dict.keys():
            process = self.process_dict[proc]
            if process != "" and not proc.__contains__("serial"):
                process.kill()
                process.terminate()
                subprocess.Popen(
                    f"rosnode kill /{proc}".split(),
                    shell=False)
        self.send_motor_cmd(np.array([0 for x in range(N_motor)]))
        self.log_area.insert(tkinter.END,
                             f"[{datetime.now().strftime('%y-%m-%d %H:%M:%S')}] All motions have stopped\n")

    def run_controller(self):
        self.run_node("rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0",
                      "serial_joint")
        self.run_node("rosrun rosserial_python serial_node.py _port:=/dev/ttyACM1",
                      "serial_motor")
        self.run_node("rosrun rosserial_python serial_node.py _port:=/dev/ttyACM2",
                      "serial_tension")
        self.run_node("rosrun rosserial_python serial_node.py _port:=/dev/ttyACM3",
                      "serial_linear")
        self.log_area.insert(tkinter.END,
                             f"[{datetime.now().strftime('%y-%m-%d %H:%M:%S')}] snake controller launched\n")

    def run_node(self, command, proc_name):
        if self.rosbag_val.get() and not proc_name.__contains__("serial") and not proc_name == "controller_v10_4":
            if "rosbag" in self.process_dict:
                self.process_dict["rosbag"] = subprocess.Popen(
                    "rosnode kill /my_bag".split(),
                    shell=False)
                self.process_dict["rosbag"].kill()
                self.process_dict["rosbag"].terminate()
            self.process_dict["rosbag"] = subprocess.Popen(
                "rosbag record -a -o /home/robot-snake/rs_ws/src/robot_snake_10/bags/ __name:=my_bag".split(),
                shell=False)
        if not proc_name.__contains__("serial") and not proc_name == "controller_v10_4":
            self.run_node("rosrun robot_snake_10 controller_v10_4", "controller_v10_4")
        self.process_dict[proc_name] = subprocess.Popen((command+f" __name:={proc_name}").split(), shell=False)
        self.log_area.insert(tkinter.END, f"[{datetime.now().strftime('%y-%m-%d %H:%M:%S')}] {proc_name} launched\n")
        if proc_name == "homing":
            threshold = 0.01
            self.log_area.insert(tkinter.END,
                                 f"[{datetime.now().strftime('%y-%m-%d %H:%M:%S')}] run until get to {threshold}\n")

            while not all([x <= threshold for x in self.joint_val_tab1]):
                continue
            self.process_dict[proc_name].kill()
            self.process_dict[proc_name].terminate()
            self.log_area.insert(tkinter.END,
                                 f"[{datetime.now().strftime('%y-%m-%d %H:%M:%S')}] homing successfully done\n")

    # def kill_homing(self):
    #     self.process.kill()
    #     self.process.terminate()

    def slider_pwm_change(self, event, i):
        # print(self.slider_pwm[i][j].get())
        arr = np.zeros(N_string)
        arr[i] = self.sliders_tab1[i].get()
        self.send_motor_cmd(arr)

    def send_motor_cmd(self, arr):
        msg = Int32MultiArray(data=arr.astype(int))
        self.motor_pwm_pub.publish(msg)

    def slider_joint_buttons(self, diff, i):
        if diff == 0:
            self.sliders_tab2[i].set(0)
            self.joint_cmd[i] = self.sliders_tab2[i].get()
            self.send_joint_cmd(self.joint_cmd)
        self.sliders_tab2[i].set(self.sliders_tab2[i].get() + diff)
        self.joint_cmd[i] = self.sliders_tab2[i].get()
        self.send_joint_cmd(self.joint_cmd)

    def slider_joint_change(self, event, i):
        self.joint_cmd[i] = self.sliders_tab2[i].get()
        self.send_joint_cmd(self.joint_cmd)

    def slider_pwm_reset(self, event, i):
        self.sliders_tab1[i].set(0)
        arr = np.zeros(N_string)
        self.send_motor_cmd(arr)
        return

    def joint_val_update(self, msg):
        if len(msg.data) > 0:
            for i in range(N_links):
                self.joint_val_text_tab1[i].set("Joint {}: {:.3f} deg".format(i + 1, msg.data[i]))
                self.joint_val_text_tab2[i].set("Joint {}: {:.3f} deg".format(i + 1, msg.data[i]))

    def send_joint_cmd(self, arr):
        msg = Float32MultiArray(data=arr)
        self.joint_cmd_pub.publish(msg)

    def home_position(self):
        for i in range(N_joints):
            self.sliders_tab2[i].set(0)
            self.joint_cmd[i] = self.sliders_tab2[i].get()
        self.send_joint_cmd(self.joint_cmd)

    def tension_val_update(self, msg):
        if len(msg.data) > 0:
            for i in range(N_string):
                row = int(np.floor(i / 2))
                col = int(i % 2)
                if msg.data[i] > 0:
                    text = "String #{}: {:.2f} Kg".format(i + 1, msg.data[i])
                else:
                    text = "String #{}: {:.2f} Kg -- Error".format(i + 1, msg.data[i])
                self.tension_text[row][col].set(text)


if __name__ == '__main__':
    a = APP()
    rate = rospy.Rate(100)
    a.root.mainloop()
    rospy.spin()