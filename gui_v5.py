#!/usr/bin/env python3
# license removed for brevity
import subprocess
from tkinter import ttk
from tkinter import *
import numpy as np
import rospy
from std_msgs.msg import Float32MultiArray, Int32MultiArray
from functools import partial
import cv2
from PIL import Image, ImageTk
import threading
import pyrealsense2 as rs

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


def button1_clicked():
    # if not started[0]:
    t = threading.Thread(target=videoLoop, args=(videoloop_stop,))
    t.start()
    threads[0].append(t)
    started[0] = True


# else:
#     videoloop_stop[0] = False


def button2_clicked():
    videoloop_stop[0] = -1


def button3_clicked():
    # if not started[1]:
    t = threading.Thread(target=videoDepthLoop, args=(videoloop_stop,))
    t.start()
    threads[1].append(t)
    started[1] = True


# else:
#     videoloop_stop[1] = False
#

def button4_clicked():
    videoloop_stop[1] = -1


def videoDepthLoop(mirror=False):
    if videoloop_stop[0] == 1:
        # if switcher tells to stop then we switch it again and stop videoloop
        # for t in threads[0]:
        #     t.join()
        videoloop_stop[0] = -1
    No = 1
    # pipe = rs.pipeline()
    cap1 = cv2.VideoCapture(No)
    cap1.set(cv2.CAP_PROP_FRAME_WIDTH, 800)
    cap1.set(cv2.CAP_PROP_FRAME_HEIGHT, 600)
    colorizer = rs.colorizer()
    cfg = rs.config()
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
            panel.destroy()
            cap1.release()

            break
        videoloop_stop[1] = 1
        # Store next frameset for later processing:
        frameset = pipe.wait_for_frames()
        depth_frame = frameset.get_depth_frame()
        # Render images
        colorized_depth = np.asanyarray(colorizer.colorize(depth_frame).get_data())
        img = Image.fromarray(colorized_depth)
        image = ImageTk.PhotoImage(image=img)
        panel = Label(image=image)
        panel.image = image
        panel.place(x=50, y=50)
        # check switcher value


def videoLoop(mirror=False):
    if videoloop_stop[1] == 1:
        # if switcher tells to stop then we switch it again and stop videoloop
        # for t in threads[1]:
        #     t.join()
        videoloop_stop[1] = -1
    No = 1
    # pipe = rs.pipeline()
    cap1 = cv2.VideoCapture(No)
    cap1.set(cv2.CAP_PROP_FRAME_WIDTH, 800)
    cap1.set(cv2.CAP_PROP_FRAME_HEIGHT, 600)
    cfg = rs.config()
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
            panel.destroy()
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
        panel = Label(image=image)
        panel.image = image
        panel.place(x=50, y=50)
        # check switcher value




class APP:

    def __init__(self):
        # ROS
        rospy.init_node("GUI", anonymous=True)
        self.motor_pwm_pub = rospy.Publisher("/robot_snake_4/motor_cmd", Int32MultiArray, queue_size=10)
        self.joint_cmd_pub = rospy.Publisher("/robot_snake_10/joint_cmd", Float32MultiArray, queue_size=10)
        rospy.Subscriber("/robot_snake_1/tension_val", Float32MultiArray, self.tension_val_update)
        rospy.Subscriber("/robot_snake_10/joint_val", Float32MultiArray, self.joint_val_update)

        # tkinter
        self.root = Tk()
        self.root.title("Tab Widget")
        self.tabControl = ttk.Notebook(self.root)

        self.tab1 = ttk.Frame(self.tabControl)
        self.tab1.pack(fill=BOTH, expand=1)
        self.tab2 = ttk.Frame(self.tabControl)
        self.tab2.pack(fill=BOTH, expand=1)
        self.tab3 = ttk.Frame(self.tabControl)
        self.tab3.pack(fill=BOTH, expand=1)
        self.tab4 = ttk.Frame(self.tabControl)
        self.tab4.pack(fill=BOTH, expand=1)

        self.tabControl.add(self.tab1, text='Tab 1')
        self.tabControl.add(self.tab2, text='Tab 2')
        self.tabControl.add(self.tab3, text='Tab 3')
        self.tabControl.add(self.tab4, text='Tab 4')
        self.tabControl.pack(expand=1, fill="both")

        self.my_canvas_tab1 = Canvas(self.tab1)
        self.my_canvas_tab2 = Canvas(self.tab2)
        self.my_canvas_tab3 = Canvas(self.tab3)
        self.my_canvas_tab4 = Canvas(self.tab4)

        self.my_canvas_tab1.pack(side=TOP, fill=BOTH, expand=1)
        self.my_canvas_tab2.pack(side=TOP, fill=BOTH, expand=1)
        self.my_canvas_tab3.pack(side=TOP, fill=BOTH, expand=1)
        self.my_canvas_tab4.pack(side=TOP, fill=BOTH, expand=1)

        # y_scrollbar.pack(side=RIGHT, fill=Y)

        self.frame_tab1 = Frame(self.my_canvas_tab1, width=1000, height=10000)
        self.frame_tab2 = Frame(self.my_canvas_tab2, width=1000, height=10000)
        self.frame_tab3 = Frame(self.my_canvas_tab3, width=1000, height=10000)
        self.frame_tab4 = Frame(self.my_canvas_tab4, width=1000, height=10000)

        self.my_canvas_tab1.create_window(500, 38 * N_motor, window=self.frame_tab1)
        self.my_canvas_tab2.create_window(700, 34 * N_joints, window=self.frame_tab2)
        self.my_canvas_tab3.create_window(700, 34 * N_joints, window=self.frame_tab3)
        self.my_canvas_tab4.create_window(700, 34 * N_joints, window=self.frame_tab4)

        self.y_scrollbar_tab1 = ttk.Scrollbar(self.my_canvas_tab1, orient=VERTICAL, command=self.my_canvas_tab1.yview)
        self.y_scrollbar_tab2 = ttk.Scrollbar(self.my_canvas_tab2, orient=VERTICAL, command=self.my_canvas_tab2.yview)
        self.y_scrollbar_tab3 = ttk.Scrollbar(self.my_canvas_tab3, orient=VERTICAL, command=self.my_canvas_tab3.yview)
        self.y_scrollbar_tab4 = ttk.Scrollbar(self.my_canvas_tab4, orient=VERTICAL, command=self.my_canvas_tab4.yview)

        self.my_canvas_tab1.config(yscrollcommand=self.y_scrollbar_tab1.set, scrollregion=(0, 0, 1000, 2000))
        self.my_canvas_tab2.config(yscrollcommand=self.y_scrollbar_tab2.set, scrollregion=(0, 0, 1000, 2000))
        self.my_canvas_tab3.config(yscrollcommand=self.y_scrollbar_tab3.set, scrollregion=(0, 0, 1000, 2000))
        self.my_canvas_tab4.config(yscrollcommand=self.y_scrollbar_tab3.set, scrollregion=(0, 0, 1000, 2000))

        self.y_scrollbar_tab1.pack(side=RIGHT, fill=Y)
        self.y_scrollbar_tab2.pack(side=RIGHT, fill=Y)
        self.y_scrollbar_tab3.pack(side=RIGHT, fill=Y)
        self.y_scrollbar_tab4.pack(side=RIGHT, fill=Y)

        self.tab3_button1 = Button(
            self.my_canvas_tab3, text="start", bg="#fff", font=("", 20),
            command=lambda: button1_clicked())
        self.tab3_button1.place(x=350, y=750, width=200, height=100)
        self.tab3_button2 = Button(
            self.my_canvas_tab3, text="stop", bg="#fff", font=("", 20),
            command=lambda: button2_clicked())
        self.tab3_button2.place(x=150, y=750, width=200, height=100)

        self.tab4_button3 = Button(
            self.my_canvas_tab4, text="start", bg="#fff", font=("", 20),
            command=lambda: button3_clicked())
        self.tab4_button3.place(x=350, y=750, width=200, height=100)
        self.tab4_button4 = Button(
            self.my_canvas_tab4, text="stop", bg="#fff", font=("", 20),
            command=lambda: button4_clicked())
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

        self.homing_process = ''
        self.homing_button = Button(self.frame_tab1, text="Homing", bg="#fff", font=("", 20),
                                    command=partial(self.launch_homing))
        self.homing_button.grid(column=1, row=0)
        self.homing_kill_button = Button(self.frame_tab1, text="Kill Homing", bg="#fff", font=("", 20),
                                         command=partial(self.kill_homing))
        self.homing_kill_button.grid(column=2, row=0)
        self.home_position()

        # while not rospy.is_shutdown():
        #     rospy.Rate(100).sleep()

    def launch_homing(self):
        self.homing_process = subprocess.Popen("rosrun robot_snake_4 controller_v4_3".split(), shell=False)

    def kill_homing(self):
        self.homing_process.kill()
        self.homing_process.terminate()

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
