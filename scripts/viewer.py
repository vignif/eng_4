import argparse
import os
import tkinter as tk
import rosbag
from cv_bridge import CvBridge
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import (FigureCanvasTkAgg, NavigationToolbar2Tk) 
import cv2
import numpy as np

from engage.rosbag_reader import RosbagReader

class Viewer:
    def __init__(
            self,
            exp,
            bag_dir,
            image_topic,
            pose_image_topic,
            button_height = 50,
            button_width = 50,
    ) -> None:
        # Parameters
        self.bag_dir = bag_dir
        self.log_bag_dir = bag_dir + "/logbags"
        self.image_topic = image_topic
        self.pose_image_topic = pose_image_topic
        self.bridge = CvBridge()

        # Settings
        self.pose_video = True
        self.display_ids = True
        self.decision_widget_created = False
        self.query_widget_created = False
        self.explanation_widget_created = False

        self.use_tracked = False

        # Create TK root
        self.root = tk.Tk()

        # Initialise rosbag
        self.init_bag(exp)

        # Setup GUI
        self.button_height = button_height
        self.button_width = button_width

        buffer_w = 10
        buffer_h = 40
        w = self.img_width + buffer_w
        h = self.img_height + buffer_h
        graph_width = 500
    
        self.header_canvas = tk.Canvas(self.root,width=w, height=button_height)
        self.header_canvas.grid(row=0,column=0)
        self.video_canvas = tk.Canvas(self.root, width=w, height=h)
        self.video_canvas.grid(row=1,column=0)
        self.graph_canvas = tk.Canvas(self.root,width=graph_width,height=h)
        self.graph_canvas.grid(row=1,column=1)
        self.settings_canvas = tk.Canvas(self.root,width=w,height=50)
        self.settings_canvas.grid(row=2,column=0)
        self.decision_canvas = tk.Canvas(self.root,width=graph_width,height=50)
        self.decision_canvas.grid(row=2,column=1)
        self.query_canvas = tk.Canvas(self.root,width=w,height=h)
        self.query_canvas.grid(row=3,column=0)
        self.explanation_canvas = tk.Canvas(self.root,width=graph_width,height=h)
        self.explanation_canvas.grid(row=3,column=1)
        self.root.protocol("WM_DELETE_WINDOW", self.close)

        # Header GUI
        self.create_header(exp)

        # Graph
        self.graph_choice = None
        self.create_graph()
        
        # Video player
        self.update_frame(0)
        self.create_video_slider()

        # Settings
        self.create_settings()

        # Decision
        self.create_decision_view()

        # Explanations
        self.create_explanation_windows()

    def close(self):
        self.root.quit()
        self.root.destroy()
        quit()

    def clear_canvases(self):
        self.video_canvas.pack_forget()
        self.graph_canvas.pack_forget()
        self.figure_canvas.get_tk_widget().destroy()
        self.graph_dropdown.destroy()

    '''
    SETUP
    '''
    def init_bag(self,exp):
        self.exp = exp
        
        try:
            self.cam_bag = rosbag.Bag("{}/{}.bag".format(self.bag_dir,self.exp))
        except:
            self.cam_bag = None
        self.log_bag = rosbag.Bag("{}/{}.bag".format(self.log_bag_dir,self.exp))

        self.load_video()
        self.root.title(exp)

        # Set up bagreader
        self.bagreader = RosbagReader(self.cam_bag,self.log_bag)

    def load_video(self):
        self.msgs = []
        self.times = []
        self.stamps = []
        if self.pose_video:
            for _,msg,t in self.log_bag.read_messages(topics=[self.pose_image_topic]):
                self.msgs.append(msg)
                self.times.append(t)
                self.stamps.append(msg.header.stamp) # Use stamp time as that is what is used to synch everything

            if len(self.msgs) == 0:
                # No annotated pose images, must draw poses instead
                self.use_tracked = True
                self.read_tracked_messages()
            else:
                self.use_tracked = False

                self.img_height = self.msgs[0].height
                self.img_width = self.msgs[0].width
        else:
            if self.cam_bag is not None:
                self.use_tracked = False
                for _,msg,t in self.cam_bag.read_messages(topics=[self.image_topic]):
                    self.msgs.append(msg)
                    self.times.append(t)
                    self.stamps.append(msg.header.stamp) # Use stamp time as that is what is used to synch everything

                self.img_height = self.msgs[0].height
                self.img_width = self.msgs[0].width
            else:
                self.use_tracked = True
                self.read_tracked_messages()
        
        self.num_frames = len(self.times)

    def read_tracked_messages(self):
        for _,msg,t in self.log_bag.read_messages(topics=["/humans/bodies/tracked"]):
            self.msgs.append(None)
            self.times.append(t)
            self.stamps.append(msg.header.stamp)
        self.img_height = 480
        self.img_width = 640

    def create_header(self,exp):
        # Create list
        onlyfiles = [f for f in os.listdir(self.log_bag_dir) if os.path.isfile(os.path.join(self.log_bag_dir, f))]
        logbags = sorted([x[:-4] for x in onlyfiles if x.endswith(".bag")])
        print("{} logbags available".format(len(logbags)))
        if exp not in logbags:
            error_message = "{} is not a valid rosbag file detected in {}".format(exp,self.log_bag_dir)
            raise Exception(error_message)
        # Create dropdown
        logbag_choice = tk.StringVar(self.header_canvas,name="logbag_choice")
        logbag_choice.set(exp)
        logbag_choice.trace("w", self.update_logbag)
        self.logbag_dropdown = tk.OptionMenu(self.header_canvas, logbag_choice, *logbags)
        self.logbag_dropdown.pack()

    def create_settings(self):
        adjusted_button_width = 3*self.button_width
        self.save_button = tk.Button(self.settings_canvas,text="Save", width=12, command=self.save_image)
        self.save_button.place(x=0, y=0,width=adjusted_button_width,height=self.button_height)

        self.pose_button = tk.Button(self.settings_canvas,text="Hide Skeleton", width=12, command=self.toggle_skeleton)
        self.pose_button.place(x=adjusted_button_width, y=0,width=adjusted_button_width,height=self.button_height)

        self.ids_button = tk.Button(self.settings_canvas,text="Hide IDs", width=12, command=self.toggle_ids)
        self.ids_button.place(x=adjusted_button_width*2, y=0,width=adjusted_button_width,height=self.button_height)

    def create_decision_view(self):
        self.decision_widget_created = True
        self.decision_text = tk.Label(self.decision_canvas,text="Decision",anchor="w")
        self.decision_text.place(x=0,y=self.button_height/2)

        self.update_decision_view()

    def create_video_slider(self):
        self.playing = False
        self.curr_frame = 0
        self.play_button = tk.Button(self.video_canvas,text="Play",command=self.play_pause)
        self.play_button.place(x=0, y=self.img_height,width=self.button_width,height=self.button_height)

        frame_index = tk.IntVar()
        self.img_scale = tk.Scale(self.video_canvas, from_=0, to=self.num_frames-1, orient=tk.HORIZONTAL, variable=frame_index, length=self.img_width-(2*self.button_width)-1, command=self.update_frame)
        self.img_scale.place(x=self.button_width+1, y=self.img_height)

        self.backward_button = tk.Button(self.video_canvas,text="<",command=self.backward_one_frame)
        self.backward_button.place(x=self.img_width-(self.button_width) + 1, y=self.img_height,width=self.button_width/2,height=self.button_height)

        self.forward_button = tk.Button(self.video_canvas,text=">",command=self.forward_one_frame)
        self.forward_button.place(x=self.img_width-(self.button_width/2) + 1, y=self.img_height,width=self.button_width/2,height=self.button_height)

    def create_explanation_windows(self):
        #TODO
        pass

    def update_logbag(self,n,m,x):
        exp = self.root.getvar(n)
        self.init_bag(exp)
        self.clear_canvases()
        self.create_graph()

        # Video player
        self.update_frame(0)
        self.create_video_slider()

    def create_graph(self,default_graph="Decision"):
        # Get dropdown list
        self.graph_list = self.bagreader.grapher.graph_choices

        if self.graph_choice is None:
            self.graph_choice = default_graph
        self.graph_fig, self.graph_axes = self.bagreader.plot(self.graph_choice)
        plt.tight_layout()

        curr_time = self.stamps[0].secs + self.stamps[0].nsecs/1000000000
        self.graph_axes[0].axvline(x=curr_time)

        
        self.figure_canvas = FigureCanvasTkAgg(self.graph_fig, master=self.graph_canvas)

        # Create dropdown widget
        graph_choice = tk.StringVar(self.graph_canvas,name="graph_choice")
        graph_choice.set(self.graph_choice)
        graph_choice.trace("w", self.update_graph)
        self.graph_dropdown = tk.OptionMenu(self.graph_canvas, graph_choice, *self.graph_list)
        self.graph_dropdown.pack()

        self.figure_canvas.draw()
        self.figure_canvas.get_tk_widget().pack()

    def update_graph(self,n,m,x):
        
        self.graph_choice = self.root.getvar(n)
        
        self.graph_fig, self.graph_axes = self.bagreader.plot(self.graph_choice)
        plt.tight_layout()

        self.figure_canvas.get_tk_widget().destroy()
        self.figure_canvas = FigureCanvasTkAgg(self.graph_fig, master=self.graph_canvas)
        curr_time = self.stamps[self.curr_frame].secs + self.stamps[self.curr_frame].nsecs/1000000000
        for ax in self.graph_axes:
            ax.axvline(x=curr_time,color='r')
        self.figure_canvas.draw()
        self.figure_canvas.get_tk_widget().pack(expand=True)

        self.update_decision_view()

    def update_frame(self,frame_index):
        self.curr_frame = int(frame_index)
        
        if self.use_tracked:
            self.cv_img = np.zeros((self.img_height,self.img_width,3), np.uint8)
            if self.pose_video:
                self.cv_img = self.bagreader.draw_poses(self.cv_img,self.stamps[self.curr_frame])
                self.cv_img = cv2.cvtColor(self.cv_img, cv2.COLOR_BGR2RGB)
        else:
            self.cv_img = self.bridge.imgmsg_to_cv2(self.msgs[self.curr_frame], desired_encoding="passthrough")
            if not self.pose_video:
                self.cv_img = cv2.cvtColor(self.cv_img, cv2.COLOR_BGR2RGB)
        self.cv_img = self.process_image(self.cv_img)
        photo = self.photo_image(self.cv_img)
        self.video_canvas.create_image(0, 0, image=photo, anchor=tk.NW)
        self.video_canvas.image = photo

        # Update graph
        curr_time = self.stamps[self.curr_frame].secs + self.stamps[self.curr_frame].nsecs/1000000000
        for ax in self.graph_axes:
            ax.get_lines().pop().remove()
            ax.axvline(x=curr_time,color='r')
        self.figure_canvas.draw()

        # Update decision
        if self.decision_widget_created:
            self.update_decision_view()

        # Update explanation stuff
        if self.query_widget_created:
            self.update_query_window()

    def process_image(self,img):
        img = img.copy()
        if self.display_ids:
            label_positions = self.bagreader.get_face_locations(self.stamps[self.curr_frame].to_sec())
            for id in label_positions:
                if label_positions[id] is not None:
                    org = (int(img.shape[1]*label_positions[id][0]),int(img.shape[0]*label_positions[id][1]))
                    colour = list(self.bagreader.colours[id])
                    col = colour.copy()
                    col[0] = colour[2]*255
                    col[1] = colour[1]*255
                    col[2] = colour[0]*255
                    
                    img = cv2.putText(img, id, org, cv2.FONT_HERSHEY_SIMPLEX ,  1, col, 2, cv2.LINE_AA)
        return img
    
    def photo_image(self,img):
        h, w = img.shape[:2]
        data = f'P6 {w} {h} 255 '.encode() + img[..., ::-1].tobytes()
        return tk.PhotoImage(width=w, height=h, data=data, format='PPM')
    
    def play_pause(self):
        if self.playing:
            # Pause
            self.playing = False
            self.play_button.config(text="Play")
        else:
            # Play
            self.playing = True
            self.play_button.config(text="Pause")
            if self.curr_frame == self.num_frames:
                # Restart video if at the end
                self.curr_frame = 0
            self.update()

    def forward_one_frame(self):
        if self.curr_frame != self.num_frames - 1:
            self.curr_frame += 1
            self.update_frame(self.curr_frame)
            self.img_scale.set(self.curr_frame)

    def backward_one_frame(self):
        if self.curr_frame != 0:
            self.curr_frame -= 1
            self.update_frame(self.curr_frame)
            self.img_scale.set(self.curr_frame)


    def update(self):
        if self.playing:
            if self.curr_frame < self.num_frames:
                self.update_frame(self.curr_frame)
                self.img_scale.set(self.curr_frame)

                self.curr_frame += 1
                self.root.after(1, self.update)
            else:
                # Toggle back to pause after video ends
                self.play_pause()


    def update_decision_view(self):
        #TODO
        pass

    def save_image(self):
        f = tk.filedialog.asksaveasfile(mode='w', defaultextension=".png")
        if f is None: # asksaveasfile return `None` if dialog closed with "cancel".
            return
        cv2.imwrite(f.name,self.cv_img)
        print("Saved to {}".format(f.name))
        f.close()

    def toggle_skeleton(self):
        self.pose_video = not self.pose_video
        if self.pose_video:
            self.pose_button.config(text="Hide Skeleton")
        else:
            self.pose_button.config(text="Show Skeleton")

        old_time = self.stamps[self.curr_frame]
        self.load_video()
        self.curr_frame = np.argmin(np.abs(np.array(self.stamps)-old_time)) # Frames don't correspond between image and pose_image
        self.update_frame(self.curr_frame)

    def toggle_ids(self):
        self.display_ids = not self.display_ids
        if self.display_ids:
            self.ids_button.config(text="Hide IDs")
        else:
            self.ids_button.config(text="Show IDs")
        self.update_frame(self.curr_frame)

    def toggle_face_blur(self):
        self.blur_faces = not self.blur_faces
        if self.blur_faces:
            self.blur_faces_button.config(text="Show Faces")
        else:
            self.blur_faces_button.config(text="Blur Faces")
        self.update_frame(self.curr_frame)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Visualise rosbag for HRI engage experiments")
    parser.add_argument("--exp", help="Name of rosbag (without .bag)", default="Approach_0")
    parser.add_argument("--bag_dir", help="Directory of bag file", default=os.getenv("HOME")+"/engage/src/engage/rosbags")
    parser.add_argument("--image_topic", help="Image topic.", default="/camera/color/image_raw")
    parser.add_argument("--pose_image_topic", help="Pose image topic.", default="/opendr/pose_img")

    args = parser.parse_args()

    vis = Viewer(args.exp,args.bag_dir,args.image_topic,args.pose_image_topic)
    vis.root.mainloop()