import numpy as np
import pandas as pd
import glob # usefull for listing all files of a type in a directory
import time
import yaml
import json
import warnings
from sklearn.cluster import KMeans
from sklearn.metrics import silhouette_score
import sys
import argparse
import os

def parsing():
    parser = argparse.ArgumentParser(description='Create quantitative and qualitative plots for user.') # create parser object
    parser.add_argument('requestID', type=str, action='store', default="requestID",
                        help='request ID') # store userID
    args = parser.parse_args()
    return args
class get_metrics():
    def __init__(self):
        # args = parsing()
        # self.requestID = args.requestID
        # self.mount_path = "/02_evaluation/mount/" + self.requestID + "/"
        self.mount_path = os.path.dirname(os.path.abspath(__file__)) # get path for current file, does not work if os.chdir() was used
        self.data_dir = os.path.dirname(self.mount_path) + "/01_recording/"
        self.now = time.strftime("%y-%m-%d_%H-%M-%S")

        # initialise log
        self.log = {
            "status": "INITIALISED",
            "start": self.now,
            "end": "",
            "stdout": "",
            "stderr": "",
            # "requestID": self.requestID,
        }

        try:
            self.read_config()
            self.log["status"] = "RUNNING"
            self.log["parameters"] = self.config
        except Exception as e:
            self.log_termination("INTERRUPTED", "CONFIG LOAD ERROR: " + repr(e))

    def read_config(self):
        with open(self.mount_path+"/get_metrics_config.yaml") as file:
            self.config = yaml.safe_load(file)

    def evaluate_data(self): # read in all csv files and compute metrics
        print("INFO: Start data transformation and evaluation: {}".format(time.strftime("%H:%M:%S")))
        data = {}
        files = glob.glob("{0}/*.csv".format(self.data_dir)) # get all the csv files paths in the directory
        planners = []
        if len(files) == 0:
            self.log_termination("INTERRUPTED", "NO FILES PROVIDED")
        for file in files: # summarize all the csv files and add to dictionary
            try:
                file_name = file.split("/")[-1].split("--")[:-1] # cut off date and time and .csv ending
                planners.append(file_name[0])
                file_name = "--".join(file_name) # join together to only include local planner, map and obstacle number
                print("-------------------------------------------------------------------------------------------------")
                print("INFO: Beginning data tranformation and evaluation for: {}".format(file_name))
                df = self.extend_df(pd.read_csv(file, converters = {"laser_scan":self.string_to_float_list, "action": self.string_to_float_list}))
                # df = self.drop_last_episode(df)
                if self.config["random_eval"]:
                    data[file_name] = {
                        "summary_df": self.get_summary_df(df).to_dict(orient = "list")
                    }
                else:
                    data[file_name] = {
                        "summary_df": self.get_summary_df(df).to_dict(orient = "list"),
                        "paths_travelled": self.get_paths_travelled(df),
                        "collision_zones": self.get_collision_zones(df)
                    }
                print("INFO: Data tranformation and evaluation finished for: {}".format(file_name))
            except Exception as e:
                self.log_termination("INTERRUPTED", "EVALUATION ERROR: " + repr(e))
        with open(self.mount_path+"/data_{}.json".format(self.now), "w") as outfile:
            json.dump(data, outfile)
            outfile.close()
        with open(self.mount_path+"/data_{}.txt".format(self.now), "w") as outfile:
            outfile.write("\n".join(list(np.unique(planners))))
            outfile.close()
        self.grab_data(files)
        print("-------------------------------------------------------------------------------------------------")
        print("INFO: End data transformation and evaluation: {}".format(time.strftime("%y-%m-%d_%H:%M:%S")))
        self.log_termination("TERMINATED")
 
    # move data from 01_recording into 02_evaluattion into a data folder with timestamp
    def grab_data(self, files):
        os.mkdir(self.mount_path+"/data_{}".format(self.now))
        for file in files:
            file_name = file.split("/")[-1]
            # move file from dir_path to data folder
            os.rename(self.data_dir+"/"+file_name, self.mount_path +
                      "/data_{}/".format(self.now)+file_name)

    def string_to_float_list(self,df_column): # convert list from csv saved as string to list of floats
        return list(np.array((df_column.replace("[","").replace("]","").split(", "))).astype(float))

    def extend_df(self,df):
        model = np.unique(df["model"])[0]
        with warnings.catch_warnings():
            warnings.simplefilter('ignore') 
            df["collision"] = self.get_collision(df,model)
            df["action_type"] = self.get_action_type(df)
            df["min_obstacle_distance"] = [np.round(np.nanmin(x),2) for x in df["laser_scan"]]
            df["normalized_curvature"] = self.get_curvature(df)
            df["path_smoothness"] = self.get_path_smoothness(df)
            df["velocity_smoothness"] = self.get_velocity_smoothness(df)
        return df

    def get_collision(self,df,model):
        raw_collision = list(np.any(np.less_equal(x,self.config["robot_radius"][model])) for x in df["laser_scan"])
        helper_collision = [False] + raw_collision[:-1]
        return [x > 0 for x in [r - h for r,h in zip(list(map(int, raw_collision)),list(map(int, helper_collision)))]]


    def get_action_type(self,df):
        action_type_column = []
        for x in df["action"]:
            if x[0] == 0.0 and x[1] == 0.0 and x[2] == 0.0:
                action_type_column.append("stop")
            elif x[0] == 0.0 and x[1] == 0.0 and x[2] != 0.0:
                action_type_column.append("rotate")
            else:
                action_type_column.append("move")
        return action_type_column
    
    def get_curvature(self,df):
        curvature_list = []
        episodes = np.unique(df["episode"])
        for episode in episodes:
            points = [list(x) for x in zip(df.loc[df["episode"]==episode,"robot_pos_x"],df.loc[df["episode"]==episode,"robot_pos_y"])]
            for i,point in enumerate(points):
                try:
                    x = np.array(point)
                    y = np.array(points[i+1])
                    z = np.array(points[i+2])
                    curvature_list.append(self.calc_curvature(x,y,z))
                    continue
                except:
                    curvature_list.append(np.nan)
                    continue
        return curvature_list

    def calc_curvature(self,x,y,z): # Menger curvature of 3 points
        # side lengths
        xy = np.linalg.norm(x-y)
        yz = np.linalg.norm(y-z)
        xz = np.linalg.norm(z-x)
        # herons formula for triangle area
        s = (xy + yz + xz)/2
        triangle_area = (s * (s-xy) * (s-yz) * (s-xz))**0.5
        # metric calculation
        if triangle_area == 0: # identical points or collinear
            norm_curvature = 0
        else:
            curvature = 4*triangle_area / (xy * yz * xz) # menger curvature
            norm_curvature = curvature / (xy + yz) # normalize by path length of the section
        return np.round(norm_curvature,2)

    def get_path_smoothness(self,df):
        path_smoothness_list = []
        episodes = np.unique(df["episode"])
        for episode in episodes:
            points = [list(x) for x in zip(df.loc[df["episode"]==episode,"robot_pos_x"],df.loc[df["episode"]==episode,"robot_pos_y"])]
            for i,point in enumerate(points):
                try:
                    x = np.array(point)
                    x_prev = np.array(points[i-1])
                    x_succ = np.array(points[i+1])
                    path_smoothness_list.append(self.calc_path_smoothness(x,x_prev,x_succ))
                    continue
                except:
                    path_smoothness_list.append(np.nan)
                    continue
        return path_smoothness_list

    def calc_path_smoothness(self,x,x_prev,x_succ):
        delta_1 = x-x_prev
        delta_2 = x_succ-x
        path_smoothness = np.linalg.norm(delta_2-delta_1)
        return np.round(path_smoothness)

    def get_velocity_smoothness(self,df):
        velocity_smoothness_list = []
        episodes = np.unique(df["episode"])
        for episode in episodes:
            velocities = [list(x) for x in zip(df.loc[df["episode"]==episode,"robot_lin_vel_x"],df.loc[df["episode"]==episode,"robot_lin_vel_y"])]
            times = list(df.loc[df["episode"]==episode,"time"])
            for i,vel in enumerate(velocities):
                try:
                    v1 = np.array(vel)
                    v2 = np.array(velocities[i+1])
                    t1 = np.array(times[i])
                    t2 = np.array(times[i+1])
                    velocity_smoothness_list.append(self.calc_velocity_smoothness(v1,v2,t1,t2))
                    continue
                except:
                    velocity_smoothness_list.append(np.nan)
                    continue
        return velocity_smoothness_list

    def calc_velocity_smoothness(self,v1,v2,t1,t2):
        velocity_smoothness = np.linalg.norm(v2-v1) / np.abs(t2-t1) # average of acceleration
        return np.round(velocity_smoothness)

    def drop_last_episode(self,df):
        episodes = np.unique(df["episode"])
        df = df.drop(df[df["episode"] == episodes[-1]].index)
        return df

    def get_summary_df(self,df): # NOTE: column specification hardcoded !
        sum_df = df.groupby(["episode"]).sum()
        mean_df = df.groupby(["episode"]).mean()
        summary_df = mean_df
        summary_df["time"] = self.get_time(df)
        summary_df["collision"] = sum_df["collision"]
        summary_df["path_smoothness"] = sum_df["path_smoothness"]
        summary_df["path_length"] = self.get_path_length(df)
        summary_df["success"],summary_df["done_reason"]  = self.get_success(summary_df)
        summary_df = summary_df.drop(columns = ['robot_lin_vel_x', 'robot_lin_vel_y', 'robot_ang_vel', 'robot_orientation', 'robot_pos_x', 'robot_pos_y'])
        return summary_df
    
    def get_time(self,df):
        time_list = []
        episodes = np.unique(df["episode"])
        for episode in episodes:
            times = list(df.loc[df["episode"]==episode,"time"])
            time_list.append(np.round(times[-1]-times[0],2))
        return time_list

    def get_path_length(self,df):
        path_length_list = []
        episodes = np.unique(df["episode"])
        for episode in episodes:
            path_length = 0
            points = list(zip(df.loc[df["episode"]==episode,"robot_pos_x"],df.loc[df["episode"]==episode,"robot_pos_y"]))
            for i,point in enumerate(points):
                if i == 0:
                    continue
                else:
                    path_length = path_length + np.linalg.norm(np.array(point)-np.array(points[i-1]))
            path_length_list.append(np.round(path_length,2))
        return path_length_list

    def get_success(self,summary_df):
        success_list = []
        done_reason_list = []
        for episode in summary_df.index:
            if summary_df.loc[episode,"collision"] > self.config["collision_treshold"]:
                success_list.append(False)
                done_reason_list.append("collision")
            elif summary_df.loc[episode,"time"] > self.config["time_out_treshold"]:
                success_list.append(False)
                done_reason_list.append("time_out")
            else:
                success_list.append(True)
                done_reason_list.append("goal_reached")
        return success_list, done_reason_list

    def get_paths_travelled(self,df):
        paths_travelled = {}
        episodes = np.unique(df["episode"])
        for episode in episodes:
            paths_travelled[str(episode)] = list(zip(df.loc[df["episode"]==episode,"robot_pos_x"],df.loc[df["episode"]==episode,"robot_pos_y"]))
        return paths_travelled

    def get_collision_zones(self,df):
        collisions = df.loc[df["collision"]==True,["robot_pos_x","robot_pos_y","collision"]]
        points = [list(x) for x in list(zip(collisions["robot_pos_x"],collisions["robot_pos_y"]))]

        silhouette_score_list = []
        with warnings.catch_warnings():
            warnings.simplefilter('ignore') 
            kmax = len(points)-1
            if len(points) <= 3:
                return {"centroids": [], "counts": [], "collisions": []}
            for k in range(2, kmax+1):
                kmeans = KMeans(n_clusters = k).fit(points)
                labels = kmeans.labels_
                silhouette_score_list.append(silhouette_score(points, labels, metric = 'euclidean'))
            best_k = np.argmax(silhouette_score_list) + 2 # kmeans here starts at 2 centroids so argmax 0 equals 2 centroids
            kmeans = KMeans(n_clusters = best_k).fit(points)
            centroids = kmeans.cluster_centers_
            _ , counts = np.unique(kmeans.labels_, return_counts=True)
        return {"centroids": centroids.tolist(), "counts": counts.tolist(), "collisions": collisions.values.tolist()}

    def log_termination(self, status = "TERMINATED", reason = ""):
        self.log["status"] = status
        self.log["end"] = time.strftime("%y-%m-%d_%H-%M-%S")
        self.log["stdout"] = "Process terminated at {}.".format(self.log["end"])
        if status != "TERMINATED":
            self.log["stderr"] = "{}".format(reason)
        # with open(self.mount_path+"/log_" + self.requestID + ".json", "w") as outfile:
        #     json.dump(self.log, outfile, indent=4)
        #     outfile.close()
        sys.exit()

if __name__=="__main__":
    metrics = get_metrics()
    metrics.evaluate_data()