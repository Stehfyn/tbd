import os
import sys

currentdir = os.path.dirname(os.path.abspath(__file__))
q1adir = os.path.abspath(currentdir + '/../../q1a/scripts')
sys.path.append(q1adir)

from ScanContext.ScanContext import ScanContext

import matplotlib
matplotlib.use('agg')
import matplotlib.pyplot as plt
import matplotlib.lines as mlines
from sklearn.cluster import AgglomerativeClustering, DBSCAN, HDBSCAN
from scipy.spatial import ConvexHull

import numpy as np
np.random.seed(0)
import pandas as pd

def shift(seq, n):
    n = n % len(seq)
    return seq[n:] + seq[:n]

def calc_opening_coords (lidar_points_in_rect):
    lidar_points_in_rect = lidar_points_in_rect.sort_values("angles")
    lidar_points_in_rect ["angular_diff"] = lidar_points_in_rect ["angles"].diff()
    max_diff_index = lidar_points_in_rect ["angular_diff" ].argmax()

    test = plt.plot(list(lidar_points_in_rect["curr_xs"].iloc[max_diff_index -1: max_diff_index +1]),
    list(lidar_points_in_rect["curr_ys"].iloc[max_diff_index -1: max_diff_index +1]), c = "g",linewidth =2)

    test = plt.scatter(lidar_points_in_rect ["curr_xs"].iloc[max_diff_index-1: max_diff_index +1],
    lidar_points_in_rect["curr_ys"].iloc[max_diff_index-1: max_diff_index +1], s =5 , color = "r")
    
    opening_coords1 = (lidar_points_in_rect["curr_xs"].iloc[max_diff_index -1], lidar_points_in_rect["curr_ys"].iloc[max_diff_index -1])
    opening_coords2 = (lidar_points_in_rect["curr_xs"].iloc[max_diff_index], lidar_points_in_rect ["curr_ys"].iloc[max_diff_index])
    ax = plt.gca()

    c1x = "{:.2f}".format(opening_coords1[0])
    c1y = "{:.2f}".format(opening_coords1[1])
    c2x = "{:.2f}".format(opening_coords2[0])
    c2y = "{:.2f}".format(opening_coords2[1])
    
    owidth = ((opening_coords1 [0] - opening_coords2 [0]) **2 + ( opening_coords1 [1] -opening_coords2 [1]) **2) **(1/2)
    ows = "Size: {:.2f} m".format(owidth)
    print("Opening " + ows)

    ax.annotate(", ".join([c1x, c1y]), (opening_coords1[0], opening_coords1[1]))
    ax.annotate(", ".join([c2x, c2y]), (opening_coords2[0], opening_coords2[1]))
    ax.annotate(ows, (0,0))
    
    return opening_coords1 , opening_coords2
    
def get_random_color(color_type = "light"):
    if color_type == "dark":
        return (np.random.choice(range(85), size=3))/255
    elif color_type == "medium":
        return (85 + np.random.choice(range(85), size=3))/255
    elif color_type == "light":
        return (170 + np.random.choice(range(85), size=3))/255
    else:
        return (np.random.choice(range(256), size=3))/255

def line_intersect(m1, c1, m2, c2):
    x = ( c2 - c1 ) / ( m1 - m2 )
    y = m1 * x + c1
    return {"curr_xs" : x , "curr_ys" : y}

def in_hull(points, hull, epsilon = 0):
    A = hull.equations
    dist = np.array(points[["curr_xs", "curr_ys"]]) @ A [: ,:2]. T + A[: ,2]
    return points.loc[np.all(dist < epsilon , axis=1) == True]

def estimate_wls_parameters (dists , sigmas):
    y = []
    H = []
    R_diag_arr = []
    for i, d in enumerate(dists):
        y_d = d ["curr_ys"].values.reshape(( -1 , 1))
        y.append(y_d)
        x_d = np.array(d["curr_xs"])
        # H is ( len (y) , 2) matrix [ curr_xs , 1]
        H.append(np.array([x_d, np.ones_like(x_d)]).T)
        sigma_vector = np.ones_like(x_d) * (1/ sigmas[i])
        R_diag_arr.append(sigma_vector **2)# sigmas [i] * np.ones_like(x_d))
    
    y_all = np.vstack(y)
    H_all = np.vstack(H)
    R_all = np.squeeze(np.hstack(R_diag_arr))
    H_R_inv = np.matmul(H_all.T, np.linalg.inv(np.diag(R_all)))
    # print ( np . linalg . inv ( np . diag ( R_all )))
    # print ( H_R_inv , H_all )
    xhat_wls = np.matmul(np.matmul(np.linalg.inv(np.matmul(H_R_inv, H_all)), H_R_inv), y_all)
    # print (" The Weighted Least Squares soln . for x_hat ( [m , c ]^ T) is : ", xhat_wls )
    return xhat_wls

class Visualiser:
    def __init__(self):
        self.clean_cycle=50
        self.frame_num=0
        self.angles_data, self.ranges_data=np.array([]), np.array([])
        self.x_data, self.y_data=np.array([]),np.array([])
        #plt.xlim([-300,300])
        #plt.ylim([-300,300])
        plt.xlim([-5,5])
        plt.ylim([-5,5])
        #plt.savefig("Current_Environment_map.png")
        
    def calculate_cartesian_coordinates(self):
        self.polar_df=self.polar_df.groupby(['angles']).agg(list).reset_index()
        self.polar_df["ranges"]=self.polar_df["ranges"].apply(lambda x: np.median(x))
        #self.polar_df["angles"]=self.polar_df["angles"] + np.pi / 2
        self.polar_df["curr_xs"]=(self.polar_df["ranges"]*np.cos(self.polar_df["angles"]))#.astype(int)
        self.polar_df["curr_ys"]=(self.polar_df["ranges"]*np.sin(self.polar_df["angles"]))#.astype(int)
        #self.polar_df["curr_xs"] = self.polar_df["curr_xs"] * -1
        #self.polar_df["curr_ys"] = self.polar_df["curr_ys"] * -1
        print(self.polar_df)

    def find_enclosure_cluster(self):
        self.x_y_contour=np.array([self.polar_df["curr_xs"],self.polar_df["curr_ys"]]).T
        clustering=DBSCAN(eps=.025,min_samples=10).fit(self.x_y_contour)
        self.polar_df["cluster_num"]=clustering.labels_
        point_cluster_labels=self.polar_df["cluster_num"].unique()
        closest_cluster=None
        origin_point_df=pd.DataFrame({"curr_xs":[0],"curr_ys":[0]})
        for i in point_cluster_labels:
            self.cluster_polar_df=self.polar_df[self.polar_df["cluster_num"]==i]
            if len(in_hull(origin_point_df,ConvexHull(self.cluster_polar_df[["curr_xs","curr_ys"]])))==1:
                closest_cluster=i
                break
        self.closest_polar_df=self.polar_df[self.polar_df["cluster_num"]==closest_cluster].sort_values("angles")
        plt.scatter(self.closest_polar_df["curr_xs"],self.closest_polar_df["curr_ys"],c='#1f77b4',s=1)
    
    def find_rectangle_corners(self):
        """Findcornersusingpeaksintherangescolumnofthedf"""
        corner_lidar_points=self.closest_polar_df.loc[self.closest_polar_df['ranges']==self.closest_polar_df['ranges'].rolling(250,center=True).max()]
        corner_lidar_points=corner_lidar_points.groupby('ranges').apply(lambda df: df.sample(1)) #toselectonepointamongothers
        corner_lidar_points=corner_lidar_points.reset_index(drop=True).sort_values("angles")
        return corner_lidar_points

    def discard_false_corners(self,corner_lidar_points):
        cyclic_corner_lidar_points=pd.concat([corner_lidar_points,corner_lidar_points.iloc[0:2]])
        cyclic_corner_lidar_points["slopes"]=(
            cyclic_corner_lidar_points["curr_ys"].diff()/
                cyclic_corner_lidar_points["curr_xs"].diff()
                    ).shift(-1)#calculatingslopeofeachline
        corner_lidar_points=cyclic_corner_lidar_points[abs(cyclic_corner_lidar_points['slopes'].diff())>5e-2].reset_index(drop=True)
        return corner_lidar_points

    def plot_m_c_line(self,x,y,m,c,line_length=3000):
        rcolor=get_random_color(color_type="medium")#np.random.rand(3,)
        if abs(m)<=1: #iflinemorehorizontal,plotx+-line_length
            plt.plot([x-line_length,x+line_length],[m*(x-line_length)+c,m*(x+line_length)+c],color=rcolor,linewidth=0.5)
        elif abs(m)>1: #iflinemorevertical,ploty+-line_length
            plt.plot([((y-line_length)-c)/m,((y+line_length)-c)/m],[y-line_length,y+line_length],color=rcolor,linewidth=0.5)
        return

    def find_line_equations(self,corner_lidar_points):
        #plt.scatter(corner_lidar_points["curr_xs"],corner_lidar_points["curr_ys"],s=5,color="k")
        self.lines_eqn_params=pd.DataFrame()
        corner_lidar_points=corner_lidar_points.sort_values("angles")
        corner_lidar_points=pd.concat([corner_lidar_points,corner_lidar_points.iloc[0:2]]).reset_index(drop=True)
        for ind, row in corner_lidar_points.iterrows():
            if ind==len(corner_lidar_points)-1:break
            a_line_df=self.closest_polar_df[self.closest_polar_df['angles'].between(
                row["angles"],corner_lidar_points.iloc[ind+1]["angles"])]
            if len(a_line_df)<500:
                a_line_df=self.closest_polar_df[~self.closest_polar_df['angles'].between(
                    corner_lidar_points.iloc[ind+1]["angles"],row["angles"])]
            if len(a_line_df)<500:
                print("====")
                print("If not even 100 points lie on this line,the enclosure is probably not a valid rectangle")
                print("====")
                continue

            required_parameters_xhat=estimate_wls_parameters([a_line_df],sigmas=[20/10])#+/-20mm
            #required_parameters_xhat=estimate_wls_parameters([a_line_df],sigmas=[8698/10])#+/-20mm
            self.lines_eqn_params=pd.concat([self.lines_eqn_params,
                pd.DataFrame({"m":required_parameters_xhat[0],"c":required_parameters_xhat[1]})])
            self.plot_m_c_line(row["curr_xs"],row["curr_ys"],required_parameters_xhat[0][0],required_parameters_xhat[1][0])
        print("Slope-intercepts:")
        print(self.lines_eqn_params)
        print('')

        self.lines_eqn_params.to_csv(f"{currentdir}/slope_intercept.csv",index_label=None)

    def find_line_intersection_corners(self):
        intersection_corners=[]
        for i in range(len(self.lines_eqn_params)-1):
            intersection_corners.append(line_intersect(self.lines_eqn_params.iloc[i]["m"], 
                self.lines_eqn_params.iloc[i]["c"],self.lines_eqn_params.iloc[i+1]["m"],
                self.lines_eqn_params.iloc[i+1]["c"]))

        self.intersection_corners=pd.DataFrame(intersection_corners,columns=["curr_xs","curr_ys"])
        self.intersection_corners.to_csv(f"{currentdir}/corners.csv",index_label=None)
        plt.scatter(self.intersection_corners["curr_xs"],self.intersection_corners["curr_ys"],s=5,color="r")
        print("A, B, C, D:")
        print(self.intersection_corners)
        print('')

    def find_opening_in_rect(self,data):
        self.polar_df=data
        plt.scatter([0],[0],s=10,color="m")
        self.calculate_cartesian_coordinates()
        #plt.scatter(self.polar_df["curr_xs"],self.polar_df["curr_ys"],s=5,color="r")
        self.find_enclosure_cluster()
        corner_lidar_points=self.find_rectangle_corners()
        corner_lidar_points=self.discard_false_corners(corner_lidar_points)
        self.find_line_equations(corner_lidar_points)
        self.find_line_intersection_corners()
        rect_convex_hull = ConvexHull(self.intersection_corners)
        lidar_points_in_rect = in_hull(self.polar_df, rect_convex_hull, epsilon=0)
        opening_coords1, opening_coords2 = calc_opening_coords(lidar_points_in_rect)
        plt.savefig(f"{currentdir}/q1b_final.png")


    def plotting_callback(self,data):
        self.find_opening_in_rect(data)
        #plt.savefig("Current_Environment_map.png")
        plt.clf();plt.cla()
        #Resettingfornextscan-lidarmapwithnewreadings
        #plt.xlim([-300,300])
        #plt.ylim([-300,300])
        plt.xlim([-5,5])
        plt.ylim([-5,5])
        self.angles_data, self.ranges_data=np.array([]),np.array([])
        self.x_data,self.y_data=np.array([]),np.array([])
        self.frame_num+=1

def main():
    sc = ScanContext()
    sc.deserialize(f'{q1adir}/LidarScanContext.txt')
    df = sc.get_data_df()
    soln = Visualiser().plotting_callback(df)

if __name__=="__main__":
    main()