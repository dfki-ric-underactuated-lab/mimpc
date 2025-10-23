# import rosbags and load data into dataframes.
from rosbags.rosbag2.reader import Reader
from rosbags.typesys import get_types_from_msg

from rosbags.typesys import Stores, get_typestore
typestore = get_typestore(Stores.ROS2_HUMBLE)


import pandas as pd
import plotnine as p9
from plotnine import *

import glob
from pathlib import Path
import numpy as np

import pandas as pd
from plotnine import *

from PIL import Image
import matplotlib.pyplot as plt

import os
os.environ["MPLBACKEND"] = "pdf"   # or "pdf" if you prefer
import matplotlib
matplotlib.use("pdf")   


typestore = get_typestore(Stores.ROS2_HUMBLE)  # FOXY/GALACTIC/HUMBLE/IRON/JAZZY etc.
add_types = {}
for path in glob.glob("scripts/orl_interfaces/msg/*.msg"):
    msg_path = Path(path)
    msg_def = msg_path.read_text(encoding="utf-8")
    # Full type must look like: "<package>/msg/<MsgName>"
    msg_type = f"orl_interfaces/msg/{msg_path.stem}"
    add_types.update(get_types_from_msg(msg_def, msg_type))

# If you’ve got .idl files, do the same with get_types_from_idl(...)
# for path in glob.glob("orl_interfaces/msg/*.idl"):
#     idl_path = Path(path)
#     idl_def = idl_path.read_text(encoding="utf-8")
#     msg_type = f"orl_interfaces/msg/{idl_path.stem}"
#     add_types.update(get_types_from_idl(idl_def, msg_type))

typestore.register(add_types)



from scipy.spatial.transform import Rotation as R

def load_bag(path: str):
    columns_pose = {}
    columns_thrust = {"t":[], "msg_idx":[], "t0": [], "t1": [],"t2": [],"t3": [],"t4": [],"t5": [],"t6": [], "t7": []}
    columns_log = {"t":[], "msg_idx":[], "rosout": []}
    for key in [
                "x",
                "y",
                "x_d",
                "y_d",
                "theta",
                "theta_d",
                "rw_vel",
                "msg_idx",
                "t"]:
        columns_pose[key] = []


    with Reader(path) as reader:
        start_time_stamp = None
        for connection, timestamp, rawdata in reader.messages():
            if not start_time_stamp:
                start_time_stamp = timestamp
            msg = typestore.deserialize_cdr(rawdata, connection.msgtype)
            #print(connection.topic)
            if connection.topic == "/reacsa/robot_state":             
                columns_pose["x"].append(msg.pose.position.x)
                columns_pose["y"].append(msg.pose.position.y) 
                columns_pose["theta"].append(R.from_quat([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]).as_euler('ZYX', degrees=False)[0] )
                columns_pose["x_d"].append(msg.twist.linear.x)
                columns_pose["y_d"].append(msg.twist.linear.y)
                columns_pose["theta_d"].append(msg.twist.angular.z)
                columns_pose["rw_vel"].append(msg.rw_vel)
                columns_pose["t"].append((msg.header.stamp.sec * 1e9 + float(msg.header.stamp.nanosec) - start_time_stamp) * 1e-9)
                columns_pose["msg_idx"].append(connection.msgcount)
                #columns_pose["t"].append(float(timestamp- start_time_stamp) * 1e-9)
            if connection.topic == "/reacsa/thrusters_command":
                columns_thrust["t"].append((msg.header.stamp.sec * 1e9 + float(msg.header.stamp.nanosec) - start_time_stamp) * 1e-9)
                columns_thrust["msg_idx"].append(connection.msgcount)
                for i in range(8):
                    columns_thrust[f"t{i}"].append(float(msg.fire[i]))
            if connection.topic == "/rosout":
                columns_log["t"].append((msg.stamp.sec * 1e9 + float(msg.stamp.nanosec) - start_time_stamp) * 1e-9)
                columns_log["rosout"].append(msg.msg)
                columns_log["msg_idx"].append(connection.msgcount)
                

        return pd.DataFrame(columns_pose), pd.DataFrame(columns_thrust), pd.DataFrame(columns_log)
    
    
df_L1_lvl1,df_L1_lvl1_trhust,df_L1_lvl1_log = load_bag("important_bags/day2_l1_lvl1_w220_0")
df_L1_lvl2,df_L1_lvl2_trhust,df_L1_lvl2_log = load_bag("important_bags/day2_l1_lvl2_w220_0")
df_mimpc,df_mimpc_trhust,df_mimpc_log = load_bag("important_bags/day2_mimpc_w220_0")


df_L1_lvl1.insert(0, "controller", "drakemi1")
df_L1_lvl2.insert(0, "controller", "drakemi2")
df_mimpc.insert(0, "controller", "scip")

df_L1_lvl1_trhust.insert(0, "controller", "drakemi1")
df_L1_lvl2_trhust.insert(0, "controller", "drakemi2")
df_mimpc_trhust.insert(0, "controller", "scip")

df_L1_lvl1_log.insert(0, "controller", "drakemi1")
df_L1_lvl2_log.insert(0, "controller", "drakemi2")
df_mimpc_log.insert(0, "controller", "scip")

df = pd.concat([df_L1_lvl1, df_L1_lvl2, df_mimpc])
df_thrust = pd.concat([df_L1_lvl1_trhust, df_L1_lvl2_trhust, df_mimpc_trhust])
df_log = pd.concat([df_L1_lvl1_log, df_L1_lvl2_log, df_mimpc_log])

# find exact timestep of start and stop
start_t  = df_log.query("rosout=='Controller state changed: Previous State = WAITING , Current State = FOLLOWING'").groupby("controller")["t"].min()
end_t = df_log.query("rosout=='Controller state changed: Previous State = MAINTAINING , Current State = CANCELLING' | rosout=='Controller state changed: Previous State = FOLLOWING , Current State = CANCELLING'").groupby("controller")["t"].min()
print(start_t)
print(end_t)

df = df[df["t"].ge(df["controller"].map(start_t))]
df = df[df["t"].lt(df["controller"].map(end_t))]


eps = 0.1
# find the cycle_idx (the point from here it says within eps circle)
df.sort_values(["controller", "t", "msg_idx"])
df["rms"] = np.sqrt(df["x"]**2 + df["y"]**2)
df["within_eps"] = df["rms"] < eps

def inside_from_here(g):
    stays_inside = g["within_eps"][::-1][::-1].cumprod().astype(bool)
    g['within_eps_from_here'] = stays_inside
    idx = stays_inside.idxmax()
    return g.loc[idx, "t"]

def rms(g):
    duration = g["t"].max() - g["t"].min()
    time_till_next = np.diff(g["t"], append=0)
    rms_weighted = (df["rms"] * time_till_next).sum() / duration
    return rms_weighted

def thrust_usage(g):
    duration = g["t"][-1] - g["t"][0]
    time_till_next = np.diff(g["t"], append=0)
    
print(df)
print(df.describe())
print(df.groupby("controller").describe())
cycle_ts = df.groupby("controller").apply(inside_from_here)

print(cycle_ts)
df_reach = df[df["t"].lt(df["controller"].map(cycle_ts))]
df_cycle = df[df["t"].ge(df["controller"].map(cycle_ts))]

print()
print(rms(df_cycle))



FLAT_FLOOR_MIN = -0.482
FLAT_FLOOR_MAX = 0.964


#load background
map_im = Image.open("scripts/heightmap_cropped.png")
map = FLAT_FLOOR_MIN + (FLAT_FLOOR_MAX - FLAT_FLOOR_MIN) * (np.asanyarray(map_im) / 255.0)
h,w = map.shape

n_pix = 513
hm_res = 10.0 / n_pix  # m/pixel
max_h = 0.001004545239150106 * 1000  # mm

LENGTH_FLATFLOOR_X = 4.75  # Length of flatfloor along (short) x-axis [m]
LENGTH_FLATFLOOR_Y = 8.78  # Length of flatfloor along (long) y-axis [m]


# grid of centers
X, Y = np.meshgrid(np.linspace(-LENGTH_FLATFLOOR_X/2, (LENGTH_FLATFLOOR_X/2),w), np.linspace(-LENGTH_FLATFLOOR_Y / 2, LENGTH_FLATFLOOR_Y / 2, h))

print(f"h={h}, w={w}")

df_map = pd.DataFrame({"x": X.ravel(), "y": Y.ravel(), "val": map.ravel()})


euro_gnc_with_in = 6.9

height = 5.0
width_perc = [0.6,0.4]

main_plot = (
    ggplot(df, aes(x='x', y='y', color="controller"))
    + geom_raster(data=df_map, mapping=aes("x", "y", fill="val"), inherit_aes=False, interpolate=True, alpha=0.6)
    + scale_fill_continuous(cmap_name="terrain", limits=(FLAT_FLOOR_MIN, FLAT_FLOOR_MAX), breaks=[FLAT_FLOOR_MIN,0, FLAT_FLOOR_MAX])
    + geom_path(size=0.5)          # thin line for the path
    # geom_point(size=0.01, color='red') +          # very small points                           
    + coord_fixed(ratio=1)
    + scale_x_continuous(expand=(0,0),breaks=np.arange(-1, 2), limits=(-1.25, 1.25), name="$x$ (m)")
    + scale_y_continuous(expand=(0,0), breaks=np.arange(-0.5, 4, 0.5), limits=(-0.5, 3.5), name="$y$ (m)")
    #+ ylim(-0.5, 4.0)
    + theme_bw()
    + theme(legend_position='right', 
            #panel_background=element_blank(),
            #panel_grid_major=element_blank(),
            figure_size=(euro_gnc_with_in * width_perc[0] ,height), text=p9.element_text(size=10, family="Times New Roman"))
).save("main_plot.pdf")

limit_x = 0.03
limit_y = 0.05
zoomed_plot = (
    ggplot(df, aes(x='x', y='y', show_legend=False, color="controller"))
    + geom_path(size=0.5, show_legend=False)          # thin line for the path
    + facet_grid("controller")    
    + coord_fixed(ratio=1)                
    + scale_x_continuous(breaks=np.array([-limit_x, 0, limit_x]), limits=(-limit_x,limit_x), name="$x$ (m)")
    + scale_y_continuous(breaks=np.array([-limit_y, 0, limit_y]), limits=(-limit_y, limit_y), name="$y$ (m)")
    + theme_bw()
    + theme(figure_size=(euro_gnc_with_in * width_perc[1], height), text=p9.element_text(size=10, family="Times New Roman"),)
).save("zoom_plot.pdf")

# limit = 0.08
# zoomed_line_plot = (
#     ggplot(df, aes(x='x', y='y', show_legend=False, color="controller"))
#     + geom_path(size=0.5, show_legend=False)          # thin line for the path    
#     + coord_fixed(ratio=1)                
#     + ylim(-limit, 2.0)
#     + xlim(-limit, limit)
#     + scale_x_continuous(breaks=np.array([-limit, 0, limit]), limits=(-limit,limit), name="")
 
# )

#df_melt = df.melt(value_vars=[keys], var_name='key', value_name='value')

statespace_plot = ((
    ggplot(df, aes(x="t"))
    + geom_line(aes(y="x"), color="red")
    + geom_line(aes(y="y"), color="blue")
    + facet_grid(cols="controller")
)/(ggplot(df, aes(x="t"))
    + geom_line(aes(y="x_d"), color="red")
    + geom_line(aes(y="y_d"), color="blue")
    + facet_grid(cols="controller")
    
    )/(ggplot(df, aes(x="t"))
    + geom_line(aes(y="theta"), color="green")
    #+ geom_line(aes(y="theta_d"), color="green", linetype="--")
    #+ geom_line(aes(y="rw_vel"), color="blue", linetype="--")
    + facet_grid(cols="controller")
    
    )/(ggplot(df_thrust, aes(x="t"))
    + geom_point(aes(y="t0"))
    + geom_point(aes(y="t1"))
    + geom_point(aes(y="t2"))
    + geom_point(aes(y="t3"))
    + geom_point(aes(y="t4"))
    + geom_point(aes(y="t5"))
    + geom_point(aes(y="t6"))
    + geom_point(aes(y="t7"))
    + facet_grid(cols="controller")
    
    )
).save("ss.pdf")