import pandas as pd
import pathlib
import os
import shutil
import sys
import numpy as np
from numpy import linalg as LA


from plotly.offline import init_notebook_mode, iplot
from plotly.subplots import make_subplots
import plotly.graph_objs as go
import plotly.io as pio
import plotly.express as px

pio.templates.default = "plotly_dark"
pio.renderers.default = "browser"
# pio.renderers.default = "vscode"

data_dir = "/home/yifanhou/data/real/flip_up_test"
data_dir = pathlib.Path(data_dir)

# find the most recent data folder under data_dir
data_folders = [f for f in data_dir.iterdir() if f.is_dir()]
data_folders.sort(key=lambda x: x.stat().st_ctime, reverse=False)
ep_dir = data_folders[0]

print("latest data folder: ", ep_dir)
json_file = ep_dir.joinpath(ep_dir).joinpath("low_dim_data.json")

# read json raw data
df_raw_data = pd.read_json(json_file)
seq_id = df_raw_data["seq_id"].to_numpy()
low_dim_time_stamps = df_raw_data["low_dim_time_stamps"].to_numpy()
ts_pose_fb = np.vstack(df_raw_data["ts_pose_fb"])
wrench = np.vstack(df_raw_data["wrench"])

# fmt: off
fig = make_subplots(
    rows=7, cols=2,
    shared_xaxes='all',
    subplot_titles=('Px', 'Fx',
                    'Py', 'Fy',
                    'Pz', 'Fz',
                    'Qw', 'Tx',
                    'Qx', 'Ty',
                    'Qy', 'Tz',
                    'Qz', 'time')
)
# fmt: on

fig.add_trace(go.Scatter(x=seq_id, y=ts_pose_fb[:, 0], name="pose x"), row=1, col=1)
fig.add_trace(go.Scatter(x=seq_id, y=ts_pose_fb[:, 1], name="pose y"), row=2, col=1)
fig.add_trace(go.Scatter(x=seq_id, y=ts_pose_fb[:, 2], name="pose z"), row=3, col=1)
fig.add_trace(go.Scatter(x=seq_id, y=ts_pose_fb[:, 3], name="pose q0"), row=4, col=1)
fig.add_trace(go.Scatter(x=seq_id, y=ts_pose_fb[:, 4], name="pose q1"), row=5, col=1)
fig.add_trace(go.Scatter(x=seq_id, y=ts_pose_fb[:, 5], name="pose q2"), row=6, col=1)
fig.add_trace(go.Scatter(x=seq_id, y=ts_pose_fb[:, 6], name="pose q3"), row=7, col=1)

fig.add_trace(go.Scatter(x=seq_id, y=wrench[:, 0], name="wrench 0"), row=1, col=2)
fig.add_trace(go.Scatter(x=seq_id, y=wrench[:, 1], name="wrench 1"), row=2, col=2)
fig.add_trace(go.Scatter(x=seq_id, y=wrench[:, 2], name="wrench 2"), row=3, col=2)
fig.add_trace(go.Scatter(x=seq_id, y=wrench[:, 3], name="wrench 3"), row=4, col=2)
fig.add_trace(go.Scatter(x=seq_id, y=wrench[:, 4], name="wrench 4"), row=5, col=2)
fig.add_trace(go.Scatter(x=seq_id, y=wrench[:, 5], name="wrench 5"), row=6, col=2)

fig.add_trace(go.Scatter(x=seq_id, y=low_dim_time_stamps, name="time"), row=7, col=2)

# convert ep_dir to string


fig.update_layout(height=1200, width=900, title_text=str(ep_dir))
fig.update_layout(hovermode="x unified")

fig.show()
