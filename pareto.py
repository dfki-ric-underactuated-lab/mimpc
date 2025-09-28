import os
from pathlib import Path
import numpy as np
import pandas as pd
import plotnine as p9
from plotnine import ggplot, aes, geom_line, facet_wrap, labs, theme_bw, ggsave
from itertools import chain
from paretoset import paretoset

from concurrent.futures import ProcessPoolExecutor

import matplotlib.pyplot as plt

from analysis import time_weighted_rms, thruster_usage


def get_occurance_of_time_step_in_sequence(time_step: float, time_sequence: np.ndarray):
    for i in range(time_sequence.shape[0]):
        if float(time_step) <= time_sequence[i]:
            return i


data_dir = Path("gnc2_test_sept_big")

rows = []


# preprocess all data

# limit_cycle_bounds for # 3v: 0.102165 x: 26.4945
# v_max: 1
# pos_min: -1.85, -3.865
# pos_max: 1.85, 3.865

reacsa_v_max = 1
reacsa_pos_min = np.array([-1.85, -3.865])
reacsa_pos_max = np.array([1.85, 3.865])



def process_npz(npz_path: Path):
    """Return the row dict for a single .npz file."""
    data = np.load(npz_path)

    x_log = data["x_log"]
    u_log = data["u_log"]
    x_times = data["x_time"]
    u_times = data["u_time"]
    
    assert((x_times == u_times).all())

    parts = npz_path.stem.split("_")
    # Example name pattern: <prefix>_tw-<thr>_vw-<vel>_<controller>_<time_limit>
    controller = parts[4]
    controller_mi = "0"
    if(controller.__contains__('mi')):
        csplit = controller.split('mi')
        controller = csplit[0]
        try:
            controller_mi = str(int(csplit[1]))
        except:
            controller_mi = str(2)
            
    time_limit = str(parts[5])
    thrust_weight = float(parts[1].split("-")[1])
    vel_weight = float(parts[2].split("-")[1])
    final_mul = float(parts[3].split("-")[1])

    eps = 0.1 # m -> 20 cm  
    x_log_mod = np.sqrt(x_log[0,:]**2 + x_log[1,:]**2) < eps 
    #x_log_mod[1, :] = -x_log_mod[1, :]
    # shape (N,)
    # Cumulative check from the right: are *all future values* True?
    #cond = np.all(x_log_mod < eps, axis=0)
    # suffix_all = np.cumprod(cond[::-1])[::-1].astype(bool)
    # First index where suffix_all is True
    #cycle_idx = np.argmax(suffix_all) if np.any(suffix_all) else -1
    
    
    # first index where condition is true
    # Cumulative "have we seen True yet?" along time axis
    seen = np.cumsum(x_log_mod) > 0   # shape (2, N), True if row has been True up to that point
    # First index where both have been True at least once
    cycle_idx_passed_by = np.argmax(seen) if np.any(seen) else -1  
    
    # Compute cumulative "all true from here to the end"
    all_true_from = np.cumprod(x_log_mod[::-1])[::-1].astype(bool)
    # Find the first index
    cycle_idx = np.argmax(all_true_from) if np.any(all_true_from) else -1

    res = []
    
    failed = False
    fail_reason = "success"
    
    if((x_log[0,:] < reacsa_pos_min[0]).any() or (x_log[0,:] > reacsa_pos_max[0]).any()):
        failed = True
        fail_reason = "hit_wall"
    elif((x_log[1,:] < reacsa_pos_min[1]).any() or (x_log[1,:] > reacsa_pos_max[1]).any()):
        failed = True
        fail_reason = "hit_wall"
    elif((np.abs(x_log[3:5])>reacsa_v_max).any()):
        failed = True
        fail_reason = "too_fast"
    
    elif((u_log[1:,:] <= 1e-4).all()):
        failed = True
        fail_reason = "no_thrust"
    
    elif (u_times[-1] - u_times[cycle_idx]) < 1 : # failed experiments
        failed = True
        if cycle_idx_passed_by == -1:
            fail_reason = "no_reach"
        else:
            fail_reason = "diverge"
            
    elif (u_times[-1] - u_times[cycle_idx]) < 30 : # failed experiments
        failed = True
        fail_reason = "less_than_30"
        
    #if(((np.abs(x_log[:2,-1]) > np.abs(x_log[:2,cycle_idx])).any()) and (u_log[1:,cycle_idx:-1] <= 1e-6).all()):
    #    failed = True
    #    fail_reason = "diverged"            

    for lms in [(0, -1, "total"), (0, cycle_idx, "reach"), (cycle_idx, -1, "cycle")]:
        start = lms[0]
        end = lms[1]    
        
        if start == end or cycle_idx == (len(u_times) - 1):
            if not lms[2] == "cycle":
                print(f"WARN: skipped empty {lms[2]} - {npz_path}")
            continue
        
        res.append(
            {
                "controller": controller,
                "mi": controller_mi,
                "time_limit": time_limit,
                "thrust_weight": thrust_weight,
                "vel_weight": vel_weight,
                "final_mul": final_mul,
                "part": lms[2],
                "failed": failed,
                "fail_reason": fail_reason,
                # metrics
                "rms_pos": time_weighted_rms(
                    x_times[start:end], x_log[0:2, start:end]
                ).item(),
                "rms_orient": time_weighted_rms(
                    x_times[start:end], x_log[3, start:end]
                ).item(),
                "thrust": thruster_usage(
                    u_times[start:end], u_log[1:, start:end]
                ).item(),
                "time": (u_times[end] - u_times[start]).item(),
            }
        )
        
        if(res[-1]["thrust"] < 1e-8 and not res[-1]["failed"]):
            
            print("small thrust", npz_path, res[-1], start, end)
            print("- bef ", npz_path, res[-2])
            print("- bef bef", npz_path, res[-3])
            
            #res[-1]["thrust"] = 1e-8
            
            
    return res

pandas_df_store = "data.pkl"
redo = False

if redo or not os.path.exists(pandas_df_store):
    files = sorted(data_dir.glob("*.npz"))

    process_npz(files[0])


    # executor.map preserves input order
    with ProcessPoolExecutor(max_workers=8) as ex:
        per_file_rows = list(ex.map(process_npz, files))  # list of lists
    rows = list(chain.from_iterable(per_file_rows))


    df = pd.DataFrame(rows)
    df.to_pickle(pandas_df_store)
else:
    df = pd.read_pickle(pandas_df_store)
    

# df2 = pd.DataFrame(rows).melt(
#             id_vars=["controller", "time_limit", "thrust_weight", "vel_weight", "part", "mi", "failed"],
#             var_name="metric",
#             value_name="value",
#         )
# df2["vel_weight"] = df2["vel_weight"].astype(str)

# p = (
#     p9.ggplot(df2
#         ,
#         aes(x="thrust_weight", y="value", color="controller", shape="vel_weight", linetype="mi", group="vel_weight + '.' + controller + '.' + mi"),
#     )
#     + p9.scale_linetype_manual(values={"0":"solid", "1":"dashed", "2":"dotted", "3":"dashdot"})
#     + p9.geom_point(size=1)
#     + p9.geom_line()
#     + p9.facet_wrap(["metric","part"], scales="free", ncol=4)
#     + p9.scale_y_log10()
# ).show()

df["vel_weight"] = df["vel_weight"].astype(str)
df["final_mul"] = df["final_mul"].astype(str)

#df = df.query("mi=='0' | mi=='1' | mi=='3'")
#df = df.query("controller != 'acados'")

# df_pareto = df.query("part == 'cycle' and failed == False")
# pareto_mask = paretoset(df_pareto[["rms_pos", "thrust", "mi", "controller"]], ["min", "min", "diff", "diff"], distinct=False)
# df_pareto = df_pareto[pareto_mask]
# df_pareto["pareto_optimal"] = True

# keys_to_match = [
#             "controller",
#             "mi",
#             "time_limit",
#             "thrust_weight",
#             "vel_weight",
#             "final_mul",
#             "failed",
#             ]

# df = df.merge(df_pareto[keys_to_match + ["pareto_optimal"]], on=keys_to_match, how="left")

#df.loc[df_pareto.index, "pareto_optimal"] = True



df_melt = df.melt(
        id_vars=[
            "controller",
            "mi",
            "thrust",
            "time_limit",
            "thrust_weight",
            "vel_weight",
            "final_mul",
            "part",
            "failed",
            "fail_reason"
#            "pareto_optimal"
        ],
        var_name="metric",
        value_name="value",
    )


solver_name_map = {
    "scip": "MIMPC (L1)",
    "drake": "MPC (L1)",
    "acados": "MPC (L2)",
}

solver_mi_name_map = {
    "scip.0": "MIMPC (L1)",
    "drake.0": "MPC uninformed (L1)",
    "drake.1": "MPC informed (L1)",
    "drake.2": "MPC half enforced (L1)",
    "drake.3": "MPC enforced (L1)",
    "acados.0": "MPC uninformed (L2)",
    "acados.1": "MPC informed (L2)",
    "acados.2": "MPC half enforced (L2)",
    "acados.3": "MPC enforced (L2)",
}

fail_reason_name_map = {
    "success": "Sucessfull",
    "hit_wall": "Crash",
    "less_than_30": "Too short in goal",
    "diverge": "Passed by goal",
    "no_reach": "Didn't reach goal"
}




# p = (
#     p9.ggplot(df_melt[df_melt["failed"] == False], aes(x="thrust", y="value", color="controller", shape="time_limit"))
#     + p9.geom_point(size=2, alpha=0.5)
#     + p9.facet_wrap(["metric","part"], scales="free")
#     + p9.scale_x_log10()
#     + p9.scale_y_log10()
# ).show()


# for metric in ["rms_pos", "rms_orient", "time"]:

#     p = (
#         p9.ggplot(df_melt[df_melt["metric"].str.contains(metric) & (df_melt["failed"] == False)], aes(x="thrust", y="value", color="mi", shape="vel_weight"))
#         + p9.geom_point(size=2)
#         + p9.facet_grid(rows="part", cols="controller", scales="free")
#         + p9.geom_point(df_melt[df_melt["metric"].str.contains(metric) & (df_melt["failed"] == True)], p9.aes(x="thrust", y=np.inf, color="mi"), shape="x")
#         + p9.scale_x_log10()
#         + p9.scale_y_log10()
#         + labs(title=metric)
#     ).show()
    
df_melt = df_melt[(df_melt["metric"].str.contains("rms_pos") & df_melt["part"].str.contains("cycle")) | 
        # ( df_melt["metric"].str.contains("rms_pos") & df_melt["part"].str.contains("total")) | 
        (df_melt["metric"].str.contains("rms_orient") & df_melt["part"].str.contains("total"))
        | (df_melt["metric"].str.contains("time") & df_melt["part"].str.contains("reach"))
        ]

df_melt["metric"] = df_melt["metric"] + df_melt["part"]
pareto_mask = paretoset(df_melt.query("failed == False")[["value", "thrust", "mi", "controller", "metric"]], ["min", "min", "diff", "diff", "diff"], distinct=False)

df_melt.loc[df_melt.query("failed == False")[pareto_mask].index, "pareto_optimal_melt"] = True


euro_gnc_with_in = 6.9
plot_style =  p9.theme_bw() + p9.theme(figure_size=(euro_gnc_with_in,3.4), #inches
                text=p9.element_text(size=10, family="Times New Roman"),
                )


df_melt_clip = df_melt.query("part=='reach'")
df_melt_clip["thrust"] = df_melt_clip["thrust"].clip(upper=0.3)
p = (
    p9.ggplot(df_melt_clip.query("part=='reach'"), aes(x="thrust", fill="fail_reason"))
    + p9.geom_histogram(alpha=0.4, position="stack", binwidth=0.05)
    + p9.facet_grid(rows="controller + '.' + mi", labeller=p9.labeller(rows=solver_mi_name_map))
    + p9.labs(x="Average thrust usage (s/s)", y="Num experiments (#)")
    #+ p9.coord_flip()
    + p9.scale_fill_discrete(labels=fail_reason_name_map, name='Experiment status')
    #+ p9.guides(fill=p9.guide_legend(title='Experiment status'))
    + p9.theme_bw() + p9.theme(figure_size=(euro_gnc_with_in,10), #inches
                text=p9.element_text(size=10, family="Times New Roman"),
                )
).save("experiment_status.pdf")

exit()
p = (
    p9.ggplot(df_melt[df_melt["failed"] == False], aes(x="thrust", y="value", color="mi", fill="mi"))
    + p9.geom_point(aes(shape='final_mul'), size=1.5, alpha=0.3, stroke=0.0)
    + p9.facet_grid(rows="metric + '.' + part", cols="controller", scales="free")
    #+ p9.geom_point(df_melt[df_melt["failed"] == True], p9.aes(x="thrust", y=np.inf, color="mi", alpha=0.2), shape="x")
    + p9.geom_line(df_melt[df_melt["pareto_optimal_melt"]==True], size=1., alpha=0.6)
    + p9.geom_point(df_melt[df_melt["pareto_optimal_melt"]==True], aes(shape='final_mul'), size=1.5, alpha=.8, stroke=0.0)
    + p9.scale_x_log10()
    + p9.scale_y_log10()
).show()


p = (
    p9.ggplot(df_melt[df_melt["failed"] == False], aes(x="thrust", y="value", color="controller + '.' + mi", fill="controller + '.' + mi"))
    + p9.geom_point(size=0.6, alpha=0.6, stroke=0.0)
    + p9.facet_grid(rows="metric + '.' + part", scales="free")
    #+ p9.geom_point(df_melt[df_melt["failed"] == True], p9.aes(x="thrust", y=np.inf, alpha=0.2), shape="x")
    + p9.geom_line(df_melt[df_melt["pareto_optimal_melt"]==True], size=0.7, alpha=1.0)
    + p9.geom_point(df_melt[df_melt["pareto_optimal_melt"]==True], size=2., alpha=.5)
    + p9.scale_x_log10()
    + p9.scale_y_log10()
).show()




# p = (
#     p9.ggplot(df_melt, aes(x="thrust_weight", y="value", color="controller"))
#     + p9.geom_point(size=4)
#     + p9.facet_wrap("~metric", scales="free")
# ).show()


# p = (
#     p9.ggplot(df_melt, aes(x="vel_weight", y="value", color="controller"))
#     + p9.geom_point(size=4)
#     + p9.facet_wrap("~metric", scales="free")
# ).show()
