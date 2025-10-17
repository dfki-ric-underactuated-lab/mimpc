import os
from pathlib import Path
import numpy as np
import pandas as pd
import plotnine as p9
from plotnine import ggplot, aes, geom_line, facet_wrap, labs, theme_bw, ggsave
from itertools import chain
from paretoset import paretoset
from kneed import KneeLocator

from concurrent.futures import ProcessPoolExecutor

import matplotlib.pyplot as plt

from analysis import time_weighted_rms, thruster_usage


def get_occurance_of_time_step_in_sequence(time_step: float, time_sequence: np.ndarray):
    for i in range(time_sequence.shape[0]):
        if float(time_step) <= time_sequence[i]:
            return i


data_dir = Path("gnc2_test_oct_big2")

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
df.loc[df.query("controller == 'scip'").index, "mi"] = ''

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



solver_mi_name_map =  {
        "scip": "MIMPC",
        "drake": "MPC",
        "acados": "MPC (L2)",
        "": "explicit",
        ".": "",
        "0": "uninformed",
        "1": "informed",
        "2": "enforced",
        "3": "enforced",
        "scip.0.100000.": "MIMPC ($\overline{t_\mathrm{s}}=0.1\mathrm{s}$)",
        "scip.1.000000.": "MIMPC ($\overline{t_\mathrm{s}}=1\mathrm{s}$)",
        "drake.0.100000.0": "MPC uninformed",
        "acados.0": "MPC (L2) uninformed",
        "drake.0.100000.1": "MPC informed",
        "acados.1": "MPC (L2) informed",
        "drake.0.100000.2": "MPC enforced",
        "acados.2": "MPC (L2) half enforced",
        }

fail_reason_name_map = {
    "success": "Successful",
    "hit_wall": "Crash",
    "less_than_30": "Undershoot",
    "diverge": "Overshoot",
    "no_reach": "Undershoot"
}

fail_reason_colors = {
    "success": "green",
    "hit_wall": "red",
    "diverge": "purple",
    "no_reach": "blue"
}

measurement_name_mao = {
    "rms_orient.cycle": "orientation RMS (rad)",
    "rms_pos.cycle": "target position RMS (m)",
    "time.reach": "time to reach target (s)",
    "rms_orient": "orientation RMS (rad)",
    "rms_pos": "position RMS (m)",
    "time": "time (s)",
    "cycle": "",
    "reach": "",
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
        (df_melt["metric"].str.contains("rms_orient") & df_melt["part"].str.contains("cycle"))
        | (df_melt["metric"].str.contains("time") & df_melt["part"].str.contains("reach"))
        ]


#df_melt["metric"] = df_melt["metric"] + df_melt["part"]
pareto_mask = paretoset(df_melt.query("failed == False")[["value", "thrust", "mi", "controller", "metric", "time_limit"]], ["min", "min", "diff", "diff", "diff", "diff"], distinct=False)

df_melt.loc[df_melt.query("failed == False")[pareto_mask].index, "pareto_optimal_melt"] = True

print("Pareto optimal points:", df_melt.query("pareto_optimal_melt==True"))


euro_gnc_with_in = 6.9
plot_style =  p9.theme_bw() + p9.theme(figure_size=(euro_gnc_with_in,3.4), #inches
                text=p9.element_text(size=10, family="Times New Roman"),
                )
leg_below =  p9.theme(legend_position='bottom',
                      #legend_box="vertical",
                      legend_box_spacing=0.012,
                      legend_spacing=15.0,
                      legend_text=p9.element_text(size=9, hjust=0),
                      legend_title=p9.element_text(face='bold',size=10),
                      legend_direction='vertical',
                      legend_text_position='right'
                      )


df_melt_clip = df_melt.query("part=='reach'")
df_melt_clip["thrust"] = df_melt_clip["thrust"].clip(upper=0.3)
p = (
    p9.ggplot(df_melt_clip.query("part=='reach'"), aes(x="thrust", fill="fail_reason"))
    + p9.geom_histogram(alpha=1.0, position="stack", binwidth=0.05)
    + p9.facet_grid(rows="mi", cols="controller + '.' + time_limit", labeller=p9.labeller(cols=solver_mi_name_map, rows=solver_mi_name_map, multi_line=True))
    + p9.labs(x="Average thrust usage (s/s)", y="Num experiments (#)")
    #+ p9.coord_flip()
    + p9.scale_fill_discrete(labels=fail_reason_name_map, name='Experiment status')
    + p9.guides(fill=p9.guide_legend(nrow=1))
    + p9.theme_bw() + p9.theme(figure_size=(euro_gnc_with_in,5), #inches
                text=p9.element_text(size=10, family="Times New Roman"),
                ) 
    + leg_below
).save("experiment_status.pdf")

df_melt_clip.loc[df_melt_clip.query("fail_reason=='less_than_30'").index, "fail_reason"] = "no_reach"
p = (
    p9.ggplot(df_melt_clip.query("part=='reach' & (mi=='' | mi=='0' | mi=='1' | mi=='2') & (controller!='acados')"), aes(x="thrust", fill="fail_reason"))
    + p9.geom_histogram(aes(y=p9.after_stat("count")),alpha=1.0, position="stack", binwidth=0.01)
    #+p9.geom_density(aes(y=p9.after_stat("count*0.05")), alpha=0.1)
    + p9.facet_wrap("controller + '.' + time_limit + '.' + mi", labeller=p9.labeller(cols=solver_mi_name_map), nrow=1)
    + p9.labs(x="Average thrust usage (s/s)", y="Num experiments (#)")
    #+ p9.coord_flip()
    + p9.scale_fill_manual(fail_reason_colors, labels=fail_reason_name_map, name='Experiment status')

    + p9.guides(fill=p9.guide_legend(nrow=1))
    + p9.theme_bw() + p9.theme(figure_size=(euro_gnc_with_in,2.5), #inches
                text=p9.element_text(size=10, family="Times New Roman"),
                ) 
    + leg_below
).save("experiment_status_compact.pdf")


p = (
    p9.ggplot(df_melt.query("failed==False & controller!='scip'"), aes(x="thrust", y="value", color="mi", fill="mi"))
    + p9.geom_point(size=0.7, alpha=0.5, stroke=0.0)
    + p9.facet_grid(rows="metric + '.' + part", cols="controller", scales="free", labeller=p9.labeller(cols=solver_mi_name_map, rows=measurement_name_mao))
    #+ p9.geom_point(df_melt[df_melt["failed"] == True], p9.aes(x="thrust", y=np.inf, color="mi", alpha=0.2), shape="x")
    + p9.geom_line(df_melt.query("pareto_optimal_melt==True & controller!='scip'"), size=0.7, alpha=0.8, linetype="-.")
    + p9.geom_point(df_melt.query("pareto_optimal_melt==True & controller!='scip'"), size=1., alpha=.9, stroke=0.0)
    + p9.scale_x_log10()
    + p9.scale_y_log10()
    + p9.scale_color_discrete(labels=solver_mi_name_map, name="MI information level")
    + p9.scale_fill_discrete(labels=solver_mi_name_map, name="MI information level")
    + p9.labs(x='Average thrust usage (s/s)', y='')
    + p9.guides(color=p9.guide_legend(nrow=1))
    + p9.theme_bw() + p9.theme(figure_size=(euro_gnc_with_in,5), #inches
                text=p9.element_text(size=10, family="Times New Roman"),
                ) 
    + leg_below
).save("comp_mi.pdf")


knees = []
for cont in ["scip", "drake"]:
    for mi in ["", "0", "1", "2"]:
        for time_limit in ["0.100000", "1.000000"]:
            x = df_melt.query(f"failed==False & pareto_optimal_melt==True & controller=='{cont}' & mi=='{mi}' & metric=='rms_pos' & part=='cycle' & time_limit=='{time_limit}'")["thrust"]
            y = df_melt.query(f"failed==False & pareto_optimal_melt==True & controller=='{cont}' & mi=='{mi}' & metric=='rms_pos' & part=='cycle' & time_limit=='{time_limit}'")["value"]
            if x.shape[0] < 3:
                print("skip knee for", cont, mi, time_limit)
                continue
            kl = KneeLocator(x, y,
                curve="convex", direction="decreasing", S=3.0, interp_method="interp1d", online=True)
            if kl.knee is not None:
                knees.append({
                    "controller": cont,
                    "mi": mi,
                    "thrust": kl.knee,
                    "metric": "rms_pos",
                    "part": "cycle",
                    "value": kl.knee_y,
                    "time_limit": time_limit
                })

knee_df = pd.DataFrame.from_records(knees)

print(knee_df)

query = "(mi=='' | mi=='0' | mi=='1' | mi=='2') & (metric!='rms_orient') & (controller!='acados')"
p = (
    p9.ggplot(df_melt.query(f"failed==False & ({query})"), aes(x="thrust", y="value", color="controller + '.' + time_limit + '.' + mi", fill="controller + '.' + time_limit + '.' + mi"))
    + p9.geom_point(size=.6, alpha=0.9, stroke=0.0)
    + p9.facet_wrap("metric + '.' + part", scales="free", labeller=p9.labeller(cols=measurement_name_mao))
    #+ p9.geom_point(df_melt[df_melt["failed"] == True], p9.aes(x="thrust", y=np.inf, alpha=0.2), shape="x")
    + p9.geom_line(df_melt.query(f"pareto_optimal_melt==True & {query}"), size=0.5, alpha=0.6, linetype="-")
    + p9.geom_point(df_melt.query(f"pareto_optimal_melt==True & {query}"), size=.9, alpha=0.7)
    + p9.geom_point(knee_df, size=3., fill="none")
    + p9.scale_x_log10()
    + p9.scale_y_log10()
    + p9.scale_color_discrete(labels=solver_mi_name_map, name="Controller")
    + p9.scale_fill_discrete(labels=solver_mi_name_map, name="Controller")
    #+ p9.scale_shape_discrete(labels=solver_mi_name_map, name="MI information level")
    + p9.labs(x='Average thrust usage (s/s)', y='')
    + p9.guides(color=p9.guide_legend(nrow=1),shape=p9.guide_legend(nrow=1), fill=p9.guide_legend(nrow=1))
    + p9.theme_bw() + p9.theme(figure_size=(euro_gnc_with_in,3.8), #inches
                text=p9.element_text(size=10, family="Times New Roman"),
                ) 
    + leg_below
).save("comp_solver.pdf")




query = "(mi=='' | mi=='0' | mi=='1' | mi=='2') & (metric=='rms_orient') & (controller!='acados')"
p = (
    p9.ggplot(df_melt.query(f"failed==False & ({query})"), aes(x="thrust", y="value", color="controller + '.' + time_limit + '.' + mi", fill="controller + '.' + time_limit + '.' + mi"))
    + p9.geom_point(size=1.0, alpha=0.8, stroke=0.0)
    + p9.facet_wrap("metric + '.' + part", scales="free", labeller=p9.labeller(cols=measurement_name_mao))
    #+ p9.geom_point(df_melt[df_melt["failed"] == True], p9.aes(x="thrust", y=np.inf, alpha=0.2), shape="x")
    + p9.scale_x_log10()
    + p9.scale_y_log10()
    + p9.scale_color_discrete(labels=solver_mi_name_map, name="Controller")
    + p9.scale_fill_discrete(labels=solver_mi_name_map, name="Controller")
    #+ p9.scale_shape_discrete(labels=solver_mi_name_map, name="MI information level")
    + p9.labs(x='Average thrust usage (s/s)', y='')
    + p9.guides(color=p9.guide_legend(nrow=1),shape=p9.guide_legend(nrow=1), fill=p9.guide_legend(nrow=1))
    + p9.theme_bw() + p9.theme(figure_size=(euro_gnc_with_in,4.), #inches
                text=p9.element_text(size=10, family="Times New Roman"),
                ) 
    + leg_below
).save("comp_solver_orient.pdf")






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
