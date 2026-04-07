import numpy as np
import matplotlib.pyplot as plt
from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore

typestore = get_typestore(Stores.ROS2_HUMBLE)

# ─── LECTURE D'UN BAG ────────────────────────────────────────────────────────
def read_bag(path):
    data = {"t1": [], "x1": [], "y1": [], "vx1": [],
            "t2": [], "x2": [], "y2": [], "vx2": [],
            "t_cmd1": [], "vcmd1": [],
            "t_cmd2": [], "vcmd2": [], "wcmd2": []}

    with Reader(path) as reader:
        for conn, timestamp, rawdata in reader.messages():
            t = timestamp * 1e-9
            if conn.topic == "/robot1/odom":
                msg = typestore.deserialize_cdr(rawdata, conn.msgtype)
                data["t1"].append(t); data["x1"].append(msg.pose.pose.position.x)
                data["y1"].append(msg.pose.pose.position.y); data["vx1"].append(msg.twist.twist.linear.x)
            elif conn.topic == "/robot2/odom":
                msg = typestore.deserialize_cdr(rawdata, conn.msgtype)
                data["t2"].append(t); data["x2"].append(msg.pose.pose.position.x)
                data["y2"].append(msg.pose.pose.position.y); data["vx2"].append(msg.twist.twist.linear.x)
            elif conn.topic == "/robot1/cmd_vel":
                msg = typestore.deserialize_cdr(rawdata, conn.msgtype)
                data["t_cmd1"].append(t); data["vcmd1"].append(msg.linear.x)
            elif conn.topic == "/robot2/cmd_vel":
                msg = typestore.deserialize_cdr(rawdata, conn.msgtype)
                data["t_cmd2"].append(t); data["vcmd2"].append(msg.linear.x)
                data["wcmd2"].append(msg.angular.z)

    for k in data:
        data[k] = np.array(data[k])

    t0 = min(data["t1"][0], data["t2"][0])
    for k in ["t1", "t2", "t_cmd1", "t_cmd2"]:
        data[k] -= t0

    return data

# ─── CALCUL MÉTRIQUES ────────────────────────────────────────────────────────
def compute_metrics(d):
    x2i = np.interp(d["t1"], d["t2"], d["x2"])
    y2i = np.interp(d["t1"], d["t2"], d["y2"])
    dist = np.sqrt((d["x1"] - x2i)**2 + (d["y1"] - y2i)**2)
    target = dist[0]
    err = dist - target

    dx = np.diff(d["x1"]); dy = np.diff(d["y1"])
    traj_len = np.sum(np.sqrt(dx**2 + dy**2))

    return {
        "dist":     dist,
        "target":   target,
        "err":      err,
        "x2i":      x2i,
        "y2i":      y2i,
        "rmse":     np.sqrt(np.mean(err**2)),
        "mae":      np.mean(np.abs(err)),
        "max_err":  np.max(np.abs(err)),
        "std_err":  np.std(err),
        "pct_err":  np.mean(np.abs(err)) / target * 100,
        "dur":      d["t1"][-1],
        "vmax1":    np.max(d["vcmd1"]) if len(d["vcmd1"]) else 0,
        "vmax2":    np.max(d["vcmd2"]) if len(d["vcmd2"]) else 0,
        "wmax2":    np.max(np.abs(d["wcmd2"])) if len(d["wcmd2"]) else 0,
        "traj_len": traj_len,
    }

# ─── CHARGEMENT ──────────────────────────────────────────────────────────────
print("Chargement bag_sim_slam...")
sim = read_bag("bag_sim_slam_lidar2")
ms  = compute_metrics(sim)

print("Chargement bag_reel...")
reel = read_bag("bag_real_slam_lidar")
mr   = compute_metrics(reel)

# ─── TERMINAL : tableau métriques ────────────────────────────────────────────
print("\n" + "="*65)
print(f"{'PARAMETRE':<35} {'SIM SLAM':>12} {'REEL':>12}")
print("="*65)
rows_metrics = [
    ("Duree enregistree (s)",          f"{ms['dur']:.1f}",           f"{mr['dur']:.1f}"),
    ("Distance de consigne (m)",        f"{ms['target']:.3f}",        f"{mr['target']:.3f}"),
    ("Distance moyenne mesuree (m)",    f"{np.mean(ms['dist']):.3f}", f"{np.mean(mr['dist']):.3f}"),
    ("Longueur trajectoire (m)",        f"{ms['traj_len']:.2f}",      f"{mr['traj_len']:.2f}"),
    ("Erreur moyenne MAE (cm)",         f"{ms['mae']*100:.2f}",       f"{mr['mae']*100:.2f}"),
    ("Erreur max (cm)",                 f"{ms['max_err']*100:.2f}",   f"{mr['max_err']*100:.2f}"),
    ("Ecart-type erreur (cm)",          f"{ms['std_err']*100:.2f}",   f"{mr['std_err']*100:.2f}"),
    ("RMSE distance (cm)",              f"{ms['rmse']*100:.2f}",      f"{mr['rmse']*100:.2f}"),
    ("Erreur en % (MAE/consigne)",      f"{ms['pct_err']:.2f} %",     f"{mr['pct_err']:.2f} %"),
]
for name, va, vs in rows_metrics:
    print(f"  {name:<33} {va:>12} {vs:>12}")
print("="*65)

# ─── FIGURE ──────────────────────────────────────────────────────────────────
fig = plt.figure(figsize=(20, 14))
fig.suptitle("Comparaison Simulation SLAM vs Reel\nRobot Porteur (Leader/Follower)",
             fontsize=15, fontweight='bold', y=0.98)

ax_sim   = fig.add_subplot(2, 2, 1)
ax_reel  = fig.add_subplot(2, 2, 2)
ax_table = fig.add_subplot(2, 2, 3)
ax_err   = fig.add_subplot(2, 2, 4)

# ── Trajectoire Simulation SLAM
ax_sim.plot(sim["x1"], sim["y1"], color="blue", lw=2.5, label='Leader (robot1)')
ax_sim.plot(sim["x2"], sim["y2"], color="red",  lw=1.5, linestyle='--', label='Follower (robot2)')
ax_sim.plot(sim["x1"][0], sim["y1"][0], color="blue", marker='s', markersize=8)
ax_sim.plot(sim["x2"][0], sim["y2"][0], color="red",  marker='s', markersize=8)
ax_sim.set_title("Trajectoire XY - Simulation SLAM", fontsize=12, fontweight='bold')
ax_sim.set_xlabel("x (m)"); ax_sim.set_ylabel("y (m)")
ax_sim.legend(fontsize=9); ax_sim.grid(True); ax_sim.axis('equal')
ax_sim.xaxis.set_major_locator(plt.MultipleLocator(0.5))
ax_sim.yaxis.set_major_locator(plt.MultipleLocator(0.2))

# ── Trajectoire Réel
ax_reel.plot(-reel["x1"], -reel["y1"], color="blue", lw=2.5, label='Leader (robot1)')
ax_reel.plot(-reel["x2"], -reel["y2"], color="red",  lw=1.5, linestyle='--', label='Follower (robot2)')
ax_reel.plot(-reel["x1"][0], -reel["y1"][0], color="blue", marker='s', markersize=8)
ax_reel.plot(-reel["x2"][0], -reel["y2"][0], color="red",  marker='s', markersize=8)
ax_reel.set_title("Trajectoire XY - Reel", fontsize=12, fontweight='bold')
ax_reel.set_xlabel("x (m)"); ax_reel.set_ylabel("y (m)")
ax_reel.legend(fontsize=9); ax_reel.grid(True); ax_reel.axis('equal')
ax_reel.xaxis.set_major_locator(plt.MultipleLocator(0.5))
ax_reel.yaxis.set_major_locator(plt.MultipleLocator(0.2))

# ── Tableau données brutes (6 instants)
def get_samples(d, m, n=6):
    indices = np.linspace(0, len(d["t1"])-1, n, dtype=int)
    rows = []
    for i in indices:
        t    = d["t1"][i]
        x1   = d["x1"][i];  y1 = d["y1"][i]
        x2   = m["x2i"][i]; y2 = m["y2i"][i]
        dist = m["dist"][i]
        err  = (dist - m["target"]) * 100
        rows.append([f"{t:.1f}", f"{x1:.3f}", f"{y1:.3f}",
                     f"{x2:.3f}", f"{y2:.3f}", f"{dist:.3f}", f"{err:.2f}"])
    return rows

col_labels = ["t (s)", "x_lead", "y_lead", "x_foll", "y_foll", "dist (m)", "err (cm)"]

samples_s = get_samples(sim, ms)
samples_r = get_samples(reel, mr)

all_rows = [["── SIM ──"]*7] + samples_s + [["── REEL ──"]*7] + samples_r

ax_table.axis('off')
tbl = ax_table.table(
    cellText=all_rows,
    colLabels=col_labels,
    loc='center',
    cellLoc='center'
)
tbl.auto_set_font_size(False)
tbl.set_fontsize(8.5)
tbl.scale(1, 1.4)

for j in range(len(col_labels)):
    tbl[0, j].set_facecolor('#2c3e50')
    tbl[0, j].set_text_props(color='white', fontweight='bold')

for j in range(len(col_labels)):
    tbl[1, j].set_facecolor('#d0e4f7')
    tbl[1, j].set_text_props(fontweight='bold')
    tbl[8, j].set_facecolor('#fddede')
    tbl[8, j].set_text_props(fontweight='bold')

for i in range(2, 8):
    for j in range(len(col_labels)):
        if i % 2 == 0:
            tbl[i, j].set_facecolor('#eaf3fb')

for i in range(9, 15):
    for j in range(len(col_labels)):
        if i % 2 == 0:
            tbl[i, j].set_facecolor('#fff0f0')

ax_table.set_title("Donnees brutes (6 instants)", fontsize=12, fontweight='bold')

# ── Erreur de suivi (%)
pct_sim  = ms["err"] / ms["target"] * 100
pct_reel = mr["err"] / mr["target"] * 100
ax_err.plot(sim["t1"],  pct_sim,  color="blue", lw=1.5, label=f'Simulation - MAE={ms["pct_err"]:.2f}%')
ax_err.plot(reel["t1"], pct_reel, color="red",  lw=1.5, linestyle='--', label=f'Reel - MAE={mr["pct_err"]:.2f}%')
ax_err.axhline(0, color='k', lw=1, linestyle='--')
ax_err.set_title("Erreur de suivi (% de la consigne)", fontsize=12, fontweight='bold')
ax_err.set_xlabel("Temps (s)"); ax_err.set_ylabel("Erreur (%)")
ax_err.legend(fontsize=9); ax_err.grid(True)

plt.subplots_adjust(top=0.85, bottom=0.06, left=0.07, right=0.97, hspace=0.6, wspace=0.3)
plt.savefig("comparaison_slam_reel.png", dpi=150, bbox_inches='tight')
print("\nFigure sauvegardee : comparaison_slam_reel.png")
plt.show()