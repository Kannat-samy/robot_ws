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
print("Chargement bag_sim_amcl...")
amcl = read_bag("bag_sim_amcl")
ma   = compute_metrics(amcl)

print("Chargement bag_sim_slam...")
slam = read_bag("bag_sim_slam")
ms   = compute_metrics(slam)

# ─── TERMINAL : tableau métriques ────────────────────────────────────────────
print("\n" + "="*65)
print(f"{'PARAMETRE':<35} {'AMCL':>12} {'SLAM':>12}")
print("="*65)
rows_metrics = [
    ("Duree enregistree (s)",          f"{ma['dur']:.1f}",           f"{ms['dur']:.1f}"),
    ("Distance de consigne (m)",        f"{ma['target']:.3f}",        f"{ms['target']:.3f}"),
    ("Distance moyenne mesuree (m)",    f"{np.mean(ma['dist']):.3f}", f"{np.mean(ms['dist']):.3f}"),
    ("Longueur trajectoire (m)",        f"{ma['traj_len']:.2f}",      f"{ms['traj_len']:.2f}"),
    ("Erreur moyenne MAE (cm)",         f"{ma['mae']*100:.2f}",       f"{ms['mae']*100:.2f}"),
    ("Erreur max (cm)",                 f"{ma['max_err']*100:.2f}",   f"{ms['max_err']*100:.2f}"),
    ("Ecart-type erreur (cm)",          f"{ma['std_err']*100:.2f}",   f"{ms['std_err']*100:.2f}"),
    ("RMSE distance (cm)",              f"{ma['rmse']*100:.2f}",      f"{ms['rmse']*100:.2f}"),
    ("Erreur en % (MAE/consigne)",      f"{ma['pct_err']:.2f} %",     f"{ms['pct_err']:.2f} %"),
]
for name, va, vs in rows_metrics:
    print(f"  {name:<33} {va:>12} {vs:>12}")
print("="*65)

# ─── FIGURE ──────────────────────────────────────────────────────────────────
fig = plt.figure(figsize=(20, 14))
fig.suptitle("Comparaison AMCL vs SLAM - Simulation\nRobot Porteur (Leader/Follower)",
             fontsize=15, fontweight="bold", y=0.98)

ax_amcl  = fig.add_subplot(2, 2, 1)
ax_slam  = fig.add_subplot(2, 2, 2)
ax_table = fig.add_subplot(2, 2, 3)
ax_err   = fig.add_subplot(2, 2, 4)

# ── Trajectoire AMCL
ax_amcl.plot(amcl["x1"], amcl["y1"], color="blue", lw=2.5, label='Leader (robot1)')
ax_amcl.plot(amcl["x2"], amcl["y2"], color="red",  lw=1.5, linestyle='--', label='Follower (robot2)')
ax_amcl.plot(amcl["x1"][0], amcl["y1"][0], color="blue", marker='s', markersize=8)
ax_amcl.plot(amcl["x2"][0], amcl["y2"][0], color="red",  marker='s', markersize=8)
ax_amcl.set_title("Trajectoire XY - AMCL", fontsize=12, fontweight='bold')
ax_amcl.set_xlabel("x (m)"); ax_amcl.set_ylabel("y (m)")
ax_amcl.legend(fontsize=9); ax_amcl.grid(True); ax_amcl.axis('equal')

# ── Trajectoire SLAM
ax_slam.plot(slam["x1"], slam["y1"], color="blue", lw=2.5, label='Leader (robot1)')
ax_slam.plot(slam["x2"], slam["y2"], color="red",  lw=1.5, linestyle='--', label='Follower (robot2)')
ax_slam.plot(slam["x1"][0], slam["y1"][0], color="blue", marker='s', markersize=8)
ax_slam.plot(slam["x2"][0], slam["y2"][0], color="red",  marker='s', markersize=8)
ax_slam.set_title("Trajectoire XY - SLAM", fontsize=12, fontweight='bold')
ax_slam.set_xlabel("x (m)"); ax_slam.set_ylabel("y (m)")
ax_slam.legend(fontsize=9); ax_slam.grid(True); ax_slam.axis('equal')

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

samples_a = get_samples(amcl, ma)
samples_s = get_samples(slam, ms)

# all_rows : 1 header AMCL + 6 lignes + 1 header SLAM + 6 lignes = 14 lignes
all_rows = [["── AMCL ──"]*7] + samples_a + [["── SLAM ──"]*7] + samples_s

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

# Header colonnes
for j in range(len(col_labels)):
    tbl[0, j].set_facecolor('#2c3e50')
    tbl[0, j].set_text_props(color='white', fontweight='bold')

# Ligne AMCL header = row 1, ligne SLAM header = row 8 (1 + 6 + 1)
for j in range(len(col_labels)):
    tbl[1, j].set_facecolor('#d0e4f7')
    tbl[1, j].set_text_props(fontweight='bold')
    tbl[8, j].set_facecolor('#fddede')
    tbl[8, j].set_text_props(fontweight='bold')

# Alternance AMCL (rows 2-7)
for i in range(2, 8):
    for j in range(len(col_labels)):
        if i % 2 == 0:
            tbl[i, j].set_facecolor('#eaf3fb')

# Alternance SLAM (rows 9-14)
for i in range(9, 15):
    for j in range(len(col_labels)):
        if i % 2 == 0:
            tbl[i, j].set_facecolor('#fff0f0')

ax_table.set_title("Donnees brutes (6 instants)", fontsize=12, fontweight='bold')

# ── Erreur de suivi (%)
pct_amcl = ma["err"] / ma["target"] * 100
pct_slam = ms["err"] / ms["target"] * 100
ax_err.plot(amcl["t1"], pct_amcl, color="blue", lw=1.5, label=f'AMCL - MAE={ma["pct_err"]:.2f}%')
ax_err.plot(slam["t1"], pct_slam, color="red",  lw=1.5, linestyle='--', label=f'SLAM - MAE={ms["pct_err"]:.2f}%')
ax_err.axhline(0, color='k', lw=1, linestyle='--')
ax_err.set_title("Erreur de suivi (% de la consigne)", fontsize=12, fontweight='bold')
ax_err.set_xlabel("Temps (s)"); ax_err.set_ylabel("Erreur (%)")
ax_err.legend(fontsize=9); ax_err.grid(True)

plt.subplots_adjust(top=0.92, bottom=0.06, left=0.07, right=0.97, hspace=0.45, wspace=0.3)
plt.savefig("comparaison_amcl_slam.png", dpi=150, bbox_inches='tight')
print("\nFigure sauvegardee : comparaison_amcl_slam.png")
plt.show()