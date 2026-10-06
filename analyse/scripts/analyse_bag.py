import numpy as np
import matplotlib.pyplot as plt
from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore

typestore = get_typestore(Stores.ROS2_HUMBLE)

bag_path = "bag_simulation"

# ─── LECTURE DU BAG ───────────────────────────────────────────────────────────
t1, x1, y1, vx1 = [], [], [], []
t2, x2, y2, vx2 = [], [], [], []
t_cmd1, vcmd1 = [], []
t_cmd2, vcmd2, wcmd2 = [], [], []

with Reader(bag_path) as reader:
    for conn, timestamp, rawdata in reader.messages():
        t_sec = timestamp * 1e-9

        if conn.topic == "/robot1/odom":
            msg = typestore.deserialize_cdr(rawdata, conn.msgtype)
            t1.append(t_sec)
            x1.append(msg.pose.pose.position.x)
            y1.append(msg.pose.pose.position.y)
            vx1.append(msg.twist.twist.linear.x)

        elif conn.topic == "/robot2/odom":
            msg = typestore.deserialize_cdr(rawdata, conn.msgtype)
            t2.append(t_sec)
            x2.append(msg.pose.pose.position.x)
            y2.append(msg.pose.pose.position.y)
            vx2.append(msg.twist.twist.linear.x)

        elif conn.topic == "/robot1/cmd_vel":
            msg = typestore.deserialize_cdr(rawdata, conn.msgtype)
            t_cmd1.append(t_sec)
            vcmd1.append(msg.linear.x)

        elif conn.topic == "/robot2/cmd_vel":
            msg = typestore.deserialize_cdr(rawdata, conn.msgtype)
            t_cmd2.append(t_sec)
            vcmd2.append(msg.linear.x)
            wcmd2.append(msg.angular.z)

# ─── CONVERSION EN NUMPY ──────────────────────────────────────────────────────
t1 = np.array(t1);   x1 = np.array(x1);   y1 = np.array(y1);   vx1 = np.array(vx1)
t2 = np.array(t2);   x2 = np.array(x2);   y2 = np.array(y2);   vx2 = np.array(vx2)
t_cmd2 = np.array(t_cmd2); vcmd2 = np.array(vcmd2); wcmd2 = np.array(wcmd2)

# Normalisation du temps
t0 = min(t1[0], t2[0])
t1 -= t0; t2 -= t0; t_cmd2 -= t0

# ─── CALCUL DISTANCE INTER-ROBOTS ────────────────────────────────────────────
# On interpole robot2 sur les timestamps de robot1
x2_interp = np.interp(t1, t2, x2)
y2_interp = np.interp(t1, t2, y2)
dist = np.sqrt((x1 - x2_interp)**2 + (y1 - y2_interp)**2)

# ─── MÉTRIQUES ───────────────────────────────────────────────────────────────
target_dist = dist[0]  # distance initiale = consigne calibrée
dist_error = dist - target_dist

rmse   = np.sqrt(np.mean(dist_error**2))
mean_e = np.mean(np.abs(dist_error))
max_e  = np.max(np.abs(dist_error))
std_e  = np.std(dist_error)

print("\n" + "="*55)
print("       ANALYSE DU BAG - SIMULATION")
print("="*55)
print(f"  Durée enregistrée         : {t1[-1]:.1f} s")
print(f"  Distance de consigne      : {target_dist:.3f} m")
print(f"  Distance moyenne mesurée  : {np.mean(dist):.3f} m")
print(f"  Erreur moyenne (MAE)      : {mean_e*100:.2f} cm")
print(f"  Erreur max                : {max_e*100:.2f} cm")
print(f"  Écart-type erreur         : {std_e*100:.2f} cm")
print(f"  RMSE distance             : {rmse*100:.2f} cm")
print(f"  Vitesse max robot1        : {np.max(vcmd1):.3f} m/s")
print(f"  Vitesse max robot2 (lin)  : {np.max(vcmd2):.3f} m/s")
print(f"  Vitesse ang max robot2    : {np.max(np.abs(wcmd2)):.3f} rad/s")
print("="*55 + "\n")

# ─── FIGURES ─────────────────────────────────────────────────────────────────
fig, axes = plt.subplots(2, 2, figsize=(14, 10))
fig.suptitle("Analyse Rosbag – Simulation\nRobot Porteur (Leader/Follower)", fontsize=14, fontweight='bold')

# 1. Trajectoires
ax = axes[0, 0]
ax.plot(x1, y1, 'b-', linewidth=2, label='Leader (robot1)')
ax.plot(x2, y2, 'r--', linewidth=2, label='Follower (robot2)')
ax.plot(x1[0], y1[0], 'bs', markersize=8)
ax.plot(x2[0], y2[0], 'rs', markersize=8)
ax.set_title("Trajectoires XY")
ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)")
ax.legend(); ax.grid(True); ax.axis('equal')

# 2. Distance inter-robots
ax = axes[0, 1]
ax.plot(t1, dist, 'g-', linewidth=1.5, label='Distance mesurée')
ax.axhline(target_dist, color='k', linestyle='--', linewidth=1.5, label=f'Consigne ({target_dist:.2f} m)')
ax.fill_between(t1, target_dist - 0.05, target_dist + 0.05, alpha=0.15, color='green', label='±5 cm')
ax.set_title("Distance Leader → Follower")
ax.set_xlabel("Temps (s)"); ax.set_ylabel("Distance (m)")
ax.legend(); ax.grid(True)

# 3. Vitesses linéaires
ax = axes[1, 0]
ax.plot(t_cmd1, vcmd1, 'b-', linewidth=1.5, label='Leader cmd_vel')
ax.plot(t_cmd2, vcmd2, 'r-', linewidth=1.5, alpha=0.8, label='Follower cmd_vel')
ax.set_title("Commandes vitesse linéaire")
ax.set_xlabel("Temps (s)"); ax.set_ylabel("v (m/s)")
ax.legend(); ax.grid(True)

# 4. Erreur de distance
ax = axes[1, 1]
ax.plot(t1, dist_error * 100, 'm-', linewidth=1.5)
ax.axhline(0, color='k', linestyle='--', linewidth=1)
ax.axhline(rmse * 100, color='r', linestyle=':', linewidth=1.5, label=f'RMSE = {rmse*100:.2f} cm')
ax.axhline(-rmse * 100, color='r', linestyle=':', linewidth=1.5)
ax.set_title("Erreur de suivi (distance)")
ax.set_xlabel("Temps (s)"); ax.set_ylabel("Erreur (cm)")
ax.legend(); ax.grid(True)

plt.tight_layout()
plt.savefig("analyse_simulation.png", dpi=150, bbox_inches='tight')
print("Figure sauvegardée : analyse_simulation.png")
plt.show()
