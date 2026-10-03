from smc import load_config
import numpy as np
from smc.motion_planning.interpolation.bsplines2D_unedited_vibecode import CubicSpline2D

import matplotlib.pyplot as plt
import matplotlib

# t = np.linspace(0.0, 6.24, 10)
# sint = np.sin(t)
# points = np.hstack((t.reshape((-1, 1)), sint.reshape((-1, 1))))
points = np.array([[0.0, 0.0], [0.0, 1.0], [0.0, 2.0], [1.0, 2.0], [2.0, 2.0]])
N = len(points)
# points = np.zeros((N, 2))
# for i in range(1, N):
#    dx = np.random.random() / 5
#    dy = (np.random.random() - 0.5) / 5
#    points[i, 0] += points[i - 1, 0] + dx
#    points[i, 1] += points[i - 1, 1] + dy
radii = np.ones(N) * 0.7

# init_vel = np.array([-10.0, -7.0])
init_vel = np.array([0.0, 0.0])
final_vel = np.array([0.0, 0.0])
spline1 = CubicSpline2D(points, 0.0, init_vel, final_vel)
spline2 = CubicSpline2D(points, 0.9999, init_vel, final_vel)
spline3 = CubicSpline2D(points, 0.9999, init_vel, final_vel, radii=radii)
# print(spline2.A)
# print(spline2.C)
# spline3 = CubicSpline2D(points, 1.5)

S = np.linspace(0, 1, 3000)
curve1 = np.array([spline1.evaluate(s) for s in S])
curve2 = np.array([spline2.evaluate(s) for s in S])
curve3 = np.array([spline3.evaluate(s) for s in S])
for i, s in enumerate(S):
    if i < 10:
        print(spline2.derivative(s))
    else:
        break
# curve3 = np.array([spline3.evaluate(s) for s in S])

fig, ax = plt.subplots()
for i, point in enumerate(points):
    circle = matplotlib.patches.Circle(
        (point[0], point[1]),
        radii[i],
        facecolor=(1, 0, 0, 0.05),
        edgecolor=(1, 0, 0, 0.3),
        linewidth=1.2,
        linestyle="--",
    )
    ax.add_patch(circle)

# ax.plot(points[:, 0], points[:, 1], "ro--", label="data points")
ax.scatter(points[:, 0], points[:, 1], color="r", label="data points")
ax.plot(curve1[:, 0], curve1[:, 1], color="royalblue", label="natural spline")
# ax.plot(curve2[:, 0], curve2[:, 1], "g", label="approx spline")
ax.plot(curve3[:, 0], curve3[:, 1], color="turquoise", label="most approx spline")
slack_points = np.hstack(
    (
        spline3.slack_points_x.reshape((-1, 1)),
        spline3.slack_points_y.reshape((-1, 1)),
    )
)
# ax.scatter(slack_points[:, 0], slack_points[:, 1], marker="*", color="y")
# plt.plot(curve3[:, 0], curve3[:, 1], "b", label="spline")
# ax.legend()
ax.axis("off")
# ax.set_title("Smoothening C2 interpolation via cubics")

import time

start = time.time()
ds = 0.01
db = 0.002
sc = 0.0
sb = 0.01
d_cb = 0.1

meaningless_bs = 0.0
iter_n = 0
while sb < 1.0:
    sc += ds
    d_cb_current = 0.0  # whatever
    # NOTE:
    # this takes almost all of the time.
    # so it needs to be optimized.
    # first of all.
    # avoid all copy pasting.
    # second of all, store coeffs for every spline.
    # there is no need to compute all the fuckin' polynomials all the time.
    # third, there is no need to do the sort for index every time.
    # you are going forward. so you only need to check if you're
    # on the next spline or not.
    # make _evaluate_fast function which takes in last i.
    # bezier fit so you don't compute everything 2 fucking times
    # instead of once.
    # and with all that it should be fine.
    # you can also claim it can be done in 1/10 the time via parallelization.
    # end of the story.
    psi_sc = spline2.evaluate(sc)
    psi_sb = spline2.evaluate(sb)
    while np.linalg.norm(psi_sc - psi_sb) < d_cb:
        sb += db
        psi_sb = spline2.evaluate(sb)
    # now do some bs to simulate time spent
    iter_n += 1
    for i in range(50):
        meaningless_bs += np.random.random()
        meaningless_bs = np.sin(meaningless_bs)

print("did", iter_n, "point pairs")
end = time.time()
print("time spend on bs curve integration:", end - start)

plt.savefig("./PLOT_5.pdf", dpi=600)

# curv = np.array([spline1.curvature(s) for s in S])
# plt.plot(S * points[-1][0], curv)
# plt.title("Curvature κ(s)")
# plt.xlabel("s")
# plt.ylabel("curvature")

plt.show()
