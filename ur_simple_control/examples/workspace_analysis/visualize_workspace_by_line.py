from smc import load_config
import matplotlib.pylab as plt
import numpy as np
import pickle

# NOTE: sqrt warnings are super annoying and contribute exactly nothing
import warnings

warnings.filterwarnings("ignore")

# TODO:
# can add arclength option as visualization.
# then you can see what's better.
# of course this is much more complex as you're now sweeping an area,
# as the involutes are different depending on curve they depend on.
# but you know what the effective max curvature of a path you're
# taking an involute from it.

# TODO:
# and then the question is simple:
# what is the difference in steerability?
# namely. find the best circle.
# then, for a given (x,y) point on circle,
# if you go up and down along x, does you're steerability increase?
# if you go up and down y, does your steerability increase?
# how much you go up and down? --> as much as your
# all the involutes sweep.
# what is the best arclength?
# the one whose involutes cover the largest area.
# you're comparing against the best of each option.
# and that's it.


# TODO: most of this god forsaken file has to be
# rewritten to be in polar coordinates.
# this shit is ridiculous.


# NOTE: a whole bunch of things here should have been done through interpolation.
# i can't be bothered. the points are sampled densly enough. i just
# floor/ceil to closest array element.
# it's fine.

# load data
headings = np.loadtxt("./data/headings.csv", delimiter=",")
file = open("./data/workspace_mask_ALL.pickle", "rb")
workspace_mask = pickle.load(file)
file.close()
xs = np.loadtxt("./data/xs.csv", delimiter=",")
ys = np.loadtxt("./data/ys.csv", delimiter=",")

XS, YS, ZS = np.meshgrid(xs, ys, headings)

radius = 0.88
# center = (xs[len(ys) // 2], ys[len(ys) // 2])
# center = (xs[len(ys) // 2], 0.0)
center = (-0.52, 0.0)


# find indeces corresponding to a curve
def circle(center, radius, y) -> float:
    # x**2 + y**2 = radius**2
    # y = sqrt(radius**2 - x**2)
    # (x-x0)**2 + (y-y0)**2 = radius**2
    # (y-y0) = sqrt(radius**2 - (x-x0)**2)
    return np.sqrt(radius**2 - (y - center[1]) ** 2) + center[0]


# find indeces corresponding to a curve
# def circleDOWN(center, radius, y) -> float:
#    # x**2 + y**2 = radius**2
#    # y = sqrt(radius**2 - x**2)
#    # (x-x0)**2 + (y-y0)**2 = radius**2
#    # (y-y0) = sqrt(radius**2 - (x-x0)**2)
#    return -np.sqrt(radius**2 - (y - center[1]) ** 2) + center[0]


def circleViaPolar(center, radius, xs, ys):
    N_pts = 200
    thetas = np.linspace(-3.14, 3.14, N_pts)
    cts = radius * np.cos(thetas)
    sts = radius * np.sin(thetas)
    circle_xs = []
    circle_ys = []
    for i in range(N_pts):
        x = cts[i] + center[0]
        y = sts[i] + center[1]
        if (x < xs[0] or x > xs[-1]) or (y < ys[0] or y > ys[-1]):
            continue
        circle_xs.append(x)
        circle_ys.append(y)
    return np.array(circle_xs), np.array(circle_ys)


def findCircularSlice(center, radius, xs, ys, headings, workspace_mask):
    xs_circle = []
    ys_circle = []
    zs_circle_IN = []
    zs_circle_OUT = []
    for j, y_j in enumerate(ys):
        x = circle(center, radius, y_j)
        i = np.searchsorted(xs, x)
        if i >= len(xs):
            continue
        ys_circle.append(y_j)
        xs_circle.append(xs[i])
        # NOTE: this rotation is purely an artefact on a bad initial decision
        headings_rotated = headings.copy()
        headings_rotated = headings_rotated + 3.14
        headings_rotated[headings_rotated > 3.14] = (
            headings_rotated[headings_rotated > 3.14] - 6.28
        )
        zs_circle_IN.append(headings_rotated[workspace_mask[i, j, :]])
        zs_circle_OUT.append(headings_rotated[np.bitwise_not(workspace_mask[i, j, :])])

    xs_circle = np.array(xs_circle)
    ys_circle = np.array(ys_circle)
    return xs_circle, ys_circle, zs_circle_IN, zs_circle_OUT


# NOTE: in case you want more than the top half of the circle
# i have to split the circle into up and down because i need to plot it
# in cartesian coordinates.
# otherwise the pcolormesh is also in polar coordinates lmao fml
def findFullCircularSlice(center, radius, xs, ys, headings, workspace_mask):
    xs_circle_up = []
    xs_circle_down = []
    ys_circle = []
    zs_circle_IN_UP = []
    zs_circle_IN_DOWN = []
    zs_circle_OUT_UP = []
    zs_circle_OUT_DOWN = []
    for j, y_j in enumerate(ys):
        x_up = circle(center, radius, y_j)
        x_down = -x_up
        i_up = np.searchsorted(xs, x_up)
        # if it's out of scope, i'll get 0.
        # which is fine, i don't care if i plot 50 times over the same point
        i_down = np.searchsorted(xs, x_down)
        if i_up >= len(xs):
            continue
        ys_circle.append(y_j)
        xs_circle_up.append(xs[i_up])
        xs_circle_down.append(xs[i_down])
        # NOTE: this rotation is purely an artefact on a bad initial decision
        headings_rotated = headings.copy()
        headings_rotated = headings_rotated + 3.14
        headings_rotated[headings_rotated > 3.14] = (
            headings_rotated[headings_rotated > 3.14] - 6.28
        )
        zs_circle_IN_UP.append(headings_rotated[workspace_mask[i_up, j, :]])
        zs_circle_OUT_UP.append(
            headings_rotated[np.bitwise_not(workspace_mask[i_up, j, :])]
        )
        zs_circle_IN_DOWN.append(headings_rotated[workspace_mask[i_down, j, :]])
        zs_circle_OUT_DOWN.append(
            headings_rotated[np.bitwise_not(workspace_mask[i_down, j, :])]
        )

    xs_circle_up = np.array(xs_circle_up)
    ys_circle = np.array(ys_circle)
    return (
        (xs_circle_up, xs_circle_down),
        (ys_circle, ys_circle),
        (zs_circle_IN_UP, zs_circle_IN_DOWN),
        (zs_circle_OUT_UP, zs_circle_OUT_DOWN),
    )


def findFullCircularSlicePolar(center, radius, xs, ys, headings, workspace_mask):
    xs_circle = []
    ys_circle = []
    xy_indeces_checked = set()
    zs_circle_IN = []
    zs_circle_OUT = []
    phis = np.linspace(-np.pi, np.pi, 400)
    for j, phi in enumerate(phis):
        x = np.cos(phi) * radius + center[0]
        i = min(np.searchsorted(xs, x), len(xs) - 1)
        y = np.sin(phi) * radius + center[1]
        j = min(np.searchsorted(ys, y), len(ys) - 1)
        if (i, j) not in xy_indeces_checked:
            xs_circle.append(xs[i])
            ys_circle.append(ys[j])
            xy_indeces_checked.add((i, j))
            # NOTE: this rotation is purely an artefact on a bad initial decision
            headings_rotated = headings.copy()
            headings_rotated = headings_rotated + 3.14
            headings_rotated[headings_rotated > 3.14] = (
                headings_rotated[headings_rotated > 3.14] - 6.28
            )
            zs_circle_IN.append(headings_rotated[workspace_mask[i, j, :]])
            zs_circle_OUT.append(
                headings_rotated[np.bitwise_not(workspace_mask[i, j, :])]
            )

    xs_circle_up = np.array(xs_circle)
    ys_circle = np.array(ys_circle)
    return (
        xs_circle,
        ys_circle,
        zs_circle_IN,
        zs_circle_OUT,
    )


def plotCircularSlice(xs_circle, ys_circle, zs_circle_IN, zs_circle_OUT):
    fig = plt.figure()
    ax = fig.add_subplot(projection="3d")
    for i in range(len(xs_circle)):
        ax.scatter(xs_circle[i], ys_circle[i], zs_circle_IN[i], color="b")
        ax.scatter(xs_circle[i], ys_circle[i], zs_circle_OUT[i], color="r")
    ax.set_xlim(xs[0], xs[-1])
    ax.set_ylim(ys[0], ys[-1])
    ax.set_xlabel("x/m")
    ax.set_ylabel("y/m")
    ax.set_zlabel("heading/rad")
    plt.show()


def plotFullCircularSlice(xs_circle, ys_circle, zs_circle_IN, zs_circle_OUT):
    fig = plt.figure()
    ax = fig.add_subplot(projection="3d")
    xs_circle_UP, xs_circle_DOWN = xs_circle
    ys_circle_UP, ys_circle_DOWN = ys_circle
    zs_circle_IN_UP, zs_circle_IN_DOWN = zs_circle_IN
    zs_circle_OUT_UP, zs_circle_OUT_DOWN = zs_circle_OUT
    for i in range(len(xs_circle_UP)):
        ax.scatter(xs_circle_UP[i], ys_circle_UP[i], zs_circle_IN_UP[i], color="b")
        ax.scatter(xs_circle_UP[i], ys_circle_UP[i], zs_circle_OUT_UP[i], color="r")
    for i in range(len(xs_circle_DOWN)):
        ax.scatter(
            xs_circle_DOWN[i], ys_circle_DOWN[i], zs_circle_IN_DOWN[i], color="b"
        )
        ax.scatter(
            xs_circle_DOWN[i], ys_circle_DOWN[i], zs_circle_OUT_DOWN[i], color="r"
        )
    ax.set_xlim(xs[0], xs[-1])
    ax.set_ylim(ys[0], ys[-1])
    ax.set_xlabel("x/m")
    ax.set_ylabel("y/m")
    ax.set_zlabel("heading/rad")
    plt.show()


# NOTE: some headings have to be discarded for this to be valid.
# TODO: alternatively, and probably more inteligently,
# you can re-run the experiment, but also make self-collisions of cart and base invalid.
# and/or self-collision of arms also.
# this is not to be done here.
# find best radius - the one with most blues

headings_rotated = headings.copy()
headings_rotated = headings_rotated + 3.14
headings_rotated[headings_rotated > 3.14] = (
    headings_rotated[headings_rotated > 3.14] - 6.28
)
# workspace heatmap
feasible_headings = np.zeros((len(xs), len(ys)))
for i in range(len(xs)):
    for j in range(len(ys)):
        # feasible_headings[i][j] = np.sum(workspace_mask[i, j, :])
        h = headings_rotated[workspace_mask[i, j, :]]
        if len(h) > 0:
            feasible_headings[i][j] = np.max(h) - np.min(h)
        else:
            feasible_headings[i][j] = 0.0

# NOTE: go through both radii and center positions see what's best
N_checks = 100
# center_i, radius_i
scores = np.zeros((N_checks, N_checks))
# BIG NOTE: THIS WAS USED FOR PAPER
# radii = np.linspace(0.01, 0.7 + 0.52, N_checks)
#center_xs = np.linspace(-0.52, 0.0, N_checks)

# NOTE: on the real robot, this is too close to the safety distance checker.
# now, this should obviously be disabled for the cart pulling task.
# but, i can't disable it.
# so i'll use a larger radius
radii = np.linspace(0.2, 0.7 + 0.52, N_checks)
center_xs = np.linspace(-0.52, 0.0, N_checks)
SLUMI_LIMITS = True
for c, center_x in enumerate(center_xs):
    for i, radius in enumerate(radii):
        if SLUMI_LIMITS and (0.52 - abs(0.52 + center_x) + radius < 0.9):
            continue
        center = (center_x, 0.0)
        xs_circle, ys_circle, zs_circle_IN, zs_circle_OUT = findFullCircularSlice(
            center, radius, xs, ys, headings, workspace_mask
        )
        n_zs_in = 0
        n_zs_out = 0
        for zs_in_ij in zs_circle_IN[0]:
            n_zs_in += len(zs_in_ij)
        for zs_in_ij in zs_circle_IN[1]:
            n_zs_in += len(zs_in_ij)
        for zs_out_ij in zs_circle_OUT[0]:
            n_zs_out += len(zs_out_ij)
        for zs_out_ij in zs_circle_OUT[1]:
            n_zs_out += len(zs_out_ij)
        if n_zs_in == 0:
            continue

        # NOTE: alternative
        # xs_circle, ys_circle, zs_circle_IN, zs_circle_OUT = findFullCircularSlicePolar(
        #    center, radius, xs, ys, headings, workspace_mask
        # )
        # n_zs_in = 0
        # n_zs_out = 0
        # for zs_in_ij in zs_circle_IN:
        #    n_zs_in += len(zs_in_ij)
        # for zs_out_ij in zs_circle_OUT:
        #    n_zs_out += len(zs_out_ij)
        scores[c][i] = n_zs_in / (n_zs_in + n_zs_out)

best_radius_at_base_center = radii[np.argmax(scores[0, :])]
print(
    "for circle with center at mobile base frame, the best radius is:",
    radii[np.argmax(scores[0, :])],
)

best_index = np.argmax(scores)
best_index_i = best_index // N_checks
best_index_j = best_index % N_checks
best_center = center_xs[best_index_i]
best_radius = radii[best_index_j]
print("=" * 40)
print("found best radius:", best_radius)
print("it's at center:", best_center)
fig, ax = plt.subplots()
CENTER_XS, RADII = np.meshgrid(center_xs, radii)
im = ax.pcolormesh(CENTER_XS, RADII, scores)
ax.set_xlabel("center_x/m")
ax.set_ylabel("radius/m")
ax.set_title("radius and center combos for most heading")
fig.colorbar(im, ax=ax)
plt.show()

XS2D, YS2D = np.meshgrid(ys, xs)
fig, ax = plt.subplots()
im = ax.pcolormesh(XS2D, YS2D, feasible_headings)
best_center = (best_center, 0.0)
# circle_xs = np.sqrt(best_radius**2 - (ys - best_center[1]) ** 2) + best_center[0]
# ax.plot(ys, circle_xs, color="r")
# circle_xs = -np.sqrt(best_radius**2 - (ys - best_center[1]) ** 2) + best_center[0]
# ax.plot(ys, circle_xs, color="r")


xs_circle, ys_circle, zs_circle_IN, zs_circle_OUT = findFullCircularSlicePolar(
    best_center, best_radius, xs, ys, headings, workspace_mask
)
ax.plot(ys_circle, xs_circle, color="r")
ax.set_xlabel("y/m")
ax.set_ylabel("x/m")
# ax.set_title("best cart distance from base")
fig.colorbar(im, ax=ax)
plt.savefig("./technical_report/workspace_with_best_radius.pdf", dpi=600)
plt.show()

# plt.plot(radii, scores, label="average percentage of available headings")
# plt.title("score per radius")
# plt.xlabel("radius/m")
# plt.ylabel("score/percentage")
# print("the best radius is", best_radius)
# plt.show()


def circle_intersections(center1, r1, center2, r2):
    """
    Returns intersection points of two circles:
    (x1, y1, r1) and (x2, y2, r2)

    Returns:
        None -> no intersections
        [(x, y)] -> one intersection (tangent)
        [(xA, yA), (xB, yB)] -> two intersections
    """

    # Distance between centers
    dx = center2[0] - center1[0]
    dy = center2[1] - center1[1]
    d = np.sqrt(dx**2 + dy**2)

    # No solution cases
    if d > r1 + r2:
        return None  # separate circles
    if d < abs(r1 - r2):
        return None  # one inside the other
    if d == 0 and r1 == r2:
        return None  # coincident circles (infinite solutions)

    # Compute a and h
    a = (r1**2 - r2**2 + d**2) / (2 * d)
    h = np.sqrt(max(r1**2 - a**2, 0))

    # Base point
    x3 = center1[0] + a * dx / d
    y3 = center1[1] + a * dy / d

    # Intersection points
    rx = -dy * (h / d)
    ry = dx * (h / d)

    p1 = (x3 + rx, y3 + ry)
    p2 = (x3 - rx, y3 - ry)

    # Tangent case (one intersection)
    if h == 0:
        return [p1]

    return [p1, p2]


# NOTE: find the path of highest curvature for this workspace and
# robot base position.
# to make this well defined, the path is a circle as they have constant
# curvature and random curve's curvature approximation is an oscullating
# circle. so this is a well established reasonable thing to do.
# circles have fixed curvature kappa = 1/r
# since we already have a circle in the code, this circle is called kappa.
N_checks = 400
r_kappa = 0.22
# putting it to the left 'cos heatmap label is on the right
center_kappa = [-0.52, -r_kappa]
delta_r_kappa = 0.01
# thetas_kappa = np.linspace(0, np.pi / 2, N_checks)
# ctk = np.cos(thetas_kappa)
# stk = np.sin(thetas_kappa)
# find first point in workspace with feasible heading
workspace_point_not_found = True
# find first point on cart circle intersection with feasible heading
cart_circle_point_not_found = True


first_flag = False
checked_intersections_x = []
checked_intersections_y = []
psi_intersected = []
ind = 0
n_print = 100
print("=" * 40)
x_kappas_in_worksapce = []
y_kappas_in_worksapce = []
while workspace_point_not_found or cart_circle_point_not_found:
    ind += 1
    r_kappa += delta_r_kappa
    #    if ind % n_print == 0:
    #        print("checking r_kappa:", r_kappa)
    if r_kappa > 10.0:
        print("r_kappa literally does not exist, we did a bad")
        break
    # putting it to the left 'cos heatmap label is on the right
    center_kappa[1] = -r_kappa

    # this results in massive jumps for bigger radii.
    # so it must be scaled.
    dx = 0.01
    delta_theta = dx / r_kappa
    # i want to do 1.5m along circle say
    theta_max = N_checks * delta_theta
    theta_max = min(theta_max, np.pi)
    #    if ind % n_print == 0:
    #        print("theta_max", theta_max)
    thetas_kappa = np.linspace(0, N_checks * delta_theta, N_checks)
    ctk = np.cos(thetas_kappa)
    stk = np.sin(thetas_kappa)

    # NOTE: tangent at base is correct by circle_kappa construction.
    # now we need to, for every (x_kappa,y_kappa) in xs,ys range check whether
    # heading_kappa_xy is in range of headings_xy.
    x_kappas = []
    y_kappas = []
    for k in range(N_checks):
        y_kappa = r_kappa * ctk[k] + center_kappa[1]
        x_kappa = r_kappa * stk[k] + center_kappa[0]
        x_kappas.append(x_kappa)
        y_kappas.append(y_kappa)
        # print("circle_kappa_pt:", x_kappa, y_kappa)
        if x_kappa < xs[0]:
            continue
        if x_kappa > xs[-1]:
            break
        # idk if i even need this one, but who cares
        if y_kappa < ys[0] or y_kappa > ys[-1]:
            continue
        # this is the heading of circle_kappa at point x,y - ultra basic geometry on paper
        # psi = np.pi - thetas_kappa[k]
        psi = thetas_kappa[k]
        #        if ind % n_print == 0:
        #            print("requesting psi", psi)
        # psi = thetas_kappa[k]
        # psi = thetas_kappa[k]
        # print("psi:", psi)
        i = np.searchsorted(xs, x_kappa)
        j = np.searchsorted(ys, y_kappa)
        # print("closest workspace pt:", xs[i], ys[j])
        if len(headings_rotated[workspace_mask[i, j, :]]) == 0:
            continue
        # NOTE: i'm straightout assuming zs_circle_in are all monotone in one interval.
        # which they are for the most part - critically, where relevant.
        # if i get fishy results, i'll find the largest interval including 0,
        # which is the correct thing.
        if (psi > np.min(headings_rotated[workspace_mask[i, j, :]])) and (
            psi < np.max(headings_rotated[workspace_mask[i, j, :]])
        ):
            # print(headings_rotated[workspace_mask[i, j, :]])
            workspace_point_not_found = False
            if not first_flag:
                print(
                    "theoretic max curvature of robot-cart-system is:",
                    1 / r_kappa,
                    "found for oscullating radius",
                    r_kappa,
                    "at the gold star location, with required heading",
                    psi,
                )
                #                print(
                #                    "FIRST: found best kappa at radius:",
                #                    r_kappa,
                #                    "location(",
                #                    x_kappa,
                #                    y_kappa,
                #                    ")",
                #                )
                r_kappa_best = r_kappa
                center_kappa_best = [-0.52, -r_kappa]
                first_flag = True
                x_kappa_best = x_kappa
                y_kappa_best = y_kappa
            print("-" * 40)
            print(
                f"BEST_KAPPA: at radius {r_kappa}, requested heading {psi} found in range [{np.min(headings_rotated[workspace_mask[i, j, :]])}, {np.max(headings_rotated[workspace_mask[i, j, :]])}], at position on circle [{x_kappa}, {y_kappa}] which in workspace grid corresponds to [{xs[i]},{ys[j]}]"
            )
            x_kappas_in_worksapce.append(x_kappa)
            y_kappas_in_worksapce.append(y_kappa)
            break
        else:
            workspace_point_not_found = True
    #    plt.plot(np.array(y_kappas), np.array(x_kappas))  # , color="r")
    #    if ind % n_print == 0:
    #        print("largest xy", x_kappa, y_kappa)

    # print(y_kappa, x_kappa)
    # print("requesting psi", psi)
    if not workspace_point_not_found:
        # find intersection with cart circle
        #        if r_kappa > 1.0:
        #            break
        intersections = circle_intersections(
            center_kappa, r_kappa, best_center, best_radius
        )
        # print("intersections", intersections)
        # NOTE: i already know intersection exists
        # and it's the second one.
        for intersection in intersections:
            # intersection = intersections[1]
            if intersection[0] < xs[0] or intersection[0] > xs[-1]:
                continue
            # idk if i even need this one, but who cares
            if intersection[1] < ys[0] or intersection[1] > ys[-1]:
                continue
            i = np.searchsorted(xs, intersection[0])
            j = np.searchsorted(ys, intersection[1])
            checked_intersections_x.append(intersection[0])
            checked_intersections_y.append(intersection[1])
            psi_intersected.append(psi)
            #            print("-" * 40)
            #            print(
            #                f"INTERSETION: requesting heading {psi} from range [{np.min(headings_rotated[workspace_mask[i, j, :]])}, {np.max(headings_rotated[workspace_mask[i, j, :]])}], at intersection position [{intersection[0]}, {intersection[1]}] which in workspace grid corresponds to [{xs[i]},{ys[j]}]"
            #            )
            if len(headings_rotated[workspace_mask[i, j, :]]) <=0:
                continue
            if (psi > np.min(headings_rotated[workspace_mask[i, j, :]])) and (
                psi < np.max(headings_rotated[workspace_mask[i, j, :]])
            ):
                print("FOUND workable max curvature !!!")
                # print(headings_rotated[workspace_mask[i, j, :]])
                cart_circle_point_not_found = False
                r_kappa_intersected = r_kappa
                center_kappa_intersected = [-0.52, -r_kappa]
                # print("-" * 40)
                # print(
                #    f"requested heading {psi} found in range [{np.min(headings_rotated[workspace_mask[i, j, :]])}, {np.max(headings_rotated[workspace_mask[i, j, :]])}], at intersection position [{intersection[0]}, {intersection[1]}] which in workspace grid corresponds to [{xs[i]},{ys[j]}]"
                # )
                break


# plt.show()
print(
    "workable max curvature of robot-cart-system is:",
    1 / r_kappa,
    "found for oscullating radius",
    r_kappa,
    "with requested heading",
    psi,
)
XS2D, YS2D = np.meshgrid(ys, xs)
fig, ax = plt.subplots()
im = ax.pcolormesh(XS2D, YS2D, feasible_headings)
# cart distance circle
circle_xs = np.sqrt(best_radius**2 - (ys - best_center[1]) ** 2) + best_center[0]
ax.plot(ys, circle_xs, color="r")
# oscullating circle
circle_kappa_xs_best = (
    np.sqrt(r_kappa_best**2 - (ys - center_kappa_best[1]) ** 2) + center_kappa_best[0]
)
kappa_mask = (circle_kappa_xs_best > xs[0]) * (circle_kappa_xs_best < xs[-1])
ax.plot(ys[kappa_mask], circle_kappa_xs_best[kappa_mask], color="r")
# oscullating circle intersecting cart circle
circle_kappa_xs_intersected = (
    np.sqrt(r_kappa_intersected**2 - (ys - center_kappa_intersected[1]) ** 2)
    + center_kappa_intersected[0]
)
kappa_mask = (circle_kappa_xs_intersected > xs[0]) * (
    circle_kappa_xs_intersected < xs[-1]
)
ax.plot(ys[kappa_mask], circle_kappa_xs_intersected[kappa_mask], color="r")
# ax.plot(ys, circle_kappa_xs, color="r")
ax.set_xlabel("y/m")
ax.set_ylabel("x/m")
# for fun let's plot the exact point the oscullating circle is satisfied
ax.scatter(np.array([y_kappa_best]), np.array([x_kappa_best]), marker="*", color="y")
ax.scatter(
    np.array(y_kappas_in_worksapce),
    np.array(x_kappas_in_worksapce),
    marker="*",
    color="orange",
)
ax.scatter(
    np.array(checked_intersections_y),
    np.array(checked_intersections_x),
    marker="*",
    color="m",
)
CY, CX = np.meshgrid(checked_intersections_y, checked_intersections_x)
# V = np.cos(CY)  # * 0.1
# U = np.sin(CX)  # * 0.1
psi_intersectedY, psi_intersectedX = np.meshgrid(psi_intersected, psi_intersected)
V = np.cos(psi_intersectedY)
U = np.sin(psi_intersectedX)

# print("intersected psis are:", psi_intersected)
# ax.quiver(CY, CX, V, U, width=0.002)
ax.set_title("maximum curvature for robot-cart system - oscullating circle depicted")
fig.colorbar(im, ax=ax)
plt.show()


# for circle centered at base frame

#xs_circle, ys_circle, zs_circle_IN, zs_circle_OUT = findCircularSlice(
#    (-0.52, 0.0), best_radius_at_base_center, xs, ys, headings, workspace_mask
#)
#workspace = {
#    "xs": xs_circle,
#    "ys": ys_circle,
#    "theta_intervals": [
#        (np.min(headings_xy), np.max(headings_xy)) if len(headings_xy) > 0 else None
#        for headings_xy in zs_circle_IN
#    ],
#    "center": np.array([-0.52, 0.0]),
#    "radius": best_radius_at_base_center,
#}
#fil = open("./workspace_center_at_base.pickle", "wb")
#pickle.dump(workspace, fil)
#fil.close()
#plotCircularSlice(xs_circle, ys_circle, zs_circle_IN, zs_circle_OUT)

# circle slices (cyllinders) of the (x,y,theta) cube
# xs_circle, ys_circle, zs_circle_IN, zs_circle_OUT = findCircularSlice(
#    best_center, best_radius, xs, ys, headings, workspace_mask
# )
xs_circle, ys_circle, zs_circle_IN, zs_circle_OUT = findFullCircularSlicePolar(
    best_center, best_radius, xs, ys, headings, workspace_mask
)

print("best center + radius combo workspace slice")
workspace = {
    "xs": xs_circle,
    "ys": ys_circle,
    "theta_intervals": [
        (np.min(headings_xy), np.max(headings_xy)) if len(headings_xy) > 0 else None
        for headings_xy in zs_circle_IN
    ],
    "center": np.array([best_center[0], best_center[1]]),
    "radius": best_radius,
}
fil = open("./workspace_best.pickle", "wb")
pickle.dump(workspace, fil)
fil.close()
plotCircularSlice(xs_circle, ys_circle, zs_circle_IN, zs_circle_OUT)
# for radius in np.linspace(0.52, 0.7 + 0.52, 10):
#    print("plot for radius", radius)
#    xs_circle, ys_circle, zs_circle_IN, zs_circle_OUT = findCircularSlice(
#        center, radius, xs, ys, headings, workspace_mask
#    )
#    plotCircularSlice(xs_circle, ys_circle, zs_circle_IN, zs_circle_OUT)
