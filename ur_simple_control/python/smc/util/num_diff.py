# TODO:
"""

# use this as a starting point for finite differencing
def numdiff(func, x, eps=1e-6):
    f0 = copy.copy(func(x))
    xe = x.copy()
    fs = []
    for k in range(len(x)):
        xe[k] += eps
        fs.append((func(xe) - f0) / eps)
        xe[k] -= eps
    if isinstance(f0, np.ndarray) and len(f0) > 1:
        return np.stack(fs,axis=1)
    else:
        return np.matrix(fs)

# and here's example usage
# Tdiffq is used to compute the tangent application in the configuration space.
Tdiffq = lambda f,q: Tdiff1(f,lambda q,v:pin.integrate(robot.model,q,v),robot.model.nv,q)
c=costManipulability
Tg = costManipulability.calcDiff(q)
Tgn = Tdiffq(costManipulability.calc,q)
#assert( norm(Tg-Tgn)<1e-4)
"""
