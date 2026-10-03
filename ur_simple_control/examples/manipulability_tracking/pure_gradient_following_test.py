from smc import load_config
import numpy as np
from scipy import linalg
from copy import deepcopy


def logSPD(Sigma: np.ndarray, Lambda: np.ndarray) -> np.ndarray:
    Sigma_minus_pow = linalg.fractional_matrix_power(Sigma, -1 / 2)
    Sigma_plus_pow = linalg.fractional_matrix_power(Sigma, 1 / 2)
    L = (
        Sigma_plus_pow
        @ linalg.logm(Sigma_minus_pow @ Lambda @ Sigma_minus_pow)
        @ Sigma_plus_pow
    )
    return L


def expSPD(Sigma: np.ndarray, Lambda: np.ndarray) -> np.ndarray:
    Sigma_minus_pow = linalg.fractional_matrix_power(Sigma, -1 / 2)
    Sigma_plus_pow = linalg.fractional_matrix_power(Sigma, 1 / 2)
    L = (
        Sigma_plus_pow
        @ linalg.expm(Sigma_minus_pow @ Lambda @ Sigma_minus_pow)
        @ Sigma_plus_pow
    )
    return L


def distance(Sigma: np.ndarray, Lambda: np.ndarray):
    Sigma_minus_pow = linalg.fractional_matrix_power(Sigma, -1 / 2)
    A = linalg.logm(Sigma_minus_pow @ Lambda @ Sigma_minus_pow)
    return np.sqrt(np.trace(A.T @ A))


if __name__ == "__main__":
    l = np.random.random((3, 3))
    s = np.random.random((3, 3))
    Sigma_init = l @ l.T
    Lambda = s @ s.T
    Sigma = deepcopy(Sigma_init)

    tangent_vector = logSPD(Sigma_init, Lambda)
    dt = 1e-1
    iteration = 0
    while distance(Sigma, Lambda) > 1e-2:
        # tangent_vector = logSPD(Lambda, Sigma)
        Sigma = expSPD(Sigma, dt * logSPD(Sigma, Lambda))
        print(distance(Sigma, Lambda))
        iteration += 1
