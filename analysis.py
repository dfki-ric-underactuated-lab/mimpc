import numpy as np

def time_weighted_rms(t, x):
    """
    Compute time-weighted RMS of a signal with non-uniform time steps.

    Parameters
    ----------
    t : array_like
        1D array of time stamps (must be increasing).
    x : array_like
        1D or 2D array of signal values at times t.
        Shape must be (len(t),) for 1D or (len(t), n_signals) for 2D.

    Returns
    -------
    rms : float or ndarray
        Time-weighted RMS. 
        - float if x is 1D
        - 1D array of shape (n_signals,) if x is 2D
    """
    t = np.asarray(t)
    x = np.asarray(x)

    # total duration
    T = t[-1] - t[0]

    # trapezoidal integration along time axis
    dt = np.diff(t, axis=0, append=t[-1].item()).ravel()
    
    if x.shape[0] == 1 or x.ndim == 1:
        return np.sum(np.abs(x[:]) * dt) / T
    elif x.shape[0] == 2:
        return np.sum(np.sqrt((x[0,:]**2 + x[1,:]**2)) * dt) / T
    else:
        raise ValueError(f"x must have shape (len(t),1) or (len(t), 2), but has shape {x.shape}")
    

    




import numpy as np

def thruster_usage(t, u):
    """
    Compute total thruster usage over time with non-uniform time steps.

    Parameters
    ----------
    t : array_like
        1D array of time stamps (length N).
    u : array_like
        2D array of thruster activations (n_thrusters x N).
        Entries should be 0/1.
        
    Returns
    -------
    usage : ndarray
        1D array of length n_thrusters with total thrust usage
        (time-integrated thrust).
    """
    t = np.asarray(t)
    u = np.asarray(u)

    if u.shape[1] != len(t):
        raise ValueError("u must have shape (n_thrusters, len(t))")

    dt = np.diff(t, axis=0, append=t[-1].item()).ravel()
    T = t[-1] - t[0]
    return np.sum(np.sum(u, axis=0) * dt) / (T * u.shape[0])

