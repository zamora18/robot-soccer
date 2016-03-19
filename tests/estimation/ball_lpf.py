"""Ball LPF
Use this script to load a .mat file that has a bunch of BallState messages
that were captured by the MATLAB GUI. Make sure that the .mat file has an
unpacked ball struct, and not an array of BallState messages.

Tune the parameters in main and then run this script. This will create a
.mat file with the same name as what you loaded, but with '_rerun' appended
to it.

Load the '*_rerun.mat' into MATLAB and run 'process_ball_data.m' on it to 
see a graph of the original vs the rerun data with tuned parameters.

Using this script and this process will help to tune filters more quickly.
"""
import sys, time, copy

import numpy as np
import scipy.io as sio

sys.path.append('../../ros/src/playground/nodes/estimation/')
from estimators import BallEstimator

_predict_forward_seconds = (1/30.0)

def rerun(alpha, tau, update_type, filename):
    estimator = BallEstimator(_alpha=alpha, _tau=tau, _update_type=update_type)

    # Open the correct mat data file
    mat = sio.loadmat(filename + '.mat', squeeze_me=True, struct_as_record=False)
    ball = mat['ball']

    # Run through the estimator
    N = len(ball.Xhat)

    xhat_rerun = []
    yhat_rerun = []
    xhat_future_rerun = []
    yhat_future_rerun = []

    _last_time = 0

    for i in xrange(N):

        # If there was a correction, that means that _measurement is not None
        if ball.Correction[i]:
            _last_time = time.time()
            _measurement = (ball.VisionX[i], ball.VisionY[i])
        else:
            _measurement = (None, None)

        Ts = (time.time() - _last_time)
        (xhat, yhat) = estimator.update(Ts, measurement=_measurement)
        xhat_rerun.append(xhat)
        yhat_rerun.append(yhat)

        # Get prediction
        (xhat, yhat) = estimator.predict(_predict_forward_seconds)
        xhat_future_rerun.append(xhat)
        yhat_future_rerun.append(yhat)


    # save as .mat file for post-processing in MATLAB
    mat['xhat_rerun'] = xhat_rerun
    mat['yhat_rerun'] = yhat_rerun
    mat['xhat_future_rerun'] = xhat_future_rerun
    mat['yhat_future_rerun'] = yhat_future_rerun
    mat['rerun'] = True
    sio.savemat(filename + '_rerun.mat', mat)

def main():
    # Choose file
    data_dir = '../../gui/data/'
    filename = data_dir + 'ball08'

    # Choose LPF parameters
    alpha = 0.4
    tau = 0.1
    update_type = BallEstimator.UPDATE_SIMPLE

    # Rerun and save!
    rerun(alpha, tau, update_type, filename)
    

if __name__ == '__main__':
    main()