"""Robot LPF
Use this script to load a .mat file that has a bunch of RobotState messages
that were captured by the MATLAB GUI. Make sure that the .mat file has an
unpacked bot struct, and not an array of RobotState messages.

Tune the parameters in main and then run this script. This will create a
.mat file with the same name as what you loaded, but with '_rerun' appended
to it.

Load the '*_rerun.mat' into MATLAB and run 'process_bot_data.m' on it to 
see a graph of the original vs the rerun data with tuned parameters.

Using this script and this process will help to tune filters more quickly.
"""
import sys, time, copy

import numpy as np
import scipy.io as sio

sys.path.append('../../ros/src/playground/nodes/estimation/')
from estimators import RobotEstimatorLPF

_predict_forward_seconds = (1/30.0)

def rerun(alpha, tau, update_type, filename):
    estimator = RobotEstimatorLPF(_alpha=alpha, _tau=tau, _update_type=update_type)

    # Open the correct mat data file
    mat = sio.loadmat(filename + '.mat', squeeze_me=True, struct_as_record=False)
    bot = mat['bot']

    # Run through the estimator
    N = len(bot.Xhat)

    xhat_rerun = []
    yhat_rerun = []
    thetahat_rerun = []
    xhat_future_rerun = []
    yhat_future_rerun = []
    thetahat_future_rerun = []

    _last_time = 0

    for i in xrange(N):

        # If there was a correction, that means that _measurement is not None
        if bot.Correction[i]:
            _last_time = time.time()
            _measurement = (bot.VisionX[i], bot.VisionY[i], bot.VisionTheta[i])
        else:
            _measurement = (None, None, None)

        Ts = (time.time() - _last_time)
        (xhat, yhat, thetahat) = estimator.update(Ts, measurement=_measurement)
        xhat_rerun.append(xhat)
        yhat_rerun.append(yhat)
        thetahat_rerun.append(thetahat)

        # Get prediction
        (xhat, yhat, thetahat) = estimator.predict(_predict_forward_seconds)
        xhat_future_rerun.append(xhat)
        yhat_future_rerun.append(yhat)
        thetahat_future_rerun.append(thetahat)


    # save as .mat file for post-processing in MATLAB
    mat['xhat_rerun'] = xhat_rerun
    mat['yhat_rerun'] = yhat_rerun
    mat['thetahat_rerun'] = thetahat_rerun
    mat['xhat_future_rerun'] = xhat_future_rerun
    mat['yhat_future_rerun'] = yhat_future_rerun
    mat['thetahat_future_rerun'] = thetahat_future_rerun
    mat['rerun'] = True
    sio.savemat(filename + '_rerun.mat', mat)

def main():
    # Choose file
    data_dir = '../../gui/data/'
    filename = data_dir + 'ally08'

    # Choose LPF parameters
    alpha = 0.05
    tau = 0.5
    update_type = RobotEstimatorLPF.UPDATE_SIMPLE

    # Rerun and save!
    rerun(alpha, tau, update_type, filename)
    

if __name__ == '__main__':
    main()