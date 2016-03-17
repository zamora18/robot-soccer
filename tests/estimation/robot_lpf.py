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
    filename = data_dir + 'ally07'

    # Choose LPF parameters
    alpha = 0.02
    tau = 0.5
    update_type = RobotEstimatorLPF.UPDATE_SIMPLE

    # Rerun and save!
    rerun(alpha, tau, update_type, filename)
    

if __name__ == '__main__':
    main()