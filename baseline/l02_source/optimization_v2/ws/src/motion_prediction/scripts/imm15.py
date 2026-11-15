"""Causal interacting CV/CA/coordinated-turn filters.

State is [x,y,vx,vy,ax,ay].
Only causal position observations enter the update. Forecast modes are retained
separately rather than collapsed into a constant-velocity average.
"""
import math
import numpy as np

MODES = ('CV', 'CA', 'TURN_LEFT', 'TURN_RIGHT')


def transition(mode, dt, turn_rate=.4):
    f = np.zeros((6, 6))
    f[:2, :2] = np.eye(2)
    if mode in ('CV', 'CA'):
        f[:2, 2:4] = np.eye(2)*dt
        f[2:4, 2:4] = np.eye(2)
        if mode == 'CA':
            f[:2, 4:] = np.eye(2)*(.5*dt*dt)
            f[2:4, 4:] = np.eye(2)*dt
            f[4:, 4:] = np.eye(2)
    else:
        omega = turn_rate if mode == 'TURN_LEFT' else -turn_rate
        s, c = math.sin(omega*dt), math.cos(omega*dt)
        f[:2, 2:4] = np.array([[s, c-1], [1-c, s]])/omega
        rotation = np.array([[c, -s], [s, c]])
        f[2:4, 2:4] = rotation
        f[4:, 2:4] = omega*np.array([[0, -1], [1, 0]])@rotation
    return f


def covariance(value, floor=1e-7):
    p = np.asarray(value, dtype=float)
    if p.shape[0] != p.shape[1] or not np.isfinite(p).all():
        raise ValueError('Invalid covariance')
    eigenvalues, eigenvectors = np.linalg.eigh((p+p.T)*.5)
    if eigenvalues.min() < -1e-5:
        raise ValueError('Non-PSD input covariance')
    return (eigenvectors*np.maximum(eigenvalues, floor))@eigenvectors.T


def process_noise(dt, acceleration_sigma=.45):
    g = np.vstack([np.eye(2)*(.5*dt*dt), np.eye(2)*dt, np.eye(2)])
    return acceleration_sigma**2*dt*(g@g.T)+np.eye(6)*1e-7


class IMM:
    def __init__(self, t, position, measurement_covariance, velocity=(0., 0.),
                 stay_probability=.94, acceleration_sigma=.45, turn_rate=.4):
        self.t = float(t)
        self.sigma, self.turn_rate = float(acceleration_sigma), float(turn_rate)
        if not math.isfinite(self.t) or not .5 < stay_probability < 1 or not 0 < self.turn_rate <= 2 or not math.isfinite(self.sigma) or self.sigma<=0:
            raise ValueError('Invalid IMM settings')
        x = np.array([*position, *velocity, 0., 0.], dtype=float)
        if x.shape != (6,) or not np.isfinite(x).all():
            raise ValueError('Invalid observation')
        p = np.diag([1., 1., .5, .5, 1., 1.])
        p[:2, :2] = covariance(measurement_covariance)
        self.means = np.tile(x, (4, 1))
        self.covs = np.tile(p, (4, 1, 1))
        self.probabilities = np.array([.45, .25, .15, .15])
        self.switch = np.full((4, 4), (1-stay_probability)/3)
        np.fill_diagonal(self.switch, stay_probability)
        self.updates = 1

    def update(self, t, position, measurement_covariance):
        dt = float(t)-self.t
        if not 0 < dt <= 1.2:
            raise ValueError('IMM observation must advance within the track freshness bound')
        z = np.asarray(position, dtype=float)
        r = covariance(measurement_covariance)
        if z.shape != (2,) or r.shape != (2, 2) or not np.isfinite(z).all():
            raise ValueError('Invalid measurement')
        predicted_probability = self.probabilities@self.switch
        mixing = self.probabilities[:, None]*self.switch/predicted_probability[None, :]
        means, covs, log_likelihoods = [], [], []
        h = np.eye(6)[:2]
        for j, mode in enumerate(MODES):
            x = np.sum(mixing[:, j, None]*self.means, axis=0)
            delta = self.means-x
            p = np.sum(mixing[:, j, None, None]*(self.covs+delta[:, :, None]*delta[:, None, :]), axis=0)
            f = transition(mode, dt, self.turn_rate)
            x = f@x
            p = f@p@f.T+process_noise(dt, self.sigma)
            innovation = z-h@x
            s = h@p@h.T+r
            gain = np.linalg.solve(s, h@p).T
            x = x+gain@innovation
            residual = np.eye(6)-gain@h
            p = residual@p@residual.T+gain@r@gain.T
            log_likelihoods.append(-.5*(innovation@np.linalg.solve(s, innovation)+np.linalg.slogdet(s)[1]+2*math.log(2*math.pi)))
            means.append(x)
            covs.append(covariance(p))
        log_probability = np.log(predicted_probability)+np.array(log_likelihoods)
        probability = np.exp(log_probability-log_probability.max())
        self.probabilities = probability/probability.sum()
        self.means, self.covs = np.array(means), np.array(covs)
        self.t, self.updates = float(t), self.updates+1

    def predict(self, horizon=3.2, dt=.4):
        if not 2 <= horizon <= 4 or not .1 <= dt <= 1 or abs(round(horizon/dt)*dt-horizon) > 1e-8:
            raise ValueError('Forecast needs 2–4 seconds with an integral number of steps')
        output = []
        for j, name in enumerate(MODES):
            x, p = self.means[j].copy(), self.covs[j].copy()
            f = transition(name, dt, self.turn_rate)
            means, covs = [], []
            for _ in range(round(horizon/dt)):
                x = f@x
                p = f@p@f.T+process_noise(dt, self.sigma)
                means.append(x[:2].tolist())
                covs.append(covariance(p[:2, :2]).tolist())
            output.append(dict(mode=name, probability=float(self.probabilities[j]),
                               mean=means, covariance=covs))
        return dict(t=self.t, frame='odom', dt=dt, horizon_s=horizon, modes=output,
                    algorithm='IMM_CV_CA_CT_LEFT_CT_RIGHT', updates=self.updates)
