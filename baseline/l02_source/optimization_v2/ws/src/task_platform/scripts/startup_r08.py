"""Admission from observed map-to-odom corrections only, no simulator pose."""
import numpy as np
def stable_corrections(history):
 if not history:return False,dict(samples=0,span_s=0.,translation_spread_m=None,yaw_spread_rad=None)
 xy=np.array([x[1] for x in history]);angles=np.unwrap([x[2] for x in history]);span=float(history[-1][0]-history[0][0]);spread=float(np.linalg.norm(xy[:,None,:]-xy[None,:,:],axis=-1).max());turn=float(np.ptp(angles));m=dict(samples=len(history),span_s=span,translation_spread_m=spread,yaw_spread_rad=turn)
 return bool(len(history)>=5 and span>=.9 and spread<=.06 and turn<=.01),m
