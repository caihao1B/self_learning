"""Real fixed-weight ECCV Trajectron++ inference, all 25 latent components.

Only past/current observed states enter the encoder. Future encoder, map encoder
and robot future are not used by this published pedestrian checkpoint.
"""
import hashlib
import json
import time
from types import SimpleNamespace
import numpy as np
import torch
from compat15 import VENDOR, install


class Trajectron:
    def __init__(self):
        torch.set_num_threads(1)
        self.patch_hashes = install()
        from model.model_registrar import ModelRegistrar
        from model.mgcvae import MultimodalGenerativeCVAE
        from model.model_utils import ModeKeys
        from environment.node_type import NodeTypeEnum
        self.mode = ModeKeys.PREDICT
        model = VENDOR/'experiments/pedestrians/models/eth_attention_radius_3'
        weight = model/'model_registrar-100.pt'
        if hashlib.sha256(weight.read_bytes()).hexdigest() != '157f56c98e721e888bdbee82000e93ed536af3820193d9c7b5b3ca669938d9e9':
            raise ValueError('UNTRUSTED_WEIGHT_HASH')
        cfg = json.loads((model/'config.json').read_text())
        self.registrar = ModelRegistrar(str(model), 'cpu')
        # Only this hash-verified, pinned official serialized ModuleDict is loaded.
        self.registrar.model_dict = torch.load(weight, map_location='cpu', weights_only=False)
        keys = set(self.registrar.model_dict.keys())
        node_type=NodeTypeEnum(['PEDESTRIAN']).PEDESTRIAN
        self.edge = (node_type, node_type)
        env = SimpleNamespace(scenes=[SimpleNamespace(dt=.4)])
        self.model = MultimodalGenerativeCVAE(env,node_type,self.registrar,cfg,'cpu',[self.edge])
        if set(self.registrar.model_dict.keys()) != keys:
            raise ValueError('MODEL_CONFIG_CREATED_UNTRAINED_MODULES')
        self.registrar.model_dict.eval()
        self.module_keys = sorted(keys)

    @torch.inference_mode()
    def predict(self, feature, neighbors=(), horizon=3.2, distribution_only=True):
        if not 2 <= horizon <= 4 or abs(round(horizon/.4)*.4-horizon)>1e-8:
            raise ValueError('2–4 s horizon on checkpoint 0.4 s grid required')
        state = np.asarray(feature['state'],dtype=np.float32)
        if state.shape!=(8,6) or not np.isfinite(state).all():
            raise ValueError('Eight finite causal states required')
        std = np.array([3.,3.,2.,2.,1.,1.],dtype=np.float32)
        centered = state.copy(); centered[:,:2] -= state[-1,:2]
        neighbors_t, weights, used = [], [], []
        for other in neighbors:
            if other['id']==feature['id'] or other.get('classification')!='person':
                continue
            # A different time origin is not silently treated as simultaneous.
            if abs(other['t']-feature['t'])>.025:
                continue
            ns=np.asarray(other['state'],dtype=np.float32)
            if np.linalg.norm(ns[-1,:2]-state[-1,:2])>3.:
                continue
            neighbors_t.append(torch.tensor((ns-state[-1])/std))
            # Historical edge age; no future neighbor state or world ID.
            inside=np.linalg.norm(ns[:,:2]-state[:,:2],axis=1)<=3.
            age=0
            for present in inside:
                age=age+1 if present else 0
            weights.append(min(age/4.,1.));used.append(other['id'])
        ns={self.edge:[neighbors_t]}
        edge_values={self.edge:[torch.tensor(weights if weights else [0.],dtype=torch.float32)]}
        m=self.model
        start=time.perf_counter();cpu=time.process_time()
        x,xr,_,yr,_,present=m.obtain_encoded_tensors(
            mode=self.mode,inputs=torch.tensor(state[None]),inputs_st=torch.tensor((centered/std)[None]),
            labels=None,labels_st=None,first_history_indices=torch.tensor([0]),
            neighbors=ns,neighbors_edge_value=edge_values,robot=None,map=None)
        m.latent.p_dist=m.p_z_x(self.mode,x)
        z,n,k=m.latent.sample_p(1,self.mode,full_dist=True)
        dist,_=m.p_y_xz(self.mode,x,xr,yr,present,z,round(horizon/.4),n,k,
                        gmm_mode=True,return_distribution_only=distribution_only)
        means=dist.mus[0,0].cpu().numpy()
        probabilities=dist.log_pis[0,0].exp().cpu().numpy()
        cov=dist.get_covariance_matrix()[0,0].cpu().numpy()
        measurement=np.asarray(feature['covariance'],dtype=float)
        cov=cov+measurement[None,None]+np.eye(2)*1e-7
        if not all(np.isfinite(v).all() for v in [means,probabilities,cov]):
            raise ValueError('NONFINITE_MODEL_OUTPUT')
        if not np.allclose(probabilities,probabilities[0],atol=1e-5):
            raise ValueError('Unexpected time-varying latent probabilities')
        modes=[dict(mode='LATENT_'+str(i),probability=float(probabilities[0,i]),
                    mean=means[:,i].tolist(),covariance=cov[:,i].tolist()) for i in range(k)]
        return dict(t=feature['t'],id=feature['id'],frame='odom',dt=.4,horizon_s=horizon,
                    algorithm='Trajectron++_ECCV_ETH',modes=modes,neighbors=used,
                    input_covariance_added=True,wall_ms=(time.perf_counter()-start)*1000,
                    cpu_ms=(time.process_time()-cpu)*1000)
