"""单层 SAC-HP、高斯融合、回放及可恢复状态。"""
from dataclasses import dataclass, asdict
import copy
import os
from pathlib import Path
import tempfile
import random
import numpy as np
import torch
from torch import nn
from torch.distributions import Normal
from torch.nn import functional as F

@dataclass
class Config:
    """! @brief 学习和动作尺度参数。"""
    hidden: int = 256
    ensemble: int = 3
    prior_variance: float = 0.3
    gamma: float = 0.99
    tau: float = 0.005
    lr: float = 0.0003
    alpha: float = 0.2
    ablation: str = 'hybrid'  # hybrid, no_prior, base

class Actor(nn.Module):
    """! @brief 输出动作潜空间高斯参数。"""
    def __init__(self, cfg):
        super().__init__()
        self.net = nn.Sequential(nn.Linear(28,cfg.hidden), nn.ReLU(), nn.Linear(cfg.hidden,cfg.hidden), nn.ReLU())
        self.mean = nn.Linear(cfg.hidden,2)
        self.log_std = nn.Linear(cfg.hidden,2)
    def forward(self, observation):
        """! @brief 根据输入计算动作或价值；张量的第一维为批量大小。"""
        x = self.net(observation)
        return self.mean(x), self.log_std(x).clamp(-20,2)

class Critic(nn.Module):
    """! @brief 对归一化动作估计状态动作价值。"""
    def __init__(self,cfg):
        super().__init__()
        self.net=nn.Sequential(nn.Linear(30,cfg.hidden),nn.ReLU(),nn.Linear(cfg.hidden,cfg.hidden),nn.ReLU(),nn.Linear(cfg.hidden,1))
    def forward(self,obs,action):
        """! @brief 根据输入计算动作或价值；张量的第一维为批量大小。"""
        return self.net(torch.cat((obs,action),dim=-1))

class Policy(nn.Module):
    """! @brief Actor 集成矩匹配并在 tanh 前融合 MPPI 高斯先验。"""
    def __init__(self,cfg):
        super().__init__()
        if cfg.ensemble < 1 or cfg.prior_variance <= 0:
            raise ValueError('ensemble and prior variance must be positive')
        self.cfg=cfg
        self.actors=nn.ModuleList([Actor(cfg) for _ in range(cfg.ensemble)])
    def distribution(self,observation,prior,member=None):
        """! @brief 将各 Actor 按矩匹配，并以潜空间 MPPI 均值融合，返回均值和标准差。"""
        if self.cfg.ablation == 'base':
            observation=torch.cat((observation[:,:20],torch.zeros_like(observation[:,20:])),dim=1)
        means=[]
        variances=[]
        for i,actor in enumerate(self.actors):
            mu,log_std=actor(observation)
            if member is not None and member != i:
                mu=mu.detach()
                log_std=log_std.detach()
            means.append(mu)
            variances.append((2*log_std).exp())
        means=torch.stack(means)
        variances=torch.stack(variances)
        mu=means.mean(0)
        var=(variances.mean(0)+(means-mu).square().mean(0)).clamp(min=1e-8)
        if self.cfg.ablation != 'no_prior':
            # MPPI 给出的是 tanh 后动作；反变换后才可在同一空间融合。
            bounded=prior.clamp(-0.999999,0.999999)
            p=0.5*torch.log((1+bounded)/(1-bounded))
            pv=self.cfg.prior_variance
            mu=(pv*mu+var*p)/(var+pv)
            var=var*pv/(var+pv)
        else:
            mu=mu+prior*0.0  # 保留统一 ONNX 输入接口
        return mu,var.sqrt()
    def sample(self,observation,prior,deterministic=False,member=None):
        """! @brief 返回策略采样动作与对数密度，或返回经验缓冲区的采样批次。"""
        mu,std=self.distribution(observation,prior,member)
        dist=Normal(mu,std)
        z=mu if deterministic else dist.rsample()
        action=torch.tanh(z)
        logp=(dist.log_prob(z)-2*(np.log(2.)-z-F.softplus(-2*z))).sum(-1,keepdim=True)
        return action,logp
    def forward(self,observation,prior):
        """! @brief 根据输入计算动作或价值；张量的第一维为批量大小。"""
        mu,_=self.distribution(observation,prior)
        return torch.tanh(mu)

class Replay:
    """! @brief 定长回放池，保存当前与下一时刻的 MPPI 先验。"""
    def __init__(self,capacity=100000,seed=0):
        self.capacity=capacity
        self.items=[]
        self.pos=0
        self.rng=np.random.default_rng(seed)
    def add(self,obs,prior,action,reward,next_obs,next_prior,terminal):
        """! @brief 复制一个采样转移到回放缓冲区。"""
        item=tuple(np.array(x,dtype=np.float32,copy=True) for x in (obs,prior,action,reward,next_obs,next_prior,terminal))
        if len(self.items)<self.capacity:
            self.items.append(item)
        else:
            self.items[self.pos]=item
        self.pos=(self.pos+1)%self.capacity
    def sample(self,n,device):
        """! @brief 返回策略采样动作与对数密度，或返回经验缓冲区的采样批次。"""
        indices=self.rng.integers(len(self.items),size=n)
        cols=zip(*(self.items[i] for i in indices))
        return [torch.as_tensor(np.stack(list(c)),device=device).reshape(n,-1) for c in cols]
    def __len__(self):
        return len(self.items)

class SAC:
    """! @brief 双 Q SAC 与多成员融合 Actor 的训练状态。"""
    def __init__(self,cfg=Config(),device='cpu'):
        self.cfg=cfg
        self.device=torch.device(device)
        self.policy=Policy(cfg).to(device)
        self.q1=Critic(cfg).to(device)
        self.q2=Critic(cfg).to(device)
        self.t1=copy.deepcopy(self.q1)
        self.t2=copy.deepcopy(self.q2)
        self.qopt=torch.optim.Adam(list(self.q1.parameters())+list(self.q2.parameters()),lr=cfg.lr)
        self.aopts=[torch.optim.Adam(a.parameters(),lr=cfg.lr) for a in self.policy.actors]
        self.log_alpha=nn.Parameter(torch.tensor(np.log(cfg.alpha),dtype=torch.float32,device=device))
        self.alphaopt=torch.optim.Adam([self.log_alpha],lr=cfg.lr)
        self.updates=0
    def act(self,obs,prior,deterministic=False):
        """! @brief 对单个固定 28 维观测与 2 维先验推理一个归一化动作。"""
        with torch.no_grad():
            o=torch.as_tensor(obs,dtype=torch.float32,device=self.device).reshape(1,28)
            p=torch.as_tensor(prior,dtype=torch.float32,device=self.device).reshape(1,2)
            if not torch.isfinite(o).all() or not torch.isfinite(p).all() or (p.abs()>1).any():
                raise ValueError('invalid observation/prior')
            return self.policy.sample(o,p,deterministic)[0][0].cpu().numpy()
    def update(self,replay,batch):
        """! @brief 对采样批次更新双 Q、各成员 Actor 和熵温度。"""
        o,p,a,r,n,np_,done=replay.sample(batch,self.device)
        alpha=self.log_alpha.exp().detach()
        with torch.no_grad():
            na,nlp=self.policy.sample(n,np_)
            target=r+self.cfg.gamma*(1-done)*(torch.minimum(self.t1(n,na),self.t2(n,na))-alpha*nlp)
        lossq=F.mse_loss(self.q1(o,a),target)+F.mse_loss(self.q2(o,a),target)
        self.qopt.zero_grad()
        lossq.backward()
        self.qopt.step()
        for q in (self.q1,self.q2):
            q.requires_grad_(False)
        losses=[]
        for i,opt in enumerate(self.aopts):
            sampled,logp=self.policy.sample(o,p,member=i)
            loss=(alpha*logp-torch.minimum(self.q1(o,sampled),self.q2(o,sampled))).mean()
            opt.zero_grad()
            loss.backward()
            opt.step()
            losses.append(float(loss.detach()))
        for q in (self.q1,self.q2):
            q.requires_grad_(True)
        aloss=-(self.log_alpha*(logp.detach()-2)).mean()
        self.alphaopt.zero_grad()
        aloss.backward()
        self.alphaopt.step()
        with torch.no_grad():
            for target,source in ((self.t1,self.q1),(self.t2,self.q2)):
                for t,s in zip(target.parameters(),source.parameters()):
                    t.lerp_(s,self.cfg.tau)
        self.updates+=1
        return {'critic':float(lossq.detach()),'actor':float(np.mean(losses)),
                'alpha':float(self.log_alpha.exp().detach()),'alpha_loss':float(aloss.detach())}
    @staticmethod
    def _save_new(path, state):
        """! @brief 完整写入相邻临时文件，再以不可覆盖方式提交快照。"""
        path = Path(path)
        pending = None
        try:
            with tempfile.NamedTemporaryFile(dir=path.parent, delete=False) as stream:
                pending = stream.name
                torch.save(state, stream)
                stream.flush()
                os.fsync(stream.fileno())
            os.link(pending, path)
        finally:
            if pending is not None:
                os.unlink(pending)

    def weights(self,path):
        """! @brief 保存推理权重；不包含优化器和经验。"""
        self._save_new(path, {'config': asdict(self.cfg), 'policy': self.policy.state_dict()})
    def checkpoint(self,path,replay,extra=None):
        """! @brief 保存可恢复训练状态、经验池和全部随机数状态。"""
        state={'config':asdict(self.cfg),'policy':self.policy.state_dict(),'q1':self.q1.state_dict(),'q2':self.q2.state_dict(),'t1':self.t1.state_dict(),'t2':self.t2.state_dict(),'qopt':self.qopt.state_dict(),'aopts':[x.state_dict() for x in self.aopts],'log_alpha':self.log_alpha.detach(),'alphaopt':self.alphaopt.state_dict(),'updates':self.updates,'replay':replay,'extra':extra,'python_rng':random.getstate(),'numpy_rng':np.random.get_state(),'torch_rng':torch.get_rng_state(),'cuda_rng':torch.cuda.get_rng_state_all() if torch.cuda.is_available() else None}
        self._save_new(path, state)
    @classmethod
    def load(cls,path,device='cpu',resume=False):
        """! @brief 加载可信权重或恢复完整检查点。"""
        state=torch.load(path,map_location=device,weights_only=False)
        agent=cls(Config(**state['config']),device)
        agent.policy.load_state_dict(state['policy'])
        if resume:
            for key in ('q1','q2','t1','t2'):
                getattr(agent,key).load_state_dict(state[key])
            agent.qopt.load_state_dict(state['qopt'])
            for opt,saved in zip(agent.aopts,state['aopts']):
                opt.load_state_dict(saved)
            agent.log_alpha.data.copy_(state['log_alpha'])
            agent.alphaopt.load_state_dict(state['alphaopt'])
            agent.updates=state['updates']
            random.setstate(state['python_rng'])
            np.random.set_state(state['numpy_rng'])
            torch.set_rng_state(state['torch_rng'].cpu())
            if state['cuda_rng'] is not None:
                torch.cuda.set_rng_state_all(state['cuda_rng'])
        return agent,state.get('replay'),state.get('extra')
