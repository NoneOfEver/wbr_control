#!/usr/bin/env python3
"""Compare condensed QP against independent explicit-state formulation."""
from pathlib import Path
import numpy as np
import osqp
from scipy import sparse
from scipy.linalg import solve_discrete_are
from benchmark import load_model, geometry_tables, WheelPowerProblem

root=Path(__file__).resolve().parents[2]
model=load_model(root)
tables=geometry_tables()
p=WheelPowerProblem(model,20,.005,40.,'total-absolute',tables)
a,b,q,r=model
terminal=solve_discrete_are(p.ad,p.bd,q,r)
cost=sparse.block_diag([q]*20+[terminal]+[r]*20,format='csc')
eq=sparse.lil_matrix((210,290))
eq[:10,:10]=np.eye(10)
for stage in range(20):
    rr=10+10*stage
    eq[rr:rr+10,10*stage:10*(stage+1)]=-p.ad
    eq[rr:rr+10,10*(stage+1):10*(stage+2)]=np.eye(10)
    eq[rr:rr+10,210+4*stage:210+4*(stage+1)]=-p.bd
for state in [np.array([.02,0,.03,0,.025,0,-.015,0,.04,0]),np.array([0,1.6,0,.4,.04,0,-.02,0,.06,0])]:
    p.update(state,10.)
    p.solver.update_settings(eps_abs=1e-8,eps_rel=1e-8,max_iter=20000)
    reduced=p.solver.solve(raise_error=False)
    p.matrix.data[:]=p.ax
    matrix=sparse.vstack([eq.tocsc(),sparse.hstack([sparse.csc_matrix((200,210)),p.matrix])],format='csc')
    equality=np.r_[state,np.zeros(200)]
    full=osqp.OSQP()
    full.setup(P=sparse.triu(2*cost/1000,format='csc'),q=np.zeros(290),A=matrix,
               l=np.r_[equality,p.lower],u=np.r_[equality,p.upper],
               eps_abs=1e-8,eps_rel=1e-8,max_iter=30000,polishing=True,verbose=False)
    explicit=full.solve(raise_error=False)
    assert reduced.info.status_val==1 and explicit.info.status_val==1,(reduced.info.status,explicit.info.status)
    difference=float(abs(reduced.x-explicit.x[210:]).max())
    assert difference<1e-3,difference
    print('full-vs-condensed maximum input difference Nm:',difference)
    powers=np.array([1.3,-2.1])*state[1]/.058
    halfspaces=np.array([[1,1],[1,-1],[-1,1],[-1,-1]])@powers
    assert abs(halfspaces.max()-abs(powers).sum())<1e-10
print('PASS: explicit dynamics and condensed QP agree; wheel power halfspaces are exact.')
