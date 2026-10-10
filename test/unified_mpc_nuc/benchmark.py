#!/usr/bin/env python3
"""Software-only unified chassis QP benchmark. Never opens robot transports."""
import argparse
import importlib.util
import json
import os
from pathlib import Path
import sys
import time
import struct

import numpy as np
import osqp
from scipy import sparse
from scipy.linalg import expm, solve_discrete_are
from scipy.optimize import least_squares


def load_model(root):
    spec = importlib.util.spec_from_file_location('derive', root / 'tools/derive_lqr_schedule.py')
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    samples = module.read_leg_samples(root / 'tools/data/leg_mass_properties0820.csv')
    lengths = [s['leg_length_m'] for s in samples]
    sample = module.interpolate_sample(samples, lengths, .25)
    a, b = module.derive_model(sample)
    return a, b, module.Q, module.R


def discretize(a, b, dt):
    full = np.zeros((14, 14))
    full[:10, :10], full[:10, 10:] = a, b
    discrete = expm(full * dt)
    return discrete[:10, :10], discrete[:10, 10:]


def fk(q, branch):
    # Same five-bar equations and dimensions as production leg_kinematics.cc.
    p1, p2 = q
    b = .0945 * np.array([np.cos(p2), -np.sin(p2)])
    d = .090 * np.array([np.cos(p1), -np.sin(p1)])
    g = .210 * np.array([np.cos(p1), -np.sin(p1)])
    db = d - b
    distance = np.linalg.norm(db)
    along = (.1125**2 - .116**2 + distance**2) / (2 * max(distance, 1e-12))
    height = np.sqrt(max(0., .1125**2 - along**2))
    c = b + along * db / max(distance, 1e-12) + branch * height * np.array([-db[1], db[0]]) / max(distance, 1e-12)
    return g + .250 * (c-d) / max(np.linalg.norm(c-d), 1e-12)


def geometry_tables():
    # Offline IK grid; online interpolate mapping at measured theta/pitch.
    angles = np.linspace(-.6, .6, 241)
    tables = []
    for branch, offset, coordinate_sign, vmc_sign in [(1, -.036063, -1., -1.), (-1, -3.121010, 1., 1.)]:
        rows = []
        seed = None
        for alpha in angles:
            raw = alpha / coordinate_sign + offset
            target = .25 * np.array([np.sin(raw), -np.cos(raw)])
            candidates = [seed] if seed is not None else [np.array([p1, p2]) for p1 in np.linspace(-np.pi, np.pi, 5) for p2 in np.linspace(-np.pi, np.pi, 5)]
            solutions = [least_squares(lambda q: fk(q, branch)-target, candidate, max_nfev=150) for candidate in candidates]
            valid = [solution for solution in solutions if np.linalg.norm(solution.fun)<1e-5]
            def conditioning(solution):
                eps=1e-6
                return np.linalg.norm(np.column_stack([(fk(solution.x+np.eye(2)[i]*eps,branch)-fk(solution.x-np.eye(2)[i]*eps,branch))/(2*eps) for i in range(2)]))
            best = min(valid, key=conditioning) if valid else min(solutions,key=lambda solution:np.linalg.norm(solution.fun))
            if np.linalg.norm(best.fun) > 1e-5:
                raise RuntimeError('IK grid failed')
            seed = best.x
            eps = 1e-6
            j = np.column_stack([(fk(seed + np.eye(2)[i]*eps, branch)-fk(seed-np.eye(2)[i]*eps, branch))/(2*eps) for i in range(2)])
            radial = target/.25
            tangent = np.array([-radial[1], radial[0]])
            rows.append(np.r_[j.T @ radial, j.T @ tangent / .25 * vmc_sign])
        tables.append(np.array(rows))
    return angles, tables


class Problem:
    def __init__(self, model, horizon, prediction_dt, watts, power_mode, tables):
        a, b, q, r = model
        self.ad, self.bd = discretize(a, b, prediction_dt)
        self.plant_a, self.plant_b = discretize(a, b, .001)
        self.n = horizon
        self.constraint_start = 10+10*horizon
        self.watts = watts
        self.mode = power_mode
        self.angles, self.tables = tables
        terminal = solve_discrete_are(self.ad, self.bd, q, r)
        self.k = np.linalg.solve(r+self.bd.T@terminal@self.bd, self.bd.T@terminal@self.ad)
        self.nx = (horizon+1)*10
        self.iu = self.nx
        self.ip = self.iu + 4*horizon
        variables = self.ip + 2*horizon
        cost = sparse.block_diag([q]*horizon+[terminal]+[r]*horizon+[np.eye(2)*1e-6]*horizon, format='csc')
        self.p = sparse.triu(2*cost, format='csc')
        # x0 equality, N dynamics, then N stages of wheel/joint/power inequalities.
        rows = 10+10*horizon+11*horizon
        matrix = sparse.lil_matrix((rows, variables))
        matrix[:10,:10] = np.eye(10)
        for stage in range(horizon):
            row = 10+10*stage
            matrix[row:row+10,10*stage:10*(stage+1)] = -self.ad
            matrix[row:row+10,10*(stage+1):10*(stage+2)] = np.eye(10)
            matrix[row:row+10,self.iu+4*stage:self.iu+4*(stage+1)] = -self.bd
            row = 10+10*horizon+11*stage
            ui, pi = self.iu+4*stage, self.ip+2*stage
            matrix[row,ui] = matrix[row+1,ui+1] = 1.
            for side in range(2):
                for joint in range(2):
                    matrix[row+2+2*side+joint,ui+2+side] = 1.
                matrix[row+6+2*side,ui+side] = 1.
                matrix[row+7+2*side,ui+side] = -1.
                matrix[row+6+2*side,pi+side] = -1.
                matrix[row+7+2*side,pi+side] = -1.
            matrix[row+10,pi:pi+2] = 1.
        self.matrix = matrix.tocsc()
        self.base_ax = self.matrix.data.copy()
        self.indices = {(int(self.matrix.indices[k]),col): k for col in range(variables) for k in range(self.matrix.indptr[col],self.matrix.indptr[col+1])}
        self.lower = np.zeros(rows)
        self.upper = np.zeros(rows)
        self.ax = self.base_ax.copy()
        self.solver = osqp.OSQP()
        self.update(np.zeros(10), 0., setup=True)
        self.solver.setup(P=self.p, q=np.zeros(variables), A=self.matrix, l=self.lower, u=self.upper,
                          eps_abs=1e-4, eps_rel=1e-4, max_iter=400, check_termination=10,
                          polishing=False, adaptive_rho=True, verbose=False)

    def update(self, state, support_extra, setup=False):
        if self.constraint_start:
            self.lower[:10] = self.upper[:10] = state
        omega = np.array([(state[1]+.215*state[3])/.058, (state[1]-.215*state[3])/.058])
        self.joint_mapping = []
        for side in range(2):
            alpha = state[4+2*side]+state[8]
            coefficients = np.array([np.interp(alpha,self.angles,self.tables[side][:,j]) for j in range(4)])
            force = (12.054*9.80665/2 + (1 if side==0 else -1)*support_extra)/max(np.cos(state[4+2*side]),.5)
            self.joint_mapping.append((coefficients[2:],coefficients[:2]*force))
        for stage in range(self.n):
            row = self.constraint_start+11*stage
            ui = self.iu+4*stage
            self.lower[row:row+2] = -16384/3450
            self.upper[row:row+2] = 16384/3450
            for side in range(2):
                coefficient, bias = self.joint_mapping[side]
                for joint in range(2):
                    rr = row+2+2*side+joint
                    self.ax[self.indices[rr,ui+2+side]] = coefficient[joint]
                    self.lower[rr], self.upper[rr] = -54-bias[joint],54-bias[joint]
                for sign,row_offset in [(1,6+2*side),(-1,7+2*side)]:
                    self.ax[self.indices[row+row_offset,ui+side]] = sign*omega[side]
            self.lower[row+6:row+10] = -np.inf
            self.upper[row+6:row+10] = 0
            self.lower[row+10],self.upper[row+10] = -np.inf,self.watts
            if self.mode == 'per-wheel':
                # Each abs(power) <= cap/2 by torque bound, retaining total epigraph.
                bound = np.minimum(16384/3450, self.watts/2/np.maximum(abs(omega),1e-9))
                self.lower[row:row+2],self.upper[row:row+2] = -bound,bound
        if setup:
            self.matrix.data[:] = self.ax
        else:
            kwargs = {'q':self.gradient @ state} if hasattr(self,'gradient') else {}
            self.solver.update(Ax=self.ax,l=self.lower,u=self.upper,**kwargs)
        return omega


class CondensedProblem(Problem):
    def __init__(self,model,horizon,prediction_dt,watts,power_mode,tables):
        a,b,q,r=model
        self.ad,self.bd=discretize(a,b,prediction_dt)
        self.plant_a,self.plant_b=discretize(a,b,.001)
        self.n,self.watts,self.mode=horizon,watts,power_mode
        self.angles,self.tables=tables
        self.constraint_start=0
        self.nx=self.iu=0
        self.ip=4*horizon
        terminal=solve_discrete_are(self.ad,self.bd,q,r)
        self.k=np.linalg.solve(r+self.bd.T@terminal@self.bd,self.bd.T@terminal@self.ad)
        fx=np.zeros((10*(horizon+1),10))
        fu=np.zeros((10*(horizon+1),4*horizon))
        fx[:10]=np.eye(10)
        for stage in range(horizon):
            fx[10*(stage+1):10*(stage+2)]=self.ad@fx[10*stage:10*(stage+1)]
            fu[10*(stage+1):10*(stage+2)]=self.ad@fu[10*stage:10*(stage+1)]
            fu[10*(stage+1):10*(stage+2),4*stage:4*(stage+1)]=self.bd
        qb=sparse.block_diag([q]*horizon+[terminal]).toarray()
        rb=sparse.block_diag([r]*horizon).toarray()
        h=2*(fu.T@qb@fu+rb)
        scale=np.diag(h).max()
        self.gradient=2*fu.T@qb@fx/scale
        self.gradient=np.vstack([self.gradient,np.zeros((2*horizon,10))])
        self.p=sparse.triu(sparse.block_diag([h/scale,np.eye(2*horizon)*1e-8],format='csc'),format='csc')
        matrix=sparse.lil_matrix((11*horizon,6*horizon))
        for stage in range(horizon):
            row,ui,pi=11*stage,4*stage,self.ip+2*stage
            matrix[row,ui]=matrix[row+1,ui+1]=1.
            for side in range(2):
                for joint in range(2):
                    matrix[row+2+2*side+joint,ui+2+side]=1.
                for rr,sign in [(row+6+2*side,1.),(row+7+2*side,-1.)]:
                    matrix[rr,ui+side]=sign
                    matrix[rr,pi+side]=-1.
            matrix[row+10,pi:pi+2]=1.
        self.matrix=matrix.tocsc()
        self.base_ax=self.matrix.data.copy()
        self.indices={(int(self.matrix.indices[k]),col):k for col in range(6*horizon) for k in range(self.matrix.indptr[col],self.matrix.indptr[col+1])}
        self.ax=self.base_ax.copy()
        self.lower=np.zeros(11*horizon)
        self.upper=np.zeros(11*horizon)
        self.solver=osqp.OSQP()
        self.update(np.zeros(10),0.,setup=True)
        self.solver.setup(P=self.p,q=np.zeros(6*horizon),A=self.matrix,l=self.lower,u=self.upper,
                          eps_abs=1e-4,eps_rel=1e-4,max_iter=400,check_termination=10,
                          polishing=False,adaptive_rho=True,verbose=False)


class WheelPowerProblem(CondensedProblem):
    """Two-wheel |pL|+|pR| <= cap is exactly four halfspaces; no auxiliaries."""
    def __init__(self,model,horizon,prediction_dt,watts,power_mode,tables):
        super().__init__(model,horizon,prediction_dt,watts,power_mode,tables)
        self.ip=2*horizon # Historical field: ip+2*N is decision dimension.
        self.gradient=self.gradient[:4*horizon]
        self.p=self.p[:4*horizon,:4*horizon].tocsc()
        matrix=sparse.lil_matrix((10*horizon,4*horizon))
        for stage in range(horizon):
            row,ui=10*stage,4*stage
            matrix[row,ui]=matrix[row+1,ui+1]=1.
            for side in range(2):
                for joint in range(2): matrix[row+2+2*side+joint,ui+2+side]=1.
            for rr,(sl,sr) in enumerate([(1,1),(1,-1),(-1,1),(-1,-1)]):
                matrix[row+6+rr,ui]=sl
                matrix[row+6+rr,ui+1]=sr
        self.matrix=matrix.tocsc()
        self.indices={(int(self.matrix.indices[k]),col):k for col in range(4*horizon) for k in range(self.matrix.indptr[col],self.matrix.indptr[col+1])}
        self.ax=self.matrix.data.copy()
        self.lower=np.zeros(10*horizon); self.upper=np.zeros(10*horizon)
        self.solver=osqp.OSQP()
        self.update(np.zeros(10),0.,setup=True)
        self.solver.setup(P=self.p,q=np.zeros(4*horizon),A=self.matrix,l=self.lower,u=self.upper,
                          eps_abs=1e-4,eps_rel=1e-4,max_iter=400,check_termination=10,
                          polishing=False,adaptive_rho=True,verbose=False)

    def update(self,state,support_extra,setup=False):
        # During parent initialization use its epigraph implementation.
        if len(self.lower)==11*self.n:
            return super().update(state,support_extra,setup)
        omega=np.array([(state[1]+.215*state[3])/.058,(state[1]-.215*state[3])/.058])
        self.joint_mapping=[]
        for side in range(2):
            alpha=state[4+2*side]+state[8]
            coefficient=np.array([np.interp(alpha,self.angles,self.tables[side][:,j]) for j in range(4)])
            force=(12.054*9.80665/2+(1 if side==0 else -1)*support_extra)/max(np.cos(state[4+2*side]),.5)
            self.joint_mapping.append((coefficient[2:],coefficient[:2]*force))
        for stage in range(self.n):
            row,ui=10*stage,4*stage
            bound=np.full(2,16384/3450)
            if self.mode=='per-wheel': bound=np.minimum(bound,self.watts/2/np.maximum(abs(omega),1e-9))
            self.lower[row:row+2],self.upper[row:row+2]=-bound,bound
            for side in range(2):
                coeff,bias=self.joint_mapping[side]
                for joint in range(2):
                    rr=row+2+2*side+joint
                    self.ax[self.indices[rr,ui+2+side]]=coeff[joint]
                    self.lower[rr],self.upper[rr]=-54-bias[joint],54-bias[joint]
            for rr,(sl,sr) in enumerate([(1,1),(1,-1),(-1,1),(-1,-1)]):
                self.ax[self.indices[row+6+rr,ui]]=sl*omega[0]
                self.ax[self.indices[row+6+rr,ui+1]]=sr*omega[1]
            self.lower[row+6:row+10]=-np.inf
            self.upper[row+6:row+10]=self.watts
        if setup: self.matrix.data[:]=self.ax
        else: self.solver.update(Ax=self.ax,l=self.lower,u=self.upper,q=self.gradient@state)
        return omega


def summary(values):
    values = np.asarray(values)
    return {k:float(v) for k,v in zip(['mean','p50','p99','p999','max'],[values.mean(),*np.percentile(values,[50,99,99.9]),values.max()])}


def run(model,tables,n,dt,watts,mode,cycles,export=None):
    problem = WheelPowerProblem(model,n,dt,watts,mode,tables)
    trace=None
    if export:
        directory=Path(export)/f'n{n}_w{int(watts)}'
        directory.mkdir(parents=True,exist_ok=True)
        problem.solver.codegen(str(directory/'solver'),parameters='matrices',force_rewrite=True,
                               use_float=False,printing_enable=False,profiling_enable=False,compile=False)
        trace=(directory/'trace.bin').open('wb')
        trace.write(struct.pack('6i',problem.ip+2*n,len(problem.lower),len(problem.ax),cycles*3,problem.iu,problem.n))
        (directory/'structure.bin').write_bytes(problem.matrix.indices.astype('<i4').tobytes()+problem.matrix.indptr.astype('<i4').tobytes())
    scenarios = {'small': [.02,0,.03,0,.025,0,-.015,0,.04,0],
                 'disturbance': [.10,0,.1,0,.10,0,.04,0,.12,0],
                 'fast-power': [0,1.6,0,.4,.04,0,-.02,0,.06,0]}
    output=[]
    for name,initial in scenarios.items():
        state=np.array(initial,dtype=float)
        problem.solver.warm_start(x=np.zeros(problem.ip+2*n),y=np.zeros(len(problem.lower)))
        times=[]; solves=[]; iterations=[]; failed=0; violation=0.; power_violation=0.; active_power=0; joint_active=0; invalid=0; peak_state=0.
        for step in range(cycles):
            if step and step%1000==0:
                state[8]+=.08; state[9]+=.25
            start=time.perf_counter_ns()
            omega=problem.update(state,10*np.sin(step*.001*3))
            result=problem.solver.solve(raise_error=False)
            elapsed=(time.perf_counter_ns()-start)/1000
            if trace:
                trace.write(np.r_[problem.gradient@state,problem.lower,problem.upper,problem.ax].astype('<f8').tobytes())
            times.append(elapsed); solves.append(result.info.solve_time*1e6); iterations.append(result.info.iter)
            if result.info.status_val != 1:
                failed+=1
                # Bounded software fallback; not asserted to be a feasible robot controller.
                u=np.clip(-problem.k@state,-2.,2.)
            else:
                u=result.x[problem.iu:problem.iu+4]
                problem.matrix.data[:]=problem.ax
                constraint=problem.matrix@result.x
                violation=max(violation,float(np.maximum(problem.lower-constraint,constraint-problem.upper).max()))
                mechanical=float(np.abs(omega*u[:2]).sum())
                power_violation=max(power_violation,mechanical-watts)
                active_power+=mechanical>watts*.95
                for side in range(2):
                    coeff,bias=problem.joint_mapping[side]
                    torque=coeff*u[2+side]+bias
                    joint_active+=int(np.max(abs(torque))>53.)
            state=problem.plant_a@state+problem.plant_b@u
            peak_state=max(peak_state,float(abs(state).max()))
            if not np.isfinite(state).all() or max(abs(state[[4,6,8]]))>.6:
                invalid+=1
                state=np.array(initial,dtype=float)
        output.append(dict(scenario=name,horizon=n,prediction_dt_s=dt,prediction_span_s=n*dt,watts_total=watts,
                           cycles=cycles,variables=problem.ip+2*n,constraint_rows=len(problem.lower),
                           update_and_solve_us=summary(times),solver_only_us=summary(solves),iterations=summary(iterations),
                           over_1ms=sum(t>1000 for t in times),not_solved=failed,max_qp_violation=violation,
                           max_frozen_speed_power_violation_w=max(0.,power_violation),power_active_cycles=active_power,
                           joint_active_side_cycles=joint_active,model_envelope_resets=invalid,peak_state=peak_state,
                           final_state=state.tolist()))
    if trace:
        trace.close()
    return output


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--root',type=Path,default=Path(__file__).resolve().parents[2])
    parser.add_argument('--cycles',type=int,default=3000)
    parser.add_argument('--horizons',type=int,nargs='+',default=[20,40,80])
    parser.add_argument('--prediction-dt',type=float,default=.005)
    parser.add_argument('--watts',type=float,nargs='+',default=[40,80])
    parser.add_argument('--mode',choices=['total-absolute','per-wheel'],default='total-absolute')
    parser.add_argument('--output',type=Path,default=Path('results.json'))
    parser.add_argument('--export',type=Path)
    args=parser.parse_args()
    model=load_model(args.root)
    tables=geometry_tables()
    report={'method':'Python orchestration + native OSQP; unpaced CPU-bound simulation; no transport',
            'power_assumption':'sum of absolute mechanical wheel power; speeds frozen across horizon; cap is total, per-wheel mode uses cap/2',
            'geometry':'fixed 0.25m legs; production FK/Jacobian; offline IK table, online measured-angle interpolation; 54Nm joints',
            'timing_includes':'Python constraint coefficient/bounds updates, OSQP matrix update/refactor and solve; excludes validation, plant step, communication',
            'cpu_affinity':sorted(os.sched_getaffinity(0)), 'osqp_version':osqp.__version__, 'results':[]}
    for n in args.horizons:
        for watts in args.watts:
            rows=run(model,tables,n,args.prediction_dt,watts,args.mode,args.cycles,args.export)
            report['results'].extend(rows)
            args.output.write_text(json.dumps(report,indent=2))
            for row in rows:
                print(json.dumps(row),flush=True)


if __name__=='__main__':
    main()
