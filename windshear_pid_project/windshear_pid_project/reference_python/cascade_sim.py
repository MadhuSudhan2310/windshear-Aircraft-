import numpy as np
from scipy.integrate import solve_ivp
from aircraft_sim import *
import aircraft_sim as sim

def run2(inner,outer,shear_on=True,Ah=15,Aw=8,tf=100,hcmd=0.0,N=20,tcmax=np.deg2rad(10)):
    Kp,Ki,Kd=inner; Kph,Kih,Kdh=outer
    def f(t,x):
        a,q,th,h,dVa,I,dfs,de,Ih=x
        uw,duw,ww=sim.shear(t,Ah,Aw) if shear_on else (0,0,0)
        Va=V0+dVa; k=(Va/V0)**2
        a_aero=a-ww/Va
        hdot=Va*(th-a)
        eh=hcmd-h
        tc_u=Kph*eh+Kih*Ih-Kdh*hdot
        tc=np.clip(tc_u,-tcmax,tcmax)
        dIh=eh+(tc-tc_u)/max(Kph,1e-6)
        e=tc-th
        u=Kp*e+Ki*I-Kd*dfs
        du=np.clip(u,-dmax,dmax)
        dI=e+(du-u)/Kp
        return [-Za*a_aero+q, k*(Mal*a_aero+Md*de)+Mq*q, q, hdot,
                -dVa/tauV+duw, dI, N*(q-dfs), (du-de)/tau_act, dIh]
    t=np.linspace(0,tf,5001)
    s=solve_ivp(f,[0,tf],np.zeros(9),t_eval=t,max_step=0.02)
    return s.t,s.y
inner=(2.0,0.8,0.8)
for outer in [(0.0,0,0),(0.002,0.0,0.01),(0.004,0.0004,0.01),(0.006,0.0006,0.015),(0.01,0.001,0.02)]:
    t,y=run2(inner,outer)
    h=y[3]; th=np.rad2deg(y[2])
    # step in altitude without shear
    t2,y2=run2(inner,outer,shear_on=False,hcmd=50.0)
    ov=(y2[3].max()-50)/50*100
    print(outer,"h_dev %.1f"%np.abs(h).max(),"th_max %.2f"%np.abs(th).max(),"de_max %.2f"%np.rad2deg(np.abs(y[7]).max()),"| step os%% %.1f"%ov, "h_end %.1f"%y2[3][-1])
print("sweep intensity with outer (0.004,0.0004,0.01)")
for A in [0,5,10,15,20,25]:
    t,y=run2(inner,(0.004,0.0004,0.01),Ah=A,Aw=A*8/15)
    t0,y0=run2(inner,(0,0,0),Ah=A,Aw=A*8/15)
    print(A,"h_dev outer %.1f"%np.abs(y[3]).max(),"th_max %.2f"%np.rad2deg(np.abs(y[2]).max()),"de_max %.2f"%np.rad2deg(np.abs(y[7]).max()),"| pitch-only h %.1f"%np.abs(y0[3]).max())
