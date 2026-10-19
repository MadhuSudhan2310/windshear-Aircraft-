import numpy as np
from scipy.integrate import solve_ivp

V0=150.0
Za,Mal,Mq,Md=1.0,-6.0,-1.5,5.0   # alpha_dot=-Za*a_aero+q ; q_dot=k*(Mal*a_aero+Md*de)+Mq*q
tau_act=0.05; dmax=np.deg2rad(25); tauV=15.0

def shear(t,A_h=15.0,A_w=8.0,t0=20.0,T=20.0):
    # headwind -> tailwind transition (A_h each way) and downdraft bump
    s=np.clip((t-t0)/T,0,1)
    uw=A_h*np.cos(np.pi*s)          # +15 headwind -> -15 tailwind
    duw=-A_h*np.pi/T*np.sin(np.pi*s)*((t>t0)&(t<t0+T))
    ww=A_w*np.sin(np.pi*s)**2*((t>t0)&(t<t0+T))   # downdraft m/s
    return uw-A_h,duw,ww   # deviation from initial headwind

def run(Kp,Ki,Kd,shear_on=True,Ah=15,Aw=8,tf=80,cmd=0.0,N=20,limit=True):
    def f(t,x):
        a,q,th,h,dVa,I,dfs,de=x
        uw,duw,ww=shear(t,Ah,Aw) if shear_on else (0,0,0)
        Va=V0+dVa
        k=(Va/V0)**2
        a_aero=a-ww/Va
        e=cmd-th
        # derivative on filtered measurement
        dth=q
        u=Kp*e+Ki*I-Kd*dfs   # dfs filtered derivative of theta (N filter)
        du=np.clip(u,-dmax,dmax) if limit else u
        dI=e+( (du-u)/ (Kp if Kp else 1) if limit else 0)  # back-calc anti-windup
        ddfs=N*(q-dfs)
        dde=(du-de)/tau_act
        return [-Za*a_aero+q, k*(Mal*a_aero+Md*de)+Mq*q, q, Va*(th-a),
                -dVa/tauV+duw, dI, ddfs, dde]
    t=np.linspace(0,tf,4001)
    s=solve_ivp(f,[0,tf],np.zeros(8),t_eval=t,max_step=0.02,method='RK45')
    return s.t,s.y
def metrics(Kp,Ki,Kd,**kw):
    t,y=run(Kp,Ki,Kd,**kw)
    th=np.rad2deg(y[2]);h=y[3]
    return dict(th_max=np.max(np.abs(th)),th_rms=np.sqrt(np.mean(th**2)),h_dev=np.max(np.abs(h)),de_max=np.rad2deg(np.max(np.abs(y[7]))))
if __name__=="__main__":
    # step response, no shear
    for g in [(1.0,0.3,0.4),(1.5,0.5,0.6),(2.0,0.8,0.8),(3,1,1)]:
        t,y=run(*g,shear_on=False,cmd=np.deg2rad(5))
        th=np.rad2deg(y[2]); 
        print(g,"overshoot%",(th.max()-5)/5*100,"final",th[-1], "ts(2%)",t[np.where(abs(th-5)>0.1)[0][-1]])
    for g in [(1.0,0.3,0.4),(1.5,0.5,0.6),(2.0,0.8,0.8),(3,1,1)]:
        print(g,metrics(*g))
    print("open loop (no PID):",metrics(0,0,0))
