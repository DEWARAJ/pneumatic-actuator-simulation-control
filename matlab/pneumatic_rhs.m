function dy = pneumatic_rhs(t,y,p)
% Independent MATLAB implementation; SI units, isothermal chambers.
% Synthetic actuator; not calibrated to the portfolio robot.
x=y(1); v=y(2); ma=y(3); mb=y(4); integral=y(5); spool=y(6);
aa=pi*p.bore^2/4; ab=pi*(p.bore^2-p.rod^2)/4;
va=p.dead_volume+aa*x; vb=p.dead_volume+ab*(p.stroke-x);
assert(min([va,vb,ma,mb])>0,'Nonphysical chamber state');
pa=ma*p.R*p.temperature/va; pb=mb*p.R*p.temperature/vb;
target=.010; if t>=.5, target=.035; end; if t>=3, target=.015; end
error=target-x; raw=p.kp*error+p.ki*integral-p.kd*v;
command=max(-1,min(1,raw)); dint=error;
if (raw>1 && error>0)||(raw< -1 && error<0), dint=0; end
aperture=min(abs(spool),1)*p.valve_area;
if spool>=0
    sa=flow(p.supply,pa,aperture,p); sb=0;
    ea=0; eb=flow(pb,p.atmosphere,aperture,p);
else
    sa=0; sb=flow(p.supply,pb,aperture,p);
    ea=flow(pa,p.atmosphere,aperture,p); eb=0;
end
leak=flow(pa,pb,p.leakage_area,p);
load=0; if t>=2, load=p.load; end
force=aa*(pa-p.atmosphere)-ab*(pb-p.atmosphere);
force=force-p.spring*(x-p.initial_position)-p.damping*v;
force=force-p.friction*tanh(v/p.friction_velocity)-load;
if x<0
    force=force-p.stop_stiffness*x-p.stop_damping*min(v,0);
elseif x>p.stroke
    force=force-p.stop_stiffness*(x-p.stroke)-p.stop_damping*max(v,0);
end
dy=[v;force/p.mass;sa-ea-leak;sb-eb+leak;dint;...
    (command-spool)/p.valve_tau;sa+sb;ea+eb];
end

function q=flow(p1,p2,area,p)
assert(min(p1,p2)>0,'Absolute pressure must stay positive');
if area<=0 || abs(p1-p2)<1e-9, q=0; return; end
up=max(p1,p2); ratio=min(p1,p2)/up;
critical=(2/(p.gamma+1))^(p.gamma/(p.gamma-1));
if ratio<=critical
    factor=sqrt(p.gamma)*(2/(p.gamma+1))^((p.gamma+1)/(2*(p.gamma-1)));
else
    factor=sqrt(max(0,2*p.gamma/(p.gamma-1)*...
        (ratio^(2/p.gamma)-ratio^((p.gamma+1)/p.gamma))));
end
q=sign(p1-p2)*p.discharge*area*up/sqrt(p.R*p.temperature)*factor;
end
