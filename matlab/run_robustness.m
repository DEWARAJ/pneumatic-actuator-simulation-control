function run_robustness()
% Independent cross-check of nominal/heavy sampled-controller traces.
% Reuses the verified MATLAB plant, disabling its continuous controller.
root=fileparts(fileparts(mfilename('fullpath')));
folder=fullfile(root,'robustness_results');
study=jsondecode(fileread(fullfile(folder,'report.json')));
gains=study.selected_gains; dt=.0005; period=.005;
report=struct();
for caseName={'nominal','heavy'}
    name=caseName{1}; scenario=study.scenarios.(name).scenario;
    p=pneumatic_parameters('tuned');
    p.mass=scenario.mass; p.load=scenario.load; p.friction=scenario.friction;
    p.supply=scenario.supply; p.valve_tau=scenario.valve_tau;
    p.kp=0; p.ki=0; p.kd=0;
    n=round(5/dt); times=(0:n)'*dt; states=zeros(n+1,8);
    y=pneumatic_initial(p); states(1,:)=y'; command=0; dint=0;
    for k=0:n-1
        t=times(k+1);
        if mod(k,round(period/dt))==0
            target=.010; if t>=.5, target=.035; end; if t>=3, target=.015; end
            error=target-y(1); raw=gains(1)*error+gains(2)*y(5)-gains(3)*y(2);
            command=max(-scenario.valve_limit,min(scenario.valve_limit,raw)); dint=error;
            if (raw>scenario.valve_limit && error>0)||(raw< -scenario.valve_limit && error<0)
                dint=0;
            end
        end
        k1=plant(t,y,p,command,dint);
        k2=plant(t+dt/2,y+dt*k1/2,p,command,dint);
        k3=plant(t+dt/2,y+dt*k2/2,p,command,dint);
        k4=plant(t+dt-eps(t+dt),y+dt*k3,p,command,dint);
        y=y+dt*(k1+2*k2+2*k3+k4)/6; states(k+2,:)=y';
    end
    source=readtable(fullfile(folder,[name,'_robust.csv']));
    assert(max(abs(times-source.time_s))<1e-10,'Time grid differs');
    discrepancy=max(abs(states(:,1)-source.position_m))*1000;
    report.(name)=struct('max_position_difference_mm',discrepancy,'passed',discrepancy<.01);
    writetable(table(times,states(:,1),'VariableNames',{'time_s','position_m'}),...
        fullfile(folder,['matlab_',name,'.csv']));
    assert(report.(name).passed,'Sampled MATLAB/Python position check failed');
end
f=fopen(fullfile(folder,'matlab_validation.json'),'w');
fprintf(f,'%s',jsonencode(report,PrettyPrint=true)); fclose(f); disp(report);
end

function dy=plant(t,y,p,command,dint)
dy=pneumatic_rhs(t,y,p);
dy(5)=dint;
dy(6)=(command-y(6))/p.valve_tau;
end
