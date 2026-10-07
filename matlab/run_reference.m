function run_reference()
% Run from MATLAB after adding this directory to the path.
% Cross-checks Python CSV on the same grid using MATLAB ode45.
root=fileparts(fileparts(mfilename('fullpath')));
report=struct();
for name={'baseline','tuned'}
    caseName=name{1}; p=pneumatic_parameters(caseName);
    y0=pneumatic_initial(p); times=[]; states=[];
    breaks=[0,.5,2,3,5];
    opts=odeset('RelTol',1e-8,'AbsTol',[1e-10,1e-9,1e-13,1e-13,1e-10,1e-9,1e-13,1e-13],...
        'MaxStep',.002);
    for i=1:4
        left=breaks(i); right=breaks(i+1);
        grid=linspace(left,right,round((right-left)/.002)+1);
        terminalLeft=right-eps(right);
        [tt,yy]=ode45(@(t,y)pneumatic_rhs(min(t,terminalLeft),y,p),grid,y0,opts);
        y0=yy(end,:)';
        if i>1, tt=tt(2:end); yy=yy(2:end,:); end
        times=[times;tt]; states=[states;yy]; %#ok<AGROW>
    end
    target=.010*ones(size(times)); target(times>=.5)=.035; target(times>=3)=.015;
    aa=pi*p.bore^2/4; ab=pi*(p.bore^2-p.rod^2)/4;
    pa=states(:,3)*p.R*p.temperature./(p.dead_volume+aa*states(:,1));
    pb=states(:,4)*p.R*p.temperature./(p.dead_volume+ab*(p.stroke-states(:,1)));
    output=table(times,target,states(:,1),states(:,2),pa,pb,states(:,6),states(:,7),states(:,8),...
        'VariableNames',{'time_s','reference_m','position_m','velocity_m_s','pressure_a_pa_abs',...
        'pressure_b_pa_abs','valve_spool','signed_supply_mass_kg','signed_exhaust_mass_kg'});
    writetable(output,fullfile(root,'results',['matlab_',caseName,'.csv']));
    source=readtable(fullfile(root,'results',[caseName,'.csv']));
    assert(max(abs(source.time_s-times))<1e-10,'Time grids differ');
    discrepancy=max(abs(source.position_m-states(:,1)))*1000;
    report.(caseName)=struct('max_position_difference_mm',discrepancy,'passed',discrepancy<.01);
    assert(discrepancy<.01,'MATLAB/Python comparison exceeds 0.01 mm');
end
f=fopen(fullfile(root,'results','matlab_validation.json'),'w');
fprintf(f,'%s',jsonencode(report,PrettyPrint=true)); fclose(f);
disp(report);
end
