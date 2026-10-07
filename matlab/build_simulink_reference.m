function build_simulink_reference()
% Build a native .slx integrating the reference RHS in Simulink.
% Level-2 MATLAB S-function: simulation only, no code-generation claim.
assert(license('test','Simulink'),'Simulink license required');
clear pneumatic_simulink_rhs
root=fileparts(fileparts(mfilename('fullpath')));
name='pneumatic_reference';
if bdIsLoaded(name), close_system(name,0); end
new_system(name);
set_param(name,'Solver','ode45','StopTime','5','MaxStep','0.002',...
    'RelTol','1e-7','AbsTol','1e-12');
add_block('simulink/Sources/Clock',[name,'/Time'],'Position',[30,60,60,80]);
add_block('simulink/Signal Routing/Mux',[name,'/TimeAndStates'],...
    'Inputs','2','Position',[110,60,115,140]);
add_block('simulink/User-Defined Functions/Level-2 MATLAB S-Function',[name,'/Equations'],...
    'FunctionName','pneumatic_sfunction',...
    'Position',[170,80,350,120]);
add_block('simulink/Continuous/Integrator',[name,'/States'],...
    'InitialCondition','pneumatic_initial(pneumatic_parameters(''tuned''))',...
    'Position',[410,80,440,110]);
add_block('simulink/Sinks/To Workspace',[name,'/Trace'],...
    'VariableName','pneumatic_trace','SaveFormat','Structure With Time',...
    'Position',[500,80,610,110]);
add_line(name,'Time/1','TimeAndStates/1');
add_line(name,'States/1','TimeAndStates/2','autorouting','on');
add_line(name,'TimeAndStates/1','Equations/1');
add_line(name,'Equations/1','States/1');
add_line(name,'States/1','Trace/1');
annotation=Simulink.Annotation(name,['Isothermal synthetic reference; 8 states: x, v, mA, mB, I, spool, supply mass, exhaust mass.',...
    newline,'Simulation-only wrapper. Amesim plant and co-simulation remain pending.']);
annotation.Position=[30,200];
save_system(name,fullfile(root,'pneumatic_reference.slx'));
simulation=sim(name);
trace=simulation.get('pneumatic_trace');
save(fullfile(root,'results','simulink_trace.mat'),'trace');
source=readtable(fullfile(root,'results','tuned.csv'));
position=interp1(trace.time,trace.signals.values(:,1),source.time_s,'linear');
discrepancy=max(abs(position-source.position_m))*1000;
report=struct('max_position_difference_mm',discrepancy,'passed',discrepancy<.01,...
    'scope','Native Simulink reference integration; not Amesim co-simulation');
f=fopen(fullfile(root,'results','simulink_validation.json'),'w');
fprintf(f,'%s',jsonencode(report,PrettyPrint=true)); fclose(f);
assert(report.passed,'Simulink/Python position disagreement exceeds 0.01 mm');
close_system(name,0);
end
