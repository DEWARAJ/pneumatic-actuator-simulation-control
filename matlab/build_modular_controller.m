function build_modular_controller()
% Separate sampled controller and continuous plant, verified nominal/heavy.
% Disturbance/noise/delay studies remain in Python; no Amesim co-simulation.
assert(license('test','Simulink'),'Simulink required');
root=fileparts(fileparts(mfilename('fullpath')));
folder=fullfile(root,'upgrade_results');
report=jsondecode(fileread(fullfile(folder,'report.json')));
name='pneumatic_sampled_controller';
if bdIsLoaded(name), close_system(name,0); end
new_system(name);
set_param(name,'Solver','ode45','StopTime','5','MaxStep','0.0005',...
    'RelTol','1e-8','AbsTol','1e-12');
set_param(name,'InitFcn','addpath(fullfile(fileparts(get_param(bdroot,''FileName'')),''matlab''));');
workspace=get_param(name,'ModelWorkspace');
workspace.DataSource='Model File';
controller_config=[double(report.controller_gains(:))',...
    report.selected_estimator_alpha,report.selected_estimator_beta,...
    report.pressure_protection.trip_pa_abs,report.pressure_protection.debounce_samples];
assignin(workspace,'controller_config',controller_config);
assignin(workspace,'plant_parameters',pneumatic_parameters('tuned'));
add_block('simulink/User-Defined Functions/Level-2 MATLAB S-Function',[name,'/SampledController'],...
    'FunctionName','pneumatic_controller_sfunction','Parameters','controller_config',...
    'Position',[60,80,240,140]);
add_block('simulink/User-Defined Functions/Level-2 MATLAB S-Function',[name,'/PneumaticPlant'],...
    'FunctionName','pneumatic_plant_sfunction','Parameters','plant_parameters',...
    'Position',[340,80,520,140]);
add_block('simulink/Sinks/To Workspace',[name,'/Trace'],...
    'VariableName','modular_trace','SaveFormat','Structure With Time',...
    'Position',[600,80,700,120]);
add_block('simulink/Sinks/To Workspace',[name,'/ControllerDiagnostics'],...
    'VariableName','controller_diagnostics','SaveFormat','Structure With Time',...
    'Position',[535,180,705,220]);
add_line(name,'SampledController/1','PneumaticPlant/1');
add_line(name,'PneumaticPlant/1','SampledController/1','autorouting','on');
add_line(name,'PneumaticPlant/1','Trace/1');
add_line(name,'SampledController/2','ControllerDiagnostics/1','autorouting','on');
a=Simulink.Annotation(name,['5 ms sampled PI; alpha-beta velocity estimate from position; conditional anti-windup.',...
    newline,'Supply pressure interlock latches after 20 ms below 2 bar absolute and commands neutral.',...
    newline,'Separate isothermal plant; nominal model saved. Native Amesim remains pending.']);
a.Position=[30,270];
validation=struct();
for cases={'nominal','heavy'}
    caseName=cases{1}; scenario=report.scenarios.(caseName).scenario;
    p=pneumatic_parameters('tuned'); p.mass=scenario.mass; p.load=scenario.load;
    p.friction=scenario.friction; p.supply=scenario.supply; p.valve_tau=scenario.valve_tau;
    assignin(workspace,'plant_parameters',p);
    save_system(name,fullfile(root,'pneumatic_sampled_controller.slx'));
    result=sim(name); trace=result.get('modular_trace');
    source=readtable(fullfile(folder,[caseName,'.csv']));
    position=interp1(trace.time,trace.signals.values(:,1),source.time_s,'linear');
    discrepancy=max(abs(position-source.position_m))*1000;
    validation.(caseName)=struct('max_position_difference_mm',discrepancy,'passed',discrepancy<.01);
    save(fullfile(folder,['simulink_',caseName,'.mat']),'trace');
    assert(validation.(caseName).passed,'Modular Simulink comparison failed');
end
% Restore nominal parameters for the delivered model.
p=pneumatic_parameters('tuned'); assignin(workspace,'plant_parameters',p);
save_system(name,fullfile(root,'pneumatic_sampled_controller.slx'));
print(['-s',name],'-dpng',fullfile(folder,'simulink_architecture.png'));
f=fopen(fullfile(folder,'simulink_validation.json'),'w');
fprintf(f,'%s',jsonencode(validation,PrettyPrint=true)); fclose(f);
close_system(name,0); disp(validation);
end
