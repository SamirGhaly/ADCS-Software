function Control_Algorithm(controller)

if strcmp(controller,'No Controller')
    assignin('base','Control_Method',0)

elseif strcmp(controller,'PD')
    assignin('base','Control_Method',1)

elseif strcmp(controller,'SMC')
    assignin('base','Control_Method',2)

elseif strcmp(controller,'Bdot')
    assignin('base','Control_Method',3)

elseif strcmp(controller,'K Omega')
    assignin('base','Control_Method',4)

elseif strcmp(controller,'LQR')
    assignin('base','Control_Method',5)
end

end