function typical_Reference_Time_Varying(agent,f_name,param)
%% reference class demo
% reference property ��Reference class�̃C���X�^���X�z��Ƃ��Ē�`
clear Reference
Reference.type=["TimeVaryingReference"];
Reference.name=["timeVarying"];
Reference.param={f_name,param,"HL"};
for i = 1:length(agent)
    agent(i).set_reference(Reference);
end
end