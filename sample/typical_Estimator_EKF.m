function typical_Estimator_EKF(agent)

%% estimator class demo
% estimator property ��Estimator class�̃C���X�^���X�z��Ƃ��Ē�`
% ���ׂĂ̋@�̂œ���ݒ�
Estimator.name="ekf";
Estimator.type="EKF";
dt = agent(1).model.dt;
    EKF_param.Q = eye(6)*7.058E-5;%.*[50;50;50;1E04;1E04;1E04];%1.0e-1; % �V�X�e���m�C�Y�iModel�N���X�R���j
    EKF_param.R = diag([[6.716E-5; 7.058E-5; 7.058E-5];[6.716E-5; 7.058E-5; 7.058E-5]/100]);%eye(6).*1.0E-4;%1.0e-1; % �ϑ��m�C�Y�iSensor�N���X�R���j
    
    n = agent(1).model.dim(1);
%    EKF_param.JacobianH = @(~,~) eye(n); % C�s�� �S��Ԋϑ�
    pqnum = sum(agent(1).model.state.num_list(contains(agent(1).model.state.list,["p","q"]))); % p, q �̎����̘a
    EKF_param.JacobianH = @(~,~) [eye(pqnum),zeros(pqnum,n-pqnum)];% C�s�� p, q�ϑ�
if isfield(agent(1).sensor,'imu')
    EKF_param.JacobianH = ["w","a"];
end
    EKF_param.H = EKF_param.JacobianH;
    EKF_param.B = [eye(6)*dt^2;eye(6)*dt];%eye(sum(agent(1).model.state.num_list)); % �V�X�e���m�C�Y�������`�����l��
    EKF_param.P = eye(n); % ���������U�s��
    EKF_param.list=["p","q"];
Estimator.param=EKF_param;
for i = 1:length(agent)
    agent(i).set_estimator(Estimator);
end
disp('Execute "do" method with following parameter to operate EKF estimator.');
disp('param.estimator={{agent(i),agent(i).sensor.result.state.get(["q","p"]),[]}};')
end