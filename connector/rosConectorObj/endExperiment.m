function endExperiment(obj)
%     obj.Message{obj.cmdVel}.Header.Stamp    = rostime('now');
%     obj.Message{obj.cmdVel}.Twist.Linear.X  = DataSet.V(1,1);
%     obj.Message{obj.cmdVel}.Twist.Angular.Z = DataSet.V(2,1);
    obj.pubMessage{obj.cmdVel}.Linear.X  = 0.;
    obj.pubMessage{obj.cmdVel}.Angular.Z = 0.;
    send(obj.pubTopic{obj.cmdVel}, obj.pubMessage{obj.cmdVel});
	rosshutdown;
end