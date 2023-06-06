ts = 0;
dt = 0.025;
te = 5;
time = TIME(ts,dt,te);
in_prog_func = @(app) dfunc(app);
post_func = @(app) dfunc(app);
motive = Connector_Natnet_sim(1, dt, 0);              % 3rd arg is a flag for noise (1 : active )
logger = LOGGER(1, size(ts:dt:te, 2), 0, [],[]);
initial_state.p = arranged_position([0, 0], 1, 1, 0);
initial_state.q = [1; 0; 0; 0];
initial_state.v = [0; 0; 0];
initial_state.w = [0; 0; 0];

agent = DRONE;
agent.plant = MODEL_CLASS(agent,Model_Quat13(dt, initial_state, 1));
agent.parameter = DRONE_PARAM("DIATONE");
agent.model = MODEL_CLASS(agent,Model_EulerAngle(dt, initial_state, 1));
agent.sensor = MOTIVE(agent, Sensor_Motive(1,0, motive).param);
agent.estimator = EKF(agent, Estimator_EKF(agent, ["p", "q"]).param);
agent.reference = TIME_VARYING_REFERENCE(agent,Reference_Time_Varying("gen_ref_saddle",{"freq",5,"orig",[0;0;1],"size",[2,2,0.5]}).param);
agent.controller = HLC(agent,Controller_HL(dt).param);

function dfunc(app)
app.logger.plot({1, "p", "er"},"FH",app.UIAxes,"xrange",[app.time.ts,app.time.te]);
app.logger.plot({1, "q", "er"},"FH",app.UIAxes2,"xrange",[app.time.ts,app.time.te]);
app.logger.plot({1, "input", ""},"FH",app.UIAxes3,"xrange",[app.time.ts,app.time.te]);
end