import matlab.engine

eng = matlab.engine.start_matlab()

# call the mav_response function
out = eng.mav_response(matlab.double([0.88,.041]),matlab.double([.88,.16,.304]),
matlab.double([.0181,.0039]),matlab.double([-1.6,-.4338,0]),matlab.double([.234,.52]),0)
