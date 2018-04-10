import matlab.engine

eng = matlab.engine.start_matlab()

# call the mav_response function
out = eng.mav_response([0.88,.041],[.88,.16,.304],[.0181,.0039],[-1.6,-.4338,0],[.234,.52],0)
