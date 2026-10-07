# AKFSF Simulation (Python port): project context

Technite "Advanced Kalman Filtering and Sensor Fusion" course. The student works in `python/kalmanfilter.py`
(the other `kalmanfilter_*_student/answer.py` files are course-provided references; do not edit them).

## Run
- GUI: `cd python && python3 main.py` (pygame; keys 1-9,0 = profiles, Space pause, `[` `]` speed, R reset, Esc quit).
- Headless check: set `SDL_VIDEODRIVER=dummy`, build `Simulation()`, `reset(main.loadSimulationNParameters())`,
  loop `while sim.isRunning(): sim.update()`, then read `sim.m_filter_error_*_history` and `utils.calculateRMSE`.
- Running `kalmanfilter.py` directly does nothing (no `__main__`).

## Exercise status (LKF)
- Exercise 1 (prediction, PDF `Linear_Vehicle_Tracker_Prediction_Step_Exercise.pdf`): DONE, Steps 1-8.
  State [X,Y,VX,VY]; F constant velocity; Q=diag(ACCEL_STD^2); L=[[.5dt^2,0],[0,.5dt^2],[dt,0],[0,dt]]; P=FPF'+LQL'.
  Init in `predictionStepDt` when `INIT_ON_FIRST_PREDICTION`: state=[0,0,5cos45,5sin45], P=diag(pos^2,pos^2,vel^2,vel^2).
- Exercise 2 (update, PDF `Linear_Vehicle_Tracker_Update_Step_Exercise.pdf`): Steps 2-3 DONE in `handleGPSMeasurement`
  (H picks X,Y; R=diag(GPS_POS_STD^2); y=z-Hx, S=HPH'+R, K=PH'S^-1, x+=Ky, P=(I-KH)P).
  Step 4 (experiments) measured, see below. The GPS-initialisation `else` branch is still an empty stub
  (only used when `INIT_ON_FIRST_PREDICTION = False`). Lidar handlers are stubs.
- Constants live at the top of `kalmanfilter.py`: INIT_POS_STD, INIT_VEL_STD, ACCEL_STD, GPS_POS_STD (=3).

## Useful measured numbers
- Profile 1 with INIT 0/0, ACCEL 0: RMSE exactly 0.00 (estimate == truth, K=0). RMSE moves only once P is nonzero.
- Position RMSE sqrt(X^2+Y^2) for (INIT_POS, INIT_VEL, ACCEL) on profiles 1/2/3/4:
  (0,0,0): 0.0/363/318/545; (0,10,0.1): 1.6/167/23.6/20.8; (5,5,0.1): 1.7/23.6/23.4/20.8.
- Profile 1 (5,5,0.1): 3-sigma position radius settles ~3.5 m by t=30 s; prediction-only reaches ~1800 m at 120 s.
- Prediction-only growth: Step 6 const 15 m; Step 7 5*t (600 m at 120 s); Step 8 ~72 m at 120 s.

## Reports (Claude Docs artifacts)
- "LKF Prediction Step Report": https://claude.ai/artifact/PM4YQ9JAQwkey3V59tsEPx
- "Kalman Filter From Scratch" (Parts 1-11, figures + animations, Part 11 = update step):
  https://claude.ai/artifact/7S1a5tzvoWgLcuTvurmCLg
Notation used in reports: sigma_x^2 (position var), sigma_v^2 (velocity var), sigma_xv (their link), sigma_a (ACCEL_STD).

## Working preferences
- Explain intuitively from basics, with a figure or animation per concept; use proper symbols, never a/b/c.
- Do not implement the next exercise step unless asked (user goes step by step following the PDF).
