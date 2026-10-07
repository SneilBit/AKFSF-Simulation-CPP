---
name: lkf-exercise
description: Implement and verify a step of the Technite LKF exercise PDFs in python/kalmanfilter.py, then check it headlessly and report RMSE and covariance numbers.
---

# LKF exercise workflow

1. Read the exercise PDF at the repo root (`Linear_Vehicle_Tracker_*_Exercise.pdf`) and implement ONLY the step asked.
2. Edit the `# ENTER YOUR CODE HERE` sections of `python/kalmanfilter.py`; keep comments explaining each matrix and equation.
3. Verify headlessly from `python/` with `SDL_VIDEODRIVER=dummy`:
   - build `Simulation()`, `reset(main.loadSimulationNParameters())`, run `while sim.isRunning(): sim.update()`
   - print `calculateRMSE` of `sim.m_filter_error_x_position_history`, `..._y_...`, heading (x180/pi), velocity
   - for covariance checks use `sim.m_kalman_filter.getCovariance()`; the sim draws a 3-sigma ellipse
   - override constants via `kalmanfilter.INIT_POS_STD` etc. before each run
4. Launch the GUI on request with `python3 main.py` (run in background; it exits when the window closes).
5. Report measured numbers, and say what is still a stub (GPS init branch, lidar).
6. If asked for a report, extend the Claude Docs artifact in CLAUDE.md: matplotlib figures/GIFs -> Artifact asset upload
   -> `create blob` -> insert `![alt](blob/<id>)`; use sigma symbols and label every term.
