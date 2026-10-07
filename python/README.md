# AKFSF Simulation - Python Port

A line-by-line Python port of the C++ simulation in `../src`. Each module maps to a C++ file with the same name:

| C++ | Python |
|---|---|
| `main.cpp` | `main.py` |
| `simulation.h/.cpp` | `simulation.py` |
| `car.h` | `car.py` |
| `sensors.h/.cpp` | `sensors.py` |
| `beacons.h/.cpp` | `beacons.py` |
| `display.h/.cpp` (SDL2) | `display.py` (pygame) |
| `utils.h/.cpp` (Eigen) | `utils.py` (numpy) |
| `kalmanfilter.h` | `kalmanfilter_base.py` |
| `kalmanfilter*.cpp` | `kalmanfilter*.py` |
| `<random>` (`std::mt19937`, distributions) | `cpp_random.py` |

## Setup

```
pip install -r requirements.txt
python main.py
```

The font is loaded from `../data/Roboto-Regular.ttf`.

## Usage

The workflow is the same as the C++ version. To use one of the filter files, copy it over `kalmanfilter.py`. For example, copy `kalmanfilter_ekf_answer.py` to `kalmanfilter.py`. Keys and motion profiles 1-9,0 are unchanged.

In Python, the two C++ overloads of `KalmanFilter::predictionStep` are named `predictionStepDt(dt)` and `predictionStepGyro(gyro, dt)`. Calls to `self.predictionStep(dt)` and `self.predictionStep(gyro, dt)` still work as they did in C++.

## Fidelity notes

- `cpp_random.py` reproduces GNU libstdc++'s `std::mt19937`, `uniform_real_distribution` and `normal_distribution` algorithms. The beacon map and sensor noise therefore match the Ubuntu/g++ build. This includes g++'s right-to-left argument evaluation in `addBeacon(pos_dis(..), pos_dis(..))`.
- `llt_matrixL()` reproduces Eigen's Cholesky, including how it behaves when a matrix is not positive-definite. `inverse()` does not raise on a singular matrix, which matches Eigen.
- The display uses pygame, which is built on SDL2. `SDL_RENDERER_PRESENTVSYNC` is emulated with a 60 FPS frame limit.
