# ------------------------------------------------------------------------------- //
# Exact Python re-implementations of the C++ <random> components used by the
# simulation, following the GNU libstdc++ algorithms (the Ubuntu/g++ build target).
# This makes the generated sensor noise and beacon map identical to the C++ build.
#
#   std::mt19937                              -> mt19937
#   std::generate_canonical<double,53>        -> generate_canonical
#   std::uniform_real_distribution<double>    -> uniform_real_distribution
#   std::normal_distribution<double>          -> normal_distribution (Marsaglia polar)
# ------------------------------------------------------------------------------- //

import math


class mt19937:
    """std::mt19937 (32-bit Mersenne Twister). Default seed is 5489, as in C++."""

    default_seed = 5489
    _N = 624
    _M = 397

    def __init__(self, seed=default_seed):
        self.seed(seed)

    def seed(self, value=default_seed):
        mt = [0] * self._N
        mt[0] = value & 0xFFFFFFFF
        for i in range(1, self._N):
            mt[i] = (1812433253 * (mt[i - 1] ^ (mt[i - 1] >> 30)) + i) & 0xFFFFFFFF
        self._mt = mt
        self._index = self._N

    @staticmethod
    def min():
        return 0

    @staticmethod
    def max():
        return 0xFFFFFFFF

    def _twist(self):
        mt = self._mt
        N, M = self._N, self._M
        for i in range(N):
            y = (mt[i] & 0x80000000) | (mt[(i + 1) % N] & 0x7FFFFFFF)
            mt[i] = mt[(i + M) % N] ^ (y >> 1) ^ (0x9908B0DF if (y & 1) else 0)
        self._index = 0

    def __call__(self):
        if self._index >= self._N:
            self._twist()
        y = self._mt[self._index]
        self._index += 1
        y ^= (y >> 11)
        y ^= (y << 7) & 0x9D2C5680
        y ^= (y << 15) & 0xEFC60000
        y ^= (y >> 18)
        return y & 0xFFFFFFFF


def generate_canonical(urng):
    """std::generate_canonical<double, 53>(urng) as implemented by libstdc++."""
    r = float(urng.max() - urng.min() + 1)  # 2^32
    k = 2  # max(1, (53 + log2(r) - 1) / log2(r))
    total = 0.0
    tmp = 1.0
    for _ in range(k):
        total += float(urng() - urng.min()) * tmp
        tmp *= r
    ret = total / tmp
    if ret >= 1.0:
        ret = math.nextafter(1.0, 0.0)
    return ret


class uniform_real_distribution:
    """std::uniform_real_distribution<double>(a, b)"""

    def __init__(self, a=0.0, b=1.0):
        self._a = a
        self._b = b

    def __call__(self, urng):
        return (generate_canonical(urng) * (self._b - self._a)) + self._a


class normal_distribution:
    """std::normal_distribution<double>(mean, stddev) (libstdc++ Marsaglia polar method).

    Like the C++ object, each instance caches the second value of every generated pair.
    """

    def __init__(self, mean=0.0, stddev=1.0):
        self._mean = mean
        self._stddev = stddev
        self._saved = 0.0
        self._saved_available = False

    def reset(self):
        self._saved_available = False

    def __call__(self, urng):
        if self._saved_available:
            self._saved_available = False
            ret = self._saved
        else:
            while True:
                x = 2.0 * generate_canonical(urng) - 1.0
                y = 2.0 * generate_canonical(urng) - 1.0
                r2 = x * x + y * y
                if not (r2 > 1.0 or r2 == 0.0):
                    break
            mult = math.sqrt(-2 * math.log(r2) / r2)
            self._saved = x * mult
            self._saved_available = True
            ret = y * mult
        return ret * self._stddev + self._mean
