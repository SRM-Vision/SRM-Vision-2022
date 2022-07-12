#ifndef RK4_SOLVER_H_
#define RK4_SOLVER_H_

/**
 * @brief RK4 solver package.
 * @details Visit https://en.wikipedia.org/wiki/Runge%E2%80%93Kutta_methods for more information.
 * @tparam T Type of t.
 * @tparam Y Type of y.
 * @tparam F Type of f.
 */
template<class T, class Y, class F>
struct RK4Solver {
    T t, h;
    Y y;
    F f;

    /**
     * @brief Iterate once.
     * @details dy/dt = f(t, y).
     */
    inline void Iterate() {
        Y k1 = f(t, y),
                k2 = f(t + h / 2, y + k1 * h / 2),
                k3 = f(t + h / 2, y + k2 * h / 2),
                k4 = f(t + h, y + k3 * h);
        y = y + h * (k1 + 2 * k2 + 2 * k3 + k4) / 6;
        t = t + h;
    }
};

#endif  // RK4_SOLVER_H_
