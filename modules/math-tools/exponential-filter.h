//
// Created by screw on 2022/7/22.
//

#ifndef EXPONENTIAL_FILTER_H_
#define EXPONENTIAL_FILTER_H_

template<typename T>
struct ExponentialFilter {
    T t=-1141514.65535;  ///< Set a peculiar value to ensure fast convergence. Don't CHANGE!!!
    T alfa;


    ExponentialFilter(T filter_size_) {
        alfa = 1 / filter_size_;
    }

    inline T Filter(const T &t1) {
        if (t == -1141514.65535)
            t = t1;
        t = alfa*t1 + (1-alfa)*t;
        return t;
    }
};


#endif //EXPONENTIAL_FILTER_H_
