//
// Created by xiguang on 2022/7/17.
//

#ifndef FILTER_H_
#define FILTER_H_

/**
 * \brief a recursive mean filter used
 * \tparam T type need to filtering.
 * \tparam LENGTH the max length to count.
 */
template<typename T, int LENGTH>
class FilterDTMean{
public:
    explicit FilterDTMean() = default;

    ~FilterDTMean() = default;

    T operator()(const T& data){
        data_queue_.push(data);

        if(data_queue_.size() > LENGTH){
            sum_ += ((data - data_queue_.front()) / LENGTH);
            data_queue_.pop();
        }else
            sum_ = (sum_ * (data_queue_.size() - 1) + data) / data_queue_.size();
        return sum_;
    }

    void Reset(){
        data_queue_ = std::queue<T>{};
        sum_ = 0;
    }

private:
    std::queue<T> data_queue_;
    T sum_{0};

};

#endif //FILTER_H_
