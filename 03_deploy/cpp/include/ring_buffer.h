#pragma once
#include <array> <mutex> <condition_variable> <optional>
template<typename T, size_t N=2>
class RingBuffer {
public:
    RingBuffer():head_(0),tail_(0),count_(0){}
    void push(T item){std::unique_lock<std::mutex>l(mtx_);not_full_.wait(l,[this]{return count_<N;});buffer_[head_]=std::move(item);head_=(head_+1)%N;count_++;not_empty_.notify_one();}
    bool try_push(T item){std::unique_lock<std::mutex>l(mtx_,std::try_to_lock);if(!l||count_>=N)return false;buffer_[head_]=std::move(item);head_=(head_+1)%N;count_++;not_empty_.notify_one();return true;}
    T pop(){std::unique_lock<std::mutex>l(mtx_);not_empty_.wait(l,[this]{return count_>0;});T item=std::move(buffer_[tail_]);tail_=(tail_+1)%N;count_--;not_full_.notify_one();return item;}
    std::optional<T> try_pop(){std::unique_lock<std::mutex>l(mtx_,std::try_to_lock);if(!l||count_==0)return std::nullopt;T item=std::move(buffer_[tail_]);tail_=(tail_+1)%N;count_--;not_full_.notify_one();return item;}
    size_t size()const{return count_;}
    bool empty()const{return count_==0;}
private:
    std::array<T,N> buffer_;size_t head_,tail_,count_;
    std::mutex mtx_;std::condition_variable not_full_,not_empty_;
};
