// deep_stack.hpp
#pragma once

#include <deque>
#include "stm32f1xx_hal.h"
#include <type_traits>

template<typename T>
class DeepStack {
    static_assert(std::is_same<T, float>::value || std::is_same<T, uint32_t>::value,
                  "DeepStack only supports float or uint32_t");

private:
    std::deque<T> stack;
    static constexpr size_t maxDepth = 15;

public:
    void push(T value) {
        stack.push_back(value);
        if (stack.size() > maxDepth) {
            stack.pop_front();
        }
    }

    T top() const {
        return stack.back();
    }

    T bottom() const {
        return stack.front();
    }

    T delta() const {
        if (stack.empty()) {
            return 0;
        }
        return top() - bottom();
    }

    T sum() const {
    	if (stack.empty()) {
    		return 0;
    	}

    	T total = 0;
		for (const T& val : stack) {
			total += val;
		}
		return total;
    }

    size_t size() const {
        return stack.size();
    }
};
