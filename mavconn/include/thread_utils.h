#pragma once

#include <thread>   // C++11 标准库提供的跨平台线程接口，封装了线程创建、管理、同步等功能，属于 C++ 原生多线程库。
#include <string>
#include <cstdio>
#include <sstream>
#include <pthread.h>    // POSIX 线程（pthread）的 C 语言接口，属于系统级线程库（仅支持 Linux/Unix 系统），提供比 <thread> 更底层的线程控制能力

namespace mavconn{
namespace utils{

    /**
     * format
     * @brief 用于将多个参数，短字符串等格式化成标准的字符串。
     *
     * @param fmt       用于格式化args以及表达含义的字符串
     * @param args      模版多参数（值传递）
     *
     * @return std::string
     */
    template<typename ... Args>
    std::string format(const std::string &fmt, Args ... args) {
        std::string ret;

        // 动态缓冲区分配（避免截断)
        auto sz = std::snprintf(nullptr, 0, fmt.c_str(), args ...);
        if (sz < 0) {
            return "Format string failed!";
        }

        // ret.reserve(sz + 1);
        // ret.resize(sz);
        ret.resize(sz + 1);

        // std::snprintf(&ret.front(), ret.capacity() + 1, fmt.c_str(), args ...);
        std::snprintf(ret.data(), sz + 1, fmt.c_str(), args ...);
        ret.pop_back(); // 去除尾部隐藏的“\0”

        return ret;
    }

    /**
     * set_this_thread_name
     * @brief 为线程专门命名
     *
     * @param name:     线程新名称
     * @param args:     模版多参数（万能应用）
     *
     * @return bool  命名成功(true)/命名失败(false)
     */
    template<typename ... Args>
    bool set_this_thread_name(const std::string &name, Args&& ... args) {
        auto new_name = format(name, std::forward<Args>(args)...);

        // Linux 系统中 pthread_setname_np 对线程名称长度有限制（通常最多 15 个字符，含 \0 终止符）
        // pthread_self 用于获取当前调用线程的线程 ID（pthread_t 类型）
        pthread_t pth = pthread_self();
        return pthread_setname_np(pth, new_name.c_str()) == 0;
    }

    /**
     * to_string_ss
     * @brief 将任意流数据转换成字符串
     *
     * @param obj:      流数据
     *
     * @return std::string
     */
    template<typename T>
    inline const std::string to_string_ss(T &obj) {
        std::ostringstream ss;
        ss << obj;
        return ss.str();
    }

    /**
     * operator"" _KiB
     * @brief KB字节计算转B字节
     *
     * @return size_t
     */
    constexpr size_t operator"" _KiB(unsigned long long sz) {
        return sz * 1024;
    }



} //namespace utils
} //namespace mavconn
