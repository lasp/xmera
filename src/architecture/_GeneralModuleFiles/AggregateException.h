// SPDX-License-Identifier: ISC

#ifndef XMERA_AGGREGATEEXCEPTION_H
#define XMERA_AGGREGATEEXCEPTION_H

#include <exception>
#include <string>
#include <vector>
#include <memory>

class AggregateException final : public std::exception {
   public:
    AggregateException(const std::string& message) : msg(message) {}

    const char* what() const noexcept override { return this->msg.c_str(); }

    void add_exception(const std::exception_ptr& ex_ptr) { this->exceptions.push_back(ex_ptr); }

    const std::vector<std::exception_ptr>& get_exceptions() const { return this->exceptions; }

   private:
    std::string msg;
    std::vector<std::exception_ptr> exceptions;
};

#endif  // XMERA_AGGREGATEEXCEPTION_H
