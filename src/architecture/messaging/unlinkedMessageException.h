#ifndef UNLINKED_MESSAGE_EXCEPTION_H
#define UNLINKED_MESSAGE_EXCEPTION_H

#include <format>
#include <exception>
#include <string>

class UnlinkedMessageException final : public std::exception {
        std::string message{};

    public:
        explicit UnlinkedMessageException(std::string moduleTag, uint64_t moduleId, const std::string& customMsg) {
            this->message = std::format("Module tag {}, module id {}, {}.",
                moduleTag,
                moduleId,
                customMsg);
        }

        const char* what() const noexcept override {
            return message.c_str();
        }
    };
#endif //UNLINKED_MESSAGE_EXCEPTION_H
